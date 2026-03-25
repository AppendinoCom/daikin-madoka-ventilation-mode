/**
 * madoka_ble.cpp — ESP-IDF BLE GATTC client for Daikin Madoka BRC1H.
 *
 * Handles:
 * - BLE scanning and connection
 * - LE Secure Connections pairing with auto-confirm (Numeric Comparison)
 * - GATT service/characteristic discovery
 * - Notification subscription (CCCD write)
 * - Madoka TLV protocol (command encoding, response parsing, chunk assembly)
 * - Periodic polling of thermostat state
 * - ESPHome sensor/climate entity updates
 */

#include "madoka_ble.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "nvs_flash.h"

#include <cmath>

namespace madoka_ble {

static const char *const TAG = "madoka_ble";

using esphome::millis;

// Singleton
MadokaBLE *MadokaBLE::instance_ = nullptr;

// ─── TLV command scan table ─────────────────────────────────
// Phase 6: Test SET 0x4030 (mode) submode 0x1F as ventilation fan speed control.
// Current: 0x1F=0x02. Hypothesis: 0x01=LOW, 0x02=HIGH for ventilation fan.
// Also test: SET 0x5013 (EKEA addressing), and SET without priv mode.
const uint16_t MadokaBLE::SCAN_CMDS[] = {
    // Baseline reads
    0x0030,  // mode (has 0x1F=02, 0x20=05=VENT)
    0x0410,  // extended status (has 0x1A=01)
    // Enter priv mode
    0xFF01,  // ENTER_PRIV_MODE
    // Test 1: SET 0x4030 {0x1F=0x01, 0x20=0x05} — change submode from 02 to 01 (LOW?)
    0xFFC0,  // sentinel
    0x0030,  // verify mode change
    0x0410,  // check extended status
    // Wait and re-read
    0x0030,  // second read
    // Test 2: SET 0x4030 {0x1F=0x02, 0x20=0x05} — restore to HIGH
    0xFFC1,  // sentinel
    0x0030,  // verify
    0x0410,  // check extended status
    // Test 3: SET 0x4030 {0x1F=0x00, 0x20=0x05} — try submode 0 (AUTO?)
    0xFFC2,  // sentinel
    0x0030,  // verify
    // Test 4: SET 0x4030 {0x1F=0x02, 0x20=0x05} — restore
    0xFFC1,  // sentinel
    0x0030,  // verify
    // Test 5: try SET 0x5013 {0x31=0x02} (EKEA_10_13 with SET prefix)
    0xFFC3,  // sentinel
    0x1013,  // readback
    // Test 6: try GET 0x0413 (EKEA alternative mapping)
    0x0413,
    // Test 7: try SET 0x4030 {0x1F=0x01} — WITHOUT 0x20 param
    0xFFC4,  // sentinel
    0x0030,  // verify
};
const int MadokaBLE::SCAN_CMD_COUNT = sizeof(MadokaBLE::SCAN_CMDS) / sizeof(MadokaBLE::SCAN_CMDS[0]);

// GATTC app ID
static const uint8_t GATTC_APP_ID = 0;

// ─── Utility: UUID comparison ───────────────────────────────
static bool uuid128_equal(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 16) == 0;
}

// ─── MadokaBLE: setup() ────────────────────────────────────
void MadokaBLE::setup() {
  instance_ = this;

  ESP_LOGI(TAG, "Initializing Madoka BLE bridge for %02X:%02X:%02X:%02X:%02X:%02X",
           target_addr_[0], target_addr_[1], target_addr_[2],
           target_addr_[3], target_addr_[4], target_addr_[5]);

  // Initialize NVS (needed for BLE bonding storage)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }

  // Release classic BT memory (BLE only)
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  // Init BT controller
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
    state_ = STATE_ERROR;
    return;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
    state_ = STATE_ERROR;
    return;
  }

  // Init Bluedroid
  ret = esp_bluedroid_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
    state_ = STATE_ERROR;
    return;
  }

  ret = esp_bluedroid_enable();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
    state_ = STATE_ERROR;
    return;
  }

  // Register callbacks
  esp_ble_gap_register_callback(gap_event_handler);
  esp_ble_gattc_register_callback(gattc_event_handler);

  // Configure security: LE Secure Connections with Numeric Comparison
  // IO capability = DisplayYesNo (we auto-confirm in the callback)
  esp_ble_io_cap_t io_cap = ESP_IO_CAP_IO;  // DisplayYesNo
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
  uint8_t key_size = 16;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &io_cap, sizeof(io_cap));
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(rsp_key));

  // Set MTU
  esp_ble_gatt_set_local_mtu(23);  // BRC1H uses 20-byte chunks, standard MTU

  // Register GATTC application
  esp_ble_gattc_app_register(GATTC_APP_ID);

  // Link climate parent
  if (climate_) {
    climate_->set_parent(this);
  }

  state_ = STATE_WAIT_SCAN;
  last_state_change_ms_ = millis();
  ESP_LOGI(TAG, "BLE stack initialized, will start scanning");
}

// ─── MadokaBLE: loop() ─────────────────────────────────────
void MadokaBLE::loop() {
  uint32_t now = millis();

  switch (state_) {
    case STATE_WAIT_SCAN:
      // Startup delay: wait before first BLE scan to allow phone to connect for comparison test
      if (!app_registered_) break;
      if (!first_scan_started_) {
        uint32_t delay = now - last_state_change_ms_;
        if (delay < startup_delay_ms_) {
          if (delay % 10000 < 20) {  // Log roughly every 10s
            ESP_LOGI(TAG, "BLE scan delayed: %lus remaining (connect phone now!)",
                     (unsigned long)(startup_delay_ms_ - delay) / 1000);
          }
          break;
        }
        first_scan_started_ = true;
      }
      start_scan_();
      break;

    case STATE_READY:
      // Process pending write commands immediately
      if (!write_queue_.empty()) {
        auto chunk = write_queue_.front();
        write_queue_.erase(write_queue_.begin());
        esp_ble_gattc_write_char(gattc_if_, conn_id_, write_handle_,
                                 chunk.size(), chunk.data(),
                                 ESP_GATT_WRITE_TYPE_NO_RSP,
                                 ESP_GATT_AUTH_REQ_NONE);
      }
      // Fan SET sequence timeout (3s per step)
      if (fan_set_step_ != FAN_SET_IDLE && fan_set_sent_ms_ != 0 &&
          (now - fan_set_sent_ms_) > 3000) {
        ESP_LOGW(TAG, "Fan SET step %d timed out (vent=%d), advancing...", (int) fan_set_step_, vent_fan_set_);
        if (fan_set_step_ == FAN_SET_WAIT_PRIV) {
          // HVAC: Priv timed out — send SET_FAN anyway
          ESP_LOGI(TAG, "Timeout: sending CMD_SET_FAN (0x4050) 0x20=%d, 0x21=%d",
                   pending_fan_val_, pending_fan_val_);
          auto fan_cmd = build_command_(CMD_SET_FAN, {
              {PARAM_FAN_COOL, {pending_fan_val_}},
              {PARAM_FAN_HEAT, {pending_fan_val_}},
          });
          send_chunks_(fan_cmd);
          fan_set_step_ = FAN_SET_WAIT_FAN;
          fan_set_sent_ms_ = now;
        } else if (fan_set_step_ == FAN_SET_WAIT_FAN) {
          // SET_FAN timed out — send SET_MODE anyway (HVAC path only)
          ESP_LOGI(TAG, "Timeout: sending CMD_SET_MODE 0x20=mode");
          auto mode_cmd = build_command_(CMD_SET_MODE, {
              {PARAM_MODE, {(uint8_t) op_mode_}},
          });
          send_chunks_(mode_cmd);
          fan_set_step_ = FAN_SET_WAIT_MODE;
          fan_set_sent_ms_ = now;
        } else {
          // SET_MODE or SET_VENTILATION timed out — give up
          ESP_LOGW(TAG, "Fan SET sequence timed out completely");
          fan_set_step_ = FAN_SET_IDLE;
        }
      }
      // Periodic polling
      if (now - last_poll_ms_ >= poll_interval_ms_) {
        start_polling_();
      }
      // ─── One-time TLV command scan ───
      if (scan_mode_ && fan_set_step_ == FAN_SET_IDLE && write_queue_.empty()) {
        if (scan_next_at_ms_ == 0) {
          // Start scan after first poll cycle
          scan_next_at_ms_ = now + 2000;  // 2s delay after READY
          ESP_LOGI(TAG, "🔍 TLV scan phase 6: %d commands (submode 0x1F tests)", SCAN_CMD_COUNT);
        } else if (now >= scan_next_at_ms_ && scan_idx_ < SCAN_CMD_COUNT) {
          uint16_t cmd = SCAN_CMDS[scan_idx_];
          std::vector<std::vector<uint8_t>> chunks;
          uint32_t delay = 2000;  // default 2s between commands

          if (cmd == 0xFF01) {
            ESP_LOGI(TAG, "🔍 SCAN [%d/%d] ENTER_PRIV_MODE (0x4112)", scan_idx_ + 1, SCAN_CMD_COUNT);
            chunks = build_command_(CMD_ENTER_PRIV_MODE, {{0xFE, {0x01}}});
          } else if (cmd == 0xFFC0) {
            ESP_LOGI(TAG, "🔍 SCAN [%d/%d] SET 0x4030 {0x1F=0x01, 0x20=0x05} (submode→LOW?)", scan_idx_ + 1, SCAN_CMD_COUNT);
            chunks = build_command_(CMD_SET_MODE, {{0x1F, {0x01}}, {PARAM_MODE, {0x05}}});
            delay = 5000;
          } else if (cmd == 0xFFC1) {
            ESP_LOGI(TAG, "🔍 SCAN [%d/%d] SET 0x4030 {0x1F=0x02, 0x20=0x05} (submode→HIGH/restore)", scan_idx_ + 1, SCAN_CMD_COUNT);
            chunks = build_command_(CMD_SET_MODE, {{0x1F, {0x02}}, {PARAM_MODE, {0x05}}});
            delay = 5000;
          } else if (cmd == 0xFFC2) {
            ESP_LOGI(TAG, "🔍 SCAN [%d/%d] SET 0x4030 {0x1F=0x00, 0x20=0x05} (submode→AUTO?)", scan_idx_ + 1, SCAN_CMD_COUNT);
            chunks = build_command_(CMD_SET_MODE, {{0x1F, {0x00}}, {PARAM_MODE, {0x05}}});
            delay = 5000;
          } else if (cmd == 0xFFC3) {
            ESP_LOGI(TAG, "🔍 SCAN [%d/%d] SET 0x5013 {0x31=0x02} (EKEA_10_13 SET)", scan_idx_ + 1, SCAN_CMD_COUNT);
            chunks = build_command_(0x5013, {{0x31, {0x02}}});
            delay = 5000;
          } else if (cmd == 0xFFC4) {
            ESP_LOGI(TAG, "🔍 SCAN [%d/%d] SET 0x4030 {0x1F=0x01} (submode only, no mode)", scan_idx_ + 1, SCAN_CMD_COUNT);
            chunks = build_command_(CMD_SET_MODE, {{0x1F, {0x01}}});
            delay = 5000;
          } else {
            ESP_LOGI(TAG, "🔍 SCAN [%d/%d] sending GET 0x%04X", scan_idx_ + 1, SCAN_CMD_COUNT, cmd);
            chunks = build_command_(cmd);
          }

          send_chunks_(chunks);
          scan_idx_++;
          scan_next_at_ms_ = now + delay;
        } else if (scan_idx_ >= SCAN_CMD_COUNT) {
          ESP_LOGI(TAG, "🔍 TLV scan phase 6 complete (%d commands tested)", SCAN_CMD_COUNT);
          scan_mode_ = false;
        }
      }
      break;

    case STATE_POLLING:
      // Wait between queries (BT callbacks set response_received_)
      if (response_received_) {
        response_received_ = false;
        poll_next_at_ms_ = now + 200;  // 200ms gap between queries
      }
      if (poll_next_at_ms_ != 0 && now >= poll_next_at_ms_) {
        poll_next_at_ms_ = 0;
        poll_step_++;
        send_next_query_();
      }
      // Timeout: if no response within 5s, skip to next step
      if (poll_next_at_ms_ == 0 && !response_received_ &&
          query_sent_ms_ != 0 && (now - query_sent_ms_) > 5000) {
        ESP_LOGW(TAG, "Poll step %d: no response in 5s, skipping", poll_step_);
        poll_next_at_ms_ = 0;
        poll_step_++;
        send_next_query_();
      }
      // Write queue processing — send one chunk per loop iteration
      if (!write_queue_.empty()) {
        auto chunk = write_queue_.front();
        write_queue_.erase(write_queue_.begin());
        esp_ble_gattc_write_char(gattc_if_, conn_id_, write_handle_,
                                 chunk.size(), chunk.data(),
                                 ESP_GATT_WRITE_TYPE_NO_RSP,
                                 ESP_GATT_AUTH_REQ_NONE);
      }
      break;

    case STATE_DISCONNECTED:
      // Reconnect with backoff
      if (now >= next_reconnect_ms_) {
        ESP_LOGI(TAG, "Reconnecting (attempt %d)...", reconnect_attempts_ + 1);
        state_ = STATE_WAIT_SCAN;
        last_state_change_ms_ = now;
      }
      break;

    case STATE_ERROR:
      // Retry after 30s
      if (now - last_state_change_ms_ > 30000) {
        ESP_LOGW(TAG, "Retrying from error state");
        // Close any lingering GATTC connection so the peripheral can readvertise
        esp_ble_gattc_close(gattc_if_, conn_id_);
        state_ = STATE_WAIT_SCAN;
        last_state_change_ms_ = now;
      }
      break;

    default:
      // Timeout watchdog: if stuck in a state for >30s, reset
      if (state_ != STATE_INIT && now - last_state_change_ms_ > 30000) {
        ESP_LOGW(TAG, "State %d timeout, resetting to scan", (int) state_);
        if (state_ == STATE_CONNECTING || state_ == STATE_CONNECTED ||
            state_ == STATE_DISCOVERING || state_ == STATE_REGISTERING_NOTIFY ||
            state_ == STATE_PAIRING) {
          esp_ble_gattc_close(gattc_if_, conn_id_);
        }
        state_ = STATE_DISCONNECTED;
        reconnect_attempts_++;
        uint32_t delay = std::min<uint32_t>(60000u, 5000u * reconnect_attempts_);
        next_reconnect_ms_ = now + delay;
        last_state_change_ms_ = now;
      }
      break;
  }
}

void MadokaBLE::dump_config() {
  ESP_LOGCONFIG(TAG, "Madoka BLE Bridge:");
  ESP_LOGCONFIG(TAG, "  MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                target_addr_[0], target_addr_[1], target_addr_[2],
                target_addr_[3], target_addr_[4], target_addr_[5]);
  ESP_LOGCONFIG(TAG, "  Poll interval: %u ms", poll_interval_ms_);
}

void MadokaBLE::set_mac_address(uint64_t addr) {
  // ESPHome mac_address.as_hex gives a 64-bit value, extract 6 bytes
  target_addr_[0] = (addr >> 40) & 0xFF;
  target_addr_[1] = (addr >> 32) & 0xFF;
  target_addr_[2] = (addr >> 24) & 0xFF;
  target_addr_[3] = (addr >> 16) & 0xFF;
  target_addr_[4] = (addr >> 8) & 0xFF;
  target_addr_[5] = addr & 0xFF;
}

// ─── BLE Operations ─────────────────────────────────────────

void MadokaBLE::start_scan_() {
  ESP_LOGI(TAG, "Starting BLE scan for Madoka...");
  state_ = STATE_SCANNING;
  last_state_change_ms_ = millis();

  esp_ble_scan_params_t scan_params = {};
  scan_params.scan_type = BLE_SCAN_TYPE_ACTIVE;
  scan_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
  scan_params.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL;
  scan_params.scan_interval = 0x50;  // 50ms
  scan_params.scan_window = 0x30;    // 30ms
  scan_params.scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE;
  esp_ble_gap_set_scan_params(&scan_params);
}

void MadokaBLE::connect_() {
  ESP_LOGI(TAG, "Connecting to %02X:%02X:%02X:%02X:%02X:%02X ...",
           target_addr_[0], target_addr_[1], target_addr_[2],
           target_addr_[3], target_addr_[4], target_addr_[5]);
  state_ = STATE_CONNECTING;
  last_state_change_ms_ = millis();

  // Stop scanning before connecting
  esp_ble_gap_stop_scanning();

  // Clear GATT cache before connecting — force fresh service discovery
  // This ensures we see ALL services including any that might be hidden in cache
  esp_ble_gattc_cache_refresh(target_addr_);
  cccd_done_ = false;  // reset so fresh discovery does full subscribe
  ESP_LOGI(TAG, "GATT cache cleared — will do fresh discovery");

  esp_ble_gattc_open(gattc_if_, target_addr_, BLE_ADDR_TYPE_PUBLIC, true);
}

void MadokaBLE::discover_services_() {
  ESP_LOGI(TAG, "Discovering GATT services...");
  state_ = STATE_DISCOVERING;
  last_state_change_ms_ = millis();
  esp_ble_gattc_search_service(gattc_if_, conn_id_, nullptr);
}

void MadokaBLE::register_notify_() {
  if (notify_handle_ == 0) {
    ESP_LOGE(TAG, "Notify handle not found!");
    state_ = STATE_ERROR;
    last_state_change_ms_ = millis();
    return;
  }

  ESP_LOGI(TAG, "Subscribing to notifications (handle=%d, cccd=%d)...",
           notify_handle_, cccd_handle_);
  state_ = STATE_REGISTERING_NOTIFY;
  last_state_change_ms_ = millis();

  // Register for notifications in the ESP-IDF BLE stack
  esp_ble_gattc_register_for_notify(gattc_if_, target_addr_, notify_handle_);
}

void MadokaBLE::start_polling_() {
  poll_step_ = 0;
  state_ = STATE_POLLING;
  last_state_change_ms_ = millis();
  last_poll_ms_ = millis();
  send_next_query_();
}

void MadokaBLE::send_next_query_() {
  std::vector<std::vector<uint8_t>> chunks;

  switch (poll_step_) {
    case 0: chunks = build_command_(CMD_GET_POWER); break;
    case 1: chunks = build_command_(CMD_GET_MODE); break;
    case 2: chunks = build_command_(CMD_GET_SETPOINT); break;
    case 3: chunks = build_command_(CMD_GET_FAN); break;
    case 4: chunks = build_command_(CMD_GET_TEMPERATURES); break;
    case 5: chunks = build_command_(CMD_GET_CLEAN_FILTER); break;
    case 6:  // Ventilation state (TLV command 0x0031)
      chunks = build_command_(CMD_GET_VENTILATION);
      break;
    case 7:  // Optional: firmware version (once)
      if (!version_fetched_) {
        chunks = build_command_(CMD_GET_VERSION);
      } else {
        poll_step_++;
        // Fall through to eye brightness
      }
      break;
    case 8:  // Optional: eye brightness (once)
      if (!eye_brightness_fetched_) {
        chunks = build_command_(CMD_GET_EYE_BRIGHT, {
            {PARAM_EYE_BRIGHT, {0x00}},
        });
      } else {
        poll_step_++;
        // Fall through to end of polling
      }
      break;
    default:
      state_ = STATE_READY;
      last_state_change_ms_ = millis();
      update_climate_state_();
      return;
  }

  if (!chunks.empty()) {
    send_chunks_(chunks);
    query_sent_ms_ = millis();
  }
}

// ─── Protocol Implementation ────────────────────────────────

std::vector<std::vector<uint8_t>> MadokaBLE::build_command_(
    uint16_t cmd_id, const std::map<uint8_t, std::vector<uint8_t>> &params) {
  // Build payload: [total_len] [0x00] [cmd_hi] [cmd_lo] [TLV params...]
  std::vector<uint8_t> tlv;
  if (params.empty()) {
    tlv.push_back(0x00);
    tlv.push_back(0x00);
  } else {
    for (auto &[pid, val] : params) {
      tlv.push_back(pid);
      tlv.push_back((uint8_t) val.size());
      tlv.insert(tlv.end(), val.begin(), val.end());
    }
  }

  std::vector<uint8_t> payload;
  payload.push_back((uint8_t) (4 + tlv.size()));  // total_len
  payload.push_back(0x00);                         // fixed
  payload.push_back((cmd_id >> 8) & 0xFF);         // cmd_hi
  payload.push_back(cmd_id & 0xFF);                // cmd_lo
  payload.insert(payload.end(), tlv.begin(), tlv.end());

  // Split into 20-byte BLE chunks: [chunk_id (1B)] [up to 19B payload]
  std::vector<std::vector<uint8_t>> chunks;
  size_t offset = 0;
  uint8_t chunk_id = 0;
  while (offset < payload.size()) {
    size_t chunk_len = std::min((size_t) 19, payload.size() - offset);
    std::vector<uint8_t> chunk;
    chunk.push_back(chunk_id);
    chunk.insert(chunk.end(), payload.begin() + offset, payload.begin() + offset + chunk_len);
    chunks.push_back(std::move(chunk));
    offset += chunk_len;
    chunk_id++;
  }
  return chunks;
}

void MadokaBLE::send_chunks_(const std::vector<std::vector<uint8_t>> &chunks) {
  // Queue all chunks; loop() sends one per iteration
  for (auto &c : chunks) {
    write_queue_.push_back(c);
  }
}

std::pair<uint16_t, std::map<uint8_t, std::vector<uint8_t>>>
MadokaBLE::parse_response_(const std::vector<uint8_t> &data) {
  uint16_t cmd_id = 0;
  std::map<uint8_t, std::vector<uint8_t>> values;

  if (data.size() < 4) return {0, values};

  // data[0] = total_len, data[1] = 0x00, data[2..3] = cmd_id
  cmd_id = ((uint16_t) data[2] << 8) | data[3];

  size_t i = 4;
  while (i + 1 < data.size()) {
    uint8_t pid = data[i];
    uint8_t plen = data[i + 1];
    if (plen == 0xFF) plen = 0;
    i += 2;
    std::vector<uint8_t> val;
    if (plen > 0 && i + plen <= data.size()) {
      val.assign(data.begin() + i, data.begin() + i + plen);
    }
    if (val.empty()) val.push_back(0x00);
    values[pid] = std::move(val);
    i += plen;
  }

  return {cmd_id, values};
}

void MadokaBLE::process_response_(uint16_t cmd_id,
    const std::map<uint8_t, std::vector<uint8_t>> &values) {
  ESP_LOGD(TAG, "Response cmd=0x%04X, %d params", cmd_id, (int) values.size());

  switch (cmd_id) {
    case CMD_GET_POWER: {
      // Dump all params for debugging
      for (auto &kv : values) {
        std::string hex;
        for (auto b : kv.second) {
          char tmp[4];
          snprintf(tmp, sizeof(tmp), "%02X ", b);
          hex += tmp;
        }
        ESP_LOGI(TAG, "Power param 0x%02X [%d bytes]: %s", kv.first, (int)kv.second.size(), hex.c_str());
      }
      auto it = values.find(PARAM_POWER);
      if (it != values.end()) {
        power_on_ = (it->second[0] == 0x01);
        ESP_LOGD(TAG, "Power: %s", power_on_ ? "ON" : "OFF");
      }
      break;
    }
    case CMD_GET_MODE: {
      // Dump all params for debugging
      for (auto &kv : values) {
        std::string hex;
        for (auto b : kv.second) {
          char tmp[4];
          snprintf(tmp, sizeof(tmp), "%02X ", b);
          hex += tmp;
        }
        ESP_LOGI(TAG, "Mode param 0x%02X [%d bytes]: %s", kv.first, (int)kv.second.size(), hex.c_str());
      }
      auto it = values.find(PARAM_MODE);
      if (it != values.end()) {
        uint8_t raw_mode = it->second[0];
        ESP_LOGD(TAG, "Mode raw: %d", raw_mode);
        // BRC1H values: 0=FAN, 1=DRY, 2=AUTO, 3=COOL, 4=HEAT, 5=VENTILATION
        if (raw_mode <= 5) {
          op_mode_ = (OpMode) raw_mode;
        } else {
          ESP_LOGW(TAG, "Unknown mode: %d, defaulting to AUTO", raw_mode);
          op_mode_ = OpMode::AUTO;
        }
      }
      break;
    }
    case CMD_GET_SETPOINT: {
      // Dump ALL params for debugging (22 params on BRC1H52W!)
      for (auto &kv : values) {
        std::string hex;
        for (auto b : kv.second) {
          char tmp[4];
          snprintf(tmp, sizeof(tmp), "%02X ", b);
          hex += tmp;
        }
        ESP_LOGI(TAG, "Setpoint param 0x%02X [%d bytes]: %s", kv.first, (int)kv.second.size(), hex.c_str());
      }
      auto it_c = values.find(PARAM_SETPOINT_COOL);
      auto it_h = values.find(PARAM_SETPOINT_HEAT);
      if (it_c != values.end() && it_c->second.size() >= 2) {
        uint16_t raw = ((uint16_t) it_c->second[0] << 8) | it_c->second[1];
        cooling_sp_ = roundf((float) raw / 128.0f);
        ESP_LOGD(TAG, "Cooling SP: %.0f°C", cooling_sp_);
      }
      if (it_h != values.end() && it_h->second.size() >= 2) {
        uint16_t raw = ((uint16_t) it_h->second[0] << 8) | it_h->second[1];
        heating_sp_ = roundf((float) raw / 128.0f);
        ESP_LOGD(TAG, "Heating SP: %.0f°C", heating_sp_);
      }
      break;
    }
    case CMD_GET_FAN: {
      // Dump all params for debugging
      for (auto &kv : values) {
        std::string hex;
        for (auto b : kv.second) {
          char tmp[4];
          snprintf(tmp, sizeof(tmp), "%02X ", b);
          hex += tmp;
        }
        ESP_LOGI(TAG, "Fan param 0x%02X [%d bytes]: %s", kv.first, (int)kv.second.size(), hex.c_str());
      }
      auto it_c = values.find(PARAM_FAN_COOL);
      auto it_h = values.find(PARAM_FAN_HEAT);
      if (it_c != values.end()) {
        uint8_t v = it_c->second[0];
        ESP_LOGI(TAG, "Fan cool raw: %d", v);
        if (v >= 2 && v <= 4) v = 3;  // normalize to MID
        cool_fan_ = (FanSpeed) v;
      }
      if (it_h != values.end()) {
        uint8_t v = it_h->second[0];
        ESP_LOGI(TAG, "Fan heat raw: %d", v);
        if (v >= 2 && v <= 4) v = 3;
        heat_fan_ = (FanSpeed) v;
      }
      break;
    }
    case CMD_GET_TEMPERATURES: {
      // Dump all params for debugging
      for (auto &kv : values) {
        std::string hex;
        for (auto b : kv.second) {
          char tmp[4];
          snprintf(tmp, sizeof(tmp), "%02X ", b);
          hex += tmp;
        }
        ESP_LOGI(TAG, "Temp param 0x%02X [%d bytes]: %s", kv.first, (int)kv.second.size(), hex.c_str());
      }
      auto it_i = values.find(PARAM_TEMP_INDOOR);
      auto it_o = values.find(PARAM_TEMP_OUTDOOR);
      if (it_i != values.end()) {
        indoor_temp_ = (float) it_i->second[0];
        ESP_LOGD(TAG, "Indoor: %.0f°C", indoor_temp_);
        if (indoor_temp_sensor_) indoor_temp_sensor_->publish_state(indoor_temp_);
      }
      if (it_o != values.end()) {
        uint8_t v = it_o->second[0];
        if (v != 0xFF) {
          outdoor_temp_ = (float) v;
          ESP_LOGD(TAG, "Outdoor: %.0f°C", outdoor_temp_);
          if (outdoor_temp_sensor_) outdoor_temp_sensor_->publish_state(outdoor_temp_);
        }
      }
      break;
    }
    case CMD_GET_CLEAN_FILTER: {
      // Dump all params for debugging (16 params on BRC1H52W)
      for (auto &kv : values) {
        std::string hex;
        for (auto b : kv.second) {
          char tmp[4];
          snprintf(tmp, sizeof(tmp), "%02X ", b);
          hex += tmp;
        }
        ESP_LOGI(TAG, "CleanFilter param 0x%02X [%d bytes]: %s", kv.first, (int)kv.second.size(), hex.c_str());
      }
      auto it = values.find(0x62);
      if (it != values.end()) {
        clean_filter_ = (it->second[0] & 0x01) == 1;
      }
      break;
    }
    case CMD_GET_VERSION: {
      // Dump all params for debugging
      for (auto &kv : values) {
        std::string hex;
        for (auto b : kv.second) {
          char tmp[4];
          snprintf(tmp, sizeof(tmp), "%02X ", b);
          hex += tmp;
        }
        ESP_LOGI(TAG, "Version param 0x%02X [%d bytes]: %s", kv.first, (int)kv.second.size(), hex.c_str());
      }
      version_fetched_ = true;
      auto it_rc = values.find(0x45);
      if (it_rc != values.end() && it_rc->second.size() >= 3) {
        char ver[16];
        snprintf(ver, sizeof(ver), "%d.%d.%d",
                 it_rc->second[0], it_rc->second[1], it_rc->second[2]);
        firmware_version_ = ver;
        ESP_LOGI(TAG, "Firmware RC: %s", ver);
      }
      break;
    }
    case CMD_GET_EYE_BRIGHT: {
      auto it = values.find(PARAM_EYE_BRIGHT);
      if (it != values.end()) {
        eye_brightness_ = it->second[0];
        eye_brightness_fetched_ = true;
        ESP_LOGI(TAG, "Eye brightness: %d", eye_brightness_);
      }
      break;
    }
    case CMD_SET_EYE_BRIGHT:
      ESP_LOGI(TAG, "SET_EYE_BRIGHT acknowledged");
      break;
    case CMD_RESET_FILTER:
      ESP_LOGI(TAG, "RESET_FILTER acknowledged");
      clean_filter_ = false;
      break;
    // SET command responses
    case CMD_SET_POWER:
      ESP_LOGD(TAG, "Set command acknowledged: 0x%04X", cmd_id);
      break;
    case CMD_SET_MODE: {
      ESP_LOGI(TAG, "SET_MODE response (0x%04X), %d params:", cmd_id, (int)values.size());
      for (auto &kv : values) {
        std::string hex;
        for (auto b : kv.second) {
          char tmp[4];
          snprintf(tmp, sizeof(tmp), "%02X ", b);
          hex += tmp;
        }
        ESP_LOGI(TAG, "  param 0x%02X [%d bytes]: %s", kv.first, (int)kv.second.size(), hex.c_str());
      }
      // Sequential fan SET complete (HVAC mode path)
      if (fan_set_step_ == FAN_SET_WAIT_MODE && !vent_fan_set_) {
        ESP_LOGI(TAG, "Fan SET sequence complete (HVAC)");
        fan_set_step_ = FAN_SET_IDLE;
      }
      break;
    }
    case CMD_GET_VENTILATION: {
      // Response to GET_VENTILATION (0x0031) — parse ventilation params
      for (auto &kv : values) {
        std::string hex;
        for (auto b : kv.second) {
          char tmp[4];
          snprintf(tmp, sizeof(tmp), "%02X ", b);
          hex += tmp;
        }
        ESP_LOGI(TAG, "Ventilation param 0x%02X [%d bytes]: %s", kv.first, (int)kv.second.size(), hex.c_str());
      }
      // Mark ventilation as supported since we got a valid response
      if (!ventilation_supported_) {
        ventilation_supported_ = true;
        ESP_LOGI(TAG, "Ventilation support detected via TLV 0x0031");
      }
      auto it_rate = values.find(PARAM_VENT_RATE);
      if (it_rate != values.end() && !it_rate->second.empty()) {
        uint8_t rate = it_rate->second[0];
        ESP_LOGI(TAG, "Ventilation rate: %d", rate);
        // Store ventilation rate for fan speed display
        // Device values: 0x01=LOW, 0x05=HIGH (confirmed from device readback)
        if (rate == 0x01) {
          vent_rate_known_ = true;
          ventilation_rate_ = VentilationRate::VENT_LOW;
        } else if (rate >= 0x03) {
          vent_rate_known_ = true;
          ventilation_rate_ = VentilationRate::VENT_HIGH;
        } else {
          vent_rate_known_ = true;
          ventilation_rate_ = VentilationRate::VENT_AUTO;
        }
      }
      auto it_mode = values.find(PARAM_VENT_MODE);
      if (it_mode != values.end() && !it_mode->second.empty()) {
        ESP_LOGI(TAG, "Ventilation mode: %d (0=AUTO, 1=ERV, 2=BYPASS)", it_mode->second[0]);
      }
      break;
    }
    case CMD_SET_VENTILATION: {
      // Response to SET_VENTILATION (0x4031)
      ESP_LOGI(TAG, "SET_VENTILATION response (0x%04X), %d params:", cmd_id, (int)values.size());
      for (auto &kv : values) {
        std::string hex;
        for (auto b : kv.second) {
          char tmp[4];
          snprintf(tmp, sizeof(tmp), "%02X ", b);
          hex += tmp;
        }
        ESP_LOGI(TAG, "  param 0x%02X [%d bytes]: %s", kv.first, (int)kv.second.size(), hex.c_str());
      }
      // Fan SET complete for ventilation mode
      if (fan_set_step_ == FAN_SET_WAIT_MODE && vent_fan_set_) {
        ESP_LOGI(TAG, "Fan SET sequence complete (ventilation)");
        fan_set_step_ = FAN_SET_IDLE;
        // Verify by requesting GET_VENTILATION
        ESP_LOGI(TAG, "Verifying: sending GET_VENTILATION (0x0031)...");
        auto verify_cmd = build_command_(CMD_GET_VENTILATION);
        send_chunks_(verify_cmd);
      }
      break;
    }
    case CMD_SET_SETPOINT:
      ESP_LOGD(TAG, "Set command acknowledged: 0x%04X", cmd_id);
      break;
    case CMD_SET_FAN: {
      ESP_LOGI(TAG, "SET_FAN response (0x%04X), %d params:", cmd_id, (int)values.size());
      for (auto &kv : values) {
        std::string hex;
        for (auto b : kv.second) {
          char tmp[4];
          snprintf(tmp, sizeof(tmp), "%02X ", b);
          hex += tmp;
        }
        ESP_LOGI(TAG, "  param 0x%02X [%d bytes]: %s", kv.first, (int)kv.second.size(), hex.c_str());
      }
      // Sequential fan SET step 3: send CMD_SET_MODE to confirm mode (HVAC path only)
      if (fan_set_step_ == FAN_SET_WAIT_FAN) {
        ESP_LOGI(TAG, "Fan SET step 3/3: Sending CMD_SET_MODE to confirm mode");
        auto mode_cmd = build_command_(CMD_SET_MODE, {
            {PARAM_MODE, {(uint8_t) op_mode_}},
        });
        send_chunks_(mode_cmd);
        fan_set_step_ = FAN_SET_WAIT_MODE;
        fan_set_sent_ms_ = millis();
      }
      break;
    }
    case CMD_ENTER_PRIV_MODE: {
      ESP_LOGI(TAG, "ENTER_PRIV_MODE response (0x%04X), %d params:", cmd_id, (int)values.size());
      for (auto &kv : values) {
        std::string hex;
        for (auto b : kv.second) {
          char tmp[4];
          snprintf(tmp, sizeof(tmp), "%02X ", b);
          hex += tmp;
        }
        ESP_LOGI(TAG, "  param 0x%02X [%d bytes]: %s", kv.first, (int)kv.second.size(), hex.c_str());
      }
      // Sequential fan SET step 2: HVAC mode only (ventilation no longer uses PRIV_MODE)
      if (fan_set_step_ == FAN_SET_WAIT_PRIV) {
        // HVAC mode: send SET_FAN
        ESP_LOGI(TAG, "Fan SET step 2/3: Sending CMD_SET_FAN (0x4050) 0x20=%d, 0x21=%d",
                 pending_fan_val_, pending_fan_val_);
        auto fan_cmd = build_command_(CMD_SET_FAN, {
            {PARAM_FAN_COOL, {pending_fan_val_}},
            {PARAM_FAN_HEAT, {pending_fan_val_}},
        });
        send_chunks_(fan_cmd);
        fan_set_step_ = FAN_SET_WAIT_FAN;
        fan_set_sent_ms_ = millis();
      }
      break;
    }
    default:
      ESP_LOGI(TAG, "Unknown/explore response: 0x%04X, %d params", cmd_id, (int)values.size());
      for (auto &kv : values) {
        std::string hex;
        for (auto b : kv.second) {
          char tmp[4];
          snprintf(tmp, sizeof(tmp), "%02X ", b);
          hex += tmp;
        }
        ESP_LOGI(TAG, "  param 0x%02X [%d bytes]: %s", kv.first, (int)kv.second.size(), hex.c_str());
      }
      break;
  }

  // Signal main loop to advance poll step
  if (state_ == STATE_POLLING) {
    response_received_ = true;
  }
}

void MadokaBLE::update_climate_state_() {
  if (cooling_sp_sensor_ && !std::isnan(cooling_sp_))
    cooling_sp_sensor_->publish_state(cooling_sp_);
  if (heating_sp_sensor_ && !std::isnan(heating_sp_))
    heating_sp_sensor_->publish_state(heating_sp_);

  if (filter_alert_sensor_)
    filter_alert_sensor_->publish_state(clean_filter_);

  if (firmware_version_sensor_ && !firmware_version_.empty())
    firmware_version_sensor_->publish_state(firmware_version_);

  if (eye_brightness_number_ && eye_brightness_fetched_)
    eye_brightness_number_->publish_brightness(eye_brightness_);

  if (climate_) {
    climate_->update_state(power_on_, op_mode_, indoor_temp_,
                           cooling_sp_, heating_sp_, cool_fan_, heat_fan_);
  }

  if (fan_) {
    fan_->update_state(power_on_, ventilation_rate_);
  }
}

// ─── Control Methods (from climate entity) ──────────────────

void MadokaBLE::request_set_power(bool on) {
  ESP_LOGI(TAG, "Set power: %s, state=%d", on ? "ON" : "OFF", (int) state_);
  if (state_ != STATE_READY && state_ != STATE_POLLING) {
    ESP_LOGW(TAG, "Cannot set power: state=%d", (int) state_);
    return;
  }
  auto chunks = build_command_(CMD_SET_POWER, {
      {PARAM_POWER, {(uint8_t)(on ? 0x01 : 0x00)}},
  });
  send_chunks_(chunks);
  power_on_ = on;
}

void MadokaBLE::request_set_mode(OpMode mode) {
  ESP_LOGI(TAG, "Set mode: %d, state=%d", (int) mode, (int) state_);
  if (state_ != STATE_READY && state_ != STATE_POLLING) {
    ESP_LOGW(TAG, "Cannot set mode: state=%d", (int) state_);
    return;
  }
  auto chunks = build_command_(CMD_SET_MODE, {
      {PARAM_MODE, {(uint8_t) mode}},
  });
  send_chunks_(chunks);
  op_mode_ = mode;
}

void MadokaBLE::request_set_setpoint(float cool, float heat) {
  if (state_ != STATE_READY && state_ != STATE_POLLING) return;
  uint16_t cool_raw = (uint16_t)(cool * 128);
  uint16_t heat_raw = (uint16_t)(heat * 128);
  auto chunks = build_command_(CMD_SET_SETPOINT, {
      {PARAM_SETPOINT_COOL, {(uint8_t)(cool_raw >> 8), (uint8_t)(cool_raw & 0xFF)}},
      {PARAM_SETPOINT_HEAT, {(uint8_t)(heat_raw >> 8), (uint8_t)(heat_raw & 0xFF)}},
      {0x30, {0x00}},  // range_enabled
      {0x31, {0x02}},  // mode
      {0x32, {0x00}},  // min_differential
  });
  send_chunks_(chunks);
  cooling_sp_ = cool;
  heating_sp_ = heat;
}

void MadokaBLE::request_set_fan(FanSpeed cool, FanSpeed heat) {
  ESP_LOGI(TAG, "Set fan: cool=%d, heat=%d, state=%d, vent_supported=%d",
           (int) cool, (int) heat, (int) state_, ventilation_supported_);
  if (state_ != STATE_READY && state_ != STATE_POLLING) {
    ESP_LOGW(TAG, "Cannot set fan: state=%d (need READY or POLLING)", (int) state_);
    return;
  }

  uint8_t fan_val = (uint8_t) cool;  // 0=AUTO, 1=LOW, 3=MID, 5=HIGH

  // Store values for sequential sending
  pending_fan_val_ = fan_val;
  vent_fan_set_ = (op_mode_ == OpMode::VENTILATION);

  if (vent_fan_set_) {
    // Ventilation mode: use CMD_SET_VENTILATION (0x4031) with param 0x21 (rate)
    // APK analysis: functionId=49 → command 0x0031/0x4031, rate param=0x21
    uint8_t vent_rate;
    if (cool == FanSpeed::LOW) {
      vent_rate = 0x01;  // Low speed (confirmed from device readback)
    } else if (cool == FanSpeed::MID || cool == FanSpeed::HIGH) {
      vent_rate = 0x05;  // High speed (confirmed: device reports 0x05 for HIGH)
    } else {
      vent_rate = 0x00;  // Auto
    }
    pending_vent_speed_ = vent_rate;
    ESP_LOGI(TAG, "Fan SET (vent): CMD_SET_VENTILATION (0x4031) param 0x21=%d", vent_rate);
    auto vent_cmd = build_command_(CMD_SET_VENTILATION, {
        {PARAM_VENT_RATE, {vent_rate}},
    });
    send_chunks_(vent_cmd);
    fan_set_step_ = FAN_SET_WAIT_MODE;  // reuse step for response tracking
    fan_set_sent_ms_ = millis();
  } else {
    // HVAC mode: enter priv first, then SET_FAN + SET_MODE
    ESP_LOGI(TAG, "Fan SET (HVAC): Sending EnterPrivilegedMode (0x4112)...");
    auto priv_cmd = build_command_(CMD_ENTER_PRIV_MODE, {
        {0xFE, {0x01}},
    });
    send_chunks_(priv_cmd);
    fan_set_step_ = FAN_SET_WAIT_PRIV;
    fan_set_sent_ms_ = millis();
  }

  cool_fan_ = cool;
  heat_fan_ = heat;
}

void MadokaBLE::request_set_ventilation_rate(VentilationRate rate) {
  if (state_ != STATE_READY && state_ != STATE_POLLING) {
    ESP_LOGW(TAG, "Cannot set ventilation: state=%d", (int) state_);
    return;
  }
  if (vent_char_handle_ == 0) {
    ESP_LOGW(TAG, "Ventilation char handle not found");
    return;
  }

  const char *rate_str;
  switch (rate) {
    case VentilationRate::VENT_LOW: rate_str = "LOW"; break;
    case VentilationRate::VENT_MEDIUM: rate_str = "MEDIUM"; break;
    case VentilationRate::VENT_HIGH: rate_str = "HIGH"; break;
    default: rate_str = "AUTO"; break;
  }

  char json[64];
  int len = snprintf(json, sizeof(json), "{\"ventilationRate\":\"%s\"}", rate_str);
  ESP_LOGI(TAG, "Writing ventilation rate: %s (%d bytes, handle=%d)", json, len, vent_char_handle_);

  esp_ble_gattc_write_char(gattc_if_, conn_id_, vent_char_handle_,
                           len, (uint8_t *)json,
                           ESP_GATT_WRITE_TYPE_RSP,
                           ESP_GATT_AUTH_REQ_NONE);
  ventilation_rate_ = rate;
  vent_rate_known_ = true;
}

void MadokaBLE::request_reset_filter() {
  ESP_LOGI(TAG, "Resetting clean filter indicator");
  if (state_ != STATE_READY && state_ != STATE_POLLING) {
    ESP_LOGW(TAG, "Cannot reset filter: state=%d", (int) state_);
    return;
  }
  auto chunks = build_command_(CMD_RESET_FILTER, {
      {0x51, {0x01}},  // disable indicator
      {0xFE, {0x01}},  // reset timer
  });
  send_chunks_(chunks);
}

void MadokaBLE::request_set_eye_brightness(uint8_t level) {
  if (level > 19) level = 19;
  ESP_LOGI(TAG, "Set eye brightness: %d", level);
  if (state_ != STATE_READY && state_ != STATE_POLLING) {
    ESP_LOGW(TAG, "Cannot set eye brightness: state=%d", (int) state_);
    return;
  }
  auto chunks = build_command_(CMD_SET_EYE_BRIGHT, {
      {PARAM_EYE_BRIGHT, {level}},
  });
  send_chunks_(chunks);
  eye_brightness_ = level;
}

// ─── GAP Event Handler ──────────────────────────────────────

void MadokaBLE::gap_event_handler(esp_gap_ble_cb_event_t event,
                                  esp_ble_gap_cb_param_t *param) {
  if (instance_) instance_->handle_gap_event_(event, param);
}

void MadokaBLE::handle_gap_event_(esp_gap_ble_cb_event_t event,
                                  esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
      // Scan params set, start actual scan
      esp_ble_gap_start_scanning(30);  // 30s scan
      break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
      auto &sr = param->scan_rst;
      if (sr.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
        if (memcmp(sr.bda, target_addr_, 6) == 0) {
          ESP_LOGI(TAG, "Found Madoka device (RSSI=%d)", sr.rssi);
          connect_();
        }
      } else if (sr.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
        if (state_ == STATE_SCANNING) {
          ESP_LOGW(TAG, "Scan complete, device not found. Retrying...");
          state_ = STATE_WAIT_SCAN;
          last_state_change_ms_ = millis();
        }
      }
      break;
    }

    // ─── Security events ────────────────────────────────────
    case ESP_GAP_BLE_NC_REQ_EVT:
      // Numeric Comparison request — AUTO-ACCEPT!
      ESP_LOGI(TAG, "🔐 Numeric Comparison: passkey=%06lu — AUTO-ACCEPTING",
               (unsigned long) param->ble_security.key_notif.passkey);
      esp_ble_confirm_reply(param->ble_security.key_notif.bd_addr, true);
      break;

    case ESP_GAP_BLE_SEC_REQ_EVT:
      // Peripheral requested security — accept
      ESP_LOGI(TAG, "Security request from peripheral — accepting");
      esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
      break;

    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
      ESP_LOGI(TAG, "Passkey notify: %06lu",
               (unsigned long) param->ble_security.key_notif.passkey);
      break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
      auto &auth = param->ble_security.auth_cmpl;
      if (auth.success) {
        ESP_LOGI(TAG, "✅ Pairing/bonding successful! auth_mode=%d", auth.auth_mode);
        paired_ = true;
        if (!ventilation_supported_ && !vent_rediscovery_done_) {
          // Re-discover services after bonding — some services are hidden until paired
          vent_rediscovery_done_ = true;
          ESP_LOGI(TAG, "Re-discovering services after bonding (ventilation not yet found)...");
          esp_ble_gattc_cache_refresh(target_addr_);
          discover_services_();
        } else if (state_ == STATE_PAIRING || state_ == STATE_ERROR) {
          // Retry notification subscription after pairing
          register_notify_();
        }
      } else {
        ESP_LOGW(TAG, "❌ Pairing failed: reason=0x%x", auth.fail_reason);
        // Will be retried on reconnect
      }
      break;
    }

    default:
      break;
  }
}

// ─── GATTC Event Handler ────────────────────────────────────

void MadokaBLE::gattc_event_handler(esp_gattc_cb_event_t event,
                                    esp_gatt_if_t gattc_if,
                                    esp_ble_gattc_cb_param_t *param) {
  if (instance_) instance_->handle_gattc_event_(event, gattc_if, param);
}

void MadokaBLE::handle_gattc_event_(esp_gattc_cb_event_t event,
                                    esp_gatt_if_t gattc_if,
                                    esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_REG_EVT: {
      if (param->reg.status == ESP_GATT_OK) {
        gattc_if_ = gattc_if;
        app_registered_ = true;
        ESP_LOGI(TAG, "GATTC app registered (if=%d)", gattc_if);
      } else {
        ESP_LOGE(TAG, "GATTC reg failed: %d", param->reg.status);
        state_ = STATE_ERROR;
        last_state_change_ms_ = millis();
      }
      break;
    }

    case ESP_GATTC_OPEN_EVT: {
      if (param->open.status == ESP_GATT_OK) {
        conn_id_ = param->open.conn_id;
        ESP_LOGI(TAG, "✅ Connected (conn_id=%d)", conn_id_);
        state_ = STATE_CONNECTED;
        last_state_change_ms_ = millis();
        reconnect_attempts_ = 0;

        // Configure MTU
        esp_ble_gattc_send_mtu_req(gattc_if, conn_id_);
      } else {
        ESP_LOGW(TAG, "Connect failed: status=%d", param->open.status);
        state_ = STATE_DISCONNECTED;
        reconnect_attempts_++;
        uint32_t delay = std::min<uint32_t>(60000u, 5000u * reconnect_attempts_);
        next_reconnect_ms_ = millis() + delay;
        last_state_change_ms_ = millis();
      }
      break;
    }

    case ESP_GATTC_CFG_MTU_EVT:
      ESP_LOGD(TAG, "MTU configured: %d", param->cfg_mtu.mtu);
      discover_services_();
      break;

    case ESP_GATTC_SEARCH_RES_EVT: {
      // ─── Log ALL discovered services for enumeration ───
      uint16_t sh = param->search_res.start_handle;
      uint16_t eh = param->search_res.end_handle;
      bool is_madoka = false;
      bool is_vent = false;

      if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128) {
        const uint8_t *u = param->search_res.srvc_id.uuid.uuid.uuid128;
        ESP_LOGI(TAG, "SVC [128] %02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x  h=%d..%d",
                 u[15],u[14],u[13],u[12], u[11],u[10], u[9],u[8],
                 u[7],u[6], u[5],u[4],u[3],u[2],u[1],u[0], sh, eh);
        is_madoka = uuid128_equal(u, MADOKA_SERVICE_UUID);
        is_vent = uuid128_equal(u, VENT_SERVICE_UUID);
        if (is_vent) {
          ventilation_supported_ = true;
          ESP_LOGI(TAG, "✅ Ventilation GATT service FOUND!");
        }
        if (uuid128_equal(u, AIRFLOW_SERVICE_UUID)) {
          airflow_supported_ = true;
          ESP_LOGI(TAG, "✅ Airflow/FanSpeed GATT service FOUND! h=%d..%d", sh, eh);
        }
      } else if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16) {
        ESP_LOGI(TAG, "SVC [16] 0x%04x  h=%d..%d",
                 param->search_res.srvc_id.uuid.uuid.uuid16, sh, eh);
      } else if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_32) {
        ESP_LOGI(TAG, "SVC [32] 0x%08lx  h=%d..%d",
                 (unsigned long)param->search_res.srvc_id.uuid.uuid.uuid32, sh, eh);
      }

      // ─── Enumerate characteristics for EVERY service ───
      {
        uint16_t count = 0;
        esp_gatt_status_t status = esp_ble_gattc_get_attr_count(
            gattc_if_, conn_id_, ESP_GATT_DB_CHARACTERISTIC, sh, eh, 0, &count);

        if (status == ESP_GATT_OK && count > 0) {
          auto *chars = new esp_gattc_char_elem_t[count];
          status = esp_ble_gattc_get_all_char(gattc_if_, conn_id_, sh, eh, chars, &count, 0);

          if (status == ESP_GATT_OK) {
            for (uint16_t i = 0; i < count; i++) {
              if (chars[i].uuid.len == ESP_UUID_LEN_128) {
                const uint8_t *c = chars[i].uuid.uuid.uuid128;
                ESP_LOGI(TAG, "  CHR [128] %02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x  h=%d props=0x%02x",
                         c[15],c[14],c[13],c[12], c[11],c[10], c[9],c[8],
                         c[7],c[6], c[5],c[4],c[3],c[2],c[1],c[0],
                         chars[i].char_handle, chars[i].properties);

                // ─── For Madoka service: save handles ───
                if (is_madoka) {
                  if (uuid128_equal(c, NOTIFY_CHAR_UUID)) {
                    notify_handle_ = chars[i].char_handle;
                    ESP_LOGI(TAG, "    → NOTIFY handle: %d", notify_handle_);

                    // Find CCCD descriptor
                    uint16_t desc_count = 0;
                    esp_ble_gattc_get_attr_count(
                        gattc_if_, conn_id_, ESP_GATT_DB_DESCRIPTOR,
                        chars[i].char_handle,
                        (i + 1 < count) ? chars[i + 1].char_handle - 1 : eh,
                        chars[i].char_handle, &desc_count);

                    if (desc_count > 0) {
                      auto *descs = new esp_gattc_descr_elem_t[desc_count];
                      esp_ble_gattc_get_all_descr(
                          gattc_if_, conn_id_, chars[i].char_handle,
                          descs, &desc_count, 0);
                      for (uint16_t j = 0; j < desc_count; j++) {
                        if (descs[j].uuid.len == ESP_UUID_LEN_16 &&
                            descs[j].uuid.uuid.uuid16 == 0x2902) {
                          cccd_handle_ = descs[j].handle;
                          ESP_LOGI(TAG, "    → CCCD handle: %d", cccd_handle_);
                        }
                      }
                      delete[] descs;
                    }
                  } else if (uuid128_equal(c, WRITE_CHAR_UUID)) {
                    write_handle_ = chars[i].char_handle;
                    ESP_LOGI(TAG, "    → WRITE handle: %d", write_handle_);
                  }
                }
                // ─── For Ventilation service: save char handle ───
                if (is_vent && uuid128_equal(c, VENT_CHAR_UUID)) {
                  vent_char_handle_ = chars[i].char_handle;
                  ESP_LOGI(TAG, "    → VENTILATION handle: %d", vent_char_handle_);
                }
                // ─── For Airflow service: save fan speed char handle ───
                if (airflow_supported_ && uuid128_equal(c, FAN_SPEED_CHAR_UUID)) {
                  airflow_char_handle_ = chars[i].char_handle;
                  ESP_LOGI(TAG, "    → FAN_SPEED handle: %d", airflow_char_handle_);
                }
              } else if (chars[i].uuid.len == ESP_UUID_LEN_16) {
                ESP_LOGI(TAG, "  CHR [16] 0x%04x  h=%d props=0x%02x",
                         chars[i].uuid.uuid.uuid16, chars[i].char_handle, chars[i].properties);
              }
              // Read ALL readable characteristics to discover their values
              if (chars[i].properties & ESP_GATT_CHAR_PROP_BIT_READ) {
                ESP_LOGI(TAG, "  → Reading h=%d ...", chars[i].char_handle);
                esp_ble_gattc_read_char(gattc_if_, conn_id_, chars[i].char_handle,
                                        ESP_GATT_AUTH_REQ_NONE);
              }
            }
          }
          delete[] chars;
        } else {
          ESP_LOGD(TAG, "  (no characteristics, count=%d status=%d)", count, status);
        }
      }
      break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {
      ESP_LOGI(TAG, "Service discovery complete. notify=%d, write=%d, cccd=%d, vent_char=%d",
               notify_handle_, write_handle_, cccd_handle_, vent_char_handle_);
      ESP_LOGI(TAG, "Ventilation GATT service: %s", ventilation_supported_ ? "SUPPORTED" : "not found");
      ESP_LOGI(TAG, "Airflow GATT service: %s", airflow_supported_ ? "SUPPORTED" : "not found");
      // Dump complete GATT database for analysis
      {
        uint16_t db_count = 0;
        esp_ble_gattc_get_db(gattc_if_, conn_id_, 0, 0xFFFF, nullptr, &db_count);
        ESP_LOGI(TAG, "GATT DB total entries: %d", db_count);
        if (db_count > 0 && db_count < 200) {
          auto *db = new esp_gattc_db_elem_t[db_count];
          esp_ble_gattc_get_db(gattc_if_, conn_id_, 0, 0xFFFF, db, &db_count);
          for (uint16_t i = 0; i < db_count; i++) {
            const char *type_str = "UNK";
            switch (db[i].type) {
              case ESP_GATT_DB_PRIMARY_SERVICE: type_str = "PRI_SVC"; break;
              case ESP_GATT_DB_SECONDARY_SERVICE: type_str = "SEC_SVC"; break;
              case ESP_GATT_DB_CHARACTERISTIC: type_str = "CHAR"; break;
              case ESP_GATT_DB_DESCRIPTOR: type_str = "DESC"; break;
              case ESP_GATT_DB_INCLUDED_SERVICE: type_str = "INC_SVC"; break;
              default: break;
            }
            if (db[i].uuid.len == ESP_UUID_LEN_128) {
              const uint8_t *u = db[i].uuid.uuid.uuid128;
              ESP_LOGI(TAG, "DB[%d] %s h=%d %02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                       i, type_str, db[i].attribute_handle,
                       u[15],u[14],u[13],u[12], u[11],u[10], u[9],u[8],
                       u[7],u[6], u[5],u[4],u[3],u[2],u[1],u[0]);
            } else if (db[i].uuid.len == ESP_UUID_LEN_16) {
              ESP_LOGI(TAG, "DB[%d] %s h=%d uuid16=0x%04x",
                       i, type_str, db[i].attribute_handle, db[i].uuid.uuid.uuid16);
            }
          }
          delete[] db;
        }
      }
      if (notify_handle_ && write_handle_) {
        if (cccd_done_) {
          // Re-discovery after bonding — CCCD already active, go straight to READY
          ESP_LOGI(TAG, "CCCD already active, skipping re-subscribe. → STATE_READY");
          state_ = STATE_READY;
          last_state_change_ms_ = millis();
          last_poll_ms_ = 0;  // trigger immediate poll
        } else {
          register_notify_();
        }
      } else {
        ESP_LOGE(TAG, "Required characteristics not found!");
        state_ = STATE_ERROR;
        last_state_change_ms_ = millis();
      }
      break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      if (param->reg_for_notify.status == ESP_GATT_OK) {
        ESP_LOGI(TAG, "Registered for notifications, writing CCCD...");
        cccd_done_ = true;  // Mark as initiated — re-discovery will skip
        // Write CCCD to enable notifications
        // This may trigger "Insufficient Authentication" → pairing
        uint16_t cccd_val = 0x0001;  // Enable notifications
        esp_ble_gattc_write_char_descr(gattc_if_, conn_id_, cccd_handle_,
                                       sizeof(cccd_val), (uint8_t *) &cccd_val,
                                       ESP_GATT_WRITE_TYPE_RSP,
                                       ESP_GATT_AUTH_REQ_MITM);
      } else {
        ESP_LOGW(TAG, "Register for notify failed: %d", param->reg_for_notify.status);
        // May need pairing first
        ESP_LOGI(TAG, "Initiating security...");
        state_ = STATE_PAIRING;
        last_state_change_ms_ = millis();
        esp_ble_set_encryption(target_addr_, ESP_BLE_SEC_ENCRYPT_MITM);
      }
      break;
    }

    case ESP_GATTC_WRITE_DESCR_EVT: {
      if (param->write.status == ESP_GATT_OK) {
        ESP_LOGI(TAG, "📡 CCCD written successfully — notifications active!");
        state_ = STATE_READY;
        last_state_change_ms_ = millis();
        // Trigger first poll immediately
        last_poll_ms_ = 0;
      } else if (param->write.status == ESP_GATT_INSUF_AUTHENTICATION ||
                 param->write.status == ESP_GATT_INSUF_ENCRYPTION ||
                 (!paired_ && param->write.status != ESP_GATT_OK)) {
        // Encryption not ready or other pre-pairing failure — initiate pairing
        ESP_LOGI(TAG, "CCCD write needs authentication (status=%d) — initiating pairing...",
                 param->write.status);
        state_ = STATE_PAIRING;
        last_state_change_ms_ = millis();
        esp_ble_set_encryption(target_addr_, ESP_BLE_SEC_ENCRYPT_MITM);
      } else {
        ESP_LOGW(TAG, "CCCD write failed: status=%d", param->write.status);
        state_ = STATE_ERROR;
        last_state_change_ms_ = millis();
      }
      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      // Notification received — process Madoka TLV chunk
      if (param->notify.handle == notify_handle_) {
        bool complete = assembler_.add_chunk(param->notify.value, param->notify.value_len);
        if (complete) {
          auto payload = assembler_.get_payload();
          auto [cmd_id, values] = parse_response_(payload);
          if (cmd_id != 0) {
            process_response_(cmd_id, values);
          }
        }
      }
      break;
    }

    case ESP_GATTC_WRITE_CHAR_EVT:
      // Write NoResponse doesn't generate this, but Write with Response does
      if (param->write.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Char write failed: handle=%d status=%d", param->write.handle, param->write.status);
      } else if (param->write.handle == vent_char_handle_) {
        ESP_LOGI(TAG, "✅ Ventilation rate write succeeded");
      }
      break;

    case ESP_GATTC_READ_CHAR_EVT: {
      if (param->read.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Char read failed: handle=%d status=%d", param->read.handle, param->read.status);
      } else {
        // Log ALL characteristic reads with raw hex data
        {
          std::string hex;
          std::string ascii;
          for (uint16_t i = 0; i < param->read.value_len && i < 64; i++) {
            char tmp[4];
            snprintf(tmp, sizeof(tmp), "%02X ", param->read.value[i]);
            hex += tmp;
            ascii += (param->read.value[i] >= 0x20 && param->read.value[i] < 0x7F)
                     ? (char)param->read.value[i] : '.';
          }
          ESP_LOGI(TAG, "CHAR READ h=%d len=%d hex=[%s] ascii=[%s]",
                   param->read.handle, param->read.value_len, hex.c_str(), ascii.c_str());
        }

        if (param->read.handle == vent_char_handle_ && param->read.value_len > 0) {
          // Parse JSON from ventilation characteristic
          std::string json_str((const char *)param->read.value, param->read.value_len);
          ESP_LOGI(TAG, "Ventilation read (%d bytes): %s", param->read.value_len, json_str.c_str());

          auto pos = json_str.find("\"ventilationRate\":\"");
          if (pos != std::string::npos) {
            pos += 19;
            auto end = json_str.find("\"", pos);
            if (end != std::string::npos) {
              std::string rate = json_str.substr(pos, end - pos);
              if (rate == "AUTO") {
                ventilation_rate_ = VentilationRate::VENT_AUTO;
              } else if (rate == "LOW") {
                ventilation_rate_ = VentilationRate::VENT_LOW;
              } else if (rate == "MEDIUM" || rate == "LOW_MEDIUM") {
                ventilation_rate_ = VentilationRate::VENT_MEDIUM;
              } else if (rate == "HIGH" || rate == "MEDIUM_HIGH") {
                ventilation_rate_ = VentilationRate::VENT_HIGH;
              }
              vent_rate_known_ = true;
              ESP_LOGI(TAG, "Ventilation rate: %s → %d", rate.c_str(), (int)ventilation_rate_);
            }
          }
        }
      }
      // Advance poll step if we're polling
      if (state_ == STATE_POLLING) {
        response_received_ = true;
      }
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "⚠️ Disconnected (reason=0x%x)", param->disconnect.reason);
      state_ = STATE_DISCONNECTED;
      notify_handle_ = 0;
      write_handle_ = 0;
      cccd_handle_ = 0;
      vent_char_handle_ = 0;
      airflow_char_handle_ = 0;
      ventilation_supported_ = false;
      airflow_supported_ = false;
      vent_rediscovery_done_ = false;
      cccd_done_ = false;
      write_queue_.clear();
      reconnect_attempts_++;
      uint32_t delay = std::min<uint32_t>(60000u, 5000u * reconnect_attempts_);
      next_reconnect_ms_ = millis() + delay;
      last_state_change_ms_ = millis();
      break;
    }

    default:
      break;
  }
}

// ─── Fan Entity Implementation ──────────────────────────────

void MadokaFan::control(const esphome::fan::FanCall &call) {
  if (!parent_) return;

  if (call.get_state().has_value()) {
    bool new_state = *call.get_state();
    parent_->request_set_power(new_state);
    if (new_state) {
      parent_->request_set_mode(OpMode::VENTILATION);
    }
  }

  if (call.has_preset_mode()) {
    const char *mode = call.get_preset_mode();
    if (strcmp(mode, "low") == 0) {
      parent_->request_set_fan(FanSpeed::LOW, FanSpeed::LOW);
    } else if (strcmp(mode, "high") == 0) {
      parent_->request_set_fan(FanSpeed::HIGH, FanSpeed::HIGH);
    }
  }
}

void MadokaFan::update_state(bool power, VentilationRate rate) {
  this->state = power;
  if (rate == VentilationRate::VENT_HIGH) {
    this->set_preset_mode_("high");
  } else {
    this->set_preset_mode_("low");
  }
  this->publish_state();
}

// ─── Climate Entity Implementation ──────────────────────────

void MadokaClimate::control(const esphome::climate::ClimateCall &call) {
  ESP_LOGI("madoka_climate", "control() called, parent_=%p", (void*) parent_);
  if (!parent_) {
    ESP_LOGW("madoka_climate", "parent_ is NULL, ignoring control call");
    return;
  }

  if (call.get_mode().has_value()) {
    auto mode = *call.get_mode();
    switch (mode) {
      case esphome::climate::CLIMATE_MODE_OFF:
        parent_->request_set_power(false);
        break;
      case esphome::climate::CLIMATE_MODE_HEAT:
        parent_->request_set_power(true);
        parent_->request_set_mode(OpMode::HEAT);
        break;
      case esphome::climate::CLIMATE_MODE_COOL:
        parent_->request_set_power(true);
        parent_->request_set_mode(OpMode::COOL);
        break;
      case esphome::climate::CLIMATE_MODE_HEAT_COOL:
        parent_->request_set_power(true);
        parent_->request_set_mode(OpMode::AUTO);
        break;
      case esphome::climate::CLIMATE_MODE_DRY:
        parent_->request_set_power(true);
        parent_->request_set_mode(OpMode::DRY);
        break;
      case esphome::climate::CLIMATE_MODE_FAN_ONLY:
        parent_->request_set_power(true);
        parent_->request_set_mode(OpMode::VENTILATION);
        break;
      default:
        break;
    }
  }

  if (call.get_target_temperature().has_value()) {
    float temp = *call.get_target_temperature();
    parent_->request_set_setpoint(temp, temp);
  }

  if (call.get_target_temperature_low().has_value() &&
      call.get_target_temperature_high().has_value()) {
    parent_->request_set_setpoint(
        *call.get_target_temperature_high(),
        *call.get_target_temperature_low());
  }

  if (call.get_fan_mode().has_value()) {
    ESP_LOGI("madoka_climate", "Fan mode change requested: %d", (int) *call.get_fan_mode());
    FanSpeed speed;
    switch (*call.get_fan_mode()) {
      case esphome::climate::CLIMATE_FAN_LOW: speed = FanSpeed::LOW; break;
      case esphome::climate::CLIMATE_FAN_MEDIUM: speed = FanSpeed::MID; break;
      case esphome::climate::CLIMATE_FAN_HIGH: speed = FanSpeed::HIGH; break;
      default: speed = FanSpeed::AUTO_SPEED; break;
    }
    ESP_LOGI("madoka_climate", "Sending fan speed: %d", (int) speed);
    parent_->request_set_fan(speed, speed);
  }
}

void MadokaClimate::update_state(bool power, OpMode mode, float indoor_temp,
                                 float cool_sp, float heat_sp,
                                 FanSpeed cool_fan, FanSpeed heat_fan) {
  // Map Madoka state to ESPHome climate state
  if (!power) {
    this->mode = esphome::climate::CLIMATE_MODE_OFF;
  } else {
    switch (mode) {
      case OpMode::HEAT: this->mode = esphome::climate::CLIMATE_MODE_HEAT; break;
      case OpMode::COOL: this->mode = esphome::climate::CLIMATE_MODE_COOL; break;
      case OpMode::AUTO: this->mode = esphome::climate::CLIMATE_MODE_HEAT_COOL; break;
      case OpMode::DRY:  this->mode = esphome::climate::CLIMATE_MODE_DRY; break;
      case OpMode::FAN:
      case OpMode::VENTILATION:
        this->mode = esphome::climate::CLIMATE_MODE_FAN_ONLY; break;
      default: this->mode = esphome::climate::CLIMATE_MODE_HEAT_COOL; break;
    }
  }

  if (!std::isnan(indoor_temp))
    this->current_temperature = indoor_temp;

  // Set target temperature based on mode
  if (this->mode == esphome::climate::CLIMATE_MODE_HEAT) {
    this->target_temperature = heat_sp;
  } else if (this->mode == esphome::climate::CLIMATE_MODE_COOL) {
    this->target_temperature = cool_sp;
  } else if (this->mode == esphome::climate::CLIMATE_MODE_HEAT_COOL) {
    this->target_temperature_low = heat_sp;
    this->target_temperature_high = cool_sp;
  } else {
    this->target_temperature = cool_sp;
  }

  // Fan mode
  if (parent_ && parent_->ventilation_supported()) {
    switch (parent_->ventilation_rate()) {
      case VentilationRate::VENT_LOW: this->fan_mode = esphome::climate::CLIMATE_FAN_LOW; break;
      case VentilationRate::VENT_MEDIUM: this->fan_mode = esphome::climate::CLIMATE_FAN_MEDIUM; break;
      case VentilationRate::VENT_HIGH: this->fan_mode = esphome::climate::CLIMATE_FAN_HIGH; break;
      default: this->fan_mode = esphome::climate::CLIMATE_FAN_AUTO; break;
    }
  } else {
    FanSpeed active_fan = (this->mode == esphome::climate::CLIMATE_MODE_HEAT) ? heat_fan : cool_fan;
    switch (active_fan) {
      case FanSpeed::LOW: this->fan_mode = esphome::climate::CLIMATE_FAN_LOW; break;
      case FanSpeed::MID: this->fan_mode = esphome::climate::CLIMATE_FAN_MEDIUM; break;
      case FanSpeed::HIGH: this->fan_mode = esphome::climate::CLIMATE_FAN_HIGH; break;
      default: this->fan_mode = esphome::climate::CLIMATE_FAN_AUTO; break;
    }
  }

  this->publish_state();
}

// ─── Reset Filter Button Entity ─────────────────────────────

void MadokaResetFilterButton::press_action() {
  if (parent_) {
    parent_->request_reset_filter();
  }
}

// ─── Eye Brightness Number Entity ───────────────────────────

void MadokaEyeBrightness::control(float value) {
  uint8_t level = (uint8_t) value;
  if (level > 19) level = 19;
  if (parent_) {
    parent_->request_set_eye_brightness(level);
  }
  this->publish_state(value);
}

}  // namespace madoka_ble
