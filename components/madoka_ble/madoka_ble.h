#pragma once
/**
 * madoka_ble.h — Daikin Madoka BRC1H BLE bridge for ESPHome.
 *
 * Connects to BRC1H via ESP-IDF BLE stack, handles LE Secure Connections
 * pairing (auto-confirm Numeric Comparison), and implements the Madoka
 * TLV protocol to expose climate + sensor entities in Home Assistant.
 */

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/button/button.h"
#include "esphome/components/fan/fan.h"
#include "esphome/components/climate/climate.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include <map>
#include <vector>
#include <cstring>

namespace madoka_ble {

// ─── BLE UUIDs (128-bit, little-endian for ESP-IDF) ─────────
// Service: 2141e110-213a-11e6-b67b-9e71128cae77
static const uint8_t MADOKA_SERVICE_UUID[16] = {
    0x77, 0xae, 0x8c, 0x12, 0x71, 0x9e, 0x7b, 0xb6,
    0xe6, 0x11, 0x3a, 0x21, 0x10, 0xe1, 0x41, 0x21};
// Notify: 2141e111-213a-11e6-b67b-9e71128cae77
static const uint8_t NOTIFY_CHAR_UUID[16] = {
    0x77, 0xae, 0x8c, 0x12, 0x71, 0x9e, 0x7b, 0xb6,
    0xe6, 0x11, 0x3a, 0x21, 0x11, 0xe1, 0x41, 0x21};
// Write: 2141e112-213a-11e6-b67b-9e71128cae77
static const uint8_t WRITE_CHAR_UUID[16] = {
    0x77, 0xae, 0x8c, 0x12, 0x71, 0x9e, 0x7b, 0xb6,
    0xe6, 0x11, 0x3a, 0x21, 0x12, 0xe1, 0x41, 0x21};

// ─── Ventilation JSON-GATT UUIDs (for recuperation units) ──
// Service: 3ce06519-bc5c-432c-ad3f-8801b224ee2a
static const uint8_t VENT_SERVICE_UUID[16] = {
    0x2a, 0xee, 0x24, 0xb2, 0x01, 0x88, 0x3f, 0xad,
    0x2c, 0x43, 0x5c, 0xbc, 0x19, 0x65, 0xe0, 0x3c};
// Characteristic: 3ce06519-bc5c-432c-ad3f-8801b224ee2b
static const uint8_t VENT_CHAR_UUID[16] = {
    0x2b, 0xee, 0x24, 0xb2, 0x01, 0x88, 0x3f, 0xad,
    0x2c, 0x43, 0x5c, 0xbc, 0x19, 0x65, 0xe0, 0x3c};
// ─── Airflow/Fan Speed JSON-GATT UUIDs ──
// Service: 3ce06519-bc5c-432c-ad3d-8801b224ee2a
static const uint8_t AIRFLOW_SERVICE_UUID[16] = {
    0x2a, 0xee, 0x24, 0xb2, 0x01, 0x88, 0x3d, 0xad,
    0x2c, 0x43, 0x5c, 0xbc, 0x19, 0x65, 0xe0, 0x3c};
// Characteristic: 3ce06519-bc5c-432c-ad3d-8801b224ee2b
static const uint8_t FAN_SPEED_CHAR_UUID[16] = {
    0x2b, 0xee, 0x24, 0xb2, 0x01, 0x88, 0x3d, 0xad,
    0x2c, 0x43, 0x5c, 0xbc, 0x19, 0x65, 0xe0, 0x3c};

// ─── Madoka Protocol Constants ──────────────────────────────
static const uint16_t CMD_GET_POWER        = 0x0020;
static const uint16_t CMD_SET_POWER        = 0x4020;
static const uint16_t CMD_GET_MODE         = 0x0030;
static const uint16_t CMD_SET_MODE         = 0x4030;
static const uint16_t CMD_GET_SETPOINT     = 0x0040;
static const uint16_t CMD_SET_SETPOINT     = 0x4040;
static const uint16_t CMD_GET_FAN          = 0x0050;
static const uint16_t CMD_SET_FAN          = 0x4050;
static const uint16_t CMD_GET_TEMPERATURES = 0x0110;
static const uint16_t CMD_GET_CLEAN_FILTER = 0x0100;
static const uint16_t CMD_GET_VERSION      = 0x0130;
static const uint16_t CMD_GET_EYE_BRIGHT   = 0x0302;
static const uint16_t CMD_RESET_FILTER     = 0x4220;
static const uint16_t CMD_SET_EYE_BRIGHT   = 0x4302;
static const uint16_t CMD_ENTER_PRIV_MODE  = 0x4112;
static const uint16_t CMD_GET_VENTILATION  = 0x0031;  // functionId=49
static const uint16_t CMD_SET_VENTILATION  = 0x4031;  // functionId=49

// TLV param IDs
static const uint8_t PARAM_POWER           = 0x20;
static const uint8_t PARAM_MODE            = 0x20;
static const uint8_t PARAM_SETPOINT_COOL   = 0x20;
static const uint8_t PARAM_SETPOINT_HEAT   = 0x21;
static const uint8_t PARAM_FAN_COOL        = 0x20;
static const uint8_t PARAM_FAN_HEAT        = 0x21;
static const uint8_t PARAM_TEMP_INDOOR     = 0x40;
static const uint8_t PARAM_TEMP_OUTDOOR    = 0x41;
static const uint8_t PARAM_EYE_BRIGHT      = 0x33;
static const uint8_t PARAM_VENT_MODE       = 0x20;  // ventilation mode (AUTO/ERV/BYPASS)
static const uint8_t PARAM_VENT_RATE       = 0x21;  // ventilation rate (fan speed)

enum class OpMode : uint8_t { FAN = 0, DRY = 1, AUTO = 2, COOL = 3, HEAT = 4, VENTILATION = 5 };
enum class FanSpeed : uint8_t { AUTO_SPEED = 0, LOW = 1, MID = 3, HIGH = 5 };
enum class VentilationRate : uint8_t { VENT_AUTO = 0, VENT_LOW = 1, VENT_MEDIUM = 2, VENT_HIGH = 3 };

// ─── Chunk Assembler ────────────────────────────────────────
class ChunkAssembler {
 public:
  /** Add a notification chunk. Returns true when a complete message is ready. */
  bool add_chunk(const uint8_t *data, uint16_t len) {
    if (len < 2) return false;
    uint8_t chunk_id = data[0];
    if (chunk_id == 0) {
      chunks_.clear();
      expected_len_ = data[1];
    }
    chunks_[chunk_id].assign(data + 1, data + len);

    int expected_chunks = (expected_len_ + 18) / 19;
    return (expected_chunks > 0 && (int) chunks_.size() >= expected_chunks);
  }

  /** Get the reassembled payload (call after add_chunk returns true). */
  std::vector<uint8_t> get_payload() {
    std::vector<uint8_t> result;
    int n = (int) chunks_.size();
    for (int i = 0; i < n; i++) {
      auto it = chunks_.find(i);
      if (it != chunks_.end())
        result.insert(result.end(), it->second.begin(), it->second.end());
    }
    chunks_.clear();
    return result;
  }

 private:
  std::map<int, std::vector<uint8_t>> chunks_;
  uint8_t expected_len_ = 0;
};

// ─── Forward declare entity classes ─────────────────────────
class MadokaClimate;
class MadokaFan;
class MadokaEyeBrightness;
class MadokaResetFilterButton;

// ─── Main BLE Component ─────────────────────────────────────
class MadokaBLE : public esphome::Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return 600.0f; }  // after WiFi

  void set_mac_address(uint64_t addr);
  void set_poll_interval(uint32_t ms) { poll_interval_ms_ = ms; }

  // Sensor setters (called from Python codegen)
  void set_indoor_temp_sensor(esphome::sensor::Sensor *s) { indoor_temp_sensor_ = s; }
  void set_outdoor_temp_sensor(esphome::sensor::Sensor *s) { outdoor_temp_sensor_ = s; }
  void set_cooling_setpoint_sensor(esphome::sensor::Sensor *s) { cooling_sp_sensor_ = s; }
  void set_heating_setpoint_sensor(esphome::sensor::Sensor *s) { heating_sp_sensor_ = s; }
  void set_climate(MadokaClimate *c) { climate_ = c; }
  void set_fan(MadokaFan *f) { fan_ = f; }
  void set_filter_alert_sensor(esphome::binary_sensor::BinarySensor *s) { filter_alert_sensor_ = s; }
  void set_firmware_version_sensor(esphome::text_sensor::TextSensor *s) { firmware_version_sensor_ = s; }
  void set_eye_brightness_number(MadokaEyeBrightness *n) { eye_brightness_number_ = n; }
  void set_reset_filter_button(MadokaResetFilterButton *b) { reset_filter_button_ = b; }

  // Control methods (called from climate entity)
  void request_set_power(bool on);
  void request_set_mode(OpMode mode);
  void request_set_setpoint(float cool, float heat);
  void request_set_fan(FanSpeed cool, FanSpeed heat);
  void request_set_ventilation_rate(VentilationRate rate);
  void request_reset_filter();
  void request_set_eye_brightness(uint8_t level);
  bool ventilation_supported() const { return ventilation_supported_; }
  VentilationRate ventilation_rate() const { return ventilation_rate_; }

  // ESP-IDF BLE callbacks (static, forwarded to singleton)
  static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
  static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                  esp_ble_gattc_cb_param_t *param);

 protected:
  // ─── State machine ────────────────────────────────────────
  enum State {
    STATE_INIT,
    STATE_WAIT_SCAN,
    STATE_SCANNING,
    STATE_CONNECTING,
    STATE_CONNECTED,
    STATE_DISCOVERING,
    STATE_REGISTERING_NOTIFY,
    STATE_PAIRING,
    STATE_READY,
    STATE_POLLING,
    STATE_DISCONNECTED,
    STATE_ERROR,
  };

  void start_scan_();
  void connect_();
  void discover_services_();
  void register_notify_();
  void start_polling_();
  void send_next_query_();
  void process_response_(uint16_t cmd_id, const std::map<uint8_t, std::vector<uint8_t>> &values);
  void update_climate_state_();

  // ─── Protocol helpers ─────────────────────────────────────
  std::vector<std::vector<uint8_t>> build_command_(uint16_t cmd_id,
      const std::map<uint8_t, std::vector<uint8_t>> &params = {});
  void send_chunks_(const std::vector<std::vector<uint8_t>> &chunks);
  std::pair<uint16_t, std::map<uint8_t, std::vector<uint8_t>>> parse_response_(
      const std::vector<uint8_t> &data);

  // ─── GAP/GATTC event handling ─────────────────────────────
  void handle_gap_event_(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
  void handle_gattc_event_(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param);

  // ─── Member data ──────────────────────────────────────────
  State state_ = STATE_INIT;
  uint8_t target_addr_[6]{};  // BDA in ESP-IDF order
  esp_gatt_if_t gattc_if_ = 0;
  uint16_t conn_id_ = 0;
  uint16_t notify_handle_ = 0;
  uint16_t write_handle_ = 0;
  uint16_t cccd_handle_ = 0;
  uint16_t vent_char_handle_ = 0;  // JSON-GATT ventilation characteristic
  uint16_t airflow_char_handle_ = 0;  // JSON-GATT airflow/fan speed characteristic
  bool airflow_supported_ = false;
  bool paired_ = false;
  bool app_registered_ = false;

  uint32_t poll_interval_ms_ = 60000;
  uint32_t last_poll_ms_ = 0;
  uint32_t last_state_change_ms_ = 0;

  // Startup delay before first BLE scan (allows API log stream to connect)
  uint32_t startup_delay_ms_ = 5000;  // 5s startup delay before BLE scan
  bool first_scan_started_ = false;

  // Polling state
  int poll_step_ = 0;
  bool version_fetched_ = false;
  bool eye_brightness_fetched_ = false;
  volatile bool response_received_ = false;
  uint32_t poll_next_at_ms_ = 0;
  uint32_t query_sent_ms_ = 0;  // when the current query was sent
  static const int POLL_STEPS = 9;  // power, mode, setpoint, fan, temps, filter, ventilation, version, eye_brightness

  // TLV command scan: one-time scan of unknown command IDs after first poll
  bool scan_mode_ = false;  // disabled — using JSON-GATT for ventilation
  int scan_idx_ = 0;
  uint32_t scan_next_at_ms_ = 0;
  static const uint16_t SCAN_CMDS[];
  static const int SCAN_CMD_COUNT;

  // Pending write queue
  std::vector<std::vector<uint8_t>> write_queue_;

  // Sequential fan SET state machine
  enum FanSetStep { FAN_SET_IDLE, FAN_SET_WAIT_PRIV, FAN_SET_WAIT_FAN, FAN_SET_WAIT_MODE };
  FanSetStep fan_set_step_ = FAN_SET_IDLE;
  uint8_t pending_fan_val_ = 0;
  uint8_t pending_vent_speed_ = 0;
  uint32_t fan_set_sent_ms_ = 0;  // timeout tracking
  bool vent_fan_set_ = false;  // true when fan set is for ventilation mode
  bool cccd_done_ = false;  // CCCD already written this connection

  // Chunk assembler
  ChunkAssembler assembler_;

  // Current device state
  bool power_on_ = false;
  OpMode op_mode_ = OpMode::AUTO;
  float indoor_temp_ = NAN;
  float outdoor_temp_ = NAN;
  float cooling_sp_ = 26.0f;
  float heating_sp_ = 22.0f;
  FanSpeed cool_fan_ = FanSpeed::AUTO_SPEED;
  FanSpeed heat_fan_ = FanSpeed::AUTO_SPEED;
  bool clean_filter_ = false;
  std::string firmware_version_;
  uint8_t eye_brightness_ = 0;
  bool ventilation_supported_ = false;
  VentilationRate ventilation_rate_ = VentilationRate::VENT_AUTO;
  bool vent_rate_known_ = false;
  bool vent_rediscovery_done_ = false;

  // ESPHome entities
  esphome::sensor::Sensor *indoor_temp_sensor_ = nullptr;
  esphome::sensor::Sensor *outdoor_temp_sensor_ = nullptr;
  esphome::sensor::Sensor *cooling_sp_sensor_ = nullptr;
  esphome::sensor::Sensor *heating_sp_sensor_ = nullptr;
  MadokaClimate *climate_ = nullptr;
  MadokaFan *fan_ = nullptr;
  esphome::binary_sensor::BinarySensor *filter_alert_sensor_ = nullptr;
  esphome::text_sensor::TextSensor *firmware_version_sensor_ = nullptr;
  MadokaEyeBrightness *eye_brightness_number_ = nullptr;
  MadokaResetFilterButton *reset_filter_button_ = nullptr;

  // Reconnect backoff
  int reconnect_attempts_ = 0;
  uint32_t next_reconnect_ms_ = 0;

  // Singleton for static callbacks
  static MadokaBLE *instance_;
};

// ─── Climate Entity ─────────────────────────────────────────
class MadokaClimate : public esphome::climate::Climate, public esphome::Component {
 public:
  void setup() override {}
  float get_setup_priority() const override { return esphome::setup_priority::LATE; }

  void set_parent(MadokaBLE *parent) { parent_ = parent; }

  esphome::climate::ClimateTraits traits() override {
    auto traits = esphome::climate::ClimateTraits();
    traits.set_supports_current_temperature(true);
    traits.set_supported_modes({
        esphome::climate::CLIMATE_MODE_OFF,
        esphome::climate::CLIMATE_MODE_FAN_ONLY,
    });
    traits.set_supported_fan_modes({
        esphome::climate::CLIMATE_FAN_LOW,
        esphome::climate::CLIMATE_FAN_HIGH,
    });
    return traits;
  }

  void control(const esphome::climate::ClimateCall &call) override;

  // Update from BLE data
  void update_state(bool power, OpMode mode, float indoor_temp,
                    float cool_sp, float heat_sp,
                    FanSpeed cool_fan, FanSpeed heat_fan);

 protected:
  MadokaBLE *parent_ = nullptr;
};

// ─── Fan Entity ─────────────────────────────────────────────
class MadokaFan : public esphome::fan::Fan, public esphome::Component {
 public:
  void setup() override {}
  float get_setup_priority() const override { return esphome::setup_priority::LATE; }

  void set_parent(MadokaBLE *parent) { parent_ = parent; }

  esphome::fan::FanTraits get_traits() override {
    auto traits = esphome::fan::FanTraits();
    traits.set_supported_preset_modes({"low", "high"});
    return traits;
  }

  void update_state(bool power, VentilationRate rate);

 protected:
  void control(const esphome::fan::FanCall &call) override;
  MadokaBLE *parent_ = nullptr;
};

// ─── Eye Brightness Number Entity ───────────────────────────
class MadokaEyeBrightness : public esphome::number::Number, public esphome::Component {
 public:
  void setup() override {}
  float get_setup_priority() const override { return esphome::setup_priority::LATE; }

  void set_parent(MadokaBLE *parent) { parent_ = parent; }

  void publish_brightness(uint8_t level) {
    this->publish_state((float) level);
  }

 protected:
  void control(float value) override;
  MadokaBLE *parent_ = nullptr;
};

// ─── Reset Filter Button Entity ─────────────────────────────
class MadokaResetFilterButton : public esphome::button::Button, public esphome::Component {
 public:
  void setup() override {}
  float get_setup_priority() const override { return esphome::setup_priority::LATE; }

  void set_parent(MadokaBLE *parent) { parent_ = parent; }

 protected:
  void press_action() override;
  MadokaBLE *parent_ = nullptr;
};

}  // namespace madoka_ble
