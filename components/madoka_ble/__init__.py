"""ESPHome external component: madoka_ble — Daikin Madoka BRC1H BLE bridge."""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_MAC_ADDRESS

CODEOWNERS = ["@peterfridrich"]
DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["sensor", "climate", "fan", "binary_sensor", "text_sensor", "number", "button"]

CONF_POLL_INTERVAL = "poll_interval"

madoka_ble_ns = cg.esphome_ns.namespace("madoka_ble")
MadokaBLE = madoka_ble_ns.class_("MadokaBLE", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MadokaBLE),
        cv.Required(CONF_MAC_ADDRESS): cv.mac_address,
        cv.Optional(CONF_POLL_INTERVAL, default="60s"): cv.positive_time_period_milliseconds,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    mac = config[CONF_MAC_ADDRESS]
    cg.add(var.set_mac_address(mac.as_hex))
    cg.add(var.set_poll_interval(config[CONF_POLL_INTERVAL]))
