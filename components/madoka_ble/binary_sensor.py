"""ESPHome binary_sensor platform for Madoka BLE — filter alert."""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID, DEVICE_CLASS_PROBLEM
from . import madoka_ble_ns, MadokaBLE

CONF_MADOKA_BLE_ID = "madoka_ble_id"
CONF_FILTER_ALERT = "filter_alert"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MADOKA_BLE_ID): cv.use_id(MadokaBLE),
        cv.Optional(CONF_FILTER_ALERT): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MADOKA_BLE_ID])

    if CONF_FILTER_ALERT in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_FILTER_ALERT])
        cg.add(parent.set_filter_alert_sensor(sens))
