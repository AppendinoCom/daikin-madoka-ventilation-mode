"""ESPHome sensor platform for Madoka BLE."""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)
from . import madoka_ble_ns, MadokaBLE

CONF_MADOKA_BLE_ID = "madoka_ble_id"
CONF_INDOOR_TEMPERATURE = "indoor_temperature"
CONF_OUTDOOR_TEMPERATURE = "outdoor_temperature"
CONF_COOLING_SETPOINT = "cooling_setpoint"
CONF_HEATING_SETPOINT = "heating_setpoint"

_TEMP_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=0,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MADOKA_BLE_ID): cv.use_id(MadokaBLE),
        cv.Optional(CONF_INDOOR_TEMPERATURE): _TEMP_SCHEMA,
        cv.Optional(CONF_OUTDOOR_TEMPERATURE): _TEMP_SCHEMA,
        cv.Optional(CONF_COOLING_SETPOINT): _TEMP_SCHEMA,
        cv.Optional(CONF_HEATING_SETPOINT): _TEMP_SCHEMA,
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MADOKA_BLE_ID])

    for key, setter in [
        (CONF_INDOOR_TEMPERATURE, "set_indoor_temp_sensor"),
        (CONF_OUTDOOR_TEMPERATURE, "set_outdoor_temp_sensor"),
        (CONF_COOLING_SETPOINT, "set_cooling_setpoint_sensor"),
        (CONF_HEATING_SETPOINT, "set_heating_setpoint_sensor"),
    ]:
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(parent, setter)(sens))
