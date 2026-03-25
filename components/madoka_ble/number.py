"""ESPHome number platform for Madoka BLE — eye brightness."""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_ID, ENTITY_CATEGORY_CONFIG
from . import madoka_ble_ns, MadokaBLE

MadokaEyeBrightness = madoka_ble_ns.class_(
    "MadokaEyeBrightness", number.Number, cg.Component
)

CONF_MADOKA_BLE_ID = "madoka_ble_id"
CONF_EYE_BRIGHTNESS = "eye_brightness"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MADOKA_BLE_ID): cv.use_id(MadokaBLE),
        cv.Optional(CONF_EYE_BRIGHTNESS): number.number_schema(
            MadokaEyeBrightness,
            entity_category=ENTITY_CATEGORY_CONFIG,
        ).extend(cv.COMPONENT_SCHEMA),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MADOKA_BLE_ID])

    if CONF_EYE_BRIGHTNESS in config:
        var = cg.new_Pvariable(config[CONF_EYE_BRIGHTNESS][CONF_ID])
        await cg.register_component(var, config[CONF_EYE_BRIGHTNESS])
        await number.register_number(
            var,
            config[CONF_EYE_BRIGHTNESS],
            min_value=0,
            max_value=19,
            step=1,
        )
        cg.add(var.set_parent(parent))
        cg.add(parent.set_eye_brightness_number(var))
