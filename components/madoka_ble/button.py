"""ESPHome button platform for Madoka BLE — reset filter."""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import CONF_ID, ENTITY_CATEGORY_CONFIG
from . import madoka_ble_ns, MadokaBLE

MadokaResetFilterButton = madoka_ble_ns.class_(
    "MadokaResetFilterButton", button.Button, cg.Component
)

CONF_MADOKA_BLE_ID = "madoka_ble_id"
CONF_RESET_FILTER = "reset_filter"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MADOKA_BLE_ID): cv.use_id(MadokaBLE),
        cv.Optional(CONF_RESET_FILTER): button.button_schema(
            MadokaResetFilterButton,
            entity_category=ENTITY_CATEGORY_CONFIG,
        ).extend(cv.COMPONENT_SCHEMA),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MADOKA_BLE_ID])

    if CONF_RESET_FILTER in config:
        var = cg.new_Pvariable(config[CONF_RESET_FILTER][CONF_ID])
        await cg.register_component(var, config[CONF_RESET_FILTER])
        await button.register_button(var, config[CONF_RESET_FILTER])
        cg.add(var.set_parent(parent))
        cg.add(parent.set_reset_filter_button(var))
