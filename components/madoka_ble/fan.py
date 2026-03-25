"""ESPHome fan platform for Madoka BLE — ventilation control."""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import fan
from esphome.const import CONF_ID
from . import madoka_ble_ns, MadokaBLE

MadokaFan = madoka_ble_ns.class_(
    "MadokaFan", fan.Fan, cg.Component
)

CONF_MADOKA_BLE_ID = "madoka_ble_id"

CONFIG_SCHEMA = (
    fan.fan_schema(MadokaFan)
    .extend(
        {
            cv.GenerateID(CONF_MADOKA_BLE_ID): cv.use_id(MadokaBLE),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await fan.register_fan(var, config)

    parent = await cg.get_variable(config[CONF_MADOKA_BLE_ID])
    cg.add(var.set_parent(parent))
    cg.add(parent.set_fan(var))
