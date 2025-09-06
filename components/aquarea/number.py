import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    DEVICE_CLASS_TEMPERATURE,
    UNIT_CELSIUS,
    ICON_THERMOMETER,
    ICON_GAUGE,
    CONF_INITIAL_VALUE,
)
from . import AquareaComponent, aquarea_ns, CONF_AQUAREA_ID

HeatOutsideTempLowNumber = aquarea_ns.class_("HeatOutsideTempLowNumber", number.Number)
HeatOutsideTempHighNumber = aquarea_ns.class_("HeatOutsideTempHighNumber", number.Number)
HeatWaterTempLowNumber = aquarea_ns.class_("HeatWaterTempLowNumber", number.Number)
HeatWaterTempHighNumber = aquarea_ns.class_("HeatWaterTempHighNumber", number.Number)
HeatOffOutsideTempNumber = aquarea_ns.class_("HeatOffOutsideTempNumber", number.Number)
HeaterOnOutsideTempNumber = aquarea_ns.class_("HeaterOnOutsideTempNumber", number.Number)
CoolSetpointTempNumber = aquarea_ns.class_("CoolSetpointTempNumber", number.Number)
TankSetpointTempNumber = aquarea_ns.class_("TankSetpointTempNumber", number.Number)
ShiftSetpointTempNumber = aquarea_ns.class_("ShiftSetpointTempNumber", number.Number)
PumpSpeedNumber = aquarea_ns.class_("PumpSpeedNumber", number.Number)

CONF_HeatOutsideTempLow = ("heat_out_temp_low", -20, 5, 1)
CONF_HeatOutsideTempHigh = ("heat_out_temp_high", -5, 15, 1)
CONF_HeatWaterTempLow = ("heat_water_temp_low", 25, 55, 1)
CONF_HeatWaterTempHigh = ("heat_water_temp_high", 25, 45, 1)
CONF_HeatOffOutsideTemp = ("heat_off_out_temp", 0, 20, 1)
CONF_HeaterOnOutsideTemp = ("heater_on_out_temp", -15, 0, 1)
CONF_CoolSetpointTemp = ("cool_setpoint_temp", 13, 20, 1)
CONF_TankSetpointTemp = ("tank_setpoint_temp", 40, 70, 1)
CONF_ShiftSetpointTemp = ("shift_setpoint_temp", -5, 5, 1)
CONF_PumpSpeed = ("pump_speed", 1, 7, 1)

TYPES = [
    CONF_HeatOutsideTempLow,
    CONF_HeatOutsideTempHigh,
    CONF_HeatWaterTempLow,
    CONF_HeatWaterTempHigh,
    CONF_HeatOffOutsideTemp,
    CONF_HeaterOnOutsideTemp,
    CONF_CoolSetpointTemp,
    CONF_TankSetpointTemp,
    CONF_ShiftSetpointTemp,
    CONF_PumpSpeed
]

def ensure_within_range(value, config):
    if value < config[1] or value > config[2]:
        raise cv.Invalid(f"Value: {value} out supported range for {config[0]} \
range is {config[1]}:{config[2]}")
    return value

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_AQUAREA_ID): cv.use_id(AquareaComponent),
            cv.Optional(CONF_HeatOutsideTempLow[0]): number.number_schema(
                HeatOutsideTempLowNumber,
                icon=ICON_THERMOMETER,
                unit_of_measurement=UNIT_CELSIUS,
            ).extend({
                cv.Optional(CONF_INITIAL_VALUE): cv.All(cv.float_, lambda v: ensure_within_range(v, CONF_HeatOutsideTempLow)),
            }),
            cv.Optional(CONF_HeatOutsideTempHigh[0]): number.number_schema(
                HeatOutsideTempHighNumber,
                icon=ICON_THERMOMETER,
                unit_of_measurement=UNIT_CELSIUS,
            ).extend({
                cv.Optional(CONF_INITIAL_VALUE): cv.All(cv.float_, lambda v: ensure_within_range(v, CONF_HeatOutsideTempHigh)),
            }),
            cv.Optional(CONF_HeatWaterTempLow[0]): number.number_schema(
                HeatWaterTempLowNumber,
                icon=ICON_THERMOMETER,
                unit_of_measurement=UNIT_CELSIUS,
            ).extend({
                cv.Optional(CONF_INITIAL_VALUE): cv.All(cv.float_, lambda v: ensure_within_range(v, CONF_HeatWaterTempLow)),
            }),
            cv.Optional(CONF_HeatWaterTempHigh[0]): number.number_schema(
                HeatWaterTempHighNumber,
                icon=ICON_THERMOMETER,
                unit_of_measurement=UNIT_CELSIUS,
            ).extend({
                cv.Optional(CONF_INITIAL_VALUE): cv.All(cv.float_, lambda v: ensure_within_range(v, CONF_HeatWaterTempHigh)),
            }),
            cv.Optional(CONF_HeatOffOutsideTemp[0]): number.number_schema(
                HeatOffOutsideTempNumber,
                icon=ICON_THERMOMETER,
                unit_of_measurement=UNIT_CELSIUS,
            ).extend({
                cv.Optional(CONF_INITIAL_VALUE): cv.All(cv.float_, lambda v: ensure_within_range(v, CONF_HeatOffOutsideTemp)),
            }),
            cv.Optional(CONF_HeaterOnOutsideTemp[0]): number.number_schema(
                HeaterOnOutsideTempNumber,
                icon=ICON_THERMOMETER,
                unit_of_measurement=UNIT_CELSIUS,
            ).extend({
                cv.Optional(CONF_INITIAL_VALUE): cv.All(cv.float_, lambda v: ensure_within_range(v, CONF_HeaterOnOutsideTemp)),
            }),
            cv.Optional(CONF_CoolSetpointTemp[0]): number.number_schema(
                CoolSetpointTempNumber,
                icon=ICON_THERMOMETER,
                unit_of_measurement=UNIT_CELSIUS,
            ).extend({
                cv.Optional(CONF_INITIAL_VALUE): cv.All(cv.float_, lambda v: ensure_within_range(v, CONF_CoolSetpointTemp)),
            }),
            cv.Optional(CONF_TankSetpointTemp[0]): number.number_schema(
                TankSetpointTempNumber,
                icon=ICON_THERMOMETER,
                unit_of_measurement=UNIT_CELSIUS,
            ).extend({
                cv.Optional(CONF_INITIAL_VALUE): cv.All(cv.float_, lambda v: ensure_within_range(v, CONF_TankSetpointTemp)),
            }),
            cv.Optional(CONF_ShiftSetpointTemp[0]): number.number_schema(
                ShiftSetpointTempNumber,
                icon=ICON_THERMOMETER,
                unit_of_measurement=UNIT_CELSIUS,
            ).extend({
                cv.Optional(CONF_INITIAL_VALUE): cv.All(cv.float_, lambda v: ensure_within_range(v, CONF_ShiftSetpointTemp)),
            }),
            cv.Optional(CONF_PumpSpeed[0]): number.number_schema(
                PumpSpeedNumber,
                icon=ICON_GAUGE,
            ).extend({
                cv.Optional(CONF_INITIAL_VALUE): cv.All(cv.float_, lambda v: ensure_within_range(v, CONF_PumpSpeed)),
            }),
        }
    ).extend(cv.COMPONENT_SCHEMA)
)


async def setup_conf(config, key, hub):
    if number_config := config.get(key[0]):
        nr = await number.new_number(number_config, min_value=key[1], max_value=key[2], step=key[3])
        cg.add(getattr(hub, f"set_{key[0]}_number")(nr))
        await cg.register_parented(nr, config[CONF_AQUAREA_ID])
        cg.add(nr.make_call().set_value(number_config[CONF_INITIAL_VALUE]).perform())

async def to_code(config):
    hub = await cg.get_variable(config[CONF_AQUAREA_ID])
    for key in TYPES:
        await setup_conf(config, key, hub)
