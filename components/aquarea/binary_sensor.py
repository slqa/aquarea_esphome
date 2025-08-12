import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import ICON_RADIATOR, ICON_HEATING_COIL
from . import AquareaComponent, CONF_AQUAREA_ID

CONF_Defrost = "defrost"
CONF_Booster = "booster"

TYPES = [
   CONF_Defrost,
   CONF_Booster,
]

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_AQUAREA_ID): cv.use_id(AquareaComponent),
            cv.Optional(CONF_Defrost): binary_sensor.binary_sensor_schema(
                icon=ICON_RADIATOR,
            ),
            cv.Optional(CONF_Booster): binary_sensor.binary_sensor_schema(
                icon=ICON_HEATING_COIL,
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA)
)


async def setup_conf(config, key, hub):
    if sensor_config := config.get(key):
        sens = await binary_sensor.new_binary_sensor(sensor_config)
        cg.add(getattr(hub, f"set_{key}_binary_sensor")(sens))


async def to_code(config):
    hub = await cg.get_variable(config[CONF_AQUAREA_ID])
    for key in TYPES:
        await setup_conf(config, key, hub)
