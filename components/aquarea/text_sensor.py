import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import ICON_BUG
from . import AquareaComponent, CONF_AQUAREA_ID

CONF_ErrorCode = "error_code"
CONF_LastErrorCode = "last_error_code"
CONF_Mode = "current_mode"

TYPES = [
   CONF_ErrorCode,
   CONF_LastErrorCode,
   CONF_Mode
]

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_AQUAREA_ID): cv.use_id(AquareaComponent),
            cv.Optional(CONF_ErrorCode): text_sensor.text_sensor_schema(
                icon=ICON_BUG,
            ),
            cv.Optional(CONF_LastErrorCode): text_sensor.text_sensor_schema(
                icon=ICON_BUG,
            ),
            cv.Optional(CONF_Mode): text_sensor.text_sensor_schema(),
        }
    ).extend(cv.COMPONENT_SCHEMA)
)


async def setup_conf(config, key, hub):
    if sensor_config := config.get(key):
        sens = await text_sensor.new_text_sensor(sensor_config)
        cg.add(getattr(hub, f"set_{key}_text_sensor")(sens))


async def to_code(config):
    hub = await cg.get_variable(config[CONF_AQUAREA_ID])
    for key in TYPES:
        await setup_conf(config, key, hub)
