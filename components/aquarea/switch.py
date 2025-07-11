import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from . import AquareaComponent, aquarea_ns, CONF_AQUAREA_ID

ForceSwitch = aquarea_ns.class_("ForceSwitch", switch.Switch)
CONF_Force = "force"

TYPES = [
   CONF_Force
]

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_AQUAREA_ID): cv.use_id(AquareaComponent),
            cv.Optional(CONF_Force): switch.switch_schema(ForceSwitch),
        }
    ).extend(cv.COMPONENT_SCHEMA)
)


async def setup_conf(config, key, hub):
    if switch_config := config.get(key):
        sw = await switch.new_switch(switch_config)
        cg.add(getattr(hub, f"set_{key}_switch")(sw))
        await cg.register_parented(sw, config[CONF_AQUAREA_ID])


async def to_code(config):
    hub = await cg.get_variable(config[CONF_AQUAREA_ID])
    for key in TYPES:
        await setup_conf(config, key, hub)
