import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from . import AquareaComponent, aquarea_ns, CONF_AQUAREA_ID

ResetErrorButton = aquarea_ns.class_("ResetErrorButton", button.Button)
CONF_ResetError = "reset_error"

TYPES = [
   CONF_ResetError
]

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_AQUAREA_ID): cv.use_id(AquareaComponent),
            cv.Optional(CONF_ResetError): button.button_schema(ResetErrorButton),
        }
    ).extend(cv.COMPONENT_SCHEMA)
)


async def setup_conf(config, key, hub):
    if button_config := config.get(key):
        btn = await button.new_button(button_config)
        cg.add(getattr(hub, f"set_{key}_button")(btn))
        await cg.register_parented(btn, config[CONF_AQUAREA_ID])


async def to_code(config):
    hub = await cg.get_variable(config[CONF_AQUAREA_ID])
    for key in TYPES:
        await setup_conf(config, key, hub)
