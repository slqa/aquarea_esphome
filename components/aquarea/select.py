import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import CONF_OPTIONS, CONF_OPTIMISTIC, CONF_RESTORE_VALUE
from . import AquareaComponent, aquarea_ns, CONF_AQUAREA_ID

ReqModeSelect = aquarea_ns.class_("ReqModeSelect", select.Select)

CONF_ReqMode = "request_mode"
CONF_ReqModes = ["off heat", "on heat", "on cool", "on tank", "on heat tank", "on cool tank"]

def ensure_option_map(options):
    cv.check_not_templatable(options)
    for option in options:
        if option not in CONF_ReqModes:
            raise cv.Invalid("Invalid option selected " + option)
    return options

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_AQUAREA_ID): cv.use_id(AquareaComponent),
            cv.Optional(CONF_ReqMode): select.select_schema(ReqModeSelect)
                .extend(
                    {
                        cv.Required(CONF_OPTIONS): ensure_option_map,
                    }
                )
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_AQUAREA_ID])
    sel = await select.new_select(config[CONF_ReqMode], options=config[CONF_ReqMode][CONF_OPTIONS])
    cg.add(getattr(hub, f"set_{CONF_ReqMode}_select")(sel))
    await cg.register_parented(sel, config[CONF_AQUAREA_ID])
