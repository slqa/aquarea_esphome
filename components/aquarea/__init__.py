import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import uart, output
from esphome.const import CONF_ID

MULTI_CONF = True
CONF_AQUAREA_ID = "aquarea_id"
CONF_RTS_PIN = "rts_pin"

aquarea_ns = cg.esphome_ns.namespace("aquarea")
AquareaComponent = aquarea_ns.class_("AquareaComponent", cg.Component)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AquareaComponent),
            cv.Required(CONF_RTS_PIN): pins.internal_gpio_output_pin_schema,
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    rts_pin = await cg.gpio_pin_expression(config[CONF_RTS_PIN])
    var = cg.new_Pvariable(config[CONF_ID], rts_pin)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
