#pragma once

#include <array>
#include <functional>
#include <vector>
#include <cstdint>
#include <queue>
#include "esphome/core/hal.h"
#include "esphome/core/gpio.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace aquarea {

enum Registers
{
  Defrost = 17, // -> 0x02 booster
  OutsideTemp = 18,
  OutletTemp = 19,
  ErrorCode =  20,
  InletTemp = 21,
  TankTemp  =  22,
  Compressor = 23,
  LastErrorCode = 24,
  Mode = 27,
  HeatLSB = 29,
  HeatMSB = 30,
  CoolLSB = 31,
  CoolMSB = 32,
  TankLSB = 33,
  TankMSB = 34,
  PumpStage = 35,
  Force = 128, //-> 0x40
  ResetError = 129,
  HeatOutsideTempLow = 130,
  HeatOutsideTempHigh = 131,
  HeatWaterTempLow = 132,
  HeatWaterTempHigh = 133,
  HeatOffOutsideTemp = 134,
  HeaterOnOutsideTemp = 135,
  CoolSetpointTemp = 136,
  TankSetpointTemp = 137,
  ShiftSetpoint = 138,
  ReqMode = 144,
  PumpSpeed = 147,
  // Settings 192; // -> 0x1 room thermo, 0x80, anitfreeze, 0x04 solar prio, 0x40 cool prio, 0x08 heat prio, 0x10 steril
  // HeatInterval 193; // -> 1 == 30min, 20 == 10h
  // TankInterval 194; // in min
  // BoosterDelay 195; // in min
  // SterilTemp 196;
  // SterilTime 197; // in min
  // SettingsCont 198; // -> 0x04 booster, 0x08 bpan
  // AutoToCoolTemp 199;
  // AutoToHeatTemp 200;
  // UnknownCommand 255;
  Num
};

std::array<std::function<void(uint8_t value)>, (size_t)Registers::Num> registers;

class AquareaComponent : public Component, public uart::UARTDevice
{
public:
    AquareaComponent() = delete;
    AquareaComponent(InternalGPIOPin *rts_pin_)
    {
      rx_packet.reserve(4);
      rts_pin = rts_pin_;
    }

protected:
    SUB_BINARY_SENSOR(defrost);
    SUB_SENSOR(outside_temperature);
    SUB_SENSOR(outlet_temperature);
    SUB_SENSOR(inlet_temperature);
    SUB_SENSOR(tank_temperature);
    SUB_SENSOR(compressor_value);
    SUB_SENSOR(heat_power);
    SUB_SENSOR(cool_power);
    SUB_SENSOR(tank_power);
    SUB_SENSOR(pump_speed);
    SUB_TEXT_SENSOR(error_code);
    SUB_TEXT_SENSOR(last_error_code);
    SUB_TEXT_SENSOR(current_mode);
    SUB_BUTTON(reset_error);
    SUB_SWITCH(force);
    SUB_NUMBER(heat_out_temp_low);
    SUB_NUMBER(heat_out_temp_high);
    SUB_NUMBER(heat_water_temp_low);
    SUB_NUMBER(heat_water_temp_high);
    SUB_NUMBER(heat_off_out_temp);
    SUB_NUMBER(heater_on_out_temp);
    SUB_NUMBER(cool_setpoint_temp);
    SUB_NUMBER(tank_setpoint_temp);
    SUB_NUMBER(shift_setpoint_temp);
    SUB_NUMBER(pump_speed);
    SUB_SELECT(request_mode);

  std::queue<std::pair<uint8_t /*register*/, uint8_t /* value */>> write_queue;
private:
  struct Power
  {
    void write_lb(long timestamp, uint8_t value)
    {
      valid = false;
      lb_timestamp = timestamp;
      power=value;
    }

    void write_mb(long timestamp, uint8_t value)
    {
      power |= ((uint16_t)value)<<8;
      if((timestamp - lb_timestamp) < 200) valid = true;
    }

    bool is_valid() const
    {
      return valid;
    }

    uint16_t get_power() const
    {
      return power;
    }

  private:
    long lb_timestamp = 0;
    uint16_t power=0u;
    bool valid=false;
  };

  void update_binary_sensor(binary_sensor::BinarySensor* sensor, uint8_t value)
  {
    sensor->publish_state(value & 0x64);
  }

  void update_sensor(sensor::Sensor* sensor, uint8_t value)
  {
    sensor->publish_state(*((int8_t*)&value));
  }

  void update_power_lsb(Power& power, uint8_t value)
  {
    power.write_lb(millis(), value);
  }

  void update_power_msb(sensor::Sensor* sensor, Power& power, uint8_t value)
  {
    power.write_mb(millis(), value);
    if(power.is_valid())
      sensor->publish_state(power.get_power());
  }

  void update_text_sensor(text_sensor::TextSensor* sensor, uint8_t value)
  {
    sensor->publish_state(std::to_string((uint32_t)value));
  }

  template<typename Type>
  void update_current_mode(Type* type, uint8_t value)
  {
    std::string mode(value & 1 ? "on" : "off");
    if (value & 2) mode += " heat";
    if (value & 4) mode += " cool";
    if (value & 16) mode += " tank";
    if (value & 32) mode += " auto";
    if (value & 64) mode += " quiet";
    if (value & 128) mode += " heater";

    type->publish_state(mode);
  }

  void update_switch(switch_::Switch* sw, uint8_t value)
  {
    sw->publish_state(value & 0x40);
  }

  void update_number(number::Number* number, uint8_t value)
  {
    number->publish_state(*((int8_t*)&value));
  }

  void setup_registers_cbk()
  {
    registers[Registers::Defrost] = std::bind(&AquareaComponent::update_binary_sensor, this, defrost_binary_sensor_, std::placeholders::_1);
    registers[Registers::OutsideTemp] = std::bind(&AquareaComponent::update_sensor, this, outside_temperature_sensor_, std::placeholders::_1);
    registers[Registers::OutletTemp] = std::bind(&AquareaComponent::update_sensor, this, outlet_temperature_sensor_, std::placeholders::_1);
    registers[Registers::InletTemp] = std::bind(&AquareaComponent::update_sensor, this, inlet_temperature_sensor_, std::placeholders::_1);
    registers[Registers::TankTemp] = std::bind(&AquareaComponent::update_sensor, this, tank_temperature_sensor_, std::placeholders::_1);
    registers[Registers::Compressor] = std::bind(&AquareaComponent::update_sensor, this, compressor_value_sensor_, std::placeholders::_1);
    registers[Registers::HeatLSB] = std::bind(&AquareaComponent::update_power_lsb, this, std::ref(heat_power), std::placeholders::_1);
    registers[Registers::HeatMSB] = std::bind(&AquareaComponent::update_power_msb, this, heat_power_sensor_, std::ref(heat_power), std::placeholders::_1);
    registers[Registers::CoolLSB] = std::bind(&AquareaComponent::update_power_lsb, this, std::ref(cool_power), std::placeholders::_1);
    registers[Registers::CoolMSB] = std::bind(&AquareaComponent::update_power_msb, this, cool_power_sensor_, std::ref(cool_power), std::placeholders::_1);
    registers[Registers::TankLSB] = std::bind(&AquareaComponent::update_power_lsb, this, std::ref(tank_power), std::placeholders::_1);
    registers[Registers::TankMSB] = std::bind(&AquareaComponent::update_power_msb, this, tank_power_sensor_, std::ref(tank_power), std::placeholders::_1);
    registers[Registers::PumpStage] = [&](uint8_t value){this->update_sensor(pump_speed_sensor_, value/16);};
    registers[Registers::ErrorCode] = std::bind(&AquareaComponent::update_text_sensor, this, error_code_text_sensor_, std::placeholders::_1);
    registers[Registers::LastErrorCode] = std::bind(&AquareaComponent::update_text_sensor, this, last_error_code_text_sensor_, std::placeholders::_1);
    registers[Registers::Mode] = std::bind(&AquareaComponent::update_current_mode<text_sensor::TextSensor>, this, current_mode_text_sensor_, std::placeholders::_1);
    registers[Registers::Force] = std::bind(&AquareaComponent::update_switch, this, force_switch_, std::placeholders::_1);
    registers[Registers::HeatOutsideTempLow] = std::bind(&AquareaComponent::update_number, this, heat_out_temp_low_number_, std::placeholders::_1);
    registers[Registers::HeatOutsideTempHigh] = std::bind(&AquareaComponent::update_number, this, heat_out_temp_high_number_, std::placeholders::_1);
    registers[Registers::HeatWaterTempLow] = std::bind(&AquareaComponent::update_number, this, heat_water_temp_low_number_, std::placeholders::_1);
    registers[Registers::HeatWaterTempHigh] = std::bind(&AquareaComponent::update_number, this, heat_water_temp_high_number_, std::placeholders::_1);
    registers[Registers::HeatOffOutsideTemp] = std::bind(&AquareaComponent::update_number, this, heat_off_out_temp_number_, std::placeholders::_1);
    registers[Registers::HeaterOnOutsideTemp] = std::bind(&AquareaComponent::update_number, this, heater_on_out_temp_number_, std::placeholders::_1);
    registers[Registers::CoolSetpointTemp] = std::bind(&AquareaComponent::update_number, this, cool_setpoint_temp_number_, std::placeholders::_1);
    registers[Registers::TankSetpointTemp] = std::bind(&AquareaComponent::update_number, this, tank_setpoint_temp_number_, std::placeholders::_1);
    registers[Registers::ShiftSetpoint] = std::bind(&AquareaComponent::update_number, this, shift_setpoint_temp_number_, std::placeholders::_1);
    registers[Registers::PumpSpeed] = std::bind(&AquareaComponent::update_number, this, pump_speed_number_, std::placeholders::_1);
    registers[Registers::ReqMode] = std::bind(&AquareaComponent::update_current_mode<select::Select>, this, request_mode_select_, std::placeholders::_1);
  }

  float get_setup_priority() const override
  {
    return setup_priority::BUS;
  }

  void setup() override
  {
    ESP_LOGD("aquarea", "%s", __func__);
    setup_registers_cbk();
    rts_pin->digital_write(false);
  }

  void loop() override
  {
    bool wait = false;
    long ts = millis();
    do
    {
      if(heatpump_read())
      {
        if(wait)
        {
          rts_pin->digital_write(false);
          ESP_LOGD("aquarea", "rts low, done after %lu", millis() - ts);
          wait = false;
        }

        if((rx_packet[0] == 0x55))
        {
          ESP_LOGV("aquarea", "data %x %x %x %x", rx_packet[0], rx_packet[1], rx_packet[2], rx_packet[3]);
          heatpump_update(rx_packet[1], rx_packet[2]);
          const bool gotMagick = rx_packet[1] == Registers::Defrost;
          if(gotMagick && !write_queue.empty())
          {
            wait = true;
            ESP_LOGD("aquarea", "rts high");
            rts_pin->digital_write(true);
            delay(1);
            heatpump_write();
          }
        }
        rx_packet.clear();
      }

      if((millis() - ts) > 300u)
      {
          rts_pin->digital_write(false);
          ESP_LOGD("aquarea", "rts low, timeout");
          break;
      }
    }while(wait);
  }

  uint8_t heatpump_checksum(const std::vector<uint8_t>& packet)
  {
    return (packet[0] + packet[1] + packet[2]) & 0xff;
  }

  bool heatpump_read(bool log = false)
  {
    if(this->available())
    {
      rx_packet.push_back(this->read());
      ESP_LOGVV("aquarea", "data %x", rx_packet[0]);
      if((rx_packet[0] == 0xAA || rx_packet[0] == 0x55))
      {
        if(rx_packet.size() == 4)
        {
          uint8_t rx_chsum = heatpump_checksum(rx_packet);
          ESP_LOGVV("aquarea", "data %x %x %x %x", rx_packet[0], rx_packet[1], rx_packet[2], rx_packet[3]);
          if(rx_chsum == rx_packet[3])
          {
              return true;
          }
          else
          {
            ESP_LOGW("aquarea", "Checksum mismatch %x %x", rx_chsum, rx_packet[3]);
            rx_packet.clear();
          }
        }
      }
      else
      {
        rx_packet.clear();
      }
    }
    return false;
  }

  void heatpump_write()
  {
    std::vector<uint8_t> packet({0xAA, write_queue.front().first, write_queue.front().second});
    packet.push_back(heatpump_checksum(packet));

    ESP_LOGD("aquarea", "send: %x %x", packet[1], packet[2]);
    this->write_array(packet);
    this->flush();
    write_queue.pop();
  }

  void heatpump_update(uint8_t reg, uint8_t value)
  {
    if(registers[reg])
      registers[reg](value);
  }

  Power heat_power;
  Power tank_power;
  Power cool_power;
  GPIOPin* rts_pin;
  std::vector<uint8_t> rx_packet;
};

class ResetErrorButton : public button::Button, public Parented<AquareaComponent>
{
 protected:
  void press_action() override
  {
    ESP_LOGD("aquarea", "ResetErrorButton");
    parent_->write_queue.push(std::make_pair(Registers::ResetError, 1));
  }
};

class ForceSwitch : public switch_::Switch, public Parented<AquareaComponent>
{
protected:
  void write_state(bool state) override
  {
    ESP_LOGD("aquarea", "ForceSwitch");
    parent_->write_queue.push(std::make_pair(Registers::Force, state ? 0x40 : 0x00));
  }
};

class HeatOutsideTempLowNumber : public number::Number, public Parented<AquareaComponent>
{
protected:
  void control(float value) override
  {
    ESP_LOGD("aquarea", "HeatOutsideTempLowNumber");
    parent_->write_queue.push(std::make_pair(Registers::HeatOutsideTempLow, value));
  }
};

class HeatOutsideTempHighNumber : public number::Number, public Parented<AquareaComponent>
{
protected:
  void control(float value) override
  {
    ESP_LOGD("aquarea", "HeatOutsideTempHighNumber");
    parent_->write_queue.push(std::make_pair(Registers::HeatOutsideTempHigh, value));
  }
};

class HeatWaterTempLowNumber : public number::Number, public Parented<AquareaComponent>
{
protected:
  void control(float value) override
  {
    ESP_LOGD("aquarea", "HeatWaterTempLowNumber");
    parent_->write_queue.push(std::make_pair(Registers::HeatWaterTempLow, value));
  }
};

class HeatWaterTempHighNumber : public number::Number, public Parented<AquareaComponent>
{
protected:
  void control(float value) override
  {
    ESP_LOGD("aquarea", "HeatWaterTempHighNumber");
    parent_->write_queue.push(std::make_pair(Registers::HeatWaterTempHigh, value));
  }
};

class HeatOffOutsideTempNumber : public number::Number, public Parented<AquareaComponent>
{
protected:
  void control(float value) override
  {
    ESP_LOGD("aquarea", "HeatOffOutsideTempNumber");
    parent_->write_queue.push(std::make_pair(Registers::HeatOffOutsideTemp, value));
  }
};

class HeaterOnOutsideTempNumber : public number::Number, public Parented<AquareaComponent>
{
protected:
  void control(float value) override
  {
    ESP_LOGD("aquarea", "HeaterOnOutsideTempNumber");
    parent_->write_queue.push(std::make_pair(Registers::HeaterOnOutsideTemp, value));
  }
};

class CoolSetpointTempNumber : public number::Number, public Parented<AquareaComponent>
{
protected:
  void control(float value) override
  {
    ESP_LOGD("aquarea", "CoolSetpointTempNumber");
    parent_->write_queue.push(std::make_pair(Registers::CoolSetpointTemp, value));
  }
};

class TankSetpointTempNumber : public number::Number, public Parented<AquareaComponent>
{
protected:
  void control(float value) override
  {
    ESP_LOGD("aquarea", "TankSetpointTempNumber");
    parent_->write_queue.push(std::make_pair(Registers::TankSetpointTemp, value));
  }
};

class ShiftSetpointTempNumber : public number::Number, public Parented<AquareaComponent>
{
protected:
  void control(float value) override
  {
    ESP_LOGD("aquarea", "ShiftSetpointTempNumber");
    parent_->write_queue.push(std::make_pair(Registers::ShiftSetpoint, value));
  }
};

class PumpSpeedNumber : public number::Number, public Parented<AquareaComponent>
{
protected:
  void control(float value) override
  {
    ESP_LOGD("aquarea", "PumpSpeedNumber");
    parent_->write_queue.push(std::make_pair(Registers::PumpSpeed, value));
  }
};

class ReqModeSelect : public select::Select, public Parented<AquareaComponent>
{
protected:
  void control(const std::string& value) override
  {
    ESP_LOGD("aquarea", "ReqModeSelect: %s", value.c_str());
    uint8_t mode = 0u;
    if(std::string::npos != value.find("on")) mode += 1;
    if(std::string::npos != value.find("heat")) mode += 2;
    if(std::string::npos != value.find("cool")) mode += 4;
    if(std::string::npos != value.find("tank")) mode += 16;
    if(std::string::npos != value.find("auto")) mode += 32;
    if(std::string::npos != value.find("quiet"))  mode += 64;
    if(std::string::npos != value.find("heater"))  mode += 128;

    parent_->write_queue.push(std::make_pair(Registers::ReqMode, mode));
  }
};

} // namespace aquarea
} // namespace esphome
