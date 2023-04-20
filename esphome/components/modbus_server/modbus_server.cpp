// This ESPHome component wraps around the modbus-esp8266 by @emelianov:
// https://github.com/emelianov/modbus-esp8266
//
// by @jpeletier - Epic Labs, 2022

#include "modbus_server.h"

#define TAG "ModbusServer"

namespace esphome {
namespace modbus_server {
ModbusServer::ModbusServer() {}

uint32_t ModbusServer::baudRate() { return this->parent_->get_baud_rate(); }

void ModbusServer::setup() { 
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
  }
  this->set_re_pin(false);
  mb.begin(this);
}

void ModbusServer::dump_config() {
    ESP_LOGCONFIG(TAG, "ModbusServer:");
    LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
}
float ModbusServer::get_setup_priority() const {
  // After UART bus
  return setup_priority::BUS - 1.0f;
}

void ModbusServer::set_address(uint8_t address) { mb.slave(address); }

bool ModbusServer::add_holding_register(uint16_t start_address, uint16_t value, uint16_t numregs) {
  return mb.addHreg(start_address, value, numregs);
}

bool ModbusServer::add_input_register(uint16_t start_address, uint16_t value, uint16_t numregs) {
  return mb.addIreg(start_address, value, numregs);
}

bool ModbusServer::write_holding_register(uint16_t address, uint16_t value) { return mb.Hreg(address, value); }

bool ModbusServer::write_input_register(uint16_t address, uint16_t value) { return mb.Ireg(address, value); }

uint16_t ModbusServer::read_holding_register(uint16_t address) { return mb.Hreg(address); }

uint16_t ModbusServer::read_input_register(uint16_t address) { return mb.Ireg(address); }

void ModbusServer::on_read_holding_register(uint16_t address, cbOnReadWrite cb, uint16_t numregs) {
  mb.onGet(
      HREG(address), [cb](TRegister *reg, uint16_t val) -> uint16_t { return cb(reg->address.address, val); }, numregs);
}

void ModbusServer::on_read_input_register(uint16_t address, cbOnReadWrite cb, uint16_t numregs) {
  mb.onGet(
      IREG(address), [cb](TRegister *reg, uint16_t val) -> uint16_t { return cb(reg->address.address, val); }, numregs);
}

void ModbusServer::on_write_holding_register(uint16_t address, cbOnReadWrite cb, uint16_t numregs) {
  mb.onSet(
      HREG(address), [cb](TRegister *reg, uint16_t val) -> uint16_t { return cb(reg->address.address, val); }, numregs);
}

void ModbusServer::on_write_input_register(uint16_t address, cbOnReadWrite cb, uint16_t numregs) {
  mb.onSet(
      IREG(address), [cb](TRegister *reg, uint16_t val) -> uint16_t { return cb(reg->address.address, val); }, numregs);
}

// Stream class implementation:
size_t ModbusServer::write(uint8_t data) {
  ESP_LOGD(TAG, "write");
  this->set_re_pin(true);
  this->sending = true;

  uart::UARTDevice::write_byte(data);

  return 1;
}
size_t ModbusServer::write(const uint8_t *data, size_t size) {
  ESP_LOGD(TAG, "write array");
  this->set_re_pin(true);
  this->sending = true;
  uart::UARTDevice::write_array(data, size);

  return size;
}

int ModbusServer::read() {
  this->set_re_pin(false);
  return uart::UARTDevice::read();
}

int ModbusServer::available() { return uart::UARTDevice::available(); }
//int ModbusServer::read() { return uart::UARTDevice::read(); }
int ModbusServer::peek() { return uart::UARTDevice::peek(); }
void ModbusServer::flush() {
  ESP_LOGD(TAG, "Flush!");
  if ( this->sending ) {
    this->set_re_pin(true);
    auto hw_serial = this->get_hw_serial();
    if ( hw_serial ) {
      ESP_LOGD(TAG, "Flush HW TX! %i");
      hw_serial->flush(true);
    }
    this->sending = false;
  } else {
    ESP_LOGD(TAG, "Flush none hw!");
    uart::UARTDevice::flush();
  }
  this->set_re_pin(false);
}

void ModbusServer::loop() { 
  if( !this->sending ) {
    this->set_re_pin(false);
  }
  mb.task(); 
};
void ModbusServer::set_re_pin(bool val) {
  if (this->flow_control_pin_ != nullptr) {
    if (this->flow_control_pin_->digital_read() != val) {
      ESP_LOGD(TAG, "Set Flow Pin %s", val ? "HIGH" : "LOW");
      this->flow_control_pin_->digital_write(val);
    }
  }
}
}  // namespace modbus_server

}  // namespace esphome
