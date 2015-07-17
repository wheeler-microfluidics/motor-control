#ifndef ___NODE__H___
#define ___NODE__H___

#include <stdint.h>
#include <Arduino.h>
#include <NadaMQ.h>
#include <BaseNodeRpc.h>
#include <BaseNodeEeprom.h>
#include <BaseNodeI2c.h>
#include <BaseNodeConfig.h>
#include <BaseNodeState.h>
#include <BaseNodeSerialHandler.h>
#include <BaseNodeI2cHandler.h>
#include <Array.h>
#include <I2cHandler.h>
#include <SerialHandler.h>
#include <pb_validate.h>
#include <pb_eeprom.h>
#include "TimerOne/TimerOne.h"
#include "motor_control_config_validate.h"
#include "motor_control_state_validate.h"
#include "motor_control_config_pb.h"


void on_timer_tick();

namespace motor_control {
const size_t FRAME_SIZE = (3 * sizeof(uint8_t)  // Frame boundary
                           - sizeof(uint16_t)  // UUID
                           - sizeof(uint16_t)  // Payload length
                           - sizeof(uint16_t));  // CRC

class Node;

typedef nanopb::EepromMessage<motor_control_Config,
                              config_validate::Validator<Node> > config_t;
typedef nanopb::Message<motor_control_State,
                        state_validate::Validator<Node> > state_t;


class Node :
  public BaseNode,
  public BaseNodeEeprom,
  public BaseNodeI2c,
  public BaseNodeConfig<config_t>,
  public BaseNodeState<state_t>,
#ifndef DISABLE_SERIAL
  public BaseNodeSerialHandler,
#endif  // #ifndef DISABLE_SERIAL
  public BaseNodeI2cHandler<base_node_rpc::i2c_handler_t> {
public:
  typedef PacketParser<FixedPacket> parser_t;
  static const uint16_t BUFFER_SIZE = 128;  // >= longest property string

  uint8_t buffer_[BUFFER_SIZE];
  uint32_t tick_count_;
  int32_t target_position_;
  static const uint8_t DIRECTION_PIN = 13;
  static const uint8_t ENABLE_PIN = 9;
  static const uint8_t STEP_PIN = 10;
  static const uint8_t MS1_PIN = 5;
  static const uint8_t MS2_PIN = 6;
  static const uint8_t MS3_PIN = 7;

  Node() : BaseNode(), BaseNodeConfig<config_t>(motor_control_Config_fields),
           BaseNodeState<state_t>(motor_control_State_fields),
           tick_count_(0), target_position_(0) {
    pinMode(DIRECTION_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(MS1_PIN, OUTPUT);
    pinMode(MS2_PIN, OUTPUT);
    pinMode(MS3_PIN, OUTPUT);

    digitalWrite(ENABLE_PIN, HIGH);  // ENABLE_PIN is active LOW
    set_MS(0);
  }

  void set_MS(uint8_t ms) {
    digitalWrite(MS1_PIN, 0x01 & ms);
    digitalWrite(MS2_PIN, 0x02 & ms);
    digitalWrite(MS3_PIN, 0x04 & ms);
  }

  UInt8Array get_buffer() { return UInt8Array(sizeof(buffer_), buffer_); }
  /* This is a required method to provide a temporary buffer to the
   * `BaseNode...` classes. */

  void begin();
  void set_i2c_address(uint8_t value);  // Override to validate i2c address

  int32_t target_position() { return target_position_; }
  void set_target_position(int32_t absolute) { target_position_ = absolute; }

  void on_tick() {
    tick_count_ += 1;
    if (!state_._.motor_enabled) { return; }
    if (state_._.motor_continuous) {
      /* In continuous mode, set new target position one step in the current
       * direction. */
      target_position_ += (state_._.motor_direction) ? 1 : -1;
    }
    if (target_position_ != state_._.motor_position) {
      if (target_position_ < state_._.motor_position) {
        on_state_motor_direction_changed(false);
      } else {
        on_state_motor_direction_changed(true);
      }
      // Pulse the step pin output.
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(state_._.motor_pulse_us);
      /* TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
       * TODO: Should *not* delay here, since this method may be called while
       * servicing an interrupt.  See [here][1] for more details.
       *
       * [1]: https://learn.adafruit.com/multi-tasking-the-arduino-part-2/interrupt-etiquette
       * TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
       */
      digitalWrite(STEP_PIN, LOW);
      state_._.motor_position += (state_._.motor_direction) ? 1 : -1;
    }
  }
  uint32_t tick_count() { return tick_count_; }
  void reset_tick_count() { tick_count_ = 0; }
  void set_period(uint32_t period) { Timer1.setPeriod(period); }
  void enable_timer(uint32_t period) { Timer1.attachInterrupt(on_timer_tick, period); }
  void disable_timer() { Timer1.detachInterrupt(); }

  int32_t position() const { return state_._.motor_position; }
  void move(int32_t relative) { set_target_position(position() + relative); }
  uint32_t pulse_us() const { return state_._.motor_pulse_us; }
  uint32_t delay_us() const { return state_._.motor_delay_us; }
  void motor_set_speed(int32_t steps_per_second) {
    uint32_t delay_us = ((steps_per_second < 0) ? -1e6 / steps_per_second
                                                : 1e6 / steps_per_second);
    on_state_motor_direction_changed(steps_per_second < 0);
    on_state_motor_delay_us_changed(delay_us);
  }
  void motor_start(int32_t steps_per_second) {
    on_state_motor_continuous_changed(true);
    motor_set_speed(steps_per_second);
  }
  void motor_stop() { on_state_motor_continuous_changed(false); }
  void motor_set_home() {
    target_position_ = 0;
    state_._.motor_position = 0;
  }

  bool on_state_motor_delay_us_changed(uint32_t new_value) {
    if (new_value == 0) {
      disable_timer();
    }
    else if (new_value < 200) { return false; }

    if (new_value != state_._.motor_delay_us) {
      if (state_._.motor_delay_us > 0) { disable_timer(); }
      enable_timer(new_value);
      state_._.motor_delay_us = new_value;
      state_._.has_motor_delay_us = true;
    }
    return true;
  }

  bool on_state_motor_continuous_changed(bool new_value) {
    state_._.motor_continuous = new_value;
    state_._.has_motor_continuous = true;
  }

  bool on_state_motor_enabled_changed(bool new_value) {
    if (new_value != state_._.motor_enabled) {
      if (new_value) {
        enable_timer(state_._.motor_delay_us);
        digitalWrite(ENABLE_PIN, LOW);  // ENABLE_PIN is active LOW
      } else {
        disable_timer();
        digitalWrite(ENABLE_PIN, HIGH);  // ENABLE_PIN is active LOW
      }
      state_._.motor_enabled = new_value;
      state_._.has_motor_enabled = true;
    }
    return true;
  }

  bool on_state_motor_direction_changed(bool new_value) {
    if (new_value != state_._.motor_direction) {
      // Change motor direction.
      digitalWrite(DIRECTION_PIN, new_value);
      state_._.motor_direction = new_value;
      state_._.has_motor_direction = true;
    }
    return true;
  }
};


}  // namespace motor_control


#endif  // #ifndef ___NODE__H___
