#ifndef MOTOR_THREE_WIRE_MOTOR_H
#define MOTOR_THREE_WIRE_MOTOR_H

#include <variant>

#include "motor.h"

namespace jemotor {

class ThreeWireMotor : public Motor {
 public:
  // Creates a motor with the given pins.
  ThreeWireMotor(int pwm_pin, int fw_pin, int rev_pin);

  void Begin();

  void Stop(StopMode mode) override;
  void SetDirection(Direction direction) override;

  // Sets the duty cycle of the motor by passing the value to
  // analogWrite. The effective duty cycle depends on the current
  // analogWriteResolution of the pwm_pin_.
  void SetDuty(int duty) override;

 private:
  std::variant<StopMode, Direction> last_mode_;

  const int pwm_pin_;
  const int forward_pin_;
  const int reverse_pin_;
};

}  // namespace jemotor

#endif  // MOTOR_THREE_WIRE_MOTOR_H