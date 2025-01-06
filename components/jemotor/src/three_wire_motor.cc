#include "three_wire_motor.h"

#include <Arduino.h>

namespace jemotor {

ThreeWireMotor::ThreeWireMotor(int pwm_pin, int fw_pin, int rev_pin)
    : pwm_pin_(pwm_pin), forward_pin_(fw_pin), reverse_pin_(rev_pin) {}

void ThreeWireMotor::Begin() {
  pinMode(pwm_pin_, OUTPUT);
  pinMode(forward_pin_, OUTPUT);
  pinMode(reverse_pin_, OUTPUT);
  Stop(kCoast);
}

void ThreeWireMotor::Stop(StopMode mode) {
  switch (mode) {
    case kBrake:
      digitalWrite(forward_pin_, HIGH);
      digitalWrite(reverse_pin_, HIGH);
      break;
    case kCoast:
      digitalWrite(forward_pin_, LOW);
      digitalWrite(reverse_pin_, LOW);
      break;
  }
  analogWrite(pwm_pin_, 0);
}

void ThreeWireMotor::SetDirection(Direction direction) {
  if (std::holds_alternative<Direction>(last_mode_) &&
      std::get<Direction>(last_mode_) == direction) {
    return;
  }
  switch (direction) {
    case kClockwise:
      digitalWrite(forward_pin_, HIGH);
      digitalWrite(reverse_pin_, LOW);
      break;
    case kCounterClockwise:
      digitalWrite(forward_pin_, LOW);
      digitalWrite(reverse_pin_, HIGH);
      break;
  }
}

void ThreeWireMotor::SetDuty(int duty) { analogWrite(pwm_pin_, duty); }

}  // namespace jemotor