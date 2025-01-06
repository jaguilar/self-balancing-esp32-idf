#ifndef MOTOR_FEEDBACK_MOTOR_H
#define MOTOR_FEEDBACK_MOTOR_H

#include <Arduino.h>
#include <FixedPointsCommon.h>

#include <optional>

#include "intpid.h"
#include "motor.h"
#include "sensor.h"

namespace jemotor {

// A motor that has a position feedback sensor. This type of motor
// can be set to a target speed or a target position.
//
// In either angle or speed mode, the motor has a PID controller that attempts
// to hold the motor at the target position or speed by adjusting the duty
// cycle. The PID's parameters may be updated while the motor is idle.
class FeedbackMotor {
 public:
  FeedbackMotor(Motor& motor, Sensor& sensor);

  // Stops the motor using the given StopMode. Cancels control of the motor.
  void Stop(StopMode mode);

  // Returns the current angle of the motor in decidegrees from zero.
  // Positive indicates clockwise, unless SetPositiveDirection has been set.
  SQ15x16 angle() const;

  // Returns the speed of the motor in decidegrees per second. Positive
  // indicates clockwise unless SetPositiveDirection has been set.
  SQ15x16 speed() const;

  // Puts this motor into duty mode, where the duty cycle is controlled
  // directly. Negative duty changes the direction of the motor.
  void SetDuty(int duty);

  // Sets the target speed of the motor in degrees per second.
  void SetSpeed(int speed);

  struct UpdateDebug {
    float dt;
    float setpoint;
    float measurement;
    float p;
    float i;
    float d;
    float output;
  };

  // Updates the control loop. Requests the sensor to update inline, no need to
  // call update on it separately.
  UpdateDebug UpdateControl(uint64_t t_us);

  // Causes us to rotate the motor counterclockwise when the motor is running
  // in a positive direction.
  //
  // Illegal to call while the motor is running.
  void SetPositiveDirection(Direction direction);

  // Resets the current motor position to have the given angle value.
  //
  // Illegal to call while the motor is running.
  void ResetAngle(SQ15x16 angle = 0);

  // Returns a reference to the default speed PID configuration.
  static const intpid::Config& DefaultSpeedConfig();

  // Returns a reference to teh default angle PID configuration.
  static const intpid::Config& DefaultAngleConfig();

  // Override the default speed PID config. The provided pointer must remain
  // valid for the duration of motor control. Must only be called when the motor
  // is idle.
  //
  // This may return an error if there is a problem with the config. If so,
  // the default will not be overridden.
  std::expected<void, std::string> SetSpeedConfig(const intpid::Config* config);

  // Override the default angle PID config. The provided pointer must remain
  // valid for the duration of motor control. Must only be called when the motor
  // is idle.
  //
  // This may return an error if there is a problem with the config. If so,
  // the default will not be overridden.
  std::expected<void, std::string> SetAngleConfig(const intpid::Config* config);

 private:
  // Converts +- duty into a direction and hardware duty cycle, and sets those
  // on the motor.
  void InternalSetDuty(int duty);

  // Calls SetDirection as appropriate on the motor_ given the desired duty.
  void HardwareSetDirection(int duty);

  Motor& motor_;
  Sensor& sensor_;

  bool invert_direction_ = false;
  int duty_ = 0;

  uint64_t last_update_ = micros();

  enum ControlMode {
    kIdle,
    kDc,
    kAngle,
    kSpeed,
  };
  ControlMode control_mode_ = kIdle;

  // In kAngle, attempts to minimize the difference between the desired angle
  // and current angle by controlling the motor duty cycle.
  // Maybe?
  //
  // In kSpeed, attempts to minimize the difference between the desired speed
  // and current speed by controlling the motor duty cycle.
  std::optional<intpid::Pid> controller_;

  // Overrideable configuration for the PID controllers.
  const intpid::Config* speed_config_ = &DefaultSpeedConfig();
  const intpid::Config* angle_config_ = &DefaultAngleConfig();
};

inline void FeedbackMotor::Stop(StopMode mode) { motor_.Stop(mode); }

inline SQ15x16 FeedbackMotor::angle() const {
  return invert_direction_ ? -sensor_.angle() : sensor_.angle();
}

inline SQ15x16 FeedbackMotor::speed() const {
  return invert_direction_ ? -sensor_.rate() : sensor_.rate();
}

}  // namespace jemotor

#endif  // MOTOR_FEEDBACK_MOTOR_H