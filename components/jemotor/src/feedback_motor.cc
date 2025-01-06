#include "feedback_motor.h"

#include <Arduino.h>

#include "esp32-hal-log.h"

namespace jemotor {

FeedbackMotor::FeedbackMotor(Motor& motor, Sensor& sensor)
    : motor_(motor), sensor_(sensor) {}

void FeedbackMotor::SetDuty(int duty) {
  HardwareSetDirection(duty);
  duty_ = duty;
  motor_.SetDuty(duty);
}

void FeedbackMotor::SetSpeed(int speed) {
  if (control_mode_ != kSpeed) {
    control_mode_ = kSpeed;
    auto pid = intpid::Pid::Create(*speed_config_);
    if (!pid) {
      ESP_LOGE("FeedbackMotor", "Failed to create speed PID: %s",
               pid.error().c_str());
      control_mode_ = kIdle;
      return;
    }
    controller_.emplace(*std::move(pid));
    controller_->set_setpoint(speed);
    controller_->Update(sensor_.rate(), 0);
  }
  controller_->set_setpoint(speed);
}

const intpid::Config& FeedbackMotor::DefaultSpeedConfig() {
  constexpr float kp = 0.3;
  constexpr float ki = kp / 250;
  static intpid::Config config = {
      .kp = kp,
      .ki = ki,
      .kd = 0,
      .output_min = -255,
      .output_max = 255,
  };
  return config;
}

const intpid::Config& FeedbackMotor::DefaultAngleConfig() {
  static intpid::Config config = {
      // TODO: tune these values.
  };
  return config;
}

FeedbackMotor::UpdateDebug FeedbackMotor::UpdateControl(uint64_t t_us) {
  // Each tick is considered to be 256us, which gives us slightly more
  // resolution than milliseconds but is less likely to overflow our fixed point
  // maths on 32 bits.
  const uint64_t dt_us = t_us - last_update_;
  last_update_ = t_us;
  const auto dt_ms = SQ15x16{SFixed<24, 3>{dt_us} / 1'000};

  UpdateDebug debug;
  debug.dt = static_cast<float>(dt_ms);

  SQ15x16 duty;
  switch (control_mode_) {
    case kIdle:
    case kDc:
      debug.output = static_cast<float>(duty_);
      debug.measurement = static_cast<float>(sensor_.rate());
      return debug;
    case kSpeed: {
      const SQ15x16 speed = this->speed();
      duty = -controller_->Update(speed, dt_ms);
      debug.setpoint = static_cast<float>(controller_->setpoint());
      debug.measurement = static_cast<float>(speed);
      debug.p = static_cast<float>(controller_->p());
      debug.i = static_cast<float>(controller_->i());
      debug.d = static_cast<float>(controller_->d());
      debug.output = static_cast<float>(duty);
      break;
    }
    default:
      assert(false);  // Not yet implemented.
  }

  int iduty = int{duty};
  HardwareSetDirection(iduty);
  duty_ = iduty;
  if (std::abs(iduty) < 30) {
    motor_.Stop(kCoast);
  } else {
    motor_.SetDuty(std::abs(iduty));
  }
  return debug;
}

void FeedbackMotor::HardwareSetDirection(int duty) {
  const Direction fw = invert_direction_ ? kCounterClockwise : kClockwise;
  const Direction rev = invert_direction_ ? kClockwise : kCounterClockwise;
  if (duty < 0) {
    motor_.SetDirection(rev);
  } else {
    motor_.SetDirection(fw);
  }
}

void FeedbackMotor::SetPositiveDirection(Direction direction) {
  invert_direction_ = direction == kCounterClockwise;
}

void FeedbackMotor::ResetAngle(SQ15x16 angle) {
  assert(duty_ == 0);
  sensor_.SetAngle(angle);
}

std::expected<void, std::string> FeedbackMotor::SetSpeedConfig(
    const intpid::Config* config) {
  auto pid = intpid::Pid::Create(*config);
  if (!pid) {
    return std::unexpected(pid.error());
  }
  speed_config_ = config;
  return {};
}

std::expected<void, std::string> FeedbackMotor::SetAngleConfig(
    const intpid::Config* config) {
  auto pid = intpid::Pid::Create(*config);
  if (!pid) {
    return std::unexpected(pid.error());
  }
  angle_config_ = config;
  return {};
}

}  // namespace jemotor