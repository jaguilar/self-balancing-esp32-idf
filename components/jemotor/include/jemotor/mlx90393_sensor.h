#ifndef MOTOR_MLX90393_SENSOR_H
#define MOTOR_MLX90393_SENSOR_H

#include <Arduino.h>
#include <FixedPointsCommon.h>

#include "Adafruit_MLX90393.h"
#include "sensor.h"

namespace jemotor {

class MLX90393Sensor : public Sensor {
 public:
  MLX90393Sensor(Adafruit_MLX90393* sensor) : sensor_(sensor) {}
  ~MLX90393Sensor() override {}

  bool Update(uint64_t t);

  // Returns the current angle of the in decidegrees. Note that this is the
  // total accumulated angle since startup. The absolute angle is not provided.
  SQ15x16 angle() override { return angle_; }

  // Returns the rate of rotation in degrees/sec during the last sensing
  // period. Positive is clockwise.
  SQ15x16 rate() override { return speed_; }

  void SetAngle(SQ15x16 angle) override { angle_ = angle; };

 private:
  Adafruit_MLX90393* const sensor_;

  // Last angle reading in decidegrees.
  uint64_t t_ = micros();
  SQ15x16 rawangle_ = 0;  // The last sensed angle.
  SQ15x16 angle_ = 0;     // The total angle accumulated since startup.
  SQ15x16 speed_ = 0;
};

}  // namespace jemotor

#endif  // MOTOR_MLX90393_SENSOR_H