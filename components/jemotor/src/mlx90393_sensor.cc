#include "mlx90393_sensor.h"

#include <cmath>

namespace jemotor {

SQ15x16 VectorToAngleDecidegrees(float x, float y) {
  if (x == 0.0 && y == 0.0) {
    return 0;  // Angle is undefined for the zero vector
  } else if (x == 0.0 && y > 0.0) {
    return 90;  // Angle is 90 degrees
  } else if (x == 0.0 && y < 0.0) {
    return 270;  // Angle is 270 degrees
  }

  float angle = atan2(y, x) * 180 / PI;
  if (angle < 0) {
    angle = 360 + angle;
  }
  return angle;
}

bool MLX90393Sensor::Update(uint64_t t) {
  std::array<float, 2> data;
  if (!sensor_->readMeasurement(MLX90393_X | MLX90393_Y, data)) {
    return false;
  }

  // Note we only update sample time when we can successfully take a sample.
  const SQ15x16 dt_ms = SQ15x16{SFixed<24, 4>{t - t_} / 1'000};
  t_ = t;

  const SQ15x16 newangle = VectorToAngleDecidegrees(data[0], data[1]);
  const SQ15x16 delta = newangle - rawangle_;
  rawangle_ = newangle;

  SQ15x16 corrected_delta;
  if (absFixed(delta) < 180) {
    // If the delta was less than 180, we'll assume that it's a normal move --
    // that is, the sensor did not wrap around.
    corrected_delta = delta;
  } else {
    // If the move was more than 180 degrees, we assume that we've wrapped
    // around. That means the *sign* of the move is the opposite delta, and the
    // magnitude is 360 - abs(delta).
    corrected_delta = (delta > 0 ? -360 + delta : 360 + delta);
  }

  // Since we want to accumulate the total angle, we add the adjusted delta.
  angle_ += corrected_delta;

  const SQ15x16 raw_speed = corrected_delta / (dt_ms / 1000);

  // There is too much noise in the speed measurement to use it directly. We
  // compute an exponentially weighted moving average of the speed to smooth out
  // the signal.
  speed_ = speed_ * 0.5 + raw_speed * 0.5;

  return true;
}

}  // namespace jemotor