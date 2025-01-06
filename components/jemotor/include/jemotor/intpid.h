#ifndef INTPID_INTPID_H
#define INTPID_INTPID_H

#include <FixedPointsCommon.h>

#include <cassert>
#include <cstdint>
#include <expected>
#include <limits>
#include <memory>
#include <string>

namespace intpid {

// Since this pid controller uses fixed point under the covers, it is important
// to keep the output within the range of the underlying type. In this case,
// that means your output should be centered somewhere near zero, and have an
// absolute value of 2^14-1 (which is 16383). On the other hand, be careful to
// stay away from outputs so small they require more than 13-14 bits of
// precision.
struct Config {
  // Each update, we multiply the error by this number and add it to the output
  // signal.
  float kp;

  // Each update, we multiply the accumulated error * time by this number and
  // add it to the output signal. The error is accumulated only when the output
  // is not saturated by the P and D terms.
  //
  // Hint: if this number is equal to kp, then if the error is constant for one
  // tick, the I term will be equal to the P term. If it's 1/10 of kp, then
  // it will take ten ticks before I = P.
  float ki;

  // Each update, we multiply the rate of change in the error by this number
  // and add it to the output signal.
  //
  // Hint: if this number is equal to kp, then if the error is changing at rate
  // X/tick, and the current error is also X, then the D term will equal the P
  // term. If this is 1/10th of kp, then the change in error has to be 10X/tick
  // before D = P.
  float kd;

  // The minimum and maximum outputs.
  float output_min;
  float output_max;
};

class Pid {
 public:
  Pid(Pid&&) = default;
  Pid& operator=(Pid&&) = default;

  static std::expected<Pid, std::string> Create(const Config& config);

  // Adjusts the setpoint. This must be called once before the first call
  // to Update.
  void set_setpoint(SQ15x16 setpoint) { setpoint_ = setpoint; }

  // Updates the PID controller with feedback and returns the new output value.
  // dt is unitless -- it just needs to be consistent with the unit for
  // integral_time and derivative_time.
  SQ15x16 Update(SQ15x16 measurement, SQ15x16 dt);

 private:
  Pid(const Config& config)
      : kp_(config.kp),
        ki_(config.ki),
        kd_(config.kd),
        output_min_(config.output_min),
        output_max_(config.output_max),
        integrator_lower_cutoff_(output_min_ -
                                 .25 * (output_max_ - output_min_)),
        integrator_upper_cutoff_(output_max_ +
                                 .25 * (output_max_ - output_min_)) {}

  static SQ15x16 MaxI(SQ15x16 output_min, SQ15x16 output_max) {
    const auto range = output_max - output_min;
    const auto absmin = absFixed(output_min);
    const auto absmax = absFixed(output_max);
    // We only have an offset from zero if our output does not include zero in
    // its range.
    const bool has_offset = (output_min > 0) == (output_max != 0);
    const auto offset = absmin > absmax ? absmin : absmax;
    return 2 * (range + (has_offset ? offset : 0));
  }

  const SQ15x16 kp_, ki_, kd_;
  const SQ15x16 integrator_clamp_;
  const SQ15x16 output_min_, output_max_;

  SQ15x16 setpoint_ = 0;

  // Note: unlike other PID controller implementations I've seen, we multiply
  // the error * time by ki before adding it to i_sum_. This means that it's
  // easy to clamp the integrator to the desired range without the risk of
  // overflow (that range being 2x the output range).
  SQ15x16 i_sum_ = 0;

  // If the raw output is outside of this range, the integrator term is reduced
  // until the raw output would be at the edge of the range.
  const SQ15x16 integrator_lower_cutoff_;
  const SQ15x16 integrator_upper_cutoff_;

  SQ15x16 prev_measurement_ = 0;
  SQ15x16 derr_ = 0;

#if INTPID_SUPPRESS_LOGGING == 0
 public:
  // If logging is enabled, we record the values we saw for the
  // setpoint, measurement, and the calculated P I and D terms.
  // We also expose the state of the integrator and derr.
  // These values are for testing only.
  SQ15x16 setpoint() const { return setpoint_; }
  SQ15x16 measurement() const { return measurement_; }
  SQ15x16 p() const { return p_; }
  SQ15x16 i() const { return i_; }
  SQ15x16 d() const { return d_; }
  SQ15x16 derr() const { return derr_; }
  SQ15x16 sum() const { return sum_; }

 private:
  SQ15x16 measurement_ = 0;
  SQ15x16 p_ = 0;
  SQ15x16 i_ = 0;
  SQ15x16 d_ = 0;
  SQ15x16 sum_ = 0;
#endif
};

}  // namespace intpid

#endif  // INTPID_INTPID_H