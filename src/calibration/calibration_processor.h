#pragma once
#include "calibration_data.h"

namespace calib
{
class CalibrationProcessor
{
  public:
    explicit CalibrationProcessor(const CalibrationData &data)
    : data_(data)
    {
    }

    float CalibratedKnob01(int idx, float raw01) const;
    float CalibratedCvVolts(int idx, float rawBi) const;
    float CalibratedCvNorm(int idx, float rawBi) const;
    float PitchHzFromVolts(float volts, float base_hz, float coarse_oct) const;

  private:
    const CalibrationData &data_;
};
} // namespace calib
