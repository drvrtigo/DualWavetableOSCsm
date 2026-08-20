#pragma once
#include "calibration_processor.h"
#include "calibration_store.h"

namespace calib
{
class CalibrationRuntime
{
  public:
    explicit CalibrationRuntime(DaisyPatchSM &patch);

    void Init();
    bool IsValid();
    bool IsUserCalibration() const;
    const CalibrationData &Data();
    daisy::PersistentStorage<CalibrationData>::State State() const;

    float GetKnob01(int ctrl) const;
    float GetCvNorm(int ctrl) const;
    float GetCvVolts(int ctrl) const;
    float GetPitchHz(int ctrl, float base_hz, float coarse_oct) const;

  private:
    DaisyPatchSM &         patch_;
    CalibrationStore      store_;
    CalibrationProcessor  processor_;
};
} // namespace calib
