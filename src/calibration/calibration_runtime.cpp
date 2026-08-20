#include "calibration_runtime.h"

namespace calib
{
CalibrationRuntime::CalibrationRuntime(DaisyPatchSM &patch)
: patch_(patch), store_(patch), processor_(store_.Data())
{
}

void CalibrationRuntime::Init()
{
    store_.Init();
}

bool CalibrationRuntime::IsValid()
{
    return store_.IsValid();
}

bool CalibrationRuntime::IsUserCalibration() const
{
    return store_.IsUserData();
}

const CalibrationData &CalibrationRuntime::Data()
{
    return store_.DataView();
}

daisy::PersistentStorage<CalibrationData>::State CalibrationRuntime::State() const
{
    return store_.State();
}

float CalibrationRuntime::GetKnob01(int ctrl) const
{
    const int idx = UnipolarCtrlToIndex(ctrl);
    return idx >= 0 ? processor_.CalibratedKnob01(idx, patch_.GetAdcValue(ctrl)) : 0.0f;
}

float CalibrationRuntime::GetCvNorm(int ctrl) const
{
    const int idx = BipolarCtrlToIndex(ctrl);
    return idx >= 0 ? processor_.CalibratedCvNorm(idx, patch_.GetAdcValue(ctrl)) : 0.0f;
}

float CalibrationRuntime::GetCvVolts(int ctrl) const
{
    const int idx = BipolarCtrlToIndex(ctrl);
    return idx >= 0 ? processor_.CalibratedCvVolts(idx, patch_.GetAdcValue(ctrl)) : 0.0f;
}

float CalibrationRuntime::GetPitchHz(int ctrl, float base_hz, float coarse_oct) const
{
    return processor_.PitchHzFromVolts(GetCvVolts(ctrl), base_hz, coarse_oct);
}

} // namespace calib
