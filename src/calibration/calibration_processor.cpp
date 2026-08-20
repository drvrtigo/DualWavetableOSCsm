#include "calibration_processor.h"
#include "daisysp.h"
#include <cmath>

namespace calib
{
namespace
{
float InvLerp(float a, float b, float x)
{
    if(b == a)
        return 0.0f;
    return (x - a) / (b - a);
}

float Lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}
} // namespace

float CalibrationProcessor::CalibratedKnob01(int idx, float raw01) const
{
    const float lo = data_.knobs[idx].raw_min;
    const float hi = data_.knobs[idx].raw_max;
    return daisysp::fclamp((raw01 - lo) / (hi - lo), 0.0f, 1.0f);
}

float CalibrationProcessor::CalibratedCvVolts(int idx, float rawBi) const
{
    const float *p = data_.cvs[idx].raw_points;

    if(rawBi <= p[0])
        return CV_POINT_VOLTS[0];
    if(rawBi >= p[NUM_CV_POINTS - 1])
        return CV_POINT_VOLTS[NUM_CV_POINTS - 1];

    for(int i = 0; i < NUM_CV_POINTS - 1; ++i)
    {
        if(rawBi >= p[i] && rawBi <= p[i + 1])
        {
            const float t = InvLerp(p[i], p[i + 1], rawBi);
            return Lerp(CV_POINT_VOLTS[i], CV_POINT_VOLTS[i + 1], t);
        }
    }

    return 0.0f;
}

float CalibrationProcessor::CalibratedCvNorm(int idx, float rawBi) const
{
    const float center = data_.cvs[idx].raw_points[3];
    if(rawBi == center)
        return 0.0f;

    const float volts = CalibratedCvVolts(idx, rawBi);
    return daisysp::fclamp(volts / 5.0f, -1.0f, 1.0f);
}

float CalibrationProcessor::PitchHzFromVolts(float volts, float base_hz, float coarse_oct) const
{
    const float total_v = daisysp::fclamp(volts + coarse_oct, -5.0f, 5.0f);
    return daisysp::fclamp(base_hz * std::pow(2.0f, total_v), 10.0f, 20000.0f);
}

} // namespace calib
