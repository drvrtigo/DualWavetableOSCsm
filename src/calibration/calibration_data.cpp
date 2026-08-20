#include "calibration_data.h"
#include "daisysp.h"
#include <cmath>

namespace calib
{
namespace
{
bool IsFinite(float x)
{
    return std::isfinite(x);
}
} // namespace

bool CalibrationData::operator!=(const CalibrationData &other) const
{
    if(magic != other.magic || version != other.version || save_count != other.save_count)
        return true;

    for(int i = 0; i < NUM_KNOBS; ++i)
    {
        if(knobs[i].raw_min != other.knobs[i].raw_min || knobs[i].raw_max != other.knobs[i].raw_max)
            return true;
    }

    for(int i = 0; i < NUM_CVS; ++i)
    {
        for(int j = 0; j < NUM_CV_POINTS; ++j)
        {
            if(cvs[i].raw_points[j] != other.cvs[i].raw_points[j])
                return true;
        }
    }

    return false;
}

CalibrationData MakeDefaults()
{
    CalibrationData d = {};
    d.magic           = MAGIC;
    d.version         = VERSION;
    d.save_count      = 0;

    for(int i = 0; i < NUM_KNOBS; ++i)
    {
        d.knobs[i].raw_min = 0.0f;
        d.knobs[i].raw_max = 1.0f;
    }

    const float defaults[NUM_CV_POINTS] = {-1.0f, -0.6f, -0.2f, 0.0f, 0.2f, 0.6f, 1.0f};
    for(int i = 0; i < NUM_CVS; ++i)
    {
        for(int j = 0; j < NUM_CV_POINTS; ++j)
            d.cvs[i].raw_points[j] = defaults[j];
    }

    return d;
}

bool Validate(const CalibrationData &data)
{
    if(data.magic != MAGIC || data.version != VERSION)
        return false;

    for(int i = 0; i < NUM_KNOBS; ++i)
    {
        const float lo = data.knobs[i].raw_min;
        const float hi = data.knobs[i].raw_max;
        if(!IsFinite(lo) || !IsFinite(hi))
            return false;
        if(!(hi > lo))
            return false;
        if((hi - lo) < 0.05f)
            return false;
        if(lo < -0.25f || lo > 1.25f)
            return false;
        if(hi < -0.25f || hi > 1.25f)
            return false;
    }

    for(int i = 0; i < NUM_CVS; ++i)
    {
        const float *p = data.cvs[i].raw_points;
        for(int j = 0; j < NUM_CV_POINTS; ++j)
        {
            if(!IsFinite(p[j]))
                return false;
            if(p[j] < -1.25f || p[j] > 1.25f)
                return false;
        }
        for(int j = 0; j < NUM_CV_POINTS - 1; ++j)
        {
            if(!(p[j + 1] > p[j]))
                return false;
        }
        if(std::fabs(p[3]) > 0.25f)
            return false;
    }

    return true;
}

int UnipolarCtrlToIndex(int ctrl)
{
    for(int i = 0; i < NUM_KNOBS; ++i)
    {
        if(UNI_CTRLS[i] == ctrl)
            return i;
    }
    return -1;
}

int BipolarCtrlToIndex(int ctrl)
{
    for(int i = 0; i < NUM_CVS; ++i)
    {
        if(BI_CTRLS[i] == ctrl)
            return i;
    }
    return -1;
}

} // namespace calib
