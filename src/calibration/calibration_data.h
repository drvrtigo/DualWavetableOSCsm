#pragma once
#include "daisy_patch_sm.h"
#include <stdint.h>

namespace calib
{
using daisy::patch_sm::DaisyPatchSM;

constexpr uint32_t MAGIC   = 0x43414C32;
constexpr uint32_t VERSION = 2;

constexpr int NUM_KNOBS     = 4;
constexpr int NUM_CVS       = 4;
constexpr int NUM_CV_POINTS = 7;

constexpr int UNI_CTRLS[NUM_KNOBS] = {
    daisy::patch_sm::CV_1,
    daisy::patch_sm::CV_2,
    daisy::patch_sm::CV_3,
    daisy::patch_sm::CV_4,
};

constexpr int BI_CTRLS[NUM_CVS] = {
    daisy::patch_sm::CV_5,
    daisy::patch_sm::CV_6,
    daisy::patch_sm::CV_7,
    daisy::patch_sm::CV_8,
};

constexpr float CV_POINT_VOLTS[NUM_CV_POINTS] = {-5.0f, -3.0f, -1.0f, 0.0f, 1.0f, 3.0f, 5.0f};

struct KnobCalibration
{
    float raw_min;
    float raw_max;
};

struct CvCalibration
{
    float raw_points[NUM_CV_POINTS];
};

struct CalibrationData
{
    uint32_t        magic;
    uint32_t        version;
    uint32_t        save_count;
    KnobCalibration knobs[NUM_KNOBS];
    CvCalibration   cvs[NUM_CVS];

    bool operator!=(const CalibrationData &other) const;
};

CalibrationData MakeDefaults();
bool            Validate(const CalibrationData &data);
int             UnipolarCtrlToIndex(int ctrl);
int             BipolarCtrlToIndex(int ctrl);

} // namespace calib
