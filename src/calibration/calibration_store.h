#pragma once
#include "calibration_data.h"
#include "util/PersistentStorage.h"

namespace calib
{
class CalibrationStore
{
  public:
    explicit CalibrationStore(DaisyPatchSM &patch)
    : storage_(patch.qspi)
    {
    }

    void Init();
    CalibrationData &Data();
    const CalibrationData &DataView();
    bool IsValid();
    bool IsUserData() const;
    daisy::PersistentStorage<CalibrationData>::State State() const;
    void Save();
    void RestoreDefaults();

  private:
    daisy::PersistentStorage<CalibrationData> storage_;
};
} // namespace calib
