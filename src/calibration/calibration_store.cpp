#include "calibration_store.h"

namespace calib
{
void CalibrationStore::Init()
{
    storage_.Init(MakeDefaults());
    if(!Validate(storage_.GetSettings()))
    {
        storage_.RestoreDefaults();
        storage_.GetSettings() = MakeDefaults();
    }
}

CalibrationData &CalibrationStore::Data()
{
    return storage_.GetSettings();
}

const CalibrationData &CalibrationStore::DataView()
{
    return storage_.GetSettings();
}

bool CalibrationStore::IsValid()
{
    return Validate(storage_.GetSettings());
}

bool CalibrationStore::IsUserData() const
{
    return State() == daisy::PersistentStorage<CalibrationData>::State::USER;
}

daisy::PersistentStorage<CalibrationData>::State CalibrationStore::State() const
{
    return const_cast<daisy::PersistentStorage<CalibrationData> &>(storage_).GetState();
}

void CalibrationStore::Save()
{
    storage_.GetSettings().magic = MAGIC;
    storage_.GetSettings().version = VERSION;
    storage_.GetSettings().save_count++;
    storage_.Save();
}

void CalibrationStore::RestoreDefaults()
{
    storage_.RestoreDefaults();
    storage_.GetSettings() = MakeDefaults();
}

} // namespace calib
