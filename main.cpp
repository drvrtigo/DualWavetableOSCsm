#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "src/calibration/calibration_runtime.h"
#include "wavetables/wavetable_data_a.h"
#include <cmath>

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;
using namespace calib;

DaisyPatchSM patch;
Switch btn_shift;
Switch toggle_mode;
CalibrationRuntime cal(patch);

static const float BASE_FREQ = 65.41f;
static constexpr float LED_OFF = 0.0f;
static constexpr float LED_SINGLE_BASE = 2.3f;
static constexpr float LED_INTERVAL_BASE = 4.6f;
static constexpr float LED_PREVIEW_SCALE = 0.30f;
static constexpr float PICKUP_THRESHOLD = 0.04f;
static constexpr float PREVIEW_RANGE = 0.12f;
static constexpr float SHIFT_LED_ALPHA = 0.20f;
static constexpr uint32_t SHIFT_SUPPRESS_MS = 100;

float DSY_SDRAM_BSS wavetable[kNumFrames_a][kFrameSize_a];

struct ChanControl
{
    int   main_octave_offset = 0;
    float shape_knob         = 0.0f;
    float shape_cv           = 0.0f;
    float sub_level          = 0.0f;
    float interval_select    = 0.5f;
};

enum class ShiftParamTarget
{
    NONE = 0,
    OCT_CH1,
    OCT_CH2,
    INTERVAL_CH1,
    INTERVAL_CH2
};

enum class PickupBlinkType
{
    NONE = 0,
    OCTAVE,
    INTERVAL
};

struct SoftTakeover
{
    bool  armed      = false;
    bool  picked_up  = false;
    float target     = 0.0f;
    float threshold  = PICKUP_THRESHOLD;
    float knob_raw   = 0.0f;

    void Arm(float current_param)
    {
        armed = true;
        picked_up = false;
        target = current_param;
    }

    bool Update(float knob_value)
    {
        knob_raw = knob_value;
        if(!armed)
            return true;

        if(fabsf(knob_value - target) <= threshold)
        {
            armed = false;
            picked_up = true;
            return true;
        }

        return false;
    }
};

struct ParamSmoother
{
    float a_ = 0.0f;
    float b_ = 1.0f;
    float z_ = 0.0f;

    void Init(float sample_rate, float time_ms, float initial)
    {
        float t_sec = fmaxf(0.0001f, time_ms * 0.001f);
        a_ = expf(-TWOPI_F / (t_sec * sample_rate));
        b_ = 1.0f - a_;
        z_ = initial;
    }

    inline float Process(float in)
    {
        z_ = in * b_ + z_ * a_;
        return z_;
    }
};

struct WavetableBankView
{
    const float *table      = nullptr;
    size_t       num_frames = 0;
    size_t       frame_size = 0;
};

struct WavetableOscillator
{
    float sr_        = 48000.0f;
    float sr_recip_  = 1.0f / 48000.0f;
    float freq_      = 100.0f;
    float phase_     = 0.0f;
    float phase_inc_ = 100.0f / 48000.0f;
    WavetableBankView bank_ = {};

    void Init(float sample_rate, const WavetableBankView &bank)
    {
        sr_ = sample_rate;
        sr_recip_ = 1.0f / sample_rate;
        bank_ = bank;
        phase_ = 0.0f;
        SetFreq(100.0f);
    }

    void SetFreq(float f)
    {
        freq_ = fclamp(f, 0.0f, sr_ * 0.45f);
        phase_inc_ = freq_ * sr_recip_;
    }

    float Process(float morph)
    {
        morph = fclamp(morph, 0.0f, 1.0f);

        const float frame_pos  = morph * float(bank_.num_frames - 1);
        const float sample_pos = phase_ * float(bank_.frame_size);

        const int   i0     = static_cast<int>(sample_pos);
        const int   i1     = (i0 + 1) % bank_.frame_size;
        const float frac_x = sample_pos - float(i0);

        const int   f0     = static_cast<int>(frame_pos);
        const int   f1     = (f0 + 1 < static_cast<int>(bank_.num_frames)) ? f0 + 1 : f0;
        const float frac_f = frame_pos - float(f0);

        const float *frameA = bank_.table + (f0 * bank_.frame_size);
        const float *frameB = bank_.table + (f1 * bank_.frame_size);

        const float a = frameA[i0] + (frameA[i1] - frameA[i0]) * frac_x;
        const float b = frameB[i0] + (frameB[i1] - frameB[i0]) * frac_x;
        const float out = a + (b - a) * frac_f;

        phase_ += phase_inc_;
        if(phase_ >= 1.0f)
            phase_ -= 1.0f;

        return out;
    }
};

struct PickupBlinkState
{
    bool active = false;
    PickupBlinkType type = PickupBlinkType::NONE;
    int total_toggles = 0;
    int toggles_done  = 0;
    bool level_high   = false;
    uint32_t last_ms  = 0;
};

ChanControl         chan_ctrl_[2];
WavetableOscillator osc_main[2][3];
Oscillator          osc_sub[2];
ParamSmoother       shape_smoother[2];
SoftTakeover        shape_takeover[2];
SoftTakeover        sub_takeover[2];
SoftTakeover        octave_takeover[2];
SoftTakeover        interval_takeover[2];
WavetableBankView   bank_view_;

MoogLadder filter_[2];
Overdrive  drive_[2];
Jitter     drift_[2][3];

volatile bool  shift_pressed_state_      = false;
volatile bool  shift_rising_pending_     = false;
volatile bool  shift_falling_pending_    = false;
volatile float shift_timeheld_ms_state_  = 0.0f;
volatile bool  toggle_pressed_state_     = false;
volatile float viz_oct_value_[2]         = {0.5f, 0.5f};
volatile float viz_interval_value_[2]    = {0.5f, 0.5f};
volatile float viz_oct_knob_[2]          = {0.0f, 0.0f};
volatile float viz_interval_knob_[2]     = {0.0f, 0.0f};
volatile bool  pickup_oct_event_[2]      = {false, false};
volatile bool  pickup_interval_event_[2] = {false, false};

static bool             ui_shift_active_      = false;
static bool             shift_press_active_   = false;
static bool             shift_hold_latched_   = false;
static bool             last_shift_mode_main_ = false;
static uint32_t         shift_entered_ms_     = 0;
static ShiftParamTarget shift_display_target_ = ShiftParamTarget::NONE;
static PickupBlinkState pickup_blink_;
static float            shift_led_smooth_     = 0.0f;
static float            shift_led_phase_      = 0.0f;
static float            prev_oct_edit_[2]     = {0.5f, 0.5f};
static float            prev_int_edit_[2]     = {0.5f, 0.5f};
static constexpr float  kLongHoldMs           = 400.0f;
static constexpr float  kMoveThresh           = 0.020f;

inline float Saturate(float x) { return fclamp(x, 0.0f, 1.0f); }

inline float ComputePickupPreview(float knob, float stored, float range)
{
    float dist = fabsf(knob - stored);
    if(dist >= range)
        return 0.0f;
    float closeness = 1.0f - (dist / range);
    return closeness * closeness * (3.0f - 2.0f * closeness);
}

inline int QuantizeMainOctave(float norm)
{
    int idx = static_cast<int>(norm * 5.0f);
    if(idx > 4)
        idx = 4;
    return idx - 2;
}

inline float OctaveOffsetToNorm(int octave)
{
    return float(octave + 2) / 4.0f;
}

inline int QuantizeIntervalIndex(float norm)
{
    int idx = static_cast<int>(norm * 5.0f);
    if(idx > 4)
        idx = 4;
    return idx;
}

inline float IntervalIndexToNorm(int idx)
{
    static const float norm_table[5] = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
    if(idx < 0)
        idx = 0;
    if(idx > 4)
        idx = 4;
    return norm_table[idx];
}

inline float QuantizeIntervalSemitones(float norm)
{
    static const float interval_table[5] = {-12.0f, -7.0f, 0.0f, 7.0f, 12.0f};
    int idx = QuantizeIntervalIndex(norm);
    return interval_table[idx];
}

static inline void InitSubOsc(Oscillator &osc, float sr)
{
    osc.Init(sr);
    osc.SetWaveform(Oscillator::WAVE_SQUARE);
    osc.SetAmp(1.0f);
}

template <size_t NF, size_t FS>
static void CopyWavetableToSdram(float (&dst)[NF][FS], const float *src)
{
    for(size_t f = 0; f < NF; ++f)
        for(size_t i = 0; i < FS; ++i)
            dst[f][i] = src[f * FS + i];
}

static void StartPickupBlink(PickupBlinkType type)
{
    pickup_blink_.active = true;
    pickup_blink_.type = type;
    pickup_blink_.toggles_done = 0;
    pickup_blink_.level_high = true;
    pickup_blink_.last_ms = System::GetNow();
    pickup_blink_.total_toggles = (type == PickupBlinkType::INTERVAL) ? 4 : 2;
}

static void WriteLedVolts(float volts)
{
    patch.WriteCvOut(CV_OUT_2, fclamp(volts, 0.0f, 5.0f));
}

static void UpdateLedUiMain(bool interval_mode)
{
    float base = interval_mode ? LED_INTERVAL_BASE : LED_SINGLE_BASE;
    WriteLedVolts(base);
}

static void UpdateLedUiShift()
{
    uint32_t now = System::GetNow();

    float o1 = viz_oct_value_[0];
    float o2 = viz_oct_value_[1];
    float i1 = viz_interval_value_[0];
    float i2 = viz_interval_value_[1];
    float ok1 = viz_oct_knob_[0];
    float ok2 = viz_oct_knob_[1];
    float ik1 = viz_interval_knob_[0];
    float ik2 = viz_interval_knob_[1];

    if(!last_shift_mode_main_)
    {
        shift_entered_ms_ = now;
        prev_oct_edit_[0] = o1;
        prev_oct_edit_[1] = o2;
        prev_int_edit_[0] = i1;
        prev_int_edit_[1] = i2;
        shift_display_target_ = ShiftParamTarget::NONE;
    }
    last_shift_mode_main_ = true;

    if((now - shift_entered_ms_) < SHIFT_SUPPRESS_MS)
    {
        prev_oct_edit_[0] = o1;
        prev_oct_edit_[1] = o2;
        prev_int_edit_[0] = i1;
        prev_int_edit_[1] = i2;
        WriteLedVolts(0.0f);
        return;
    }

    bool oct1_changed = fabsf(o1 - prev_oct_edit_[0]) > kMoveThresh;
    bool oct2_changed = fabsf(o2 - prev_oct_edit_[1]) > kMoveThresh;
    bool int1_changed = fabsf(i1 - prev_int_edit_[0]) > kMoveThresh;
    bool int2_changed = fabsf(i2 - prev_int_edit_[1]) > kMoveThresh;

    if(oct1_changed)
    {
        shift_display_target_ = ShiftParamTarget::OCT_CH1;
        prev_oct_edit_[0] = o1;
    }
    if(oct2_changed)
    {
        shift_display_target_ = ShiftParamTarget::OCT_CH2;
        prev_oct_edit_[1] = o2;
    }
    if(int1_changed)
    {
        shift_display_target_ = ShiftParamTarget::INTERVAL_CH1;
        prev_int_edit_[0] = i1;
    }
    if(int2_changed)
    {
        shift_display_target_ = ShiftParamTarget::INTERVAL_CH2;
        prev_int_edit_[1] = i2;
    }

    float oct_prev1 = octave_takeover[0].armed ? ComputePickupPreview(ok1, o1, PREVIEW_RANGE) : 0.0f;
    float oct_prev2 = octave_takeover[1].armed ? ComputePickupPreview(ok2, o2, PREVIEW_RANGE) : 0.0f;
    float int_prev1 = interval_takeover[0].armed ? ComputePickupPreview(ik1, i1, PREVIEW_RANGE) : 0.0f;
    float int_prev2 = interval_takeover[1].armed ? ComputePickupPreview(ik2, i2, PREVIEW_RANGE) : 0.0f;

    if(shift_display_target_ == ShiftParamTarget::NONE)
    {
        float best = 0.0f;
        if(oct_prev1 > best)
        {
            best = oct_prev1;
            shift_display_target_ = ShiftParamTarget::OCT_CH1;
        }
        if(oct_prev2 > best)
        {
            best = oct_prev2;
            shift_display_target_ = ShiftParamTarget::OCT_CH2;
        }
        if(int_prev1 > best)
        {
            best = int_prev1;
            shift_display_target_ = ShiftParamTarget::INTERVAL_CH1;
        }
        if(int_prev2 > best)
        {
            best = int_prev2;
            shift_display_target_ = ShiftParamTarget::INTERVAL_CH2;
        }
    }

    float int1_step = IntervalIndexToNorm(QuantizeIntervalIndex(i1));
    float int2_step = IntervalIndexToNorm(QuantizeIntervalIndex(i2));

    float param_led = 0.0f;
    bool pulse_mode = false;

    switch(shift_display_target_)
    {
        case ShiftParamTarget::OCT_CH1:
            param_led = octave_takeover[0].armed ? (o1 * LED_PREVIEW_SCALE + oct_prev1 * 0.35f) : o1;
            break;

        case ShiftParamTarget::OCT_CH2:
            param_led = octave_takeover[1].armed ? (o2 * LED_PREVIEW_SCALE + oct_prev2 * 0.35f) : o2;
            break;

        case ShiftParamTarget::INTERVAL_CH1:
            param_led = interval_takeover[0].armed
                            ? (int1_step * LED_PREVIEW_SCALE + int_prev1 * 0.35f)
                            : int1_step;
            pulse_mode = true;
            break;

        case ShiftParamTarget::INTERVAL_CH2:
            param_led = interval_takeover[1].armed
                            ? (int2_step * LED_PREVIEW_SCALE + int_prev2 * 0.35f)
                            : int2_step;
            pulse_mode = true;
            break;

        case ShiftParamTarget::NONE:
        default:
        {
            float near_any = fmaxf(fmaxf(oct_prev1, oct_prev2), fmaxf(int_prev1, int_prev2));
            param_led = near_any * 0.20f;
        }
        break;
    }

    param_led = Saturate(param_led);

    shift_led_phase_ += pulse_mode ? 0.035f : 0.020f;
    if(shift_led_phase_ > 6.2831853f)
        shift_led_phase_ -= 6.2831853f;

    float pulse = pulse_mode ? (0.86f + 0.14f * (0.5f + 0.5f * sinf(shift_led_phase_))) : 1.0f;
    float target_led = Saturate(param_led * pulse);

    if(pickup_blink_.active)
    {
        uint32_t on_ms = (pickup_blink_.type == PickupBlinkType::INTERVAL) ? 45 : 65;
        uint32_t off_ms = (pickup_blink_.type == PickupBlinkType::INTERVAL) ? 45 : 55;
        uint32_t interval = pickup_blink_.level_high ? on_ms : off_ms;

        if(now - pickup_blink_.last_ms >= interval)
        {
            pickup_blink_.level_high = !pickup_blink_.level_high;
            pickup_blink_.last_ms = now;
            pickup_blink_.toggles_done++;
            if(pickup_blink_.toggles_done >= pickup_blink_.total_toggles)
            {
                pickup_blink_.active = false;
                pickup_blink_.level_high = false;
            }
        }

        WriteLedVolts((pickup_blink_.level_high ? 1.0f : 0.0f) * 5.0f);
        return;
    }

    shift_led_smooth_ += SHIFT_LED_ALPHA * (target_led - shift_led_smooth_);
    shift_led_smooth_ = Saturate(shift_led_smooth_);
    WriteLedVolts(shift_led_smooth_ * 5.0f);
}

static void AudioCallback(AudioHandle::InputBuffer in,
                          AudioHandle::OutputBuffer out,
                          size_t size)
{
    patch.ProcessAllControls();
    btn_shift.Debounce();
    toggle_mode.Debounce();

    shift_pressed_state_ = btn_shift.Pressed();
    shift_timeheld_ms_state_ = btn_shift.TimeHeldMs();
    toggle_pressed_state_ = toggle_mode.Pressed();

    if(btn_shift.RisingEdge())
        shift_rising_pending_ = true;
    if(btn_shift.FallingEdge())
        shift_falling_pending_ = true;

    bool interval_mode = toggle_pressed_state_;

    for(size_t i = 0; i < size; i++)
    {
        for(int c = 0; c < 2; c++)
        {
            const int cv_voct = (c == 0) ? CV_5 : CV_7;

            float base_from_voct = cal.GetPitchHz(cv_voct, BASE_FREQ, 0.0f);
            float freq_hz = base_from_voct * powf(2.0f, float(chan_ctrl_[c].main_octave_offset));
            float morph = shape_smoother[c].Process(
                Saturate(chan_ctrl_[c].shape_knob + chan_ctrl_[c].shape_cv));
            float sub_freq_hz = freq_hz * 0.5f;
            float main = 0.0f;

            float drift0 = drift_[c][0].Process();
            float drift1 = drift_[c][1].Process();

            osc_main[c][0].SetFreq(freq_hz * (1.0f + drift0));
            float center = osc_main[c][0].Process(morph);

            if(!interval_mode)
            {
                main = center * 0.42f;
            }
            else
            {
                float interval_semitones = QuantizeIntervalSemitones(chan_ctrl_[c].interval_select);
                float interval_ratio = powf(2.0f, interval_semitones / 12.0f);
                float second_freq_hz = freq_hz * interval_ratio;

                osc_main[c][1].SetFreq(second_freq_hz * (1.0f + drift1));
                float second = osc_main[c][1].Process(morph);

                main = center * 0.40f + second * 0.30f;
            }

            osc_sub[c].SetFreq(sub_freq_hz);
            float sub = osc_sub[c].Process() * chan_ctrl_[c].sub_level * 0.22f;

            float pre_filter = main + sub;

            float cutoff = fclamp(freq_hz * 8.0f + 1800.0f, 180.0f, 14000.0f);
            filter_[c].SetFreq(cutoff);
            filter_[c].SetRes(0.12f);

            float filtered = filter_[c].Process(pre_filter);
            float sig = filtered * 0.78f + pre_filter * 0.22f;

            sig *= 2.50f;
            sig = drive_[c].Process(sig * 1.03f);
            sig = fclamp(sig, -0.98f, 0.98f);

            out[c][i] = sig;
        }
    }
}

int main(void)
{
    patch.Init();
    cal.Init();

    btn_shift.Init(patch.B7, patch.AudioCallbackRate());
    toggle_mode.Init(patch.B8, patch.AudioCallbackRate());

    CopyWavetableToSdram(wavetable, wavetable_data_a);
    bank_view_ = {&wavetable[0][0], kNumFrames_a, kFrameSize_a};

    const float sr = patch.AudioSampleRate();

    for(int c = 0; c < 2; c++)
    {
        for(int v = 0; v < 3; v++)
            osc_main[c][v].Init(sr, bank_view_);

        InitSubOsc(osc_sub[c], sr);
        shape_smoother[c].Init(sr, 2.5f, 0.0f);

        filter_[c].Init(sr);
        filter_[c].SetFreq(3200.0f);
        filter_[c].SetRes(0.12f);

        drive_[c].Init();
        drive_[c].SetDrive(0.08f);

        for(int v = 0; v < 3; v++)
        {
            drift_[c][v].Init(sr);
            drift_[c][v].SetAmp(0.0015f);
            drift_[c][v].SetCpsMin(0.03f);
            drift_[c][v].SetCpsMax(0.12f);
        }

        chan_ctrl_[c].main_octave_offset = 0;
        chan_ctrl_[c].shape_knob = 0.0f;
        chan_ctrl_[c].shape_cv = 0.0f;
        chan_ctrl_[c].sub_level = 0.0f;
        chan_ctrl_[c].interval_select = 0.5f;

        viz_oct_value_[c] = OctaveOffsetToNorm(chan_ctrl_[c].main_octave_offset);
        viz_interval_value_[c] = IntervalIndexToNorm(
            QuantizeIntervalIndex(chan_ctrl_[c].interval_select));
        prev_oct_edit_[c] = viz_oct_value_[c];
        prev_int_edit_[c] = viz_interval_value_[c];
    }

    patch.SetAudioBlockSize(8);
    patch.StartAudio(AudioCallback);

    while(1)
    {
        bool shift_pressed = shift_pressed_state_;
        bool shift_rising = shift_rising_pending_;
        bool shift_falling = shift_falling_pending_;
        float shift_held_ms = shift_timeheld_ms_state_;

        bool got_pickup_oct = pickup_oct_event_[0] || pickup_oct_event_[1];
        bool got_pickup_interval = pickup_interval_event_[0] || pickup_interval_event_[1];

        pickup_oct_event_[0] = false;
        pickup_oct_event_[1] = false;
        pickup_interval_event_[0] = false;
        pickup_interval_event_[1] = false;

        if(shift_rising_pending_)
            shift_rising_pending_ = false;
        if(shift_falling_pending_)
            shift_falling_pending_ = false;

        if(shift_rising)
        {
            shift_press_active_ = true;
            shift_hold_latched_ = false;
        }

        if(shift_press_active_ && shift_pressed && !shift_hold_latched_
           && shift_held_ms >= kLongHoldMs)
        {
            shift_hold_latched_ = true;
            ui_shift_active_ = true;
            shift_entered_ms_ = System::GetNow();

            for(int c = 0; c < 2; c++)
            {
                octave_takeover[c].Arm(OctaveOffsetToNorm(chan_ctrl_[c].main_octave_offset));
                interval_takeover[c].Arm(IntervalIndexToNorm(
                    QuantizeIntervalIndex(chan_ctrl_[c].interval_select)));
            }

            shift_display_target_ = ShiftParamTarget::NONE;
            shift_led_smooth_ = 0.0f;
        }

        if(shift_falling)
        {
            shift_press_active_ = false;
            shift_hold_latched_ = false;
            ui_shift_active_ = false;
            shift_display_target_ = ShiftParamTarget::NONE;
            pickup_blink_.active = false;

            for(int c = 0; c < 2; c++)
            {
                shape_takeover[c].Arm(chan_ctrl_[c].shape_knob);
                sub_takeover[c].Arm(chan_ctrl_[c].sub_level);
            }
        }

        if(got_pickup_interval)
            StartPickupBlink(PickupBlinkType::INTERVAL);
        else if(got_pickup_oct)
            StartPickupBlink(PickupBlinkType::OCTAVE);

        for(int c = 0; c < 2; c++)
        {
            const int knob_shape = (c == 0) ? CV_1 : CV_2;
            const int knob_level = (c == 0) ? CV_3 : CV_4;
            const int cv_shape   = (c == 0) ? CV_6 : CV_8;

            float knob_shape_val = cal.GetKnob01(knob_shape);
            float knob_level_val = cal.GetKnob01(knob_level);

            if(!ui_shift_active_)
            {
                bool shape_ok = shape_takeover[c].Update(knob_shape_val);
                bool sub_ok   = sub_takeover[c].Update(knob_level_val);

                if(shape_ok)
                    chan_ctrl_[c].shape_knob = knob_shape_val;
                if(sub_ok)
                    chan_ctrl_[c].sub_level = knob_level_val;
            }
            else
            {
                viz_oct_knob_[c] = knob_shape_val;
                viz_interval_knob_[c] = knob_level_val;

                bool oct_ok = octave_takeover[c].Update(knob_shape_val);
                bool int_ok = interval_takeover[c].Update(knob_level_val);

                if(octave_takeover[c].picked_up)
                {
                    pickup_oct_event_[c] = true;
                    octave_takeover[c].picked_up = false;
                }
                if(interval_takeover[c].picked_up)
                {
                    pickup_interval_event_[c] = true;
                    interval_takeover[c].picked_up = false;
                }

                if(oct_ok)
                    chan_ctrl_[c].main_octave_offset = QuantizeMainOctave(knob_shape_val);
                if(int_ok)
                    chan_ctrl_[c].interval_select = knob_level_val;
            }

            chan_ctrl_[c].shape_cv = cal.GetCvNorm(cv_shape);
            viz_oct_value_[c] = OctaveOffsetToNorm(chan_ctrl_[c].main_octave_offset);
            viz_interval_value_[c]
                = IntervalIndexToNorm(QuantizeIntervalIndex(chan_ctrl_[c].interval_select));
        }

        if(ui_shift_active_)
            UpdateLedUiShift();
        else
        {
            last_shift_mode_main_ = false;
            shift_display_target_ = ShiftParamTarget::NONE;
            shift_led_smooth_ = 0.0f;
            UpdateLedUiMain(toggle_pressed_state_);
        }

        System::Delay(4);
    }
}