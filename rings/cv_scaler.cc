// Copyright 2015 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
// Upravený kod pro NUSrecords (AreaGreenLand music label)

// Copyright 2015 Emilie Gillet.
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
// Upravený kod pro NUSrecords (AreaGreenLand music label)

#include "rings/cv_scaler.h"

#include <algorithm>

#include "stmlib/dsp/dsp.h"
#include "stmlib/system/storage.h"
#include "stmlib/utils/random.h"

#include "rings/dsp/part.h"
#include "rings/dsp/patch.h"

namespace rings {
  
using namespace std;
using namespace stmlib;

/* static */
ChannelSettings CvScaler::channel_settings_[ADC_CHANNEL_LAST] = {
  { LAW_LINEAR, true, 0.01f },  // ADC_CHANNEL_CV_FREQUENCY (Bypass: váš nový pot Structure)
  { LAW_LINEAR, true, 0.1f },   // ADC_CHANNEL_CV_STRUCTURE
  { LAW_LINEAR, true, 0.1f },   // ADC_CHANNEL_CV_BRIGHTNESS
  { LAW_LINEAR, true, 0.05f },  // ADC_CHANNEL_CV_DAMPING
  { LAW_LINEAR, true, 0.01f },  // ADC_CHANNEL_CV_POSITION
  { LAW_LINEAR, false, 1.00f }, // ADC_CHANNEL_CV_V_OCT
  { LAW_LINEAR, false, 0.01f }, // ADC_CHANNEL_POT_FREQUENCY
  { LAW_LINEAR, false, 0.01f }, // ADC_CHANNEL_POT_STRUCTURE
  { LAW_LINEAR, false, 0.01f }, // ADC_CHANNEL_POT_BRIGHTNESS
  { LAW_LINEAR, false, 0.01f }, // ADC_CHANNEL_POT_DAMPING
  { LAW_LINEAR, false, 0.01f }, // ADC_CHANNEL_POT_POSITION
  { LAW_QUARTIC_BIPOLAR, false, 0.005f },  // ADC_CHANNEL_ATTENUVERTER_FREQUENCY
  { LAW_QUADRATIC_BIPOLAR, false, 0.005f }, // ADC_CHANNEL_ATTENUVERTER_STRUCTURE
  { LAW_QUADRATIC_BIPOLAR, false, 0.005f }, // ADC_CHANNEL_ATTENUVERTER_BRIGHTNESS
  { LAW_QUADRATIC_BIPOLAR, false, 0.005f }, // ADC_CHANNEL_ATTENUVERTER_DAMPING
  { LAW_QUADRATIC_BIPOLAR, false, 0.005f }, // ADC_CHANNEL_ATTENUVERTER_POSITION
};

void CvScaler::Init(CalibrationData* calibration_data) {
  calibration_data_ = calibration_data;

  adc_.Init();
  trigger_input_.Init();

  transpose_ = 0.0f;
  
  // OPRAVA INDEXU FILTRU: Přidány správné ukazatele na pole
  fill(&adc_lp_[0], &adc_lp_[ADC_CHANNEL_LAST], 0.0f);
  
  normalization_probe_.Init();
  normalization_detector_exciter_.Init(0.01f, 0.5f);
  normalization_detector_trigger_.Init(0.05f, 0.9f);
  normalization_detector_v_oct_.Init(0.01f, 0.5f);
  
  inhibit_strum_ = 0;
  fm_cv_ = 0.0f;
  
  normalization_probe_enabled_ = true;
  normalization_probe_forced_state_ = false;
}

void CvScaler::DetectAudioNormalization(Codec::Frame* in, size_t size) {
  int32_t count = 0;
  short* input_samples = &in->r;
  for (size_t i = 0; i < size; i += 8) {
    short s = input_samples[i * 2];
    if (s > 50 && s < 1500) {
      ++count;
    } else if (s > -1500 && s < -50) {
      --count;
    }
  }
  float y = static_cast<float>(count) / static_cast<float>(size >> 3);
  // OPRAVA: Vrácen index [1] pro správnou detekci normalizace audia
  float x = normalization_probe_value_[1] ? -1.0f : 1.0f;
  
  normalization_detector_exciter_.Process(x, y);
  if (normalization_detector_exciter_.normalized()) {
    for (size_t i = 0; i < size; ++i) {
      input_samples[i * 2] = 0;
    }
  }
}

void CvScaler::DetectNormalization() {
  // OPRAVA: Vrácen index [0] pro dummy read triggeru
  if (normalization_probe_value_[0] == trigger_input_.DummyRead()) {
    normalization_detector_trigger_.Process(1.0f, 1.0f);
  } else {
    normalization_detector_trigger_.Process(1.0f, -1.0f);
  }
  
  float x = adc_.float_value(ADC_CHANNEL_CV_V_OCT) - calibration_data_->normalization_detection_threshold;
  // OPRAVA: Vrácen index [0] pro normalizaci V/OCT pinu
  float y = normalization_probe_value_[0] ? -1.0f : 1.0f;
  if (x > -0.5f && x < 0.5f) {
    x = x < 0.0f ? -1.0f : 1.0f;
    normalization_detector_v_oct_.Process(x, y);
  } else {
    normalization_detector_v_oct_.Process(0.0f, y);
  }
  
  // OPRAVA: Správná rotace stavů v poli a zápis náhodného slova pro detekci kabelu
  normalization_probe_value_[1] = normalization_probe_value_[0];
  normalization_probe_value_[0] = Random::GetWord() >> 31;
  bool new_state = normalization_probe_enabled_
      ? normalization_probe_value_[0]
      : normalization_probe_forced_state_;
  normalization_probe_.Write(new_state);
}

#define ATTENUVERT(destination, NAME, min, max) \
  { \
    float value = adc_lp_[ADC_CHANNEL_CV_ ## NAME]; \
    value *= adc_lp_[ADC_CHANNEL_ATTENUVERTER_ ## NAME]; \
    value += adc_lp_[ADC_CHANNEL_POT_ ## NAME]; \
    CONSTRAIN(value, min, max) \
    destination = value; \
  }

void CvScaler::Read(Patch* patch, PerformanceState* performance_state) {
  // Process all CVs / pots.
  for (size_t i = 0; i < ADC_CHANNEL_LAST; ++i) {
    const ChannelSettings& settings = channel_settings_[i];
    float value = adc_.float_value(i);
    if (settings.remove_offset) {
      value = calibration_data_->offset[i] - value;
    }
    switch (settings.law) {
      case LAW_QUADRATIC_BIPOLAR:
        {
          value = value - 0.5f;
          float value2 = value * value * 4.0f * 3.3f;
          value = value < 0.0f ? -value2 : value2;
        }
        break;

      case LAW_QUARTIC_BIPOLAR:
        {
          value = value - 0.5f;
          float value2 = value * value * 4.0f;
          float value4 = value2 * value2 * 3.3f;
          value = value < 0.0f ? -value4 : value4;
        }
        break;

      default:
        break;
    }
    adc_lp_[i] += settings.lp_coefficient * (value - adc_lp_[i]);
  }

  // DEFINITIVNÍ SRAŽENÍ NEFUNKČNÍCH A NEPOUŽÍVANÝCH PRVKŮ NA NULU
  adc_lp_[ADC_CHANNEL_POT_STRUCTURE] = 0.0f;
  adc_lp_[ADC_CHANNEL_ATTENUVERTER_STRUCTURE] = 0.0f;
  adc_lp_[ADC_CHANNEL_ATTENUVERTER_FREQUENCY] = 0.0f;

  // CHYTRÁ INVERZE ZNAMENKA: Přenásobíme -1.0f, abychom otočili směr, ale zachovali střed na 12. hodině
  float chord = -1.0f * adc_lp_[ADC_CHANNEL_CV_FREQUENCY];
  chord += adc_lp_[ADC_CHANNEL_CV_STRUCTURE];

  {
    float structure_param = chord;
    CONSTRAIN(structure_param, 0.0f, 0.9995f)
    patch->structure = structure_param;  
  }
  
  ATTENUVERT(patch->brightness, BRIGHTNESS, 0.0f, 1.0f);
  ATTENUVERT(patch->damping, DAMPING, 0.0f, 1.0f);
  ATTENUVERT(patch->position, POSITION, 0.0f, 1.0f);
  
  performance_state->fm = 0.0f;
  
  float transpose = 60.0f * adc_lp_[ADC_CHANNEL_POT_FREQUENCY];
  float hysteresis = transpose - transpose_ > 0.0f ? -0.3f : +0.3f;
  transpose_ = static_cast<int32_t>(transpose + hysteresis + 0.5f);
  
  float note = calibration_data_->pitch_offset;
  note += adc_lp_[ADC_CHANNEL_CV_V_OCT] * calibration_data_->pitch_scale;
  
  performance_state->note = note;
  performance_state->tonic = 12.0f + transpose_;
  
  DetectNormalization();
  
  // Strumming / internal exciter triggering logic.
  bool internal_strum = normalization_detector_trigger_.normalized();
  bool internal_exciter = normalization_detector_exciter_.normalized();
  bool internal_note = normalization_detector_v_oct_.normalized();
  performance_state->internal_exciter = internal_exciter;
  performance_state->internal_strum = internal_strum;
  performance_state->internal_note = internal_note;
  performance_state->strum = trigger_input_.rising_edge();
  
  if (performance_state->internal_note) {
    // Remove quantization when nothing is plugged in the V/OCT input.
    performance_state->note = 0.0f;
    performance_state->tonic = 12.0f + transpose;
  }

  // NATIVNÍ VÝPOČET AKORDŮ SE STŘEDEM NA 12. HODINĚ
  chord *= static_cast<float>(kNumChords - 1);
  
  hysteresis = chord - chord_ > 0.0f ? -0.1f : +0.1f;
  chord_ = static_cast<int32_t>(chord + hysteresis + 0.5f);
  CONSTRAIN(chord_, 0, kNumChords - 1);
  performance_state->chord = chord_;
  
  adc_.Convert();
  trigger_input_.Read();
}

}  // namespace rings
