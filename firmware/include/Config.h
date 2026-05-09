#pragma once

#include <Arduino.h>

namespace Config {

const bool ENABLE_TELEMETRY = true;
//const bool ENABLE_TELEMETRY = false;

// Hardware pins (XIAO RP2040)
//const int PIN_RIGHT_BTN = D0;
//const int PIN_LED_LS = D1;
//const int PIN_LEFT_BTN = D2;
//const int PIN_LED_DATA = D3;

//const int PIN_MAG1_LS = D10;
//const int PIN_MAG2_LS = D9;
//const int PIN_MAG3_LS = D8;

// Hardware pins (RP2040 ZERO)
const int PIN_RIGHT_BTN = 0;
const int PIN_LEFT_BTN = 2;
const int PIN_BOOT_BTN = 25; //bootloader pin
const int PIN_LED_DATA = 3;
//const int PIN_LED_DATA = 16; //rp2040_zero onboard RGB
const int PIN_LED_LS = 1;
const int PIN_MAG1_LS = 29;
const int PIN_MAG2_LS = 28;
const int PIN_MAG3_LS = 27;

// Samples for calibration offset
const int ZERO_SAMPLES = 200;

// Gains and sign fixes
const float GAIN_T[3] = {28.0, 28.0, 24.0};
const float GAIN_R[3] = {18.0, 18.0, 20.0};
const int SIGN_AXIS[6] = {-1, +1, -1, +1, +1, +1};

// Dead zones
//const float DEAD_T = 16.0;
//const float DEAD_T = 8.0;
const float DEAD_T = 4.0;
//const float DEAD_R = 20.0;
//const float DEAD_R = 10.0;
const float DEAD_R = 5.0;

// Sensitivity curve exponent.
// 1.0 = linear, 3.0 = cubic (fine control at small deflections, fast at large).
const float SENSITIVITY_EXP = 3.0;
//const float SENSITIVITY_EXP = 2.5;

// Sensor validation: reject any frame where a magnetic axis reads beyond
// this magnitude. EXTRA_SHORT_RANGE peaks around 130 mT; anything past
// this bound indicates a read fault or corrupted frame.
const float SENSOR_SANITY_LIMIT_MT = 500.0;

// Smoothing
const float SMOOTH_TAU_S = 0.08;

// Final axis output range
const float AXIS_LIMIT = 350.0;

// RGB LEDs
const int LED_COUNT = 8;
const int LED_BRIGHTNESS = 40;
//const unsigned long LED_IDLE_COLOR = 0x00FF00; //green
const unsigned long LED_IDLE_COLOR = 0x800080; // purple
//const unsigned long LED_IDLE_COLOR = 0x6B3FA0; // royal purple
//const unsigned long LED_IDLE_COLOR = 0xE6E6FA; // Lavender
//const unsigned long LED_CALIBRATING_COLOR = 0x0000FF; //blue
//const unsigned long LED_CALIBRATING_COLOR = 0xFFDF00; // gold yellow
//const unsigned long LED_CALIBRATING_COLOR = 0xFFD700; // gold
const unsigned long LED_CALIBRATING_COLOR = 0xFFBF00; // amber
const unsigned long LED_ERROR_COLOR = 0xFF0000;

// FSM timing
const long IDLE_SLEEP_TIMEOUT_MS = 2 * 60 * 1000;


// Sleep-mode wake on knob motion. Sensors stay powered while asleep,
// so we can poll them at a slow cadence and wake Idle if any axis
// deviates from the calibrated baseline by more than the threshold.
const unsigned long SLEEP_WAKE_POLL_MS = 100;
const float SLEEP_WAKE_THRESHOLD_MT = 5.0;
}  // namespace Config
