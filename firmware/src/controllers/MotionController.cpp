#include "controllers/MotionController.h"

#include <Arduino.h>
#include <math.h>

#include "Config.h"

namespace {
enum RawIndex {
  RAW_MAG1_X = 0,
  RAW_MAG1_Y,
  RAW_MAG1_Z,
  RAW_MAG2_X,
  RAW_MAG2_Y,
  RAW_MAG2_Z,
  RAW_MAG3_X,
  RAW_MAG3_Y,
  RAW_MAG3_Z
};

enum AxisIndex {
  AXIS_TX = 0,
  AXIS_TY,
  AXIS_TZ,
  AXIS_RX,
  AXIS_RY,
  AXIS_RZ
};

// Precomputed geometric constants for the equilateral sensor triangle.
const float kOneThird = 1.0f / 3.0f;
const float kSqrt3 = 1.7320508f;          // sqrt(3)
const float kSqrt3Over6 = 0.28867513f;    // sqrt(3)/6
const float kSqrt3Over3 = 0.57735027f;    // sqrt(3)/3
const float kMag2PosX = -0.5f;
const float kMag2PosY = kSqrt3Over6;
const float kMag3PosX = 0.5f;
const float kMag3PosY = kSqrt3Over6;
const float kMag1PosX = 0.0f;
const float kMag1PosY = -kSqrt3Over3;
}  // namespace

void MotionController::reset() {
  for (int i = 0; i < 6; i++) {
    filt_[i] = 0.0;
  }
  motionActive_ = false;
}

float MotionController::clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float MotionController::hardZero(float v, float thr) {
  return (fabs(v) < thr) ? 0.0 : v;
}

float MotionController::lowpass(float prev, float x, float dt, float tau) {
  if (tau <= 0.0) return x;
  const float a = dt / (tau + dt);
  return prev + a * (x - prev);
}

float MotionController::axisBaseDead(int i) {
  return (i < 3) ? Config::DEAD_T : Config::DEAD_R;
}

#if 1
float MotionController::sensitivityCurve(float value, float dead, float limit) {
  // Map the post-dead-zone range [dead, limit] onto [0, limit] with a power curve.
  // This gives fine control at small deflections and fast motion at large ones.
  const float sign = (value >= 0.0f) ? 1.0f : -1.0f;
  const float absVal = fabsf(value);
  if (absVal <= dead) return 0.0f;
  //if (absVal <= dead) return 0.1f;

  // Normalize to 0..1 within the active range
  const float range = limit - dead;
  if (range <= 0.0f) return 0.0f;
  //if (range <= 0.0f) return 0.1f;
  const float n = clampf((absVal - dead) / range, 0.0f, 1.0f);

  // Fast paths for the documented exponents. powf on Cortex-M0+ soft-float
  // costs ~1-2k cycles per call and this runs 6x per tick. The compare is
  // against a compile-time constant so the optimizer folds the branches.
  float shaped;
  if (Config::SENSITIVITY_EXP == 3.0f) {
    shaped = n * n * n;
  } else if (Config::SENSITIVITY_EXP == 2.0f) {
    shaped = n * n;
  } else if (Config::SENSITIVITY_EXP == 1.0f) {
    shaped = n;
  } else {
    shaped = powf(n, Config::SENSITIVITY_EXP);
  }
#if 0
  return sign * shaped * limit;
#else
  shaped = shaped * limit;
  if(shaped < 1){
    if(shaped <= 0.4){
      shaped = 0.0;
    }else{
      shaped = 1.0;
    }
  }
  // result 0.1 / shaped / 1 / 0; 
  return sign * shaped;
#endif
}
#endif

void MotionController::compute(const float raw[9], const float* baseline, float dt,
                               float out[6]) {
  // Baseline subtraction converts magnetic deltas around the calibrated rest pose.
  const float mag1x = raw[RAW_MAG1_X] - baseline[RAW_MAG1_X];
  const float mag1y = raw[RAW_MAG1_Y] - baseline[RAW_MAG1_Y];
  const float mag1z = raw[RAW_MAG1_Z] - baseline[RAW_MAG1_Z];
  const float mag2x = raw[RAW_MAG2_X] - baseline[RAW_MAG2_X];
  const float mag2y = raw[RAW_MAG2_Y] - baseline[RAW_MAG2_Y];
  const float mag2z = raw[RAW_MAG2_Z] - baseline[RAW_MAG2_Z];
  const float mag3x = raw[RAW_MAG3_X] - baseline[RAW_MAG3_X];
  const float mag3y = raw[RAW_MAG3_Y] - baseline[RAW_MAG3_Y];
  const float mag3z = raw[RAW_MAG3_Z] - baseline[RAW_MAG3_Z];

  // Translation:
  //   Tx = (mag1x + mag2x + mag3x) / 3
  //   Ty = (mag1y + mag2y + mag3y) / 3
  //   Tz = (mag1z + mag2z + mag3z) / 3
  // Translation: average of all three sensors.
  const float tx = (mag1x + mag2x + mag3x) * kOneThird;
  const float ty = (mag1y + mag2y + mag3y) * kOneThird;
  const float tz = (mag1z + mag2z + mag3z) * kOneThird;

  #if 0
  // Physical PCB layout:
  // MAG2 = top left, MAG3 = top right, MAG1 = bottom.
  const float mag2PosX = -0.5;
  const float mag2PosY = sqrt(3.0) / 6.0;

  const float mag3PosX = 0.5;
  const float mag3PosY = sqrt(3.0) / 6.0;

  const float mag1PosX = 0.0;
  const float mag1PosY = -sqrt(3.0) / 3.0;

  // Rotation estimates:
  //   Ry = mag3z - mag2z
  //     right sensor minus left sensor
  //     -> side to side tilt across the top edge
  //
  //   Rx = sqrt(3) * (mag2z + mag3z - 2 * mag1z) / 3
  //     top pair minus bottom sensor
  //     -> front/back tilt of the triangle
  const float rx = (sqrt(3.0) * (mag2z + mag3z - 2.0 * mag1z)) / 3.0;
  const float ry = (mag3z - mag2z);

  //   Rz = sum_i (posXi * magYi - posYi * magXi)
  // Each sensor contributes according to its x/y position in the triangle.
  const float swirlNum =
      (mag2PosX * mag2y - mag2PosY * mag2x) +
      (mag3PosX * mag3y - mag3PosY * mag3x) +
      (mag1PosX * mag1y - mag1PosY * mag1x);
  const float rz = swirlNum;
#else
  // Rotation estimates from sensor triangle geometry.
  //   Ry: side-to-side tilt (right sensor minus left)
  //   Rx: front/back tilt (top pair minus bottom)
  //   Rz: twist (cross-product per sensor position)
  const float rx = (kSqrt3 * (mag2z + mag3z - 2.0f * mag1z)) * kOneThird;
  const float ry = (mag3z - mag2z);
  const float rz =
      (kMag2PosX * mag2y - kMag2PosY * mag2x) +
      (kMag3PosX * mag3y - kMag3PosY * mag3x) +
      (kMag1PosX * mag1y - kMag1PosY * mag1x);

#endif
  // Apply sign fixes and gains
  float y[6];
  y[AXIS_TX] = Config::SIGN_AXIS[AXIS_TX] * tx * Config::GAIN_T[AXIS_TX];
  y[AXIS_TY] = Config::SIGN_AXIS[AXIS_TY] * ty * Config::GAIN_T[AXIS_TY];
  y[AXIS_TZ] = Config::SIGN_AXIS[AXIS_TZ] * tz * Config::GAIN_T[AXIS_TZ];
  y[AXIS_RX] = Config::SIGN_AXIS[AXIS_RX] * rx * Config::GAIN_R[AXIS_RX - 3];
  y[AXIS_RY] = Config::SIGN_AXIS[AXIS_RY] * ry * Config::GAIN_R[AXIS_RY - 3];
  y[AXIS_RZ] = Config::SIGN_AXIS[AXIS_RZ] * rz * Config::GAIN_R[AXIS_RZ - 3];

  // Filter, clamp to range and dead zones.
  motionActive_ = false;
  for (int i = 0; i < 6; i++) {
    const float dead = axisBaseDead(i);

//#if 0    
    if (fabs(y[i]) < dead) {
      filt_[i] = 0.0;
    } else {
      //filt_[i] = lowpass(filt_[i], y[i], dt, Config::SMOOTH_TAU_S);
      filt_[i]=y[i];
    }
#if 0
    const float limited =
        clampf(filt_[i], -Config::AXIS_LIMIT, Config::AXIS_LIMIT);
    out[i] = hardZero(limited, dead);
#else
    // Apply sensitivity curve to filtered output
    out[i] = sensitivityCurve(filt_[i], dead, Config::AXIS_LIMIT);
    //out[i] = sensitivityCurve(y[i], dead, Config::AXIS_LIMIT);
#endif

#if 1
    if (out[i] != 0.0) {
      motionActive_ = true;      
    }
#else
    if (out[i] == 0.1 | out[i] == 0.0) {
      out[i] = 0.0;
    }else{
      motionActive_ = true;
    }
#endif
  }

}

bool MotionController::hasMotionActivity() const { return motionActive_; }
