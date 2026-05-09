#pragma once

class MotionController {
 public:
  void reset();
  void compute(const float raw[9], const float* baseline, float dt, float out[6]);
  bool hasMotionActivity() const;

 private:
  static float sensitivityCurve(float value, float dead, float limit);
  static float clampf(float v, float lo, float hi);
  static float hardZero(float v, float thr);
  static float lowpass(float prev, float x, float dt, float tau);
  static float axisBaseDead(int i);

  // Per-axis Kalman filter state
  float kalmanX_[6] = {};  // Estimated state
  float kalmanP_[6] = {};  // Estimate uncertainty (covariance)
  void kalmanStep(int axis, float measurement);

  float filt_[6] = {};
  bool motionActive_ = false;
};
