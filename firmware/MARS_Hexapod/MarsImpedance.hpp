#pragma once

#include <Arduino.h>

// Simple impedance controller supporting joint-space and Cartesian-space modes.
// All math is kept lightweight for Teensy 4.1; no dynamic allocation or STL.

enum ImpedanceMode : uint8_t {
  IMP_MODE_OFF   = 0,
  IMP_MODE_JOINT = 1,
  IMP_MODE_CART  = 2
};

struct JointImpedanceGains {
  uint16_t k_spring_milli[3]; // per joint: coxa, femur, tibia
  uint16_t k_damp_milli[3];   // per joint: coxa, femur, tibia
};

struct CartesianImpedanceGains {
  uint16_t k_spring_milli[3]; // per axis: x, y, z (body frame)
  uint16_t k_damp_milli[3];   // per axis: x, y, z (body frame)
};

struct ImpedanceConfig {
  bool          enabled;
  ImpedanceMode mode;

  JointImpedanceGains     joint;
  CartesianImpedanceGains cart;

  uint16_t      output_scale_milli; // scale for corrections (0..1000), default 1000
  uint16_t      joint_deadband_cd;   // symmetric deadband on joint error (centideg)
  float         cart_deadband_mm;    // symmetric deadband on Cartesian error magnitude (mm)
};

class MarsImpedance {
  public:
    MarsImpedance() {
      reset();
    }

    void reset();

    void setEnabled(bool on) {
      cfg_.enabled = on;
    }
    void setMode(ImpedanceMode m) {
      cfg_.mode = m;
    }

    void setJointSpring(int jointIdxOrAll, uint16_t k_milli);
    void setJointDamp(int jointIdxOrAll, uint16_t c_milli);

    void setCartSpring(int axisOrAll, uint16_t k_milli);
    void setCartDamp(int axisOrAll, uint16_t c_milli);

    const ImpedanceConfig& config() const {
      return cfg_;
    }
    ImpedanceConfig& config() {
      return cfg_;
    }

    // Compute joint-space corrections for a single leg.
    // Inputs:
    //  - leg: leg index (0..5), unused for now but kept for future per-leg options
    //  - est_cd[3]: estimated joint angles (centideg)
    //  - cmd_cd[3]: nominal joint commands (centideg)
    //  - dt_s: tick duration in seconds
    //  - bodyFoot: optional BODY-frame foot position (x,y,z) in mm; may be nullptr when mode==JOINT
    //  - bodyRef: optional BODY-frame reference foot position (x,y,z) in mm; may be nullptr when mode==JOINT
    // Output:
    //  - out_corr_cd[3]: additive joint corrections (centideg)
    void computeLegCorrection(
      uint8_t leg,
      const int16_t est_cd[3],
      const int16_t cmd_cd[3],
      float dt_s,
      const float* bodyFoot,
      const float* bodyRef,
      int16_t out_corr_cd[3]);

  private:
    ImpedanceConfig cfg_;

    // For simple Cartesian damping, track last foot position per leg (x,y,z in mm).
    struct FootState {
      float last_x_mm;
      float last_y_mm;
      float last_z_mm;
      bool  valid;
    };

    FootState foot_state_[6];
};
