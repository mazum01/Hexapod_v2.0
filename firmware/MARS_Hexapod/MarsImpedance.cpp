#include "MarsImpedance.hpp"

void MarsImpedance::reset()
{
  cfg_.enabled = false;
  cfg_.mode = IMP_MODE_OFF;
  cfg_.output_scale_milli = 1000;
  cfg_.joint_deadband_cd = 0;
  cfg_.cart_deadband_mm  = 0.0f;
  for (int j = 0; j < 3; ++j) {
    cfg_.joint.k_spring_milli[j] = 0;
    cfg_.joint.k_damp_milli[j]   = 0;
    cfg_.cart.k_spring_milli[j]  = 0;
    cfg_.cart.k_damp_milli[j]    = 0;
  }
  for (int L = 0; L < 6; ++L) {
    foot_state_[L].last_x_mm = 0.0f;
    foot_state_[L].last_y_mm = 0.0f;
    foot_state_[L].last_z_mm = 0.0f;
    foot_state_[L].valid = false;
  }
}

void MarsImpedance::setJointSpring(int jointIdxOrAll, uint16_t k_milli)
{
  if (jointIdxOrAll < 0) return;
  if (jointIdxOrAll == 3) {
    for (int j = 0; j < 3; ++j) cfg_.joint.k_spring_milli[j] = k_milli;
  } else if (jointIdxOrAll >= 0 && jointIdxOrAll < 3) {
    cfg_.joint.k_spring_milli[jointIdxOrAll] = k_milli;
  }
}

void MarsImpedance::setJointDamp(int jointIdxOrAll, uint16_t c_milli)
{
  if (jointIdxOrAll < 0) return;
  if (jointIdxOrAll == 3) {
    for (int j = 0; j < 3; ++j) cfg_.joint.k_damp_milli[j] = c_milli;
  } else if (jointIdxOrAll >= 0 && jointIdxOrAll < 3) {
    cfg_.joint.k_damp_milli[jointIdxOrAll] = c_milli;
  }
}

void MarsImpedance::setCartSpring(int axisOrAll, uint16_t k_milli)
{
  if (axisOrAll < 0) return;
  if (axisOrAll == 3) {
    for (int a = 0; a < 3; ++a) cfg_.cart.k_spring_milli[a] = k_milli;
  } else if (axisOrAll >= 0 && axisOrAll < 3) {
    cfg_.cart.k_spring_milli[axisOrAll] = k_milli;
  }
}

void MarsImpedance::setCartDamp(int axisOrAll, uint16_t c_milli)
{
  if (axisOrAll < 0) return;
  if (axisOrAll == 3) {
    for (int a = 0; a < 3; ++a) cfg_.cart.k_damp_milli[a] = c_milli;
  } else if (axisOrAll >= 0 && axisOrAll < 3) {
    cfg_.cart.k_damp_milli[axisOrAll] = c_milli;
  }
}

void MarsImpedance::computeLegCorrection(
    uint8_t leg,
    const int16_t est_cd[3],
    const int16_t cmd_cd[3],
    float dt_s,
    const float* bodyFoot,
    const float* bodyRef,
    int16_t out_corr_cd[3])
{
  (void)leg;
  if (!out_corr_cd) return;
  out_corr_cd[0] = out_corr_cd[1] = out_corr_cd[2] = 0;

  if (!cfg_.enabled || cfg_.mode == IMP_MODE_OFF) {
    return;
  }

  if (dt_s <= 0.0f) dt_s = 1e-3f; // guard against divide by zero

  if (cfg_.mode == IMP_MODE_JOINT) {
    // Simple joint-space spring-damper around the nominal command.
    // error = est - cmd (cd); velocity approx from delta est over dt is left to caller;
    // here we only use position error and approximate damping from est-cmd as well.
    for (int j = 0; j < 3; ++j) {
      int32_t e_cd = (int32_t)est_cd[j] - (int32_t)cmd_cd[j];
      // Apply symmetric deadband around zero error to improve compliance.
      if (cfg_.joint_deadband_cd > 0) {
        int32_t db = (int32_t)cfg_.joint_deadband_cd;
        if (e_cd > -db && e_cd < db) {
          continue; // within deadband: no correction on this joint
        }
      }
      // Spring term: k_spring_milli * e_cd / 1000
      int32_t ks = (int32_t)cfg_.joint.k_spring_milli[j];
      int32_t spring_cd = (ks * e_cd) / 1000;
      // Damping term: approximate using same error as proxy for velocity.
      int32_t kd = (int32_t)cfg_.joint.k_damp_milli[j];
      int32_t damp_cd = (kd * e_cd) / 1000;
      int32_t corr = -(spring_cd + damp_cd);
      corr = (corr * (int32_t)cfg_.output_scale_milli) / 1000;
      if (corr < -32768) corr = -32768;
      if (corr >  32767) corr =  32767;
      out_corr_cd[j] = (int16_t)corr;
    }
    return;
  }

  if (cfg_.mode == IMP_MODE_CART) {
    if (!bodyFoot || !bodyRef) {
      return; // no data; no correction
    }

    // Compute Cartesian displacement and simple damping from finite differences
    // in BODY frame for all three axes X, Y, Z (mm).
    float ex = bodyFoot[0] - bodyRef[0];
    float ey = bodyFoot[1] - bodyRef[1];
    float ez = bodyFoot[2] - bodyRef[2];

    // Optional Cartesian deadband on displacement magnitude.
    if (cfg_.cart_deadband_mm > 0.0f) {
      float mag2 = ex*ex + ey*ey + ez*ez;
      float db2  = cfg_.cart_deadband_mm * cfg_.cart_deadband_mm;
      if (mag2 < db2) {
        return; // within deadband: no Cartesian correction for this leg
      }
    }

    FootState& fs = foot_state_[(leg < 6) ? leg : 0];
  float vx = 0.0f, vy = 0.0f, vz = 0.0f;
    if (fs.valid) {
      vx = (bodyFoot[0] - fs.last_x_mm) / dt_s;
      vy = (bodyFoot[1] - fs.last_y_mm) / dt_s;
      vz = (bodyFoot[2] - fs.last_z_mm) / dt_s;
    }
    fs.last_x_mm = bodyFoot[0];
    fs.last_y_mm = bodyFoot[1];
    fs.last_z_mm = bodyFoot[2];
    fs.valid = true;

    // Virtual Cartesian forces along X,Y,Z using per-axis spring/damping gains.
    float Fx = 0.0f, Fy = 0.0f, Fz = 0.0f;
    {
      const float ksx = (float)cfg_.cart.k_spring_milli[0] / 1000.0f;
      const float kdx = (float)cfg_.cart.k_damp_milli[0]   / 1000.0f;
      const float ksy = (float)cfg_.cart.k_spring_milli[1] / 1000.0f;
      const float kdy = (float)cfg_.cart.k_damp_milli[1]   / 1000.0f;
      const float ksz = (float)cfg_.cart.k_spring_milli[2] / 1000.0f;
      const float kdz = (float)cfg_.cart.k_damp_milli[2]   / 1000.0f;
      // Sign convention: positive force resists displacement/velocity away from reference.
      Fx = -(ksx * ex + kdx * vx);
      Fy = -(ksy * ey + kdy * vy);
      Fz = -(ksz * ez + kdz * vz);
    }

    // Heuristic mapping from Cartesian forces to joint angle corrections (cd).
    // This is intentionally simple to keep CPU cost low; constants are tuned offline.
    const float CD_PER_MM_FORCE_COXA  = 2.5f;  // centideg per unit X/Y force
    const float CD_PER_MM_FORCE_FEMUR = 5.0f;  // centideg per unit Y/Z force
    const float CD_PER_MM_FORCE_TIBIA = 10.0f; // centideg per unit Z force

    float coxa_corr_cd_f  = (Fx) * CD_PER_MM_FORCE_COXA;
    float femur_corr_cd_f = (Fy * 0.5f + Fz * 0.5f) * CD_PER_MM_FORCE_FEMUR;
    float tibia_corr_cd_f = Fz * CD_PER_MM_FORCE_TIBIA;

    const float scale = (float)cfg_.output_scale_milli / 1000.0f;
    coxa_corr_cd_f  *= scale;
    femur_corr_cd_f *= scale;
    tibia_corr_cd_f *= scale;

    if (coxa_corr_cd_f < -32768.0f) coxa_corr_cd_f = -32768.0f;
    if (coxa_corr_cd_f >  32767.0f) coxa_corr_cd_f =  32767.0f;
    if (femur_corr_cd_f < -32768.0f) femur_corr_cd_f = -32768.0f;
    if (femur_corr_cd_f >  32767.0f) femur_corr_cd_f =  32767.0f;
    if (tibia_corr_cd_f < -32768.0f) tibia_corr_cd_f = -32768.0f;
    if (tibia_corr_cd_f >  32767.0f) tibia_corr_cd_f =  32767.0f;

    out_corr_cd[0] = (int16_t)coxa_corr_cd_f;
    out_corr_cd[1] = (int16_t)femur_corr_cd_f;
    out_corr_cd[2] = (int16_t)tibia_corr_cd_f;
    return;
  }
}
