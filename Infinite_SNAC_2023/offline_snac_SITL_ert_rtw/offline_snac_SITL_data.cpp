//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: offline_snac_SITL_data.cpp
//
// Code generated for Simulink model 'offline_snac_SITL'.
//
// Model version                  : 3.38
// Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
// C/C++ source code generated on : Mon Aug 12 14:21:53 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "offline_snac_SITL.h"

// Block parameters (default storage)
P_offline_snac_SITL_T offline_snac_SITL_P = {
  // Mask Parameter: DiscreteDerivative_ICPrevScaled
  //  Referenced by: '<S13>/UD'

  0.0F,

  // Mask Parameter: DiscreteDerivative1_ICPrevScale
  //  Referenced by: '<S14>/UD'

  0.0F,

  // Mask Parameter: DiscreteDerivative2_ICPrevScale
  //  Referenced by: '<S15>/UD'

  0.0F,

  // Computed Parameter: Out1_Y0
  //  Referenced by: '<S21>/Out1'

  {
    (0ULL),                            // timestamp
    (0ULL),                            // timestamp_sample
    (0ULL),                            // ref_timestamp
    0.0,                               // ref_lat
    0.0,                               // ref_lon
    0.0F,                              // x
    0.0F,                              // y
    0.0F,                              // z

    {
      0.0F, 0.0F }
    ,                                  // delta_xy
    0.0F,                              // delta_z
    0.0F,                              // vx
    0.0F,                              // vy
    0.0F,                              // vz
    0.0F,                              // z_deriv

    {
      0.0F, 0.0F }
    ,                                  // delta_vxy
    0.0F,                              // delta_vz
    0.0F,                              // ax
    0.0F,                              // ay
    0.0F,                              // az
    0.0F,                              // heading
    0.0F,                              // delta_heading
    0.0F,                              // ref_alt
    0.0F,                              // dist_bottom
    0.0F,                              // eph
    0.0F,                              // epv
    0.0F,                              // evh
    0.0F,                              // evv
    0.0F,                              // vxy_max
    0.0F,                              // vz_max
    0.0F,                              // hagl_min
    0.0F,                              // hagl_max
    false,                             // xy_valid
    false,                             // z_valid
    false,                             // v_xy_valid
    false,                             // v_z_valid
    0U,                                // xy_reset_counter
    0U,                                // z_reset_counter
    0U,                                // vxy_reset_counter
    0U,                                // vz_reset_counter
    0U,                                // heading_reset_counter
    false,                             // heading_good_for_control
    false,                             // xy_global
    false,                             // z_global
    false,                             // dist_bottom_valid
    0U,                                // dist_bottom_sensor_bitfield
    false,                             // dead_reckoning
    0U                                 // _padding0
  },

  // Computed Parameter: Constant_Value
  //  Referenced by: '<S5>/Constant'

  {
    (0ULL),                            // timestamp
    (0ULL),                            // timestamp_sample
    (0ULL),                            // ref_timestamp
    0.0,                               // ref_lat
    0.0,                               // ref_lon
    0.0F,                              // x
    0.0F,                              // y
    0.0F,                              // z

    {
      0.0F, 0.0F }
    ,                                  // delta_xy
    0.0F,                              // delta_z
    0.0F,                              // vx
    0.0F,                              // vy
    0.0F,                              // vz
    0.0F,                              // z_deriv

    {
      0.0F, 0.0F }
    ,                                  // delta_vxy
    0.0F,                              // delta_vz
    0.0F,                              // ax
    0.0F,                              // ay
    0.0F,                              // az
    0.0F,                              // heading
    0.0F,                              // delta_heading
    0.0F,                              // ref_alt
    0.0F,                              // dist_bottom
    0.0F,                              // eph
    0.0F,                              // epv
    0.0F,                              // evh
    0.0F,                              // evv
    0.0F,                              // vxy_max
    0.0F,                              // vz_max
    0.0F,                              // hagl_min
    0.0F,                              // hagl_max
    false,                             // xy_valid
    false,                             // z_valid
    false,                             // v_xy_valid
    false,                             // v_z_valid
    0U,                                // xy_reset_counter
    0U,                                // z_reset_counter
    0U,                                // vxy_reset_counter
    0U,                                // vz_reset_counter
    0U,                                // heading_reset_counter
    false,                             // heading_good_for_control
    false,                             // xy_global
    false,                             // z_global
    false,                             // dist_bottom_valid
    0U,                                // dist_bottom_sensor_bitfield
    false,                             // dead_reckoning
    0U                                 // _padding0
  },

  // Computed Parameter: Out1_Y0_f
  //  Referenced by: '<S23>/Out1'

  {
    (0ULL),                            // timestamp
    0.0F,                              // x
    0.0F,                              // y
    0.0F,                              // z
    0.0F,                              // vx
    0.0F,                              // vy
    0.0F,                              // vz

    {
      0.0F, 0.0F, 0.0F }
    ,                                  // acceleration

    {
      0.0F, 0.0F, 0.0F }
    ,                                  // thrust
    0.0F,                              // yaw
    0.0F                               // yawspeed
  },

  // Computed Parameter: Constant_Value_p
  //  Referenced by: '<S7>/Constant'

  {
    (0ULL),                            // timestamp
    0.0F,                              // x
    0.0F,                              // y
    0.0F,                              // z
    0.0F,                              // vx
    0.0F,                              // vy
    0.0F,                              // vz

    {
      0.0F, 0.0F, 0.0F }
    ,                                  // acceleration

    {
      0.0F, 0.0F, 0.0F }
    ,                                  // thrust
    0.0F,                              // yaw
    0.0F                               // yawspeed
  },

  // Computed Parameter: Constant_Value_o
  //  Referenced by: '<S19>/Constant'

  {
    (0ULL),                            // timestamp
    0.0F,                              // x
    0.0F,                              // y
    0.0F,                              // z
    0.0F,                              // vx
    0.0F,                              // vy
    0.0F,                              // vz

    {
      0.0F, 0.0F, 0.0F }
    ,                                  // acceleration

    {
      0.0F, 0.0F, 0.0F }
    ,                                  // thrust
    0.0F,                              // yaw
    0.0F                               // yawspeed
  },

  // Computed Parameter: Out1_Y0_h
  //  Referenced by: '<S22>/Out1'

  {
    (0ULL),                            // timestamp
    (0ULL),                            // timestamp_sample

    {
      0.0F, 0.0F, 0.0F, 0.0F }
    ,                                  // q

    {
      0.0F, 0.0F, 0.0F, 0.0F }
    ,                                  // delta_q_reset
    0U,                                // quat_reset_counter

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U }
    // _padding0
  },

  // Computed Parameter: Constant_Value_a
  //  Referenced by: '<S6>/Constant'

  {
    (0ULL),                            // timestamp
    (0ULL),                            // timestamp_sample

    {
      0.0F, 0.0F, 0.0F, 0.0F }
    ,                                  // q

    {
      0.0F, 0.0F, 0.0F, 0.0F }
    ,                                  // delta_q_reset
    0U,                                // quat_reset_counter

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U }
    // _padding0
  },

  // Computed Parameter: Out1_Y0_hw
  //  Referenced by: '<S20>/Out1'

  {
    (0ULL),                            // timestamp
    (0ULL),                            // timestamp_sample
    0U,                                // device_id
    0.0F,                              // x
    0.0F,                              // y
    0.0F,                              // z
    0.0F,                              // temperature
    0U,                                // error_count

    {
      0U, 0U, 0U }
    ,                                  // clip_counter
    0U,                                // samples

    {
      0U, 0U, 0U, 0U }
    // _padding0
  },

  // Computed Parameter: Constant_Value_e
  //  Referenced by: '<S4>/Constant'

  {
    (0ULL),                            // timestamp
    (0ULL),                            // timestamp_sample
    0U,                                // device_id
    0.0F,                              // x
    0.0F,                              // y
    0.0F,                              // z
    0.0F,                              // temperature
    0U,                                // error_count

    {
      0U, 0U, 0U }
    ,                                  // clip_counter
    0U,                                // samples

    {
      0U, 0U, 0U, 0U }
    // _padding0
  },

  // Expression: 15
  //  Referenced by: '<S9>/Rate Limiter3'

  15.0,

  // Expression: -15
  //  Referenced by: '<S9>/Rate Limiter3'

  -15.0,

  // Expression: 15
  //  Referenced by: '<S9>/Rate Limiter4'

  15.0,

  // Expression: -15
  //  Referenced by: '<S9>/Rate Limiter4'

  -15.0,

  // Expression: 15
  //  Referenced by: '<S9>/Rate Limiter5'

  15.0,

  // Expression: -15
  //  Referenced by: '<S9>/Rate Limiter5'

  -15.0,

  // Expression: 15
  //  Referenced by: '<S9>/Rate Limiter'

  15.0,

  // Expression: -15
  //  Referenced by: '<S9>/Rate Limiter'

  -15.0,

  // Expression: 15
  //  Referenced by: '<S9>/Rate Limiter1'

  15.0,

  // Expression: -15
  //  Referenced by: '<S9>/Rate Limiter1'

  -15.0,

  // Expression: 15
  //  Referenced by: '<S9>/Rate Limiter2'

  15.0,

  // Expression: -15
  //  Referenced by: '<S9>/Rate Limiter2'

  -15.0,

  // Expression: 2
  //  Referenced by: '<S1>/Rate Limiter3'

  2.0,

  // Expression: -2
  //  Referenced by: '<S1>/Rate Limiter3'

  -2.0,

  // Expression: 2
  //  Referenced by: '<S1>/Rate Limiter4'

  2.0,

  // Expression: -2
  //  Referenced by: '<S1>/Rate Limiter4'

  -2.0,

  // Expression: 2
  //  Referenced by: '<S1>/Rate Limiter5'

  2.0,

  // Expression: -2
  //  Referenced by: '<S1>/Rate Limiter5'

  -2.0,

  // Computed Parameter: Constant_Value_k
  //  Referenced by: '<S29>/Constant'

  1.0F,

  // Computed Parameter: Constant_Value_d
  //  Referenced by: '<S30>/Constant'

  1.0F,

  // Computed Parameter: Constant1_Value
  //  Referenced by: '<S9>/Constant1'

  1.0F,

  // Computed Parameter: RateLimiter3_IC
  //  Referenced by: '<S9>/Rate Limiter3'

  0.0F,

  // Computed Parameter: RateLimiter4_IC
  //  Referenced by: '<S9>/Rate Limiter4'

  0.0F,

  // Computed Parameter: RateLimiter5_IC
  //  Referenced by: '<S9>/Rate Limiter5'

  0.0F,

  // Computed Parameter: RateLimiter_IC
  //  Referenced by: '<S9>/Rate Limiter'

  0.0F,

  // Computed Parameter: RateLimiter1_IC
  //  Referenced by: '<S9>/Rate Limiter1'

  0.0F,

  // Computed Parameter: RateLimiter2_IC
  //  Referenced by: '<S9>/Rate Limiter2'

  0.0F,

  // Computed Parameter: Saturation3_UpperSat
  //  Referenced by: '<S9>/Saturation3'

  2.0F,

  // Computed Parameter: Saturation3_LowerSat
  //  Referenced by: '<S9>/Saturation3'

  -2.0F,

  // Computed Parameter: Saturation4_UpperSat
  //  Referenced by: '<S9>/Saturation4'

  2.0F,

  // Computed Parameter: Saturation4_LowerSat
  //  Referenced by: '<S9>/Saturation4'

  -2.0F,

  // Computed Parameter: Saturation5_UpperSat
  //  Referenced by: '<S9>/Saturation5'

  15.0F,

  // Computed Parameter: Saturation5_LowerSat
  //  Referenced by: '<S9>/Saturation5'

  0.0F,

  // Computed Parameter: TSamp_WtEt
  //  Referenced by: '<S13>/TSamp'

  250.0F,

  // Computed Parameter: RateLimiter3_IC_b
  //  Referenced by: '<S1>/Rate Limiter3'

  0.0F,

  // Computed Parameter: TSamp_WtEt_c
  //  Referenced by: '<S14>/TSamp'

  250.0F,

  // Computed Parameter: RateLimiter4_IC_i
  //  Referenced by: '<S1>/Rate Limiter4'

  0.0F,

  // Computed Parameter: TSamp_WtEt_h
  //  Referenced by: '<S15>/TSamp'

  250.0F,

  // Computed Parameter: RateLimiter5_IC_k
  //  Referenced by: '<S1>/Rate Limiter5'

  0.0F,

  // Computed Parameter: DiscreteTimeIntegrator_gainval
  //  Referenced by: '<S3>/Discrete-Time Integrator'

  0.004F,

  // Computed Parameter: DiscreteTimeIntegrator_IC
  //  Referenced by: '<S3>/Discrete-Time Integrator'

  0.0F,

  // Computed Parameter: Constant2_Value
  //  Referenced by: '<S3>/Constant2'

  0.3F,

  // Computed Parameter: Constant_Value_g
  //  Referenced by: '<S3>/Constant'

  1.0F,

  // Computed Parameter: Reset_Value
  //  Referenced by: '<Root>/Reset'

  false,

  // Computed Parameter: DataStoreMemory_InitialValue
  //  Referenced by: '<Root>/Data Store Memory'

  false
};

//
// File trailer for generated code.
//
// [EOF]
//
