//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: offline_snac_SITL_data.cpp
//
// Code generated for Simulink model 'offline_snac_SITL'.
//
// Model version                  : 3.14
// Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
// C/C++ source code generated on : Fri Jun 21 17:48:17 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "offline_snac_SITL.h"

// Block parameters (default storage)
P_offline_snac_SITL_T offline_snac_SITL_P = {
  // Mask Parameter: DiscreteDerivative2_ICPrevScale
  //  Referenced by: '<S31>/UD'

  0.0,

  // Mask Parameter: DiscreteDerivative_ICPrevScaled
  //  Referenced by: '<S29>/UD'

  0.0,

  // Mask Parameter: DiscreteDerivative1_ICPrevScale
  //  Referenced by: '<S30>/UD'

  0.0,

  // Mask Parameter: DiscreteDerivative_ICPrevScal_f
  //  Referenced by: '<S12>/UD'

  0.0,

  // Mask Parameter: DiscreteDerivative1_ICPrevSca_g
  //  Referenced by: '<S13>/UD'

  0.0,

  // Mask Parameter: DiscreteDerivative2_ICPrevSca_i
  //  Referenced by: '<S14>/UD'

  0.0,

  // Computed Parameter: Out1_Y0
  //  Referenced by: '<S17>/Out1'

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
    0.0,                               // vx
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
  //  Referenced by: '<S4>/Constant'

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
    0.0,                               // vx
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

  // Computed Parameter: Out1_Y0_g
  //  Referenced by: '<S18>/Out1'

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

  // Computed Parameter: Constant_Value_l
  //  Referenced by: '<S5>/Constant'

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

  // Computed Parameter: Out1_Y0_e
  //  Referenced by: '<S16>/Out1'

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

  // Computed Parameter: Constant_Value_m
  //  Referenced by: '<S3>/Constant'

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

  // Expression: 1
  //  Referenced by: '<S24>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S25>/Constant'

  1.0,

  // Computed Parameter: TSamp_WtEt
  //  Referenced by: '<S31>/TSamp'

  1000.0,

  // Computed Parameter: TSamp_WtEt_g
  //  Referenced by: '<S29>/TSamp'

  1000.0,

  // Computed Parameter: TSamp_WtEt_p
  //  Referenced by: '<S30>/TSamp'

  1000.0,

  // Computed Parameter: DiscreteTimeIntegrator_gainval
  //  Referenced by: '<Root>/Discrete-Time Integrator'

  0.001,

  // Expression: 0
  //  Referenced by: '<Root>/Discrete-Time Integrator'

  0.0,

  // Expression: [1 -1 1 1;1 1 -1 1;1 -1 -1 -1;1 1 1 -1]
  //  Referenced by: '<S10>/Mixer matrix'

  { 1.0, 1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, 1.0,
    -1.0, -1.0 },

  // Expression: 15
  //  Referenced by: '<S11>/Saturation2'

  15.0,

  // Expression: -15
  //  Referenced by: '<S11>/Saturation2'

  -15.0,

  // Expression: 15
  //  Referenced by: '<S6>/Saturation'

  15.0,

  // Expression: -15
  //  Referenced by: '<S6>/Saturation'

  -15.0,

  // Expression: 15
  //  Referenced by: '<S11>/Saturation1'

  15.0,

  // Expression: -15
  //  Referenced by: '<S11>/Saturation1'

  -15.0,

  // Expression: 15
  //  Referenced by: '<S6>/Saturation1'

  15.0,

  // Expression: -15
  //  Referenced by: '<S6>/Saturation1'

  -15.0,

  // Expression: 15
  //  Referenced by: '<S11>/Saturation'

  15.0,

  // Expression: -15
  //  Referenced by: '<S11>/Saturation'

  -15.0,

  // Expression: 15
  //  Referenced by: '<S6>/Saturation2'

  15.0,

  // Expression: -15
  //  Referenced by: '<S6>/Saturation2'

  -15.0,

  // Expression: 1
  //  Referenced by: '<S6>/Constant'

  1.0,

  // Computed Parameter: TSamp_WtEt_a
  //  Referenced by: '<S12>/TSamp'

  1000.0,

  // Computed Parameter: TSamp_WtEt_c
  //  Referenced by: '<S13>/TSamp'

  1000.0,

  // Computed Parameter: TSamp_WtEt_n
  //  Referenced by: '<S14>/TSamp'

  1000.0,

  // Expression: 1000
  //  Referenced by: '<S10>/Gain1'

  1000.0,

  // Expression: 1
  //  Referenced by: '<Root>/Constant'

  1.0,

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
