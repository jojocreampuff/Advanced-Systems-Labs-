//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: offline_snac_SITL_data.cpp
//
// Code generated for Simulink model 'offline_snac_SITL'.
//
// Model version                  : 3.25
// Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
// C/C++ source code generated on : Tue Jul  9 19:06:57 2024
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

  // Mask Parameter: DiscreteDerivative_ICPrevScal_o
  //  Referenced by: '<S30>/UD'

  0.0F,

  // Mask Parameter: DiscreteDerivative1_ICPrevSca_p
  //  Referenced by: '<S31>/UD'

  0.0F,

  // Mask Parameter: DiscreteDerivative2_ICPrevSca_g
  //  Referenced by: '<S32>/UD'

  0.0F,

  // Computed Parameter: Out1_Y0
  //  Referenced by: '<S18>/Out1'

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
  //  Referenced by: '<S6>/Constant'

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

  // Computed Parameter: Out1_Y0_g
  //  Referenced by: '<S19>/Out1'

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
  //  Referenced by: '<S7>/Constant'

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
  //  Referenced by: '<S17>/Out1'

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
  //  Referenced by: '<S5>/Constant'

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

  // Computed Parameter: Constant_Value_mk
  //  Referenced by: '<S25>/Constant'

  1.0F,

  // Computed Parameter: Constant_Value_h
  //  Referenced by: '<S26>/Constant'

  1.0F,

  // Computed Parameter: DiscreteTimeIntegrator_gainval
  //  Referenced by: '<Root>/Discrete-Time Integrator'

  0.001F,

  // Computed Parameter: DiscreteTimeIntegrator_IC
  //  Referenced by: '<Root>/Discrete-Time Integrator'

  0.0F,

  // Computed Parameter: Mixermatrix_Value
  //  Referenced by: '<S12>/Mixer matrix'

  { 1.0F, 1.0F, 1.0F, 1.0F, -1.0F, 1.0F, -1.0F, 1.0F, 1.0F, -1.0F, -1.0F, 1.0F,
    1.0F, 1.0F, -1.0F, -1.0F },

  // Computed Parameter: Saturation_UpperSat
  //  Referenced by: '<S8>/Saturation'

  15.0F,

  // Computed Parameter: Saturation_LowerSat
  //  Referenced by: '<S8>/Saturation'

  -15.0F,

  // Computed Parameter: Saturation1_UpperSat
  //  Referenced by: '<S8>/Saturation1'

  15.0F,

  // Computed Parameter: Saturation1_LowerSat
  //  Referenced by: '<S8>/Saturation1'

  -15.0F,

  // Computed Parameter: Saturation2_UpperSat
  //  Referenced by: '<S8>/Saturation2'

  15.0F,

  // Computed Parameter: Saturation2_LowerSat
  //  Referenced by: '<S8>/Saturation2'

  -15.0F,

  // Computed Parameter: Constant_Value_e
  //  Referenced by: '<S8>/Constant'

  1.0F,

  // Computed Parameter: Saturation3_UpperSat
  //  Referenced by: '<S8>/Saturation3'

  72.0F,

  // Computed Parameter: Saturation3_LowerSat
  //  Referenced by: '<S8>/Saturation3'

  0.0F,

  // Computed Parameter: Saturation4_UpperSat
  //  Referenced by: '<S8>/Saturation4'

  0.79F,

  // Computed Parameter: Saturation4_LowerSat
  //  Referenced by: '<S8>/Saturation4'

  -0.79F,

  // Computed Parameter: Saturation5_UpperSat
  //  Referenced by: '<S8>/Saturation5'

  0.79F,

  // Computed Parameter: Saturation5_LowerSat
  //  Referenced by: '<S8>/Saturation5'

  -0.79F,

  // Computed Parameter: Constant1_Value
  //  Referenced by: '<Root>/Constant1'

  0.0F,

  // Computed Parameter: TSamp_WtEt
  //  Referenced by: '<S13>/TSamp'

  1000.0F,

  // Computed Parameter: TSamp_WtEt_c
  //  Referenced by: '<S14>/TSamp'

  1000.0F,

  // Computed Parameter: TSamp_WtEt_n
  //  Referenced by: '<S15>/TSamp'

  1000.0F,

  // Computed Parameter: Saturation3_UpperSat_l
  //  Referenced by: '<S1>/Saturation3'

  1.0F,

  // Computed Parameter: Saturation3_LowerSat_k
  //  Referenced by: '<S1>/Saturation3'

  -1.0F,

  // Computed Parameter: Saturation2_UpperSat_a
  //  Referenced by: '<S1>/Saturation2'

  8.0F,

  // Computed Parameter: Saturation2_LowerSat_n
  //  Referenced by: '<S1>/Saturation2'

  -8.0F,

  // Computed Parameter: Saturation_UpperSat_m
  //  Referenced by: '<S1>/Saturation'

  8.0F,

  // Computed Parameter: Saturation_LowerSat_h
  //  Referenced by: '<S1>/Saturation'

  -8.0F,

  // Computed Parameter: Saturation1_UpperSat_o
  //  Referenced by: '<S1>/Saturation1'

  3.0F,

  // Computed Parameter: Saturation1_LowerSat_n
  //  Referenced by: '<S1>/Saturation1'

  -3.0F,

  // Computed Parameter: Gain1_Gain
  //  Referenced by: '<S12>/Gain1'

  1000.0F,

  // Computed Parameter: Constant_Value_lo
  //  Referenced by: '<Root>/Constant'

  1.0F,

  // Computed Parameter: TSamp_WtEt_g
  //  Referenced by: '<S30>/TSamp'

  1000.0F,

  // Computed Parameter: TSamp_WtEt_p
  //  Referenced by: '<S31>/TSamp'

  1000.0F,

  // Computed Parameter: TSamp_WtEt_a
  //  Referenced by: '<S32>/TSamp'

  1000.0F,

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
