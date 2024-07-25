//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: offline_snac_SITL_types.h
//
// Code generated for Simulink model 'offline_snac_SITL'.
//
// Model version                  : 3.30
// Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
// C/C++ source code generated on : Wed Jul 24 18:49:50 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef offline_snac_SITL_types_h_
#define offline_snac_SITL_types_h_
#include "rtwtypes.h"
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#ifndef struct_e_px4_internal_block_SampleTi_T
#define struct_e_px4_internal_block_SampleTi_T

struct e_px4_internal_block_SampleTi_T
{
  int32_T __dummy;
};

#endif                                // struct_e_px4_internal_block_SampleTi_T

#ifndef struct_px4_internal_block_Subscriber_T
#define struct_px4_internal_block_Subscriber_T

struct px4_internal_block_Subscriber_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  e_px4_internal_block_SampleTi_T SampleTimeHandler;
  pollfd_t eventStructObj;
  orb_metadata_t * orbMetadataObj;
};

#endif                                // struct_px4_internal_block_Subscriber_T

#ifndef struct_px4_internal_block_Publisher__T
#define struct_px4_internal_block_Publisher__T

struct px4_internal_block_Publisher__T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  orb_advert_t orbAdvertiseObj;
  orb_metadata_t * orbMetadataObj;
};

#endif                                // struct_px4_internal_block_Publisher__T

#ifndef struct_px4_internal_block_PWM_offlin_T
#define struct_px4_internal_block_PWM_offlin_T

struct px4_internal_block_PWM_offlin_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  unsigned int servoCount;
  int channelMask;
  boolean_T isMain;
  orb_advert_t armAdvertiseObj;
  orb_advert_t actuatorAdvertiseObj;
  boolean_T isArmed;
};

#endif                                // struct_px4_internal_block_PWM_offlin_T

// Parameters (default storage)
typedef struct P_offline_snac_SITL_T_ P_offline_snac_SITL_T;

// Forward declaration for rtModel
typedef struct tag_RTM_offline_snac_SITL_T RT_MODEL_offline_snac_SITL_T;

#endif                                 // offline_snac_SITL_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
