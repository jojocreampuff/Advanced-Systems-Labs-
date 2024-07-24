//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: offline_snac_SITL.h
//
// Code generated for Simulink model 'offline_snac_SITL'.
//
// Model version                  : 3.26
// Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
// C/C++ source code generated on : Tue Jul 23 17:59:17 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef offline_snac_SITL_h_
#define offline_snac_SITL_h_
#include <poll.h>
#include <uORB/uORB.h>
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "MW_uORB_Read.h"
#include "MW_PX4_PWM.h"
#include "offline_snac_SITL_types.h"
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_gyro.h>

extern "C"
{

#include "rtGetInf.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

extern "C"
{

#include "rt_nonfinite.h"

}

#include <stddef.h>

// Macros for accessing real-time model data structure
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWExtModeInfo
#define rtmGetRTWExtModeInfo(rtm)      ((rtm)->extModeInfo)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                (&(rtm)->Timing.taskTime0)
#endif

// Block signals (default storage)
struct B_offline_snac_SITL_T {
  real32_T ref[300000];
  real32_T fv[84];
  px4_Bus_vehicle_local_position In1;  // '<S18>/In1'
  px4_Bus_vehicle_local_position r;
  px4_Bus_vehicle_attitude In1_b;      // '<S19>/In1'
  px4_Bus_vehicle_attitude r1;
  real32_T fv1[13];
  px4_Bus_sensor_gyro In1_l;           // '<S17>/In1'
  px4_Bus_sensor_gyro r2;
  real32_T fv2[9];
  uint16_T pwmValue[8];
  real32_T uxyz[3];
  uint16_T Gain1[4];                   // '<S12>/Gain1'
  real32_T DiscreteTimeIntegrator;     // '<Root>/Discrete-Time Integrator'
  real32_T SignalConversion;           // '<Root>/Signal Conversion'
  real32_T sensor_x;                   // '<S8>/Data Type Conversion3'
  real32_T sensor_y;                   // '<S8>/Data Type Conversion1'
  real32_T sensor_z;                   // '<S8>/Data Type Conversion2'
  real32_T sensor_vy;                  // '<S8>/Data Type Conversion6'
  real32_T sensor_vz;                  // '<S8>/Data Type Conversion4'
  real32_T r_xdot;                     // '<S8>/Saturation'
  real32_T r_ydot;                     // '<S8>/Saturation1'
  real32_T r_zdot;                     // '<S8>/Saturation2'
  real32_T pos_error[6];               // '<S8>/Subtract'
  real32_T Saturation3;                // '<S8>/Saturation3'
  real32_T SignalConversion1;          // '<Root>/Signal Conversion1'
  real32_T SignalConversion2;          // '<Root>/Signal Conversion2'
  real32_T Saturation4;                // '<S8>/Saturation4'
  real32_T Saturation5;                // '<S8>/Saturation5'
  real32_T Constant1;                  // '<Root>/Constant1'
  real32_T Diff;                       // '<S13>/Diff'
  real32_T Diff_f;                     // '<S14>/Diff'
  real32_T des_yaw_rate;               // '<S1>/Saturation3'
  real32_T att_error[6];               // '<S1>/Subtract'
  real32_T Saturation2;                // '<S1>/Saturation2'
  real32_T Saturation;                 // '<S1>/Saturation'
  real32_T Saturation1;                // '<S1>/Saturation1'
  real32_T PWM_output[4];              // '<S12>/Product1'
  real32_T z_test1;                    // '<S10>/Subtract'
  real32_T z_test2;                    // '<S10>/Subtract1'
  real32_T z_test3;                    // '<S10>/Subtract2'
  real32_T pitch_rate;                 // '<Root>/T_matrix'
  real32_T roll_rate;                  // '<Root>/T_matrix'
  real32_T yaw_rate;                   // '<Root>/T_matrix'
  real32_T ux;                         // '<S8>/MATLAB Function'
  real32_T uy;                         // '<S8>/MATLAB Function'
  real32_T uz;                         // '<S8>/MATLAB Function'
  real32_T ref_k[6];                   // '<Root>/MATLAB Function2'
  real32_T a;
  real32_T b;
  real32_T B;
  real32_T C;
  real32_T Product3;                   // '<S23>/Product3'
  real32_T Product2;                   // '<S23>/Product2'
  real32_T Product1;                   // '<S23>/Product1'
  real32_T fcn5;                       // '<S9>/fcn5'
  real32_T fcn3;                       // '<S9>/fcn3'
  boolean_T NOT;                       // '<S5>/NOT'
  boolean_T NOT_n;                     // '<S6>/NOT'
  boolean_T NOT_g;                     // '<S7>/NOT'
};

// Block states (default storage) for system '<Root>'
struct DW_offline_snac_SITL_T {
  px4_internal_block_Subscriber_T obj; // '<S7>/SourceBlock'
  px4_internal_block_Subscriber_T obj_k;// '<S6>/SourceBlock'
  px4_internal_block_Subscriber_T obj_f;// '<S5>/SourceBlock'
  px4_internal_block_PWM_offlin_T obj_d;// '<S12>/PX4 PWM Output'
  struct {
    void *LoggedData;
  } Pitch_rate_PWORK;                  // '<S10>/Pitch_rate??'

  struct {
    void *LoggedData;
  } roll_rate1_PWORK;                  // '<S10>/roll_rate??1'

  struct {
    void *LoggedData;
  } yaw_rate1_PWORK;                   // '<S10>/yaw_rate??1'

  real32_T DiscreteTimeIntegrator_DSTATE;// '<Root>/Discrete-Time Integrator'
  real32_T UD_DSTATE;                  // '<S13>/UD'
  real32_T UD_DSTATE_a;                // '<S14>/UD'
  real32_T UD_DSTATE_m;                // '<S15>/UD'
  real32_T UD_DSTATE_j;                // '<S30>/UD'
  real32_T UD_DSTATE_i;                // '<S31>/UD'
  real32_T UD_DSTATE_c;                // '<S32>/UD'
  int8_T IfActionSubsystem2_SubsysRanBC;// '<S24>/If Action Subsystem2'
  int8_T IfActionSubsystem1_SubsysRanBC;// '<S24>/If Action Subsystem1'
  int8_T IfActionSubsystem_SubsysRanBC;// '<S24>/If Action Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC; // '<S7>/Enabled Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC_m;// '<S6>/Enabled Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC_g;// '<S5>/Enabled Subsystem'
};

// Parameters (default storage)
struct P_offline_snac_SITL_T_ {
  real32_T DiscreteDerivative_ICPrevScaled;
                              // Mask Parameter: DiscreteDerivative_ICPrevScaled
                                 //  Referenced by: '<S13>/UD'

  real32_T DiscreteDerivative1_ICPrevScale;
                              // Mask Parameter: DiscreteDerivative1_ICPrevScale
                                 //  Referenced by: '<S14>/UD'

  real32_T DiscreteDerivative2_ICPrevScale;
                              // Mask Parameter: DiscreteDerivative2_ICPrevScale
                                 //  Referenced by: '<S15>/UD'

  real32_T DiscreteDerivative_ICPrevScal_o;
                              // Mask Parameter: DiscreteDerivative_ICPrevScal_o
                                 //  Referenced by: '<S30>/UD'

  real32_T DiscreteDerivative1_ICPrevSca_p;
                              // Mask Parameter: DiscreteDerivative1_ICPrevSca_p
                                 //  Referenced by: '<S31>/UD'

  real32_T DiscreteDerivative2_ICPrevSca_g;
                              // Mask Parameter: DiscreteDerivative2_ICPrevSca_g
                                 //  Referenced by: '<S32>/UD'

  px4_Bus_vehicle_local_position Out1_Y0;// Computed Parameter: Out1_Y0
                                            //  Referenced by: '<S18>/Out1'

  px4_Bus_vehicle_local_position Constant_Value;// Computed Parameter: Constant_Value
                                                   //  Referenced by: '<S6>/Constant'

  px4_Bus_vehicle_attitude Out1_Y0_g;  // Computed Parameter: Out1_Y0_g
                                          //  Referenced by: '<S19>/Out1'

  px4_Bus_vehicle_attitude Constant_Value_l;// Computed Parameter: Constant_Value_l
                                               //  Referenced by: '<S7>/Constant'

  px4_Bus_sensor_gyro Out1_Y0_e;       // Computed Parameter: Out1_Y0_e
                                          //  Referenced by: '<S17>/Out1'

  px4_Bus_sensor_gyro Constant_Value_m;// Computed Parameter: Constant_Value_m
                                          //  Referenced by: '<S5>/Constant'

  real32_T Constant_Value_mk;          // Computed Parameter: Constant_Value_mk
                                          //  Referenced by: '<S25>/Constant'

  real32_T Constant_Value_h;           // Computed Parameter: Constant_Value_h
                                          //  Referenced by: '<S26>/Constant'

  real32_T DiscreteTimeIntegrator_gainval;
                           // Computed Parameter: DiscreteTimeIntegrator_gainval
                              //  Referenced by: '<Root>/Discrete-Time Integrator'

  real32_T DiscreteTimeIntegrator_IC;
                                // Computed Parameter: DiscreteTimeIntegrator_IC
                                   //  Referenced by: '<Root>/Discrete-Time Integrator'

  real32_T Mixermatrix_Value[16];      // Computed Parameter: Mixermatrix_Value
                                          //  Referenced by: '<S12>/Mixer matrix'

  real32_T Saturation_UpperSat;       // Computed Parameter: Saturation_UpperSat
                                         //  Referenced by: '<S8>/Saturation'

  real32_T Saturation_LowerSat;       // Computed Parameter: Saturation_LowerSat
                                         //  Referenced by: '<S8>/Saturation'

  real32_T Saturation1_UpperSat;     // Computed Parameter: Saturation1_UpperSat
                                        //  Referenced by: '<S8>/Saturation1'

  real32_T Saturation1_LowerSat;     // Computed Parameter: Saturation1_LowerSat
                                        //  Referenced by: '<S8>/Saturation1'

  real32_T Saturation2_UpperSat;     // Computed Parameter: Saturation2_UpperSat
                                        //  Referenced by: '<S8>/Saturation2'

  real32_T Saturation2_LowerSat;     // Computed Parameter: Saturation2_LowerSat
                                        //  Referenced by: '<S8>/Saturation2'

  real32_T Constant_Value_e;           // Computed Parameter: Constant_Value_e
                                          //  Referenced by: '<S8>/Constant'

  real32_T Saturation3_UpperSat;     // Computed Parameter: Saturation3_UpperSat
                                        //  Referenced by: '<S8>/Saturation3'

  real32_T Saturation3_LowerSat;     // Computed Parameter: Saturation3_LowerSat
                                        //  Referenced by: '<S8>/Saturation3'

  real32_T Saturation4_UpperSat;     // Computed Parameter: Saturation4_UpperSat
                                        //  Referenced by: '<S8>/Saturation4'

  real32_T Saturation4_LowerSat;     // Computed Parameter: Saturation4_LowerSat
                                        //  Referenced by: '<S8>/Saturation4'

  real32_T Saturation5_UpperSat;     // Computed Parameter: Saturation5_UpperSat
                                        //  Referenced by: '<S8>/Saturation5'

  real32_T Saturation5_LowerSat;     // Computed Parameter: Saturation5_LowerSat
                                        //  Referenced by: '<S8>/Saturation5'

  real32_T Constant1_Value;            // Computed Parameter: Constant1_Value
                                          //  Referenced by: '<Root>/Constant1'

  real32_T TSamp_WtEt;                 // Computed Parameter: TSamp_WtEt
                                          //  Referenced by: '<S13>/TSamp'

  real32_T TSamp_WtEt_c;               // Computed Parameter: TSamp_WtEt_c
                                          //  Referenced by: '<S14>/TSamp'

  real32_T TSamp_WtEt_n;               // Computed Parameter: TSamp_WtEt_n
                                          //  Referenced by: '<S15>/TSamp'

  real32_T Saturation3_UpperSat_l; // Computed Parameter: Saturation3_UpperSat_l
                                      //  Referenced by: '<S1>/Saturation3'

  real32_T Saturation3_LowerSat_k; // Computed Parameter: Saturation3_LowerSat_k
                                      //  Referenced by: '<S1>/Saturation3'

  real32_T Saturation2_UpperSat_a; // Computed Parameter: Saturation2_UpperSat_a
                                      //  Referenced by: '<S1>/Saturation2'

  real32_T Saturation2_LowerSat_n; // Computed Parameter: Saturation2_LowerSat_n
                                      //  Referenced by: '<S1>/Saturation2'

  real32_T Saturation_UpperSat_m;   // Computed Parameter: Saturation_UpperSat_m
                                       //  Referenced by: '<S1>/Saturation'

  real32_T Saturation_LowerSat_h;   // Computed Parameter: Saturation_LowerSat_h
                                       //  Referenced by: '<S1>/Saturation'

  real32_T Saturation1_UpperSat_o; // Computed Parameter: Saturation1_UpperSat_o
                                      //  Referenced by: '<S1>/Saturation1'

  real32_T Saturation1_LowerSat_n; // Computed Parameter: Saturation1_LowerSat_n
                                      //  Referenced by: '<S1>/Saturation1'

  real32_T Gain1_Gain;                 // Computed Parameter: Gain1_Gain
                                          //  Referenced by: '<S12>/Gain1'

  real32_T Constant_Value_lo;          // Computed Parameter: Constant_Value_lo
                                          //  Referenced by: '<Root>/Constant'

  real32_T TSamp_WtEt_g;               // Computed Parameter: TSamp_WtEt_g
                                          //  Referenced by: '<S30>/TSamp'

  real32_T TSamp_WtEt_p;               // Computed Parameter: TSamp_WtEt_p
                                          //  Referenced by: '<S31>/TSamp'

  real32_T TSamp_WtEt_a;               // Computed Parameter: TSamp_WtEt_a
                                          //  Referenced by: '<S32>/TSamp'

  boolean_T Reset_Value;               // Computed Parameter: Reset_Value
                                          //  Referenced by: '<Root>/Reset'

  boolean_T DataStoreMemory_InitialValue;
                             // Computed Parameter: DataStoreMemory_InitialValue
                                //  Referenced by: '<Root>/Data Store Memory'

};

// Real-time Model Data Structure
struct tag_RTM_offline_snac_SITL_T {
  const char_T *errorStatus;
  RTWExtModeInfo *extModeInfo;

  //
  //  Sizes:
  //  The following substructure contains sizes information
  //  for many of the model attributes such as inputs, outputs,
  //  dwork, sample times, etc.

  struct {
    uint32_T checksums[4];
  } Sizes;

  //
  //  SpecialInfo:
  //  The following substructure contains special information
  //  related to other components that are dependent on RTW.

  struct {
    const void *mappingInfo;
  } SpecialInfo;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C"
{

#endif

  extern P_offline_snac_SITL_T offline_snac_SITL_P;

#ifdef __cplusplus

}

#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C"
{

#endif

  extern struct B_offline_snac_SITL_T offline_snac_SITL_B;

#ifdef __cplusplus

}

#endif

// Block states (default storage)
extern struct DW_offline_snac_SITL_T offline_snac_SITL_DW;

#ifdef __cplusplus

extern "C"
{

#endif

  // Model entry point functions
  extern void offline_snac_SITL_initialize(void);
  extern void offline_snac_SITL_step(void);
  extern void offline_snac_SITL_terminate(void);

#ifdef __cplusplus

}

#endif

// Real-time Model object
#ifdef __cplusplus

extern "C"
{

#endif

  extern RT_MODEL_offline_snac_SITL_T *const offline_snac_SITL_M;

#ifdef __cplusplus

}

#endif

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S13>/Data Type Duplicate' : Unused code path elimination
//  Block '<S14>/Data Type Duplicate' : Unused code path elimination
//  Block '<S15>/Data Type Duplicate' : Unused code path elimination
//  Block '<S30>/Data Type Duplicate' : Unused code path elimination
//  Block '<S31>/Data Type Duplicate' : Unused code path elimination
//  Block '<S32>/Data Type Duplicate' : Unused code path elimination
//  Block '<Root>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S8>/Data Type Conversion5' : Eliminate redundant data type conversion
//  Block '<S8>/Data Type Conversion7' : Eliminate redundant data type conversion


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'offline_snac_SITL'
//  '<S1>'   : 'offline_snac_SITL/Attitude controller'
//  '<S2>'   : 'offline_snac_SITL/MATLAB Function1'
//  '<S3>'   : 'offline_snac_SITL/MATLAB Function2'
//  '<S4>'   : 'offline_snac_SITL/MATLAB Function9'
//  '<S5>'   : 'offline_snac_SITL/PX4 uORB Read'
//  '<S6>'   : 'offline_snac_SITL/PX4 uORB Read1'
//  '<S7>'   : 'offline_snac_SITL/PX4 uORB Read2'
//  '<S8>'   : 'offline_snac_SITL/Position & Altitude controller'
//  '<S9>'   : 'offline_snac_SITL/Quaternions to Rotation Angles'
//  '<S10>'  : 'offline_snac_SITL/Subsystem'
//  '<S11>'  : 'offline_snac_SITL/T_matrix'
//  '<S12>'  : 'offline_snac_SITL/To Actuator'
//  '<S13>'  : 'offline_snac_SITL/Attitude controller/Discrete Derivative?'
//  '<S14>'  : 'offline_snac_SITL/Attitude controller/Discrete Derivative?1'
//  '<S15>'  : 'offline_snac_SITL/Attitude controller/Discrete Derivative?2'
//  '<S16>'  : 'offline_snac_SITL/Attitude controller/MATLAB Function3'
//  '<S17>'  : 'offline_snac_SITL/PX4 uORB Read/Enabled Subsystem'
//  '<S18>'  : 'offline_snac_SITL/PX4 uORB Read1/Enabled Subsystem'
//  '<S19>'  : 'offline_snac_SITL/PX4 uORB Read2/Enabled Subsystem'
//  '<S20>'  : 'offline_snac_SITL/Position & Altitude controller/MATLAB Function'
//  '<S21>'  : 'offline_snac_SITL/Position & Altitude controller/MATLAB Function1'
//  '<S22>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation'
//  '<S23>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Quaternion Normalize'
//  '<S24>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input'
//  '<S25>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem'
//  '<S26>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem1'
//  '<S27>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem2'
//  '<S28>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus'
//  '<S29>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S30>'  : 'offline_snac_SITL/Subsystem/Discrete Derivative'
//  '<S31>'  : 'offline_snac_SITL/Subsystem/Discrete Derivative1'
//  '<S32>'  : 'offline_snac_SITL/Subsystem/Discrete Derivative2'

#endif                                 // offline_snac_SITL_h_

//
// File trailer for generated code.
//
// [EOF]
//
