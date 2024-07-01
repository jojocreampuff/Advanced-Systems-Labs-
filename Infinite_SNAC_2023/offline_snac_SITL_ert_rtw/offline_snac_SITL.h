//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: offline_snac_SITL.h
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
  real_T dv[84];
  px4_Bus_vehicle_local_position In1;  // '<S17>/In1'
  px4_Bus_vehicle_local_position r;
  px4_Bus_vehicle_attitude In1_b;      // '<S18>/In1'
  px4_Bus_vehicle_attitude r1;
  px4_Bus_sensor_gyro In1_l;           // '<S16>/In1'
  px4_Bus_sensor_gyro r2;
  real32_T fv[9];
  real_T uxyz[3];
  uint16_T pwmValue[8];
  real32_T w[3];
  real_T SignalConversion;             // '<Root>/Signal Conversion'
  real_T SignalConversion1;            // '<Root>/Signal Conversion1'
  real_T SignalConversion2;            // '<Root>/Signal Conversion2'
  real_T Subtract2;                    // '<S8>/Subtract2'
  real_T Subtract;                     // '<S8>/Subtract'
  real_T Subtract1;                    // '<S8>/Subtract1'
  real_T DiscreteTimeIntegrator;       // '<Root>/Discrete-Time Integrator'
  real_T sensor_x;                     // '<S6>/Data Type Conversion3'
  real_T sensor_y;                     // '<S6>/Data Type Conversion1'
  real_T sensor_z;                     // '<S6>/Data Type Conversion2'
  real_T sensor_vy;                    // '<S6>/Data Type Conversion6'
  real_T sensor_vz;                    // '<S6>/Data Type Conversion4'
  real_T Saturation2;                  // '<S11>/Saturation2'
  real_T r_xdot;                       // '<S6>/Saturation'
  real_T Saturation1;                  // '<S11>/Saturation1'
  real_T r_ydot;                       // '<S6>/Saturation1'
  real_T Saturation;                   // '<S11>/Saturation'
  real_T r_zdot;                       // '<S6>/Saturation2'
  real_T pos_error[6];                 // '<S6>/Subtract'
  real_T att_error[6];                 // '<S1>/Subtract'
  real_T ref_x;                        // '<S11>/MATLAB Function'
  real_T ref_y;                        // '<S11>/MATLAB Function'
  real_T ref_z;                        // '<S11>/MATLAB Function'
  real_T ft;                           // '<S6>/MATLAB Function1'
  real_T pitch;                        // '<S6>/MATLAB Function1'
  real_T roll;                         // '<S6>/MATLAB Function1'
  real_T ux;                           // '<S6>/MATLAB Function'
  real_T uy;                           // '<S6>/MATLAB Function'
  real_T uz;                           // '<S6>/MATLAB Function'
  real_T tau_x;                        // '<S1>/MATLAB Function3'
  real_T tau_y;                        // '<S1>/MATLAB Function3'
  real_T tau_z;                        // '<S1>/MATLAB Function3'
  real_T B;
  real_T C;
  real_T DataTypeConversion4;          // '<Root>/Data Type Conversion4'
  real_T DataTypeConversion3;          // '<Root>/Data Type Conversion3'
  real_T DataTypeConversion2;          // '<Root>/Data Type Conversion2'
  real_T Diff_k;                       // '<S14>/Diff'
  real_T fcn3;                         // '<S7>/fcn3'
  real_T TSamp;                        // '<S31>/TSamp'
  real_T SignalConversion_tmp;
  real_T SignalConversion_tmp_m;
  real_T d;
  real_T d1;
  real_T d2;
  real_T d3;
  real_T d4;
  real_T d5;
  real_T d6;
  real_T d7;
  uint16_T Gain1[4];                   // '<S10>/Gain1'
  boolean_T NOT;                       // '<S3>/NOT'
  boolean_T NOT_n;                     // '<S4>/NOT'
  boolean_T NOT_g;                     // '<S5>/NOT'
};

// Block states (default storage) for system '<Root>'
struct DW_offline_snac_SITL_T {
  px4_internal_block_Subscriber_T obj; // '<S5>/SourceBlock'
  px4_internal_block_Subscriber_T obj_k;// '<S4>/SourceBlock'
  px4_internal_block_Subscriber_T obj_f;// '<S3>/SourceBlock'
  px4_internal_block_PWM_offlin_T obj_d;// '<S10>/PX4 PWM Output'
  real_T UD_DSTATE;                    // '<S31>/UD'
  real_T UD_DSTATE_j;                  // '<S29>/UD'
  real_T UD_DSTATE_i;                  // '<S30>/UD'
  real_T DiscreteTimeIntegrator_DSTATE;// '<Root>/Discrete-Time Integrator'
  real_T UD_DSTATE_o;                  // '<S12>/UD'
  real_T UD_DSTATE_a;                  // '<S13>/UD'
  real_T UD_DSTATE_m;                  // '<S14>/UD'
  struct {
    void *LoggedData;
  } yaw_rate1_PWORK;                   // '<S8>/yaw_rate??1'

  struct {
    void *LoggedData;
  } Pitch_rate_PWORK;                  // '<S8>/Pitch_rate??'

  struct {
    void *LoggedData;
  } roll_rate1_PWORK;                  // '<S8>/roll_rate??1'

  int8_T IfActionSubsystem2_SubsysRanBC;// '<S23>/If Action Subsystem2'
  int8_T IfActionSubsystem1_SubsysRanBC;// '<S23>/If Action Subsystem1'
  int8_T IfActionSubsystem_SubsysRanBC;// '<S23>/If Action Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC; // '<S5>/Enabled Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC_m;// '<S4>/Enabled Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC_g;// '<S3>/Enabled Subsystem'
};

// Parameters (default storage)
struct P_offline_snac_SITL_T_ {
  real_T DiscreteDerivative2_ICPrevScale;
                              // Mask Parameter: DiscreteDerivative2_ICPrevScale
                                 //  Referenced by: '<S31>/UD'

  real_T DiscreteDerivative_ICPrevScaled;
                              // Mask Parameter: DiscreteDerivative_ICPrevScaled
                                 //  Referenced by: '<S29>/UD'

  real_T DiscreteDerivative1_ICPrevScale;
                              // Mask Parameter: DiscreteDerivative1_ICPrevScale
                                 //  Referenced by: '<S30>/UD'

  real_T DiscreteDerivative_ICPrevScal_f;
                              // Mask Parameter: DiscreteDerivative_ICPrevScal_f
                                 //  Referenced by: '<S12>/UD'

  real_T DiscreteDerivative1_ICPrevSca_g;
                              // Mask Parameter: DiscreteDerivative1_ICPrevSca_g
                                 //  Referenced by: '<S13>/UD'

  real_T DiscreteDerivative2_ICPrevSca_i;
                              // Mask Parameter: DiscreteDerivative2_ICPrevSca_i
                                 //  Referenced by: '<S14>/UD'

  px4_Bus_vehicle_local_position Out1_Y0;// Computed Parameter: Out1_Y0
                                            //  Referenced by: '<S17>/Out1'

  px4_Bus_vehicle_local_position Constant_Value;// Computed Parameter: Constant_Value
                                                   //  Referenced by: '<S4>/Constant'

  px4_Bus_vehicle_attitude Out1_Y0_g;  // Computed Parameter: Out1_Y0_g
                                          //  Referenced by: '<S18>/Out1'

  px4_Bus_vehicle_attitude Constant_Value_l;// Computed Parameter: Constant_Value_l
                                               //  Referenced by: '<S5>/Constant'

  px4_Bus_sensor_gyro Out1_Y0_e;       // Computed Parameter: Out1_Y0_e
                                          //  Referenced by: '<S16>/Out1'

  px4_Bus_sensor_gyro Constant_Value_m;// Computed Parameter: Constant_Value_m
                                          //  Referenced by: '<S3>/Constant'

  real_T Constant_Value_mk;            // Expression: 1
                                          //  Referenced by: '<S24>/Constant'

  real_T Constant_Value_h;             // Expression: 1
                                          //  Referenced by: '<S25>/Constant'

  real_T TSamp_WtEt;                   // Computed Parameter: TSamp_WtEt
                                          //  Referenced by: '<S31>/TSamp'

  real_T TSamp_WtEt_g;                 // Computed Parameter: TSamp_WtEt_g
                                          //  Referenced by: '<S29>/TSamp'

  real_T TSamp_WtEt_p;                 // Computed Parameter: TSamp_WtEt_p
                                          //  Referenced by: '<S30>/TSamp'

  real_T DiscreteTimeIntegrator_gainval;
                           // Computed Parameter: DiscreteTimeIntegrator_gainval
                              //  Referenced by: '<Root>/Discrete-Time Integrator'

  real_T DiscreteTimeIntegrator_IC;    // Expression: 0
                                          //  Referenced by: '<Root>/Discrete-Time Integrator'

  real_T Mixermatrix_Value[16];
                          // Expression: [1 -1 1 1;1 1 -1 1;1 -1 -1 -1;1 1 1 -1]
                             //  Referenced by: '<S10>/Mixer matrix'

  real_T Saturation2_UpperSat;         // Expression: 15
                                          //  Referenced by: '<S11>/Saturation2'

  real_T Saturation2_LowerSat;         // Expression: -15
                                          //  Referenced by: '<S11>/Saturation2'

  real_T Saturation_UpperSat;          // Expression: 15
                                          //  Referenced by: '<S6>/Saturation'

  real_T Saturation_LowerSat;          // Expression: -15
                                          //  Referenced by: '<S6>/Saturation'

  real_T Saturation1_UpperSat;         // Expression: 15
                                          //  Referenced by: '<S11>/Saturation1'

  real_T Saturation1_LowerSat;         // Expression: -15
                                          //  Referenced by: '<S11>/Saturation1'

  real_T Saturation1_UpperSat_k;       // Expression: 15
                                          //  Referenced by: '<S6>/Saturation1'

  real_T Saturation1_LowerSat_a;       // Expression: -15
                                          //  Referenced by: '<S6>/Saturation1'

  real_T Saturation_UpperSat_d;        // Expression: 15
                                          //  Referenced by: '<S11>/Saturation'

  real_T Saturation_LowerSat_j;        // Expression: -15
                                          //  Referenced by: '<S11>/Saturation'

  real_T Saturation2_UpperSat_d;       // Expression: 15
                                          //  Referenced by: '<S6>/Saturation2'

  real_T Saturation2_LowerSat_h;       // Expression: -15
                                          //  Referenced by: '<S6>/Saturation2'

  real_T Constant_Value_e;             // Expression: 1
                                          //  Referenced by: '<S6>/Constant'

  real_T TSamp_WtEt_a;                 // Computed Parameter: TSamp_WtEt_a
                                          //  Referenced by: '<S12>/TSamp'

  real_T TSamp_WtEt_c;                 // Computed Parameter: TSamp_WtEt_c
                                          //  Referenced by: '<S13>/TSamp'

  real_T TSamp_WtEt_n;                 // Computed Parameter: TSamp_WtEt_n
                                          //  Referenced by: '<S14>/TSamp'

  real_T Gain1_Gain;                   // Expression: 1000
                                          //  Referenced by: '<S10>/Gain1'

  real_T Constant_Value_o;             // Expression: 1
                                          //  Referenced by: '<Root>/Constant'

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
//  Block '<S12>/Data Type Duplicate' : Unused code path elimination
//  Block '<S13>/Data Type Duplicate' : Unused code path elimination
//  Block '<S14>/Data Type Duplicate' : Unused code path elimination
//  Block '<S29>/Data Type Duplicate' : Unused code path elimination
//  Block '<S30>/Data Type Duplicate' : Unused code path elimination
//  Block '<S31>/Data Type Duplicate' : Unused code path elimination
//  Block '<S1>/Data Type Conversion2' : Eliminate redundant data type conversion
//  Block '<S1>/Data Type Conversion4' : Eliminate redundant data type conversion
//  Block '<S1>/Data Type Conversion5' : Eliminate redundant data type conversion
//  Block '<S6>/Data Type Conversion5' : Eliminate redundant data type conversion
//  Block '<S6>/Data Type Conversion7' : Eliminate redundant data type conversion
//  Block '<S6>/Data Type Conversion8' : Eliminate redundant data type conversion


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
//  '<S2>'   : 'offline_snac_SITL/MATLAB Function9'
//  '<S3>'   : 'offline_snac_SITL/PX4 uORB Read'
//  '<S4>'   : 'offline_snac_SITL/PX4 uORB Read1'
//  '<S5>'   : 'offline_snac_SITL/PX4 uORB Read2'
//  '<S6>'   : 'offline_snac_SITL/Position & Altitude controller'
//  '<S7>'   : 'offline_snac_SITL/Quaternions to Rotation Angles'
//  '<S8>'   : 'offline_snac_SITL/Subsystem'
//  '<S9>'   : 'offline_snac_SITL/T_matrix'
//  '<S10>'  : 'offline_snac_SITL/To Actuator'
//  '<S11>'  : 'offline_snac_SITL/signal generator'
//  '<S12>'  : 'offline_snac_SITL/Attitude controller/Discrete Derivative?'
//  '<S13>'  : 'offline_snac_SITL/Attitude controller/Discrete Derivative?1'
//  '<S14>'  : 'offline_snac_SITL/Attitude controller/Discrete Derivative?2'
//  '<S15>'  : 'offline_snac_SITL/Attitude controller/MATLAB Function3'
//  '<S16>'  : 'offline_snac_SITL/PX4 uORB Read/Enabled Subsystem'
//  '<S17>'  : 'offline_snac_SITL/PX4 uORB Read1/Enabled Subsystem'
//  '<S18>'  : 'offline_snac_SITL/PX4 uORB Read2/Enabled Subsystem'
//  '<S19>'  : 'offline_snac_SITL/Position & Altitude controller/MATLAB Function'
//  '<S20>'  : 'offline_snac_SITL/Position & Altitude controller/MATLAB Function1'
//  '<S21>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation'
//  '<S22>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Quaternion Normalize'
//  '<S23>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input'
//  '<S24>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem'
//  '<S25>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem1'
//  '<S26>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem2'
//  '<S27>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus'
//  '<S28>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S29>'  : 'offline_snac_SITL/Subsystem/Discrete Derivative'
//  '<S30>'  : 'offline_snac_SITL/Subsystem/Discrete Derivative1'
//  '<S31>'  : 'offline_snac_SITL/Subsystem/Discrete Derivative2'
//  '<S32>'  : 'offline_snac_SITL/signal generator/MATLAB Function'
//  '<S33>'  : 'offline_snac_SITL/signal generator/MATLAB Function1'

#endif                                 // offline_snac_SITL_h_

//
// File trailer for generated code.
//
// [EOF]
//
