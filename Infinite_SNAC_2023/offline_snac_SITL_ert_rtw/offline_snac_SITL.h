//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: offline_snac_SITL.h
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
#include "MW_uORB_Write.h"
#include "MW_PX4_PWM.h"
#include "offline_snac_SITL_types.h"
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
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
  real32_T fv[172];
  px4_Bus_vehicle_local_position In1;  // '<S21>/In1'
  px4_Bus_vehicle_local_position r;
  px4_Bus_vehicle_local_position_setpoint BusAssignment;// '<S3>/Bus Assignment' 
  px4_Bus_vehicle_local_position_setpoint In1_d;// '<S23>/In1'
  px4_Bus_vehicle_local_position_setpoint r1;
  px4_Bus_vehicle_attitude In1_l;      // '<S22>/In1'
  px4_Bus_vehicle_attitude r2;
  real32_T fv1[13];
  px4_Bus_sensor_gyro In1_ly;          // '<S20>/In1'
  px4_Bus_sensor_gyro r3;
  real32_T fv2[9];
  uint16_T pwmValue[8];
  real32_T uxyz[3];
  real32_T yaw;// '<Root>/SigConversion_InsertedFor_Bus Selector3_at_outport_6'
  real32_T sensor_x;                   // '<S9>/Data Type Conversion3'
  real32_T sensor_y;                   // '<S9>/Data Type Conversion1'
  real32_T sensor_z;                   // '<S9>/Product'
  real32_T sensor_vx;                  // '<S9>/Data Type Conversion5'
  real32_T sensor_vy;                  // '<S9>/Data Type Conversion6'
  real32_T sensor_vz;                  // '<S9>/Data Type Conversion4'
  real32_T r_x;                        // '<S9>/Rate Limiter3'
  real32_T r_y;                        // '<S9>/Rate Limiter4'
  real32_T r_z;                        // '<S9>/Rate Limiter5'
  real32_T r_xdot;                     // '<S9>/Rate Limiter'
  real32_T r_ydot;                     // '<S9>/Rate Limiter1'
  real32_T r_zdot;                     // '<S9>/Rate Limiter2'
  real32_T pos_error[6];               // '<S9>/Subtract'
  real32_T SignalConversion1;          // '<Root>/Signal Conversion1'
  real32_T SignalConversion2;          // '<Root>/Signal Conversion2'
  real32_T SignalConversion;           // '<Root>/Signal Conversion'
  real32_T des_pitch_rate;             // '<S1>/Rate Limiter3'
  real32_T des_roll_rate;              // '<S1>/Rate Limiter4'
  real32_T att_error[6];               // '<S1>/Subtract'
  real32_T pitch_rate;                 // '<Root>/T_matrix'
  real32_T roll_rate;                  // '<Root>/T_matrix'
  real32_T yaw_rate;                   // '<Root>/T_matrix'
  real32_T ft;                         // '<S9>/MATLAB Function1'
  real32_T pitch;                      // '<S9>/MATLAB Function1'
  real32_T roll;                       // '<S9>/MATLAB Function1'
  real32_T ux;                         // '<S9>/MATLAB Function'
  real32_T uy;                         // '<S9>/MATLAB Function'
  real32_T uz;                         // '<S9>/MATLAB Function'
  real32_T tau_x;                      // '<S1>/MATLAB Function3'
  real32_T tau_y;                      // '<S1>/MATLAB Function3'
  real32_T tau_z;                      // '<S1>/MATLAB Function3'
  real32_T a;
  real32_T b;
  real32_T B;
  real32_T C;
  real32_T T3;
  real32_T T4;
  real32_T Saturation5;                // '<S9>/Saturation5'
  real32_T SignalConversion2_tmp;
  real32_T f;
  real32_T f1;
  real32_T f2;
  real32_T f3;
  real32_T f4;
  real32_T f5;
  real32_T f6;
  real32_T f7;
  real32_T f8;
  real32_T f9;
  real32_T f10;
  real32_T f11;
  real32_T f12;
  real32_T f13;
  real32_T f14;
  real32_T f15;
  real32_T f16;
  int32_T i;
  int32_T i1;
  int32_T qY;
  uint16_T ch1;                        // '<S12>/MATLAB Function'
  uint16_T ch2;                        // '<S12>/MATLAB Function'
  uint16_T ch3;                        // '<S12>/MATLAB Function'
  uint16_T ch4;                        // '<S12>/MATLAB Function'
  boolean_T NOT;                       // '<S4>/NOT'
  boolean_T NOT_j;                     // '<S5>/NOT'
  boolean_T NOT_b;                     // '<S6>/NOT'
  boolean_T NOT_h;                     // '<S7>/NOT'
};

// Block states (default storage) for system '<Root>'
struct DW_offline_snac_SITL_T {
  px4_internal_block_Subscriber_T obj; // '<S7>/SourceBlock'
  px4_internal_block_Subscriber_T obj_d;// '<S6>/SourceBlock'
  px4_internal_block_Subscriber_T obj_e;// '<S5>/SourceBlock'
  px4_internal_block_Subscriber_T obj_o;// '<S4>/SourceBlock'
  px4_internal_block_Publisher__T obj_dt;// '<S8>/SinkBlock'
  px4_internal_block_PWM_offlin_T obj_i;// '<S12>/PX4 PWM Output'
  real32_T UD_DSTATE;                  // '<S13>/UD'
  real32_T UD_DSTATE_g;                // '<S14>/UD'
  real32_T UD_DSTATE_i;                // '<S15>/UD'
  real32_T DiscreteTimeIntegrator_DSTATE;// '<S3>/Discrete-Time Integrator'
  real32_T PrevY;                      // '<S9>/Rate Limiter3'
  real32_T PrevY_p;                    // '<S9>/Rate Limiter4'
  real32_T PrevY_a;                    // '<S9>/Rate Limiter5'
  real32_T PrevY_f;                    // '<S9>/Rate Limiter'
  real32_T PrevY_n;                    // '<S9>/Rate Limiter1'
  real32_T PrevY_d;                    // '<S9>/Rate Limiter2'
  real32_T PrevY_k;                    // '<S1>/Rate Limiter3'
  real32_T PrevY_c;                    // '<S1>/Rate Limiter4'
  real32_T PrevY_i;                    // '<S1>/Rate Limiter5'
  int8_T IfActionSubsystem2_SubsysRanBC;// '<S28>/If Action Subsystem2'
  int8_T IfActionSubsystem1_SubsysRanBC;// '<S28>/If Action Subsystem1'
  int8_T IfActionSubsystem_SubsysRanBC;// '<S28>/If Action Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC; // '<S7>/Enabled Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC_g;// '<S6>/Enabled Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC_p;// '<S5>/Enabled Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC_l;// '<S4>/Enabled Subsystem'
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

  px4_Bus_vehicle_local_position Out1_Y0;// Computed Parameter: Out1_Y0
                                            //  Referenced by: '<S21>/Out1'

  px4_Bus_vehicle_local_position Constant_Value;// Computed Parameter: Constant_Value
                                                   //  Referenced by: '<S5>/Constant'

  px4_Bus_vehicle_local_position_setpoint Out1_Y0_f;// Computed Parameter: Out1_Y0_f
                                                       //  Referenced by: '<S23>/Out1'

  px4_Bus_vehicle_local_position_setpoint Constant_Value_p;// Computed Parameter: Constant_Value_p
                                                              //  Referenced by: '<S7>/Constant'

  px4_Bus_vehicle_local_position_setpoint Constant_Value_o;// Computed Parameter: Constant_Value_o
                                                              //  Referenced by: '<S19>/Constant'

  px4_Bus_vehicle_attitude Out1_Y0_h;  // Computed Parameter: Out1_Y0_h
                                          //  Referenced by: '<S22>/Out1'

  px4_Bus_vehicle_attitude Constant_Value_a;// Computed Parameter: Constant_Value_a
                                               //  Referenced by: '<S6>/Constant'

  px4_Bus_sensor_gyro Out1_Y0_hw;      // Computed Parameter: Out1_Y0_hw
                                          //  Referenced by: '<S20>/Out1'

  px4_Bus_sensor_gyro Constant_Value_e;// Computed Parameter: Constant_Value_e
                                          //  Referenced by: '<S4>/Constant'

  real_T RateLimiter3_RisingLim;       // Expression: 15
                                          //  Referenced by: '<S9>/Rate Limiter3'

  real_T RateLimiter3_FallingLim;      // Expression: -15
                                          //  Referenced by: '<S9>/Rate Limiter3'

  real_T RateLimiter4_RisingLim;       // Expression: 15
                                          //  Referenced by: '<S9>/Rate Limiter4'

  real_T RateLimiter4_FallingLim;      // Expression: -15
                                          //  Referenced by: '<S9>/Rate Limiter4'

  real_T RateLimiter5_RisingLim;       // Expression: 15
                                          //  Referenced by: '<S9>/Rate Limiter5'

  real_T RateLimiter5_FallingLim;      // Expression: -15
                                          //  Referenced by: '<S9>/Rate Limiter5'

  real_T RateLimiter_RisingLim;        // Expression: 15
                                          //  Referenced by: '<S9>/Rate Limiter'

  real_T RateLimiter_FallingLim;       // Expression: -15
                                          //  Referenced by: '<S9>/Rate Limiter'

  real_T RateLimiter1_RisingLim;       // Expression: 15
                                          //  Referenced by: '<S9>/Rate Limiter1'

  real_T RateLimiter1_FallingLim;      // Expression: -15
                                          //  Referenced by: '<S9>/Rate Limiter1'

  real_T RateLimiter2_RisingLim;       // Expression: 15
                                          //  Referenced by: '<S9>/Rate Limiter2'

  real_T RateLimiter2_FallingLim;      // Expression: -15
                                          //  Referenced by: '<S9>/Rate Limiter2'

  real_T RateLimiter3_RisingLim_o;     // Expression: 2
                                          //  Referenced by: '<S1>/Rate Limiter3'

  real_T RateLimiter3_FallingLim_p;    // Expression: -2
                                          //  Referenced by: '<S1>/Rate Limiter3'

  real_T RateLimiter4_RisingLim_c;     // Expression: 2
                                          //  Referenced by: '<S1>/Rate Limiter4'

  real_T RateLimiter4_FallingLim_h;    // Expression: -2
                                          //  Referenced by: '<S1>/Rate Limiter4'

  real_T RateLimiter5_RisingLim_k;     // Expression: 2
                                          //  Referenced by: '<S1>/Rate Limiter5'

  real_T RateLimiter5_FallingLim_a;    // Expression: -2
                                          //  Referenced by: '<S1>/Rate Limiter5'

  real32_T Constant_Value_k;           // Computed Parameter: Constant_Value_k
                                          //  Referenced by: '<S29>/Constant'

  real32_T Constant_Value_d;           // Computed Parameter: Constant_Value_d
                                          //  Referenced by: '<S30>/Constant'

  real32_T Constant1_Value;            // Computed Parameter: Constant1_Value
                                          //  Referenced by: '<S9>/Constant1'

  real32_T RateLimiter3_IC;            // Computed Parameter: RateLimiter3_IC
                                          //  Referenced by: '<S9>/Rate Limiter3'

  real32_T RateLimiter4_IC;            // Computed Parameter: RateLimiter4_IC
                                          //  Referenced by: '<S9>/Rate Limiter4'

  real32_T RateLimiter5_IC;            // Computed Parameter: RateLimiter5_IC
                                          //  Referenced by: '<S9>/Rate Limiter5'

  real32_T RateLimiter_IC;             // Computed Parameter: RateLimiter_IC
                                          //  Referenced by: '<S9>/Rate Limiter'

  real32_T RateLimiter1_IC;            // Computed Parameter: RateLimiter1_IC
                                          //  Referenced by: '<S9>/Rate Limiter1'

  real32_T RateLimiter2_IC;            // Computed Parameter: RateLimiter2_IC
                                          //  Referenced by: '<S9>/Rate Limiter2'

  real32_T Saturation3_UpperSat;     // Computed Parameter: Saturation3_UpperSat
                                        //  Referenced by: '<S9>/Saturation3'

  real32_T Saturation3_LowerSat;     // Computed Parameter: Saturation3_LowerSat
                                        //  Referenced by: '<S9>/Saturation3'

  real32_T Saturation4_UpperSat;     // Computed Parameter: Saturation4_UpperSat
                                        //  Referenced by: '<S9>/Saturation4'

  real32_T Saturation4_LowerSat;     // Computed Parameter: Saturation4_LowerSat
                                        //  Referenced by: '<S9>/Saturation4'

  real32_T Saturation5_UpperSat;     // Computed Parameter: Saturation5_UpperSat
                                        //  Referenced by: '<S9>/Saturation5'

  real32_T Saturation5_LowerSat;     // Computed Parameter: Saturation5_LowerSat
                                        //  Referenced by: '<S9>/Saturation5'

  real32_T TSamp_WtEt;                 // Computed Parameter: TSamp_WtEt
                                          //  Referenced by: '<S13>/TSamp'

  real32_T RateLimiter3_IC_b;          // Computed Parameter: RateLimiter3_IC_b
                                          //  Referenced by: '<S1>/Rate Limiter3'

  real32_T TSamp_WtEt_c;               // Computed Parameter: TSamp_WtEt_c
                                          //  Referenced by: '<S14>/TSamp'

  real32_T RateLimiter4_IC_i;          // Computed Parameter: RateLimiter4_IC_i
                                          //  Referenced by: '<S1>/Rate Limiter4'

  real32_T TSamp_WtEt_h;               // Computed Parameter: TSamp_WtEt_h
                                          //  Referenced by: '<S15>/TSamp'

  real32_T RateLimiter5_IC_k;          // Computed Parameter: RateLimiter5_IC_k
                                          //  Referenced by: '<S1>/Rate Limiter5'

  real32_T DiscreteTimeIntegrator_gainval;
                           // Computed Parameter: DiscreteTimeIntegrator_gainval
                              //  Referenced by: '<S3>/Discrete-Time Integrator'

  real32_T DiscreteTimeIntegrator_IC;
                                // Computed Parameter: DiscreteTimeIntegrator_IC
                                   //  Referenced by: '<S3>/Discrete-Time Integrator'

  real32_T Constant2_Value;            // Computed Parameter: Constant2_Value
                                          //  Referenced by: '<S3>/Constant2'

  real32_T Constant_Value_g;           // Computed Parameter: Constant_Value_g
                                          //  Referenced by: '<S3>/Constant'

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
//  Block '<Root>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S9>/Data Type Conversion2' : Eliminate redundant data type conversion


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
//  '<S3>'   : 'offline_snac_SITL/Off Boarded Trajectory Stream'
//  '<S4>'   : 'offline_snac_SITL/PX4 uORB Read'
//  '<S5>'   : 'offline_snac_SITL/PX4 uORB Read1'
//  '<S6>'   : 'offline_snac_SITL/PX4 uORB Read2'
//  '<S7>'   : 'offline_snac_SITL/PX4 uORB Read3'
//  '<S8>'   : 'offline_snac_SITL/PX4 uORB Write'
//  '<S9>'   : 'offline_snac_SITL/Position '
//  '<S10>'  : 'offline_snac_SITL/Quaternions to Rotation Angles'
//  '<S11>'  : 'offline_snac_SITL/T_matrix'
//  '<S12>'  : 'offline_snac_SITL/To Actuator'
//  '<S13>'  : 'offline_snac_SITL/Attitude controller/Discrete Derivative?'
//  '<S14>'  : 'offline_snac_SITL/Attitude controller/Discrete Derivative?1'
//  '<S15>'  : 'offline_snac_SITL/Attitude controller/Discrete Derivative?2'
//  '<S16>'  : 'offline_snac_SITL/Attitude controller/MATLAB Function3'
//  '<S17>'  : 'offline_snac_SITL/Off Boarded Trajectory Stream/MATLAB Function1'
//  '<S18>'  : 'offline_snac_SITL/Off Boarded Trajectory Stream/MATLAB Function2'
//  '<S19>'  : 'offline_snac_SITL/Off Boarded Trajectory Stream/PX4 uORB Message'
//  '<S20>'  : 'offline_snac_SITL/PX4 uORB Read/Enabled Subsystem'
//  '<S21>'  : 'offline_snac_SITL/PX4 uORB Read1/Enabled Subsystem'
//  '<S22>'  : 'offline_snac_SITL/PX4 uORB Read2/Enabled Subsystem'
//  '<S23>'  : 'offline_snac_SITL/PX4 uORB Read3/Enabled Subsystem'
//  '<S24>'  : 'offline_snac_SITL/Position /MATLAB Function'
//  '<S25>'  : 'offline_snac_SITL/Position /MATLAB Function1'
//  '<S26>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation'
//  '<S27>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Quaternion Normalize'
//  '<S28>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input'
//  '<S29>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem'
//  '<S30>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem1'
//  '<S31>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem2'
//  '<S32>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus'
//  '<S33>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S34>'  : 'offline_snac_SITL/To Actuator/MATLAB Function'

#endif                                 // offline_snac_SITL_h_

//
// File trailer for generated code.
//
// [EOF]
//
