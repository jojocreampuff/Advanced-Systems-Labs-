//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: offline_snac_SITL.h
//
// Code generated for Simulink model 'offline_snac_SITL'.
//
// Model version                  : 3.32
// Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
// C/C++ source code generated on : Fri Jul 26 14:06:32 2024
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
  real32_T ref[300000];
  real32_T fv[96];
  px4_Bus_vehicle_local_position In1;  // '<S20>/In1'
  px4_Bus_vehicle_local_position r;
  px4_Bus_vehicle_local_position_setpoint BusAssignment;// '<Root>/Bus Assignment' 
  px4_Bus_vehicle_local_position_setpoint In1_d;// '<S22>/In1'
  px4_Bus_vehicle_local_position_setpoint r1;
  px4_Bus_vehicle_attitude In1_l;      // '<S21>/In1'
  px4_Bus_vehicle_attitude r2;
  real32_T fv1[13];
  px4_Bus_sensor_gyro In1_ly;          // '<S19>/In1'
  px4_Bus_sensor_gyro r3;
  real32_T fv2[9];
  uint16_T pwmValue[8];
  real32_T uxyz[3];
  real32_T yaw;// '<Root>/SigConversion_InsertedFor_Bus Selector3_at_outport_6'
  real32_T sensor_x;                   // '<S11>/Data Type Conversion3'
  real32_T sensor_y;                   // '<S11>/Data Type Conversion1'
  real32_T sensor_z;                   // '<S11>/Product'
  real32_T sensor_vx;                  // '<S11>/Data Type Conversion5'
  real32_T sensor_vy;                  // '<S11>/Data Type Conversion6'
  real32_T sensor_vz;                  // '<S11>/Data Type Conversion4'
  real32_T r_x;                        // '<S11>/Rate Limiter3'
  real32_T r_y;                        // '<S11>/Rate Limiter4'
  real32_T r_z;                        // '<S11>/Rate Limiter5'
  real32_T r_xdot;                     // '<S11>/Rate Limiter'
  real32_T r_ydot;                     // '<S11>/Rate Limiter1'
  real32_T r_zdot;                     // '<S11>/Rate Limiter2'
  real32_T pos_error[6];               // '<S11>/Subtract'
  real32_T RateLimiter6;               // '<S11>/Rate Limiter6'
  real32_T SignalConversion1;          // '<Root>/Signal Conversion1'
  real32_T SignalConversion2;          // '<Root>/Signal Conversion2'
  real32_T SignalConversion;           // '<Root>/Signal Conversion'
  real32_T RateLimiter7;               // '<S11>/Rate Limiter7'
  real32_T RateLimiter8;               // '<S11>/Rate Limiter8'
  real32_T des_pitch_rate;             // '<S1>/Rate Limiter3'
  real32_T des_roll_rate;              // '<S1>/Rate Limiter4'
  real32_T att_error[6];               // '<S1>/Subtract'
  real32_T RateLimiter1;               // '<S1>/Rate Limiter1'
  real32_T RateLimiter;                // '<S1>/Rate Limiter'
  real32_T RateLimiter2;               // '<S1>/Rate Limiter2'
  real32_T DiscreteTimeIntegrator;     // '<Root>/Discrete-Time Integrator'
  real32_T pitch_rate;                 // '<Root>/T_matrix'
  real32_T roll_rate;                  // '<Root>/T_matrix'
  real32_T yaw_rate;                   // '<Root>/T_matrix'
  real32_T ux;                         // '<S11>/MATLAB Function'
  real32_T uy;                         // '<S11>/MATLAB Function'
  real32_T uz;                         // '<S11>/MATLAB Function'
  real32_T a;
  real32_T b;
  real32_T B;
  real32_T C;
  real32_T Product2;                   // '<S26>/Product2'
  real32_T SignalConversion2_tmp;
  int32_T i;
  int32_T i1;
  int32_T i2;
  uint16_T ch1;                        // '<S14>/MATLAB Function'
  uint16_T ch2;                        // '<S14>/MATLAB Function'
  uint16_T ch3;                        // '<S14>/MATLAB Function'
  uint16_T ch4;                        // '<S14>/MATLAB Function'
  boolean_T NOT;                       // '<S6>/NOT'
  boolean_T NOT_j;                     // '<S7>/NOT'
  boolean_T NOT_b;                     // '<S8>/NOT'
  boolean_T NOT_h;                     // '<S9>/NOT'
};

// Block states (default storage) for system '<Root>'
struct DW_offline_snac_SITL_T {
  px4_internal_block_Subscriber_T obj; // '<S9>/SourceBlock'
  px4_internal_block_Subscriber_T obj_d;// '<S8>/SourceBlock'
  px4_internal_block_Subscriber_T obj_e;// '<S7>/SourceBlock'
  px4_internal_block_Subscriber_T obj_o;// '<S6>/SourceBlock'
  px4_internal_block_Publisher__T obj_dt;// '<S10>/SinkBlock'
  px4_internal_block_PWM_offlin_T obj_i;// '<S14>/PX4 PWM Output'
  real32_T UD_DSTATE;                  // '<S15>/UD'
  real32_T UD_DSTATE_g;                // '<S16>/UD'
  real32_T UD_DSTATE_i;                // '<S17>/UD'
  real32_T DiscreteTimeIntegrator_DSTATE;// '<Root>/Discrete-Time Integrator'
  real32_T PrevY;                      // '<S11>/Rate Limiter3'
  real32_T PrevY_p;                    // '<S11>/Rate Limiter4'
  real32_T PrevY_a;                    // '<S11>/Rate Limiter5'
  real32_T PrevY_f;                    // '<S11>/Rate Limiter'
  real32_T PrevY_n;                    // '<S11>/Rate Limiter1'
  real32_T PrevY_d;                    // '<S11>/Rate Limiter2'
  real32_T PrevY_l;                    // '<S11>/Rate Limiter6'
  real32_T PrevY_e;                    // '<S11>/Rate Limiter7'
  real32_T PrevY_o;                    // '<S11>/Rate Limiter8'
  real32_T PrevY_k;                    // '<S1>/Rate Limiter3'
  real32_T PrevY_c;                    // '<S1>/Rate Limiter4'
  real32_T PrevY_i;                    // '<S1>/Rate Limiter5'
  real32_T PrevY_cf;                   // '<S1>/Rate Limiter1'
  real32_T PrevY_ak;                   // '<S1>/Rate Limiter'
  real32_T PrevY_dq;                   // '<S1>/Rate Limiter2'
  int8_T IfActionSubsystem2_SubsysRanBC;// '<S27>/If Action Subsystem2'
  int8_T IfActionSubsystem1_SubsysRanBC;// '<S27>/If Action Subsystem1'
  int8_T IfActionSubsystem_SubsysRanBC;// '<S27>/If Action Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC; // '<S9>/Enabled Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC_g;// '<S8>/Enabled Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC_p;// '<S7>/Enabled Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC_l;// '<S6>/Enabled Subsystem'
};

// Parameters (default storage)
struct P_offline_snac_SITL_T_ {
  real32_T DiscreteDerivative_ICPrevScaled;
                              // Mask Parameter: DiscreteDerivative_ICPrevScaled
                                 //  Referenced by: '<S15>/UD'

  real32_T DiscreteDerivative1_ICPrevScale;
                              // Mask Parameter: DiscreteDerivative1_ICPrevScale
                                 //  Referenced by: '<S16>/UD'

  real32_T DiscreteDerivative2_ICPrevScale;
                              // Mask Parameter: DiscreteDerivative2_ICPrevScale
                                 //  Referenced by: '<S17>/UD'

  px4_Bus_vehicle_local_position Out1_Y0;// Computed Parameter: Out1_Y0
                                            //  Referenced by: '<S20>/Out1'

  px4_Bus_vehicle_local_position Constant_Value;// Computed Parameter: Constant_Value
                                                   //  Referenced by: '<S7>/Constant'

  px4_Bus_vehicle_local_position_setpoint Out1_Y0_f;// Computed Parameter: Out1_Y0_f
                                                       //  Referenced by: '<S22>/Out1'

  px4_Bus_vehicle_local_position_setpoint Constant_Value_p;// Computed Parameter: Constant_Value_p
                                                              //  Referenced by: '<S9>/Constant'

  px4_Bus_vehicle_local_position_setpoint Constant_Value_o;// Computed Parameter: Constant_Value_o
                                                              //  Referenced by: '<S5>/Constant'

  px4_Bus_vehicle_attitude Out1_Y0_h;  // Computed Parameter: Out1_Y0_h
                                          //  Referenced by: '<S21>/Out1'

  px4_Bus_vehicle_attitude Constant_Value_a;// Computed Parameter: Constant_Value_a
                                               //  Referenced by: '<S8>/Constant'

  px4_Bus_sensor_gyro Out1_Y0_hw;      // Computed Parameter: Out1_Y0_hw
                                          //  Referenced by: '<S19>/Out1'

  px4_Bus_sensor_gyro Constant_Value_e;// Computed Parameter: Constant_Value_e
                                          //  Referenced by: '<S6>/Constant'

  real_T RateLimiter3_RisingLim;       // Expression: 15
                                          //  Referenced by: '<S11>/Rate Limiter3'

  real_T RateLimiter3_FallingLim;      // Expression: -15
                                          //  Referenced by: '<S11>/Rate Limiter3'

  real_T RateLimiter4_RisingLim;       // Expression: 15
                                          //  Referenced by: '<S11>/Rate Limiter4'

  real_T RateLimiter4_FallingLim;      // Expression: -15
                                          //  Referenced by: '<S11>/Rate Limiter4'

  real_T RateLimiter5_RisingLim;       // Expression: 15
                                          //  Referenced by: '<S11>/Rate Limiter5'

  real_T RateLimiter5_FallingLim;      // Expression: -15
                                          //  Referenced by: '<S11>/Rate Limiter5'

  real_T RateLimiter_RisingLim;        // Expression: 15
                                          //  Referenced by: '<S11>/Rate Limiter'

  real_T RateLimiter_FallingLim;       // Expression: -15
                                          //  Referenced by: '<S11>/Rate Limiter'

  real_T RateLimiter1_RisingLim;       // Expression: 15
                                          //  Referenced by: '<S11>/Rate Limiter1'

  real_T RateLimiter1_FallingLim;      // Expression: -15
                                          //  Referenced by: '<S11>/Rate Limiter1'

  real_T RateLimiter2_RisingLim;       // Expression: 15
                                          //  Referenced by: '<S11>/Rate Limiter2'

  real_T RateLimiter2_FallingLim;      // Expression: -15
                                          //  Referenced by: '<S11>/Rate Limiter2'

  real_T RateLimiter6_RisingLim;       // Expression: 3
                                          //  Referenced by: '<S11>/Rate Limiter6'

  real_T RateLimiter6_FallingLim;      // Expression: -3
                                          //  Referenced by: '<S11>/Rate Limiter6'

  real_T RateLimiter7_RisingLim;       // Expression: 3
                                          //  Referenced by: '<S11>/Rate Limiter7'

  real_T RateLimiter7_FallingLim;      // Expression: -3
                                          //  Referenced by: '<S11>/Rate Limiter7'

  real_T RateLimiter8_RisingLim;       // Expression: 3
                                          //  Referenced by: '<S11>/Rate Limiter8'

  real_T RateLimiter8_FallingLim;      // Expression: -3
                                          //  Referenced by: '<S11>/Rate Limiter8'

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

  real_T RateLimiter1_RisingLim_a;     // Expression: 2
                                          //  Referenced by: '<S1>/Rate Limiter1'

  real_T RateLimiter1_FallingLim_j;    // Expression: -2
                                          //  Referenced by: '<S1>/Rate Limiter1'

  real_T RateLimiter_RisingLim_c;      // Expression: 2
                                          //  Referenced by: '<S1>/Rate Limiter'

  real_T RateLimiter_FallingLim_g;     // Expression: -2
                                          //  Referenced by: '<S1>/Rate Limiter'

  real_T RateLimiter2_RisingLim_f;     // Expression: 2
                                          //  Referenced by: '<S1>/Rate Limiter2'

  real_T RateLimiter2_FallingLim_o;    // Expression: -2
                                          //  Referenced by: '<S1>/Rate Limiter2'

  real32_T Constant_Value_k;           // Computed Parameter: Constant_Value_k
                                          //  Referenced by: '<S28>/Constant'

  real32_T Constant_Value_d;           // Computed Parameter: Constant_Value_d
                                          //  Referenced by: '<S29>/Constant'

  real32_T Constant1_Value;            // Computed Parameter: Constant1_Value
                                          //  Referenced by: '<S11>/Constant1'

  real32_T RateLimiter3_IC;            // Computed Parameter: RateLimiter3_IC
                                          //  Referenced by: '<S11>/Rate Limiter3'

  real32_T RateLimiter4_IC;            // Computed Parameter: RateLimiter4_IC
                                          //  Referenced by: '<S11>/Rate Limiter4'

  real32_T RateLimiter5_IC;            // Computed Parameter: RateLimiter5_IC
                                          //  Referenced by: '<S11>/Rate Limiter5'

  real32_T RateLimiter_IC;             // Computed Parameter: RateLimiter_IC
                                          //  Referenced by: '<S11>/Rate Limiter'

  real32_T RateLimiter1_IC;            // Computed Parameter: RateLimiter1_IC
                                          //  Referenced by: '<S11>/Rate Limiter1'

  real32_T RateLimiter2_IC;            // Computed Parameter: RateLimiter2_IC
                                          //  Referenced by: '<S11>/Rate Limiter2'

  real32_T Constant_Value_e1;          // Computed Parameter: Constant_Value_e1
                                          //  Referenced by: '<S11>/Constant'

  real32_T RateLimiter6_IC;            // Computed Parameter: RateLimiter6_IC
                                          //  Referenced by: '<S11>/Rate Limiter6'

  real32_T RateLimiter7_IC;            // Computed Parameter: RateLimiter7_IC
                                          //  Referenced by: '<S11>/Rate Limiter7'

  real32_T RateLimiter8_IC;            // Computed Parameter: RateLimiter8_IC
                                          //  Referenced by: '<S11>/Rate Limiter8'

  real32_T TSamp_WtEt;                 // Computed Parameter: TSamp_WtEt
                                          //  Referenced by: '<S15>/TSamp'

  real32_T RateLimiter3_IC_b;          // Computed Parameter: RateLimiter3_IC_b
                                          //  Referenced by: '<S1>/Rate Limiter3'

  real32_T TSamp_WtEt_c;               // Computed Parameter: TSamp_WtEt_c
                                          //  Referenced by: '<S16>/TSamp'

  real32_T RateLimiter4_IC_i;          // Computed Parameter: RateLimiter4_IC_i
                                          //  Referenced by: '<S1>/Rate Limiter4'

  real32_T TSamp_WtEt_h;               // Computed Parameter: TSamp_WtEt_h
                                          //  Referenced by: '<S17>/TSamp'

  real32_T RateLimiter5_IC_k;          // Computed Parameter: RateLimiter5_IC_k
                                          //  Referenced by: '<S1>/Rate Limiter5'

  real32_T RateLimiter1_IC_e;          // Computed Parameter: RateLimiter1_IC_e
                                          //  Referenced by: '<S1>/Rate Limiter1'

  real32_T RateLimiter_IC_o;           // Computed Parameter: RateLimiter_IC_o
                                          //  Referenced by: '<S1>/Rate Limiter'

  real32_T RateLimiter2_IC_p;          // Computed Parameter: RateLimiter2_IC_p
                                          //  Referenced by: '<S1>/Rate Limiter2'

  real32_T DiscreteTimeIntegrator_gainval;
                           // Computed Parameter: DiscreteTimeIntegrator_gainval
                              //  Referenced by: '<Root>/Discrete-Time Integrator'

  real32_T DiscreteTimeIntegrator_IC;
                                // Computed Parameter: DiscreteTimeIntegrator_IC
                                   //  Referenced by: '<Root>/Discrete-Time Integrator'

  real32_T Constant2_Value;            // Computed Parameter: Constant2_Value
                                          //  Referenced by: '<Root>/Constant2'

  real32_T Constant_Value_g;           // Computed Parameter: Constant_Value_g
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
//  Block '<S15>/Data Type Duplicate' : Unused code path elimination
//  Block '<S16>/Data Type Duplicate' : Unused code path elimination
//  Block '<S17>/Data Type Duplicate' : Unused code path elimination
//  Block '<Root>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S11>/Data Type Conversion2' : Eliminate redundant data type conversion
//  Block '<S11>/Data Type Conversion7' : Eliminate redundant data type conversion


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
//  '<S5>'   : 'offline_snac_SITL/PX4 uORB Message'
//  '<S6>'   : 'offline_snac_SITL/PX4 uORB Read'
//  '<S7>'   : 'offline_snac_SITL/PX4 uORB Read1'
//  '<S8>'   : 'offline_snac_SITL/PX4 uORB Read2'
//  '<S9>'   : 'offline_snac_SITL/PX4 uORB Read3'
//  '<S10>'  : 'offline_snac_SITL/PX4 uORB Write'
//  '<S11>'  : 'offline_snac_SITL/Position '
//  '<S12>'  : 'offline_snac_SITL/Quaternions to Rotation Angles'
//  '<S13>'  : 'offline_snac_SITL/T_matrix'
//  '<S14>'  : 'offline_snac_SITL/To Actuator'
//  '<S15>'  : 'offline_snac_SITL/Attitude controller/Discrete Derivative?'
//  '<S16>'  : 'offline_snac_SITL/Attitude controller/Discrete Derivative?1'
//  '<S17>'  : 'offline_snac_SITL/Attitude controller/Discrete Derivative?2'
//  '<S18>'  : 'offline_snac_SITL/Attitude controller/MATLAB Function3'
//  '<S19>'  : 'offline_snac_SITL/PX4 uORB Read/Enabled Subsystem'
//  '<S20>'  : 'offline_snac_SITL/PX4 uORB Read1/Enabled Subsystem'
//  '<S21>'  : 'offline_snac_SITL/PX4 uORB Read2/Enabled Subsystem'
//  '<S22>'  : 'offline_snac_SITL/PX4 uORB Read3/Enabled Subsystem'
//  '<S23>'  : 'offline_snac_SITL/Position /MATLAB Function'
//  '<S24>'  : 'offline_snac_SITL/Position /MATLAB Function1'
//  '<S25>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation'
//  '<S26>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Quaternion Normalize'
//  '<S27>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input'
//  '<S28>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem'
//  '<S29>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem1'
//  '<S30>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem2'
//  '<S31>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus'
//  '<S32>'  : 'offline_snac_SITL/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S33>'  : 'offline_snac_SITL/To Actuator/MATLAB Function'

#endif                                 // offline_snac_SITL_h_

//
// File trailer for generated code.
//
// [EOF]
//
