//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: snac_openloop_SITL.h
//
// Code generated for Simulink model 'snac_openloop_SITL'.
//
// Model version                  : 3.40
// Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
// C/C++ source code generated on : Mon Aug  5 15:59:24 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef snac_openloop_SITL_h_
#define snac_openloop_SITL_h_
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
#include "snac_openloop_SITL_types.h"
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

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

// Block signals for system '<S4>/SourceBlock'
struct B_SourceBlock_snac_openloop_S_T {
  px4_Bus_sensor_gyro SourceBlock_o2;  // '<S4>/SourceBlock'
  boolean_T SourceBlock_o1;            // '<S4>/SourceBlock'
};

// Block states (default storage) for system '<S4>/SourceBlock'
struct DW_SourceBlock_snac_openloop__T {
  px4_internal_block_Subscriber_T obj; // '<S4>/SourceBlock'
  boolean_T objisempty;                // '<S4>/SourceBlock'
};

// Block signals for system '<S5>/SourceBlock'
struct B_SourceBlock_snac_openloop_n_T {
  px4_Bus_vehicle_local_position SourceBlock_o2;// '<S5>/SourceBlock'
  boolean_T SourceBlock_o1;            // '<S5>/SourceBlock'
};

// Block states (default storage) for system '<S5>/SourceBlock'
struct DW_SourceBlock_snac_openloo_e_T {
  px4_internal_block_Subscriber_T obj; // '<S5>/SourceBlock'
  boolean_T objisempty;                // '<S5>/SourceBlock'
};

// Block signals for system '<S6>/SourceBlock'
struct B_SourceBlock_snac_openloo_n4_T {
  px4_Bus_vehicle_attitude SourceBlock_o2;// '<S6>/SourceBlock'
  boolean_T SourceBlock_o1;            // '<S6>/SourceBlock'
};

// Block states (default storage) for system '<S6>/SourceBlock'
struct DW_SourceBlock_snac_openloo_a_T {
  px4_internal_block_Subscriber_T obj; // '<S6>/SourceBlock'
  boolean_T objisempty;                // '<S6>/SourceBlock'
};

// Block signals (default storage)
struct B_snac_openloop_SITL_T {
  real32_T fv[172];
  real32_T fv1[48];
  px4_Bus_vehicle_local_position In1;  // '<S45>/In1'
  px4_Bus_vehicle_local_position In1_c;// '<S22>/In1'
  px4_Bus_vehicle_local_position_setpoint BusAssignment;// '<S3>/Bus Assignment' 
  px4_Bus_vehicle_local_position_setpoint In1_d;// '<S24>/In1'
  px4_Bus_vehicle_local_position_setpoint r;
  px4_Bus_vehicle_attitude In1_j;      // '<S44>/In1'
  px4_Bus_vehicle_attitude In1_a;      // '<S23>/In1'
  real32_T fv2[13];
  px4_Bus_sensor_gyro In1_b;           // '<S43>/In1'
  px4_Bus_sensor_gyro In1_f;           // '<S21>/In1'
  real32_T X[12];
  real32_T fv3[9];
  uint16_T pwmValue[8];
  real32_T w[3];
  real32_T w_m[3];
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
  real32_T F_tx_ty_tz_i[4];            // '<S13>/MATLAB Function1'
  real32_T pitch_rate;                 // '<Root>/T_matrix'
  real32_T roll_rate;                  // '<Root>/T_matrix'
  real32_T yaw_rate;                   // '<Root>/T_matrix'
  real32_T x_k_plus_1[12];             // '<S11>/MATLAB Function1'
  real32_T uxyz[3];                    // '<S9>/MATLAB Function3'
  real32_T ft;                         // '<S9>/MATLAB Function1'
  real32_T pitch;                      // '<S9>/MATLAB Function1'
  real32_T roll;                       // '<S9>/MATLAB Function1'
  real32_T ux;                         // '<S9>/MATLAB Function'
  real32_T uy;                         // '<S9>/MATLAB Function'
  real32_T uz;                         // '<S9>/MATLAB Function'
  real32_T tau_x;                      // '<S1>/MATLAB Function3'
  real32_T tau_y;                      // '<S1>/MATLAB Function3'
  real32_T tau_z;                      // '<S1>/MATLAB Function3'
  real32_T b;
  real32_T T4;
  real32_T fcn2;                       // '<S10>/fcn2'
  real32_T Product2;                   // '<S30>/Product2'
  real32_T Product1;                   // '<S30>/Product1'
  real32_T fcn3;                       // '<S41>/fcn3'
  real32_T fcn3_c;                     // '<S10>/fcn3'
  real32_T rtb_VectorConcatenate_idx_0;
  real32_T rtb_VectorConcatenate_idx_1;
  real32_T rtb_VectorConcatenate_idx_2;
  real32_T rtb_VectorConcatenate_idx_0_tmp;
  real32_T SignalConversion2_tmp;
  real32_T SignalConversion2_tmp_c;
  real32_T SignalConversion2_tmp_k;
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
  int32_T uxyz_tmp;
  uint16_T ch1;                        // '<S13>/MATLAB Function'
  uint16_T ch2;                        // '<S13>/MATLAB Function'
  uint16_T ch3;                        // '<S13>/MATLAB Function'
  uint16_T ch4;                        // '<S13>/MATLAB Function'
  boolean_T NOT;                       // '<S4>/NOT'
  boolean_T NOT_g;                     // '<S5>/NOT'
  boolean_T NOT_n;                     // '<S6>/NOT'
  boolean_T NOT_l;                     // '<S7>/NOT'
  boolean_T NOT_e;                     // '<S38>/NOT'
  boolean_T NOT_n2;                    // '<S39>/NOT'
  boolean_T NOT_b;                     // '<S40>/NOT'
  B_SourceBlock_snac_openloop_n_T SourceBlock_k;// '<S5>/SourceBlock'
  B_SourceBlock_snac_openloo_n4_T SourceBlock_fm;// '<S6>/SourceBlock'
  B_SourceBlock_snac_openloop_S_T SourceBlock_f;// '<S4>/SourceBlock'
  B_SourceBlock_snac_openloo_n4_T SourceBlock_g;// '<S6>/SourceBlock'
  B_SourceBlock_snac_openloop_n_T SourceBlock_m;// '<S5>/SourceBlock'
  B_SourceBlock_snac_openloop_S_T SourceBlock;// '<S4>/SourceBlock'
};

// Block states (default storage) for system '<Root>'
struct DW_snac_openloop_SITL_T {
  px4_internal_block_Subscriber_T obj; // '<S7>/SourceBlock'
  px4_internal_block_Publisher__T obj_l;// '<S8>/SinkBlock'
  px4_internal_block_PWM_snac_o_T obj_p;// '<S13>/PX4 PWM Output'
  real32_T UD_DSTATE;                  // '<S14>/UD'
  real32_T UD_DSTATE_e;                // '<S15>/UD'
  real32_T UD_DSTATE_d;                // '<S16>/UD'
  real32_T DiscreteTimeIntegrator_DSTATE;// '<S9>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegrator_DSTATE_m;// '<S13>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegrator_DSTATE_f;// '<S3>/Discrete-Time Integrator'
  real32_T PrevY;                      // '<S9>/Rate Limiter3'
  real32_T PrevY_k;                    // '<S9>/Rate Limiter4'
  real32_T PrevY_f;                    // '<S9>/Rate Limiter5'
  real32_T PrevY_g;                    // '<S9>/Rate Limiter'
  real32_T PrevY_h;                    // '<S9>/Rate Limiter1'
  real32_T PrevY_gn;                   // '<S9>/Rate Limiter2'
  real32_T PrevY_m;                    // '<S1>/Rate Limiter3'
  real32_T PrevY_e;                    // '<S1>/Rate Limiter4'
  real32_T PrevY_b;                    // '<S1>/Rate Limiter5'
  int8_T IfActionSubsystem2_SubsysRanBC;// '<S48>/If Action Subsystem2'
  int8_T IfActionSubsystem1_SubsysRanBC;// '<S48>/If Action Subsystem1'
  int8_T IfActionSubsystem_SubsysRanBC;// '<S48>/If Action Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC; // '<S40>/Enabled Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC_f;// '<S39>/Enabled Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC_k;// '<S38>/Enabled Subsystem'
  int8_T IfActionSubsystem2_SubsysRanB_n;// '<S31>/If Action Subsystem2'
  int8_T IfActionSubsystem1_SubsysRanB_n;// '<S31>/If Action Subsystem1'
  int8_T IfActionSubsystem_SubsysRanBC_o;// '<S31>/If Action Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC_h;// '<S7>/Enabled Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC_g;// '<S6>/Enabled Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC_gz;// '<S5>/Enabled Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC_l;// '<S4>/Enabled Subsystem'
  DW_SourceBlock_snac_openloo_e_T SourceBlock_k;// '<S5>/SourceBlock'
  DW_SourceBlock_snac_openloo_a_T SourceBlock_fm;// '<S6>/SourceBlock'
  DW_SourceBlock_snac_openloop__T SourceBlock_f;// '<S4>/SourceBlock'
  DW_SourceBlock_snac_openloo_a_T SourceBlock_g;// '<S6>/SourceBlock'
  DW_SourceBlock_snac_openloo_e_T SourceBlock_m;// '<S5>/SourceBlock'
  DW_SourceBlock_snac_openloop__T SourceBlock;// '<S4>/SourceBlock'
};

// Parameters (default storage)
struct P_snac_openloop_SITL_T_ {
  real32_T DiscreteDerivative_ICPrevScaled;
                              // Mask Parameter: DiscreteDerivative_ICPrevScaled
                                 //  Referenced by: '<S14>/UD'

  real32_T DiscreteDerivative1_ICPrevScale;
                              // Mask Parameter: DiscreteDerivative1_ICPrevScale
                                 //  Referenced by: '<S15>/UD'

  real32_T DiscreteDerivative2_ICPrevScale;
                              // Mask Parameter: DiscreteDerivative2_ICPrevScale
                                 //  Referenced by: '<S16>/UD'

  px4_Bus_vehicle_local_position Out1_Y0;// Computed Parameter: Out1_Y0
                                            //  Referenced by: '<S22>/Out1'

  px4_Bus_vehicle_local_position Out1_Y0_e;// Computed Parameter: Out1_Y0_e
                                              //  Referenced by: '<S45>/Out1'

  px4_Bus_vehicle_local_position Constant_Value;// Computed Parameter: Constant_Value
                                                   //  Referenced by: '<S5>/Constant'

  px4_Bus_vehicle_local_position Constant_Value_d;// Computed Parameter: Constant_Value_d
                                                     //  Referenced by: '<S40>/Constant'

  px4_Bus_vehicle_local_position_setpoint Out1_Y0_i;// Computed Parameter: Out1_Y0_i
                                                       //  Referenced by: '<S24>/Out1'

  px4_Bus_vehicle_local_position_setpoint Constant_Value_b;// Computed Parameter: Constant_Value_b
                                                              //  Referenced by: '<S7>/Constant'

  px4_Bus_vehicle_local_position_setpoint Constant_Value_n;// Computed Parameter: Constant_Value_n
                                                              //  Referenced by: '<S20>/Constant'

  px4_Bus_vehicle_attitude Out1_Y0_k;  // Computed Parameter: Out1_Y0_k
                                          //  Referenced by: '<S23>/Out1'

  px4_Bus_vehicle_attitude Out1_Y0_j;  // Computed Parameter: Out1_Y0_j
                                          //  Referenced by: '<S44>/Out1'

  px4_Bus_vehicle_attitude Constant_Value_f;// Computed Parameter: Constant_Value_f
                                               //  Referenced by: '<S6>/Constant'

  px4_Bus_vehicle_attitude Constant_Value_nu;// Computed Parameter: Constant_Value_nu
                                                //  Referenced by: '<S39>/Constant'

  px4_Bus_sensor_gyro Out1_Y0_n;       // Computed Parameter: Out1_Y0_n
                                          //  Referenced by: '<S21>/Out1'

  px4_Bus_sensor_gyro Out1_Y0_d;       // Computed Parameter: Out1_Y0_d
                                          //  Referenced by: '<S43>/Out1'

  px4_Bus_sensor_gyro Constant_Value_p;// Computed Parameter: Constant_Value_p
                                          //  Referenced by: '<S4>/Constant'

  px4_Bus_sensor_gyro Constant_Value_o;// Computed Parameter: Constant_Value_o
                                          //  Referenced by: '<S38>/Constant'

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

  real_T RateLimiter3_RisingLim_i;     // Expression: 2
                                          //  Referenced by: '<S1>/Rate Limiter3'

  real_T RateLimiter3_FallingLim_m;    // Expression: -2
                                          //  Referenced by: '<S1>/Rate Limiter3'

  real_T RateLimiter4_RisingLim_k;     // Expression: 2
                                          //  Referenced by: '<S1>/Rate Limiter4'

  real_T RateLimiter4_FallingLim_d;    // Expression: -2
                                          //  Referenced by: '<S1>/Rate Limiter4'

  real_T RateLimiter5_RisingLim_o;     // Expression: 2
                                          //  Referenced by: '<S1>/Rate Limiter5'

  real_T RateLimiter5_FallingLim_h;    // Expression: -2
                                          //  Referenced by: '<S1>/Rate Limiter5'

  real32_T Constant_Value_pg;          // Computed Parameter: Constant_Value_pg
                                          //  Referenced by: '<S32>/Constant'

  real32_T Constant_Value_e;           // Computed Parameter: Constant_Value_e
                                          //  Referenced by: '<S33>/Constant'

  real32_T Constant_Value_h;           // Computed Parameter: Constant_Value_h
                                          //  Referenced by: '<S49>/Constant'

  real32_T Constant_Value_dp;          // Computed Parameter: Constant_Value_dp
                                          //  Referenced by: '<S50>/Constant'

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

  real32_T TSamp_WtEt;                 // Computed Parameter: TSamp_WtEt
                                          //  Referenced by: '<S14>/TSamp'

  real32_T RateLimiter3_IC_j;          // Computed Parameter: RateLimiter3_IC_j
                                          //  Referenced by: '<S1>/Rate Limiter3'

  real32_T TSamp_WtEt_h;               // Computed Parameter: TSamp_WtEt_h
                                          //  Referenced by: '<S15>/TSamp'

  real32_T RateLimiter4_IC_h;          // Computed Parameter: RateLimiter4_IC_h
                                          //  Referenced by: '<S1>/Rate Limiter4'

  real32_T TSamp_WtEt_j;               // Computed Parameter: TSamp_WtEt_j
                                          //  Referenced by: '<S16>/TSamp'

  real32_T RateLimiter5_IC_k;          // Computed Parameter: RateLimiter5_IC_k
                                          //  Referenced by: '<S1>/Rate Limiter5'

  real32_T DiscreteTimeIntegrator_gainval;
                           // Computed Parameter: DiscreteTimeIntegrator_gainval
                              //  Referenced by: '<S9>/Discrete-Time Integrator'

  real32_T DiscreteTimeIntegrator_IC;
                                // Computed Parameter: DiscreteTimeIntegrator_IC
                                   //  Referenced by: '<S9>/Discrete-Time Integrator'

  real32_T Constant_Value_i;           // Computed Parameter: Constant_Value_i
                                          //  Referenced by: '<S9>/Constant'

  real32_T DiscreteTimeIntegrator_gainva_i;
                          // Computed Parameter: DiscreteTimeIntegrator_gainva_i
                             //  Referenced by: '<S13>/Discrete-Time Integrator'

  real32_T DiscreteTimeIntegrator_IC_e;
                              // Computed Parameter: DiscreteTimeIntegrator_IC_e
                                 //  Referenced by: '<S13>/Discrete-Time Integrator'

  real32_T Constant_Value_ho;          // Computed Parameter: Constant_Value_ho
                                          //  Referenced by: '<S13>/Constant'

  real32_T DiscreteTimeIntegrator_gainva_c;
                          // Computed Parameter: DiscreteTimeIntegrator_gainva_c
                             //  Referenced by: '<S3>/Discrete-Time Integrator'

  real32_T DiscreteTimeIntegrator_IC_f;
                              // Computed Parameter: DiscreteTimeIntegrator_IC_f
                                 //  Referenced by: '<S3>/Discrete-Time Integrator'

  real32_T Constant2_Value;            // Computed Parameter: Constant2_Value
                                          //  Referenced by: '<S3>/Constant2'

  real32_T Constant_Value_k;           // Computed Parameter: Constant_Value_k
                                          //  Referenced by: '<S3>/Constant'

  boolean_T Reset_Value;               // Computed Parameter: Reset_Value
                                          //  Referenced by: '<Root>/Reset'

  boolean_T DataStoreMemory_InitialValue;
                             // Computed Parameter: DataStoreMemory_InitialValue
                                //  Referenced by: '<Root>/Data Store Memory'

};

// Real-time Model Data Structure
struct tag_RTM_snac_openloop_SITL_T {
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

  extern P_snac_openloop_SITL_T snac_openloop_SITL_P;

#ifdef __cplusplus

}

#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C"
{

#endif

  extern struct B_snac_openloop_SITL_T snac_openloop_SITL_B;

#ifdef __cplusplus

}

#endif

// Block states (default storage)
extern struct DW_snac_openloop_SITL_T snac_openloop_SITL_DW;

#ifdef __cplusplus

extern "C"
{

#endif

  // Model entry point functions
  extern void snac_openloop_SITL_initialize(void);
  extern void snac_openloop_SITL_step(void);
  extern void snac_openloop_SITL_terminate(void);

#ifdef __cplusplus

}

#endif

// Real-time Model object
#ifdef __cplusplus

extern "C"
{

#endif

  extern RT_MODEL_snac_openloop_SITL_T *const snac_openloop_SITL_M;

#ifdef __cplusplus

}

#endif

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S14>/Data Type Duplicate' : Unused code path elimination
//  Block '<S15>/Data Type Duplicate' : Unused code path elimination
//  Block '<S16>/Data Type Duplicate' : Unused code path elimination
//  Block '<Root>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S9>/Data Type Conversion2' : Eliminate redundant data type conversion
//  Block '<S11>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S11>/Signal Conversion' : Eliminate redundant signal conversion block
//  Block '<S11>/Signal Conversion1' : Eliminate redundant signal conversion block
//  Block '<S11>/Signal Conversion2' : Eliminate redundant signal conversion block


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
//  '<Root>' : 'snac_openloop_SITL'
//  '<S1>'   : 'snac_openloop_SITL/Attitude controller'
//  '<S2>'   : 'snac_openloop_SITL/MATLAB Function9'
//  '<S3>'   : 'snac_openloop_SITL/Off Boarded Trajectory Stream'
//  '<S4>'   : 'snac_openloop_SITL/PX4 uORB Read'
//  '<S5>'   : 'snac_openloop_SITL/PX4 uORB Read1'
//  '<S6>'   : 'snac_openloop_SITL/PX4 uORB Read2'
//  '<S7>'   : 'snac_openloop_SITL/PX4 uORB Read3'
//  '<S8>'   : 'snac_openloop_SITL/PX4 uORB Write'
//  '<S9>'   : 'snac_openloop_SITL/Position '
//  '<S10>'  : 'snac_openloop_SITL/Quaternions to Rotation Angles'
//  '<S11>'  : 'snac_openloop_SITL/Subsystem'
//  '<S12>'  : 'snac_openloop_SITL/T_matrix'
//  '<S13>'  : 'snac_openloop_SITL/To Actuator'
//  '<S14>'  : 'snac_openloop_SITL/Attitude controller/Discrete Derivative?'
//  '<S15>'  : 'snac_openloop_SITL/Attitude controller/Discrete Derivative?1'
//  '<S16>'  : 'snac_openloop_SITL/Attitude controller/Discrete Derivative?2'
//  '<S17>'  : 'snac_openloop_SITL/Attitude controller/MATLAB Function3'
//  '<S18>'  : 'snac_openloop_SITL/Off Boarded Trajectory Stream/MATLAB Function1'
//  '<S19>'  : 'snac_openloop_SITL/Off Boarded Trajectory Stream/MATLAB Function2'
//  '<S20>'  : 'snac_openloop_SITL/Off Boarded Trajectory Stream/PX4 uORB Message'
//  '<S21>'  : 'snac_openloop_SITL/PX4 uORB Read/Enabled Subsystem'
//  '<S22>'  : 'snac_openloop_SITL/PX4 uORB Read1/Enabled Subsystem'
//  '<S23>'  : 'snac_openloop_SITL/PX4 uORB Read2/Enabled Subsystem'
//  '<S24>'  : 'snac_openloop_SITL/PX4 uORB Read3/Enabled Subsystem'
//  '<S25>'  : 'snac_openloop_SITL/Position /MATLAB Function'
//  '<S26>'  : 'snac_openloop_SITL/Position /MATLAB Function1'
//  '<S27>'  : 'snac_openloop_SITL/Position /MATLAB Function2'
//  '<S28>'  : 'snac_openloop_SITL/Position /MATLAB Function3'
//  '<S29>'  : 'snac_openloop_SITL/Quaternions to Rotation Angles/Angle Calculation'
//  '<S30>'  : 'snac_openloop_SITL/Quaternions to Rotation Angles/Quaternion Normalize'
//  '<S31>'  : 'snac_openloop_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input'
//  '<S32>'  : 'snac_openloop_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem'
//  '<S33>'  : 'snac_openloop_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem1'
//  '<S34>'  : 'snac_openloop_SITL/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem2'
//  '<S35>'  : 'snac_openloop_SITL/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus'
//  '<S36>'  : 'snac_openloop_SITL/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S37>'  : 'snac_openloop_SITL/Subsystem/MATLAB Function1'
//  '<S38>'  : 'snac_openloop_SITL/Subsystem/PX4 uORB Read'
//  '<S39>'  : 'snac_openloop_SITL/Subsystem/PX4 uORB Read2'
//  '<S40>'  : 'snac_openloop_SITL/Subsystem/PX4 uORB Read4'
//  '<S41>'  : 'snac_openloop_SITL/Subsystem/Quaternions to Rotation Angles'
//  '<S42>'  : 'snac_openloop_SITL/Subsystem/T_matrix'
//  '<S43>'  : 'snac_openloop_SITL/Subsystem/PX4 uORB Read/Enabled Subsystem'
//  '<S44>'  : 'snac_openloop_SITL/Subsystem/PX4 uORB Read2/Enabled Subsystem'
//  '<S45>'  : 'snac_openloop_SITL/Subsystem/PX4 uORB Read4/Enabled Subsystem'
//  '<S46>'  : 'snac_openloop_SITL/Subsystem/Quaternions to Rotation Angles/Angle Calculation'
//  '<S47>'  : 'snac_openloop_SITL/Subsystem/Quaternions to Rotation Angles/Quaternion Normalize'
//  '<S48>'  : 'snac_openloop_SITL/Subsystem/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input'
//  '<S49>'  : 'snac_openloop_SITL/Subsystem/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem'
//  '<S50>'  : 'snac_openloop_SITL/Subsystem/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem1'
//  '<S51>'  : 'snac_openloop_SITL/Subsystem/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem2'
//  '<S52>'  : 'snac_openloop_SITL/Subsystem/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus'
//  '<S53>'  : 'snac_openloop_SITL/Subsystem/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S54>'  : 'snac_openloop_SITL/To Actuator/MATLAB Function'
//  '<S55>'  : 'snac_openloop_SITL/To Actuator/MATLAB Function1'
//  '<S56>'  : 'snac_openloop_SITL/To Actuator/MATLAB Function2'

#endif                                 // snac_openloop_SITL_h_

//
// File trailer for generated code.
//
// [EOF]
//
