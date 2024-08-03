//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: snac_openloop_SITL_private.h
//
// Code generated for Simulink model 'snac_openloop_SITL'.
//
// Model version                  : 3.39
// Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
// C/C++ source code generated on : Fri Aug  2 14:07:47 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef snac_openloop_SITL_private_h_
#define snac_openloop_SITL_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#include "snac_openloop_SITL_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"

// Private macros used by the generated code to access rtModel
#ifndef rtmSetTFinal
#define rtmSetTFinal(rtm, val)         ((rtm)->Timing.tFinal = (val))
#endif

extern real32_T rt_roundf_snf(real32_T u);
extern real32_T rt_atan2f_snf(real32_T u0, real32_T u1);
extern real32_T rt_powf_snf(real32_T u0, real32_T u1);

#endif                                 // snac_openloop_SITL_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
