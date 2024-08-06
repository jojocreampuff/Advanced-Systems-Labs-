//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: snac_openloop_SITL_private.h
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
#ifndef snac_openloop_SITL_private_h_
#define snac_openloop_SITL_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#include "snac_openloop_SITL.h"
#include "snac_openloop_SITL_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"

// Private macros used by the generated code to access rtModel
#ifndef rtmSetTFinal
#define rtmSetTFinal(rtm, val)         ((rtm)->Timing.tFinal = (val))
#endif

extern real32_T rt_atan2f_snf(real32_T u0, real32_T u1);
extern real32_T rt_powf_snf(real32_T u0, real32_T u1);
extern real32_T rt_roundf_snf(real32_T u);
extern void snac_openloop__SourceBlock_Init(DW_SourceBlock_snac_openloop__T
  *localDW);
extern void snac_openloop_SITL_SourceBlock(B_SourceBlock_snac_openloop_S_T
  *localB, DW_SourceBlock_snac_openloop__T *localDW);
extern void snac_openloo_SourceBlock_c_Init(DW_SourceBlock_snac_openloo_e_T
  *localDW);
extern void snac_openloop_SIT_SourceBlock_m(B_SourceBlock_snac_openloop_n_T
  *localB, DW_SourceBlock_snac_openloo_e_T *localDW);
extern void snac_openloo_SourceBlock_i_Init(DW_SourceBlock_snac_openloo_a_T
  *localDW);
extern void snac_openloop_SIT_SourceBlock_g(B_SourceBlock_snac_openloo_n4_T
  *localB, DW_SourceBlock_snac_openloo_a_T *localDW);
extern void snac_openloop__SourceBlock_Term(DW_SourceBlock_snac_openloop__T
  *localDW);
extern void snac_openloo_SourceBlock_m_Term(DW_SourceBlock_snac_openloo_e_T
  *localDW);
extern void snac_openloo_SourceBlock_e_Term(DW_SourceBlock_snac_openloo_a_T
  *localDW);

#endif                                 // snac_openloop_SITL_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
