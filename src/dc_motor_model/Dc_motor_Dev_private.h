//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Dc_motor_Dev_private.h
//
// Code generated for Simulink model 'Dc_motor_Dev'.
//
// Model version                  : 1.30
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Thu Jan 21 12:42:25 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: AMD->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_Dc_motor_Dev_private_h_
#define RTW_HEADER_Dc_motor_Dev_private_h_
#include "rtwtypes.h"

// Private macros used by the generated code to access rtModel
#ifndef rtmIsMajorTimeStep
# define rtmIsMajorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
# define rtmIsMinorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
# define rtmSetTPtr(rtm, val)          ((rtm)->Timing.t = (val))
#endif

// private model entry point functions
extern void Dc_motor_Dev_derivatives(RT_MODEL_Dc_motor_Dev_T *const
  Dc_motor_Dev_M);

#endif                                 // RTW_HEADER_Dc_motor_Dev_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
