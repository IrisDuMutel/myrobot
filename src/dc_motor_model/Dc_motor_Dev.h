//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Dc_motor_Dev.h
//
// Code generated for Simulink model 'Dc_motor_Dev'.
//
// Model version                  : 1.32
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Thu Jan 21 16:22:31 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: AMD->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_Dc_motor_Dev_h_
#define RTW_HEADER_Dc_motor_Dev_h_
#include <string.h>
#include <stddef.h>
#ifndef Dc_motor_Dev_COMMON_INCLUDES_
# define Dc_motor_Dev_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "R2019b/simulink/include/rtw_continuous.h"
#include "R2019b/simulink/include/rtw_solver.h"
#endif                                 // Dc_motor_Dev_COMMON_INCLUDES_

#include "Dc_motor_Dev_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rt_defines.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetBlockIO
# define rtmGetBlockIO(rtm)            ((rtm)->blockIO)
#endif

#ifndef rtmSetBlockIO
# define rtmSetBlockIO(rtm, val)       ((rtm)->blockIO = (val))
#endif

#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ((rtm)->odeF = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

// Block signals (default storage)
typedef struct {
  real_T Gain3;                        // '<S1>/Gain3'
} B_Dc_motor_Dev_T;

// Continuous states (default storage)
typedef struct {
  real_T Integrator_CSTATE;            // '<S1>/Integrator'
} X_Dc_motor_Dev_T;

// State derivatives (default storage)
typedef struct {
  real_T Integrator_CSTATE;            // '<S1>/Integrator'
} XDot_Dc_motor_Dev_T;

// State disabled
typedef struct {
  boolean_T Integrator_CSTATE;         // '<S1>/Integrator'
} XDis_Dc_motor_Dev_T;

#ifndef ODE1_INTG
#define ODE1_INTG

// ODE1 Integration Data
typedef struct {
  real_T *f[1];                        // derivatives
} ODE1_IntgData;

#endif

// External inputs (root inport signals with default storage)
typedef struct {
  real_T Cr;                           // '<Root>/Cr'
  real_T V;                            // '<Root>/V'
} ExtU_Dc_motor_Dev_T;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real_T C_final;                      // '<Root>/C_final'
  real_T w_m;                          // '<Root>/w_m'
} ExtY_Dc_motor_Dev_T;

// Parameters (default storage)
struct P_Dc_motor_Dev_T_ {
  real_T C0;                           // Variable: C0
                                          //  Referenced by: '<S1>/Constant1'

  real_T J;                            // Variable: J
                                          //  Referenced by: '<S1>/Gain3'

  real_T Kc;                           // Variable: Kc
                                          //  Referenced by: '<S1>/Gain2'

  real_T Kem;                          // Variable: Kem
                                          //  Referenced by: '<S1>/Gain'

  real_T R;                            // Variable: R
                                          //  Referenced by: '<S1>/Gain1'

  real_T f;                            // Variable: f
                                          //  Referenced by: '<S1>/Gain4'

  real_T tau;                          // Variable: tau
                                          //  Referenced by:
                                          //    '<Root>/Gain1'
                                          //    '<Root>/Gain5'
                                          //    '<Root>/Gain6'

  real_T Integrator_IC;                // Expression: 0
                                          //  Referenced by: '<S1>/Integrator'

};

// Real-time Model Data Structure
struct tag_RTM_Dc_motor_Dev_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  B_Dc_motor_Dev_T *blockIO;
  X_Dc_motor_Dev_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeF[1][1];
  ODE1_IntgData intgData;

  //
  //  Sizes:
  //  The following substructure contains sizes information
  //  for many of the model attributes such as inputs, outputs,
  //  dwork, sample times, etc.

  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_Dc_motor_Dev_T Dc_motor_Dev_P;

#ifdef __cplusplus

}
#endif

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void Dc_motor_Dev_initialize(RT_MODEL_Dc_motor_Dev_T *const
    Dc_motor_Dev_M, ExtU_Dc_motor_Dev_T *Dc_motor_Dev_U, ExtY_Dc_motor_Dev_T
    *Dc_motor_Dev_Y);
  extern void Dc_motor_Dev_step(RT_MODEL_Dc_motor_Dev_T *const Dc_motor_Dev_M,
    ExtU_Dc_motor_Dev_T *Dc_motor_Dev_U, ExtY_Dc_motor_Dev_T *Dc_motor_Dev_Y);
  extern void Dc_motor_Dev_terminate(RT_MODEL_Dc_motor_Dev_T *const
    Dc_motor_Dev_M);

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/To Workspace' : Unused code path elimination
//  Block '<Root>/To Workspace1' : Unused code path elimination


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
//  '<Root>' : 'Dc_motor_Dev'
//  '<S1>'   : 'Dc_motor_Dev/DC MOTOR'
//  '<S2>'   : 'Dc_motor_Dev/DC MOTOR/MATLAB Function'

#endif                                 // RTW_HEADER_Dc_motor_Dev_h_

//
// File trailer for generated code.
//
// [EOF]
//
