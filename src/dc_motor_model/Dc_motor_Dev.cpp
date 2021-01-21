//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Dc_motor_Dev.cpp
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
#include "Dc_motor_Dev.h"
#include "Dc_motor_Dev_private.h"

//
// This function updates continuous states using the ODE1 fixed-step
// solver algorithm
//
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si ,
  RT_MODEL_Dc_motor_Dev_T *const Dc_motor_Dev_M, ExtU_Dc_motor_Dev_T
  *Dc_motor_Dev_U, ExtY_Dc_motor_Dev_T *Dc_motor_Dev_Y)
{
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE1_IntgData *id = static_cast<ODE1_IntgData *>(rtsiGetSolverData(si));
  real_T *f0 = id->f[0];
  int_T i;
  int_T nXc = 1;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);
  rtsiSetdX(si, f0);
  Dc_motor_Dev_derivatives(Dc_motor_Dev_M);
  rtsiSetT(si, tnew);
  for (i = 0; i < nXc; ++i) {
    x[i] += h * f0[i];
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

// Model step function
void Dc_motor_Dev_step(RT_MODEL_Dc_motor_Dev_T *const Dc_motor_Dev_M,
  ExtU_Dc_motor_Dev_T *Dc_motor_Dev_U, ExtY_Dc_motor_Dev_T *Dc_motor_Dev_Y)
{
  B_Dc_motor_Dev_T *Dc_motor_Dev_B = (static_cast<B_Dc_motor_Dev_T *>
    (Dc_motor_Dev_M->blockIO));
  X_Dc_motor_Dev_T *Dc_motor_Dev_X = ((X_Dc_motor_Dev_T *)
    Dc_motor_Dev_M->contStates);
  real_T rtb_Sum1;
  if (rtmIsMajorTimeStep(Dc_motor_Dev_M)) {
    // set solver stop time
    rtsiSetSolverStopTime(&Dc_motor_Dev_M->solverInfo,
                          ((Dc_motor_Dev_M->Timing.clockTick0+1)*
      Dc_motor_Dev_M->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep(Dc_motor_Dev_M)) {
    Dc_motor_Dev_M->Timing.t[0] = rtsiGetT(&Dc_motor_Dev_M->solverInfo);
  }

  // MATLAB Function: '<S1>/MATLAB Function' incorporates:
  //   Constant: '<S1>/Constant1'
  //   Integrator: '<S1>/Integrator'

  if (Dc_motor_Dev_X->Integrator_CSTATE == 0.0) {
    rtb_Sum1 = 0.0;
  } else {
    if (Dc_motor_Dev_X->Integrator_CSTATE < 0.0) {
      rtb_Sum1 = -1.0;
    } else if (Dc_motor_Dev_X->Integrator_CSTATE > 0.0) {
      rtb_Sum1 = 1.0;
    } else if (Dc_motor_Dev_X->Integrator_CSTATE == 0.0) {
      rtb_Sum1 = 0.0;
    } else {
      rtb_Sum1 = (rtNaN);
    }

    rtb_Sum1 *= Dc_motor_Dev_P.C0;
  }

  // End of MATLAB Function: '<S1>/MATLAB Function'

  // Sum: '<S1>/Sum1' incorporates:
  //   Gain: '<S1>/Gain'
  //   Gain: '<S1>/Gain1'
  //   Gain: '<S1>/Gain2'
  //   Gain: '<S1>/Gain4'
  //   Inport: '<Root>/V'
  //   Integrator: '<S1>/Integrator'
  //   Sum: '<S1>/Sum'

  rtb_Sum1 = ((Dc_motor_Dev_U->V - Dc_motor_Dev_P.Kem *
               Dc_motor_Dev_X->Integrator_CSTATE) * (1.0 / Dc_motor_Dev_P.R) *
              Dc_motor_Dev_P.Kc - Dc_motor_Dev_P.f *
              Dc_motor_Dev_X->Integrator_CSTATE) - rtb_Sum1;

  // Outport: '<Root>/C_final' incorporates:
  //   Gain: '<Root>/Gain1'

  Dc_motor_Dev_Y->C_final = 1.0 / Dc_motor_Dev_P.tau * rtb_Sum1;

  // Gain: '<S1>/Gain3' incorporates:
  //   Gain: '<Root>/Gain5'
  //   Inport: '<Root>/Cr'
  //   Sum: '<S1>/Sum2'

  Dc_motor_Dev_B->Gain3 = (rtb_Sum1 - Dc_motor_Dev_P.tau * Dc_motor_Dev_U->Cr) *
    (1.0 / Dc_motor_Dev_P.J);

  // Outport: '<Root>/w_m' incorporates:
  //   Gain: '<Root>/Gain6'
  //   Integrator: '<S1>/Integrator'

  Dc_motor_Dev_Y->w_m = Dc_motor_Dev_P.tau * Dc_motor_Dev_X->Integrator_CSTATE;
  if (rtmIsMajorTimeStep(Dc_motor_Dev_M)) {
    rt_ertODEUpdateContinuousStates(&Dc_motor_Dev_M->solverInfo, Dc_motor_Dev_M,
      Dc_motor_Dev_U, Dc_motor_Dev_Y);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++Dc_motor_Dev_M->Timing.clockTick0;
    Dc_motor_Dev_M->Timing.t[0] = rtsiGetSolverStopTime
      (&Dc_motor_Dev_M->solverInfo);

    {
      // Update absolute timer for sample time: [0.01s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.01, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      Dc_motor_Dev_M->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void Dc_motor_Dev_derivatives(RT_MODEL_Dc_motor_Dev_T *const Dc_motor_Dev_M)
{
  B_Dc_motor_Dev_T *Dc_motor_Dev_B = (static_cast<B_Dc_motor_Dev_T *>
    (Dc_motor_Dev_M->blockIO));
  XDot_Dc_motor_Dev_T *_rtXdot;
  _rtXdot = ((XDot_Dc_motor_Dev_T *) Dc_motor_Dev_M->derivs);

  // Derivatives for Integrator: '<S1>/Integrator'
  _rtXdot->Integrator_CSTATE = Dc_motor_Dev_B->Gain3;
}

// Model initialize function
void Dc_motor_Dev_initialize(RT_MODEL_Dc_motor_Dev_T *const Dc_motor_Dev_M,
  ExtU_Dc_motor_Dev_T *Dc_motor_Dev_U, ExtY_Dc_motor_Dev_T *Dc_motor_Dev_Y)
{
  X_Dc_motor_Dev_T *Dc_motor_Dev_X = ((X_Dc_motor_Dev_T *)
    Dc_motor_Dev_M->contStates);
  B_Dc_motor_Dev_T *Dc_motor_Dev_B = (static_cast<B_Dc_motor_Dev_T *>
    (Dc_motor_Dev_M->blockIO));

  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&Dc_motor_Dev_M->solverInfo,
                          &Dc_motor_Dev_M->Timing.simTimeStep);
    rtsiSetTPtr(&Dc_motor_Dev_M->solverInfo, &rtmGetTPtr(Dc_motor_Dev_M));
    rtsiSetStepSizePtr(&Dc_motor_Dev_M->solverInfo,
                       &Dc_motor_Dev_M->Timing.stepSize0);
    rtsiSetdXPtr(&Dc_motor_Dev_M->solverInfo, &Dc_motor_Dev_M->derivs);
    rtsiSetContStatesPtr(&Dc_motor_Dev_M->solverInfo, (real_T **)
                         &Dc_motor_Dev_M->contStates);
    rtsiSetNumContStatesPtr(&Dc_motor_Dev_M->solverInfo,
      &Dc_motor_Dev_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&Dc_motor_Dev_M->solverInfo,
      &Dc_motor_Dev_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&Dc_motor_Dev_M->solverInfo,
      &Dc_motor_Dev_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&Dc_motor_Dev_M->solverInfo,
      &Dc_motor_Dev_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&Dc_motor_Dev_M->solverInfo, (&rtmGetErrorStatus
      (Dc_motor_Dev_M)));
    rtsiSetRTModelPtr(&Dc_motor_Dev_M->solverInfo, Dc_motor_Dev_M);
  }

  rtsiSetSimTimeStep(&Dc_motor_Dev_M->solverInfo, MAJOR_TIME_STEP);
  Dc_motor_Dev_M->intgData.f[0] = Dc_motor_Dev_M->odeF[0];
  Dc_motor_Dev_M->contStates = ((X_Dc_motor_Dev_T *) Dc_motor_Dev_X);
  rtsiSetSolverData(&Dc_motor_Dev_M->solverInfo, static_cast<void *>
                    (&Dc_motor_Dev_M->intgData));
  rtsiSetSolverName(&Dc_motor_Dev_M->solverInfo,"ode1");
  rtmSetTPtr(Dc_motor_Dev_M, &Dc_motor_Dev_M->Timing.tArray[0]);
  Dc_motor_Dev_M->Timing.stepSize0 = 0.01;

  // block I/O
  (void) std::memset((static_cast<void *>(Dc_motor_Dev_B)), 0,
                     sizeof(B_Dc_motor_Dev_T));

  // states (continuous)
  {
    (void) std::memset(static_cast<void *>(Dc_motor_Dev_X), 0,
                       sizeof(X_Dc_motor_Dev_T));
  }

  // external inputs
  (void)std::memset(Dc_motor_Dev_U, 0, sizeof(ExtU_Dc_motor_Dev_T));

  // external outputs
  (void) std::memset(static_cast<void *>(Dc_motor_Dev_Y), 0,
                     sizeof(ExtY_Dc_motor_Dev_T));

  // InitializeConditions for Integrator: '<S1>/Integrator'
  Dc_motor_Dev_X->Integrator_CSTATE = Dc_motor_Dev_P.Integrator_IC;
}

// Model terminate function
void Dc_motor_Dev_terminate(RT_MODEL_Dc_motor_Dev_T *const Dc_motor_Dev_M)
{
  // (no terminate code required)
  UNUSED_PARAMETER(Dc_motor_Dev_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
