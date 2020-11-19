//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PI_contr.cpp
//
// Code generated for Simulink model 'PI_contr'.
//
// Model version                  : 1.6
// Simulink Coder version         : 9.1 (R2019a) 23-Nov-2018
// C/C++ source code generated on : Tue Nov 17 15:54:12 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "PI_contr.h"
#include "PI_contr_private.h"

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T a;
  a = std::abs(u0);
  y = std::abs(u1);
  if (a < y) {
    a /= y;
    y *= std::sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = std::sqrt(y * y + 1.0) * a;
  } else {
    if (!rtIsNaN(y)) {
      y = a * 1.4142135623730951;
    }
  }

  return y;
}

// Model step function
void PI_contr_step(RT_MODEL_PI_contr_T *const PI_contr_M, ExtU_PI_contr_T
                   *PI_contr_U, ExtY_PI_contr_T *PI_contr_Y)
{
  DW_PI_contr_T *PI_contr_DW = ((DW_PI_contr_T *) PI_contr_M->dwork);
  real_T rtb_sat_vel;
  real_T rtb_Sum3;
  real_T rtb_sat_psi;

  // Sum: '<Root>/Sum2' incorporates:
  //   Inport: '<Root>/px_goal'
  //   Inport: '<Root>/px_odom'
  //   Inport: '<Root>/px_start'
  //   Inport: '<Root>/py_goal'
  //   Inport: '<Root>/py_odom'
  //   Inport: '<Root>/py_start'
  //   Inport: '<Root>/vel_odom'
  //   Inport: '<Root>/vel_ref'
  //   Math: '<Root>/Hypot'
  //   Math: '<Root>/Hypot1'
  //   Sum: '<Root>/Sum4'
  //   Sum: '<Root>/Sum5'
  //   Sum: '<Root>/Sum6'
  //   Sum: '<Root>/Sum7'
  //   Sum: '<Root>/Sum8'
  //   Sum: '<Root>/Sum9'

  rtb_sat_vel = (rt_hypotd_snf(PI_contr_U->px_goal - PI_contr_U->px_start,
    PI_contr_U->py_goal - PI_contr_U->py_start) - rt_hypotd_snf
                 (PI_contr_U->px_odom - PI_contr_U->px_start,
                  PI_contr_U->py_odom - PI_contr_U->py_start)) +
    (PI_contr_U->vel_ref - PI_contr_U->vel_odom);

  // Saturate: '<Root>/sat_vel'
  if (rtb_sat_vel > 1.0) {
    rtb_sat_vel = 1.0;
  } else {
    if (rtb_sat_vel < -1.0) {
      rtb_sat_vel = -1.0;
    }
  }

  // End of Saturate: '<Root>/sat_vel'

  // Sum: '<Root>/Sum3' incorporates:
  //   Inport: '<Root>/psi_odom'
  //   Inport: '<Root>/psi_ref'

  rtb_Sum3 = PI_contr_U->psi_ref - PI_contr_U->psi_odom;

  // Gain: '<Root>/normlz_psi' incorporates:
  //   DiscreteIntegrator: '<Root>/Integrator_psi'
  //   Sum: '<Root>/Sum1'

  rtb_sat_psi = (rtb_Sum3 + PI_contr_DW->Integrator_psi_DSTATE) *
    0.0055555555555555558;

  // Saturate: '<Root>/sat_psi'
  if (rtb_sat_psi > 0.5) {
    rtb_sat_psi = 0.5;
  } else {
    if (rtb_sat_psi < -0.5) {
      rtb_sat_psi = -0.5;
    }
  }

  // End of Saturate: '<Root>/sat_psi'

  // Outport: '<Root>/pwm_right' incorporates:
  //   Gain: '<Root>/Gain'
  //   Sum: '<Root>/Sum'

  PI_contr_Y->pwm_right = (rtb_sat_vel + rtb_sat_psi) * 20000.0;

  // Outport: '<Root>/pwm_left' incorporates:
  //   Gain: '<Root>/Gain1'
  //   Sum: '<Root>/Sum10'

  PI_contr_Y->pwm_left = (rtb_sat_vel - rtb_sat_psi) * 20000.0;

  // Update for DiscreteIntegrator: '<Root>/Integrator_psi'
  PI_contr_DW->Integrator_psi_DSTATE += 0.2 * rtb_Sum3;
}

// Model initialize function
void PI_contr_initialize(RT_MODEL_PI_contr_T *const PI_contr_M, ExtU_PI_contr_T *
  PI_contr_U, ExtY_PI_contr_T *PI_contr_Y)
{
  DW_PI_contr_T *PI_contr_DW = ((DW_PI_contr_T *) PI_contr_M->dwork);

  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // states (dwork)
  (void) memset((void *)PI_contr_DW, 0,
                sizeof(DW_PI_contr_T));

  // external inputs
  (void)memset(PI_contr_U, 0, sizeof(ExtU_PI_contr_T));

  // external outputs
  (void) memset((void *)PI_contr_Y, 0,
                sizeof(ExtY_PI_contr_T));
}

// Model terminate function
void PI_contr_terminate(RT_MODEL_PI_contr_T *const PI_contr_M)
{
  // (no terminate code required)
  UNUSED_PARAMETER(PI_contr_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
