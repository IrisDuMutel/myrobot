//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: simple_node.cpp
//
// Code generated for Simulink model 'simple_node'.
//
// Model version                  : 1.3
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Tue Jan 26 15:31:15 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "simple_node.h"
#include "simple_node_private.h"

// Block signals (default storage)
B_simple_node_T simple_node_B;

// Block states (default storage)
DW_simple_node_T simple_node_DW;

// Real-time model
RT_MODEL_simple_node_T simple_node_M_ = RT_MODEL_simple_node_T();
RT_MODEL_simple_node_T *const simple_node_M = &simple_node_M_;

// Forward declaration for local functions
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

// Model step function
void simple_node_step(void)
{
  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S1>/SourceBlock'
  Sub_simple_node_2.getLatestMessage(&simple_node_B.b_varargout_2);

  // End of Outputs for SubSystem: '<Root>/Subscribe'
}

// Model initialize function
void simple_node_initialize(void)
{
  {
    char_T tmp[6];
    int32_T i;
    static const char_T tmp_0[5] = { '/', 'o', 'd', 'o', 'm' };

    // Start for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S1>/SourceBlock'
    simple_node_DW.obj.matlabCodegenIsDeleted = false;
    simple_node_DW.obj.isInitialized = 1;
    for (i = 0; i < 5; i++) {
      tmp[i] = tmp_0[i];
    }

    tmp[5] = '\x00';
    Sub_simple_node_2.createSubscriber(tmp, 1);
    simple_node_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S1>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe'
  }
}

// Model terminate function
void simple_node_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S1>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&simple_node_DW.obj);

  // End of Terminate for SubSystem: '<Root>/Subscribe'
}

//
// File trailer for generated code.
//
// [EOF]
//
