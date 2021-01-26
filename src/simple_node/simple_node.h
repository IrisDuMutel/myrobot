//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: simple_node.h
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
#ifndef RTW_HEADER_simple_node_h_
#define RTW_HEADER_simple_node_h_
#include <stddef.h>
#ifndef simple_node_COMMON_INCLUDES_
# define simple_node_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "slros_initialize.h"
#endif                                 // simple_node_COMMON_INCLUDES_

#include "simple_node_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block signals (default storage)
typedef struct {
  SL_Bus_simple_node_nav_msgs_Odometry b_varargout_2;
} B_simple_node_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  ros_slros_internal_block_Subs_T obj; // '<S1>/SourceBlock'
} DW_simple_node_T;

// Parameters (default storage)
struct P_simple_node_T_ {
  SL_Bus_simple_node_nav_msgs_Odometry Out1_Y0;// Computed Parameter: Out1_Y0
                                                  //  Referenced by: '<S3>/Out1'

  SL_Bus_simple_node_nav_msgs_Odometry Constant_Value;// Computed Parameter: Constant_Value
                                                         //  Referenced by: '<S1>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_simple_node_T {
  const char_T *errorStatus;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_simple_node_T simple_node_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_simple_node_T simple_node_B;

#ifdef __cplusplus

}
#endif

// Block states (default storage)
extern DW_simple_node_T simple_node_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void simple_node_initialize(void);
  extern void simple_node_step(void);
  extern void simple_node_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_simple_node_T *const simple_node_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S2>/signal1' : Unused code path elimination


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
//  '<Root>' : 'simple_node'
//  '<S1>'   : 'simple_node/Subscribe'
//  '<S2>'   : 'simple_node/Subsystem'
//  '<S3>'   : 'simple_node/Subscribe/Enabled Subsystem'

#endif                                 // RTW_HEADER_simple_node_h_

//
// File trailer for generated code.
//
// [EOF]
//
