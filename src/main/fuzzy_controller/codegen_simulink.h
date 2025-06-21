//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: codegen_simulink.h
//
// Code generated for Simulink model 'codegen_simulink'.
//
// Model version                  : 1.3
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Sun Jun  8 12:16:43 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_codegen_simulink_h_
#define RTW_HEADER_codegen_simulink_h_
#include "rtwtypes.h"
#include "codegen_simulink_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// Constant parameters (default storage)
struct ConstP_codegen_simulink_T {
  // Expression: fis.outputSamplePoints
  //  Referenced by: '<S1>/Output Sample Points'

  real_T OutputSamplePoints_Value[101];
};

// External inputs (root inport signals with default storage)
struct ExtU_codegen_simulink_T {
  real_T Time_From_Last;               // '<Root>/Time_From_Last'
  real_T Light_Diff;                   // '<Root>/Light_Diff'
  real_T Sun_Strength;                 // '<Root>/Sun_Strength'
  real_T Consider_Time;                // '<Root>/Consider_Time'
};

// External outputs (root outports fed by signals with default storage)
struct ExtY_codegen_simulink_T {
  real_T Move_Decision;                // '<Root>/Move_Decision'
};

// Real-time Model Data Structure
struct tag_RTM_codegen_simulink_T {
  const char_T * volatile errorStatus;
};

#ifdef __cplusplus

extern "C"
{

#endif

  // External inputs (root inport signals with default storage)
  extern struct ExtU_codegen_simulink_T codegen_simulink_U;

  // External outputs (root outports fed by signals with default storage)
  extern struct ExtY_codegen_simulink_T codegen_simulink_Y;

#ifdef __cplusplus

}

#endif

// Constant parameters (default storage)
extern const ConstP_codegen_simulink_T codegen_simulink_ConstP;

#ifdef __cplusplus

extern "C"
{

#endif

  // Model entry point functions
  extern void codegen_simulink_initialize(void);
  extern void codegen_simulink_step(void);
  extern void codegen_simulink_terminate(void);

#ifdef __cplusplus

}

#endif

// Real-time Model object
#ifdef __cplusplus

extern "C"
{

#endif

  extern RT_MODEL_codegen_simulink_T *const codegen_simulink_M;

#ifdef __cplusplus

}

#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S1>/InputConversion' : Eliminate redundant data type conversion


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
//  '<Root>' : 'codegen_simulink'
//  '<S1>'   : 'codegen_simulink/Fuzzy Logic  Controller'
//  '<S2>'   : 'codegen_simulink/Fuzzy Logic  Controller/Defuzzify Outputs'
//  '<S3>'   : 'codegen_simulink/Fuzzy Logic  Controller/Evaluate Rule Antecedents'
//  '<S4>'   : 'codegen_simulink/Fuzzy Logic  Controller/Evaluate Rule Consequents'

#endif                                 // RTW_HEADER_codegen_simulink_h_

//
// File trailer for generated code.
//
// [EOF]
//
