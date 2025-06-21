//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: codegen_simulink.cpp
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
#include "codegen_simulink.h"
#include "rtwtypes.h"
#include <cmath>

// External inputs (root inport signals with default storage)
ExtU_codegen_simulink_T codegen_simulink_U;

// External outputs (root outports fed by signals with default storage)
ExtY_codegen_simulink_T codegen_simulink_Y;

// Real-time model
RT_MODEL_codegen_simulink_T codegen_simulink_M_{ };

RT_MODEL_codegen_simulink_T *const codegen_simulink_M{ &codegen_simulink_M_ };

// Forward declaration for local functions
static real_T codegen_simulink_trapmf(real_T x, const real_T params[4]);
static real_T codegen_simulink_trimf(real_T x, const real_T params[3]);
static void codegen_simulink_trapmf_e(const real_T x[101], const real_T params[4],
  real_T y[101]);

// Function for MATLAB Function: '<S1>/Evaluate Rule Antecedents'
static real_T codegen_simulink_trapmf(real_T x, const real_T params[4])
{
  real_T b_x;
  real_T b_y;
  b_x = 0.0;
  b_y = 0.0;
  if (x >= params[1]) {
    b_x = 1.0;
  }

  if (x < params[0]) {
    b_x = 0.0;
  }

  if ((params[0] <= x) && (x < params[1]) && (params[0] != params[1])) {
    b_x = 1.0 / (params[1] - params[0]) * (x - params[0]);
  }

  if (x <= params[2]) {
    b_y = 1.0;
  }

  if (x > params[3]) {
    b_y = 0.0;
  }

  if ((params[2] < x) && (x <= params[3]) && (params[2] != params[3])) {
    b_y = 1.0 / (params[3] - params[2]) * (params[3] - x);
  }

  return std::fmin(b_x, b_y);
}

// Function for MATLAB Function: '<S1>/Evaluate Rule Antecedents'
static real_T codegen_simulink_trimf(real_T x, const real_T params[3])
{
  real_T y;
  y = 0.0;
  if ((params[0] != params[1]) && (params[0] < x) && (x < params[1])) {
    y = 1.0 / (params[1] - params[0]) * (x - params[0]);
  }

  if ((params[1] != params[2]) && (params[1] < x) && (x < params[2])) {
    y = 1.0 / (params[2] - params[1]) * (params[2] - x);
  }

  if (x == params[1]) {
    y = 1.0;
  }

  return y;
}

// Function for MATLAB Function: '<S1>/Evaluate Rule Consequents'
static void codegen_simulink_trapmf_e(const real_T x[101], const real_T params[4],
  real_T y[101])
{
  real_T a;
  real_T b;
  real_T c;
  real_T d;
  a = params[0];
  b = params[1];
  c = params[2];
  d = params[3];
  for (int32_T i{0}; i < 101; i++) {
    real_T b_y1;
    real_T x_0;
    real_T y2;
    b_y1 = 0.0;
    y2 = 0.0;
    x_0 = x[i];
    if (x_0 >= b) {
      b_y1 = 1.0;
    }

    if (x_0 < a) {
      b_y1 = 0.0;
    }

    if ((a <= x_0) && (x_0 < b) && (a != b)) {
      b_y1 = 1.0 / (b - a) * (x_0 - a);
    }

    if (x_0 <= c) {
      y2 = 1.0;
    }

    if (x_0 > d) {
      y2 = 0.0;
    }

    if ((c < x_0) && (x_0 <= d) && (c != d)) {
      y2 = 1.0 / (d - c) * (d - x_0);
    }

    y[i] = std::fmin(b_y1, y2);
  }
}

// Model step function
void codegen_simulink_step(void)
{
  static const real_T c_0[13]{ 1.0, 0.0, 0.5, 0.75, 0.25, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0 };

  static const real_T c[4]{ 0.5, 0.8, 1.0, 1.0 };

  static const real_T d[4]{ 15.0, 22.0, 30.0, 30.0 };

  static const real_T d_0[4]{ 0.0, 0.0, 0.2, 0.5 };

  static const real_T f[4]{ 0.0, 0.0, 7.0, 15.0 };

  static const real_T g[4]{ 2600.0, 2900.0, 3000.0, 3000.0 };

  static const real_T i[4]{ 0.0, 0.0, 500.0, 1500.0 };

  static const real_T j[4]{ 0.0, 0.0, 75.0, 200.0 };

  static const real_T l[4]{ 400.0, 700.0, 1100.0, 1100.0 };

  static const real_T m[4]{ 60.0, 180.0, 600.0, 600.0 };

  static const real_T o[4]{ 0.0, 0.0, 15.0, 30.0 };

  static const real_T e[3]{ 7.0, 15.0, 22.0 };

  static const real_T h[3]{ 1000.0, 2300.0, 2800.0 };

  static const real_T k[3]{ 100.0, 400.0, 500.0 };

  static const real_T n[3]{ 20.0, 70.0, 90.0 };

  static const int8_T b_0[52]{ 0, 0, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0,
    0, 3, -3, 1, 2, 3, 0, 0, 3, 1, 0, 0, 0, 0, 3, 3, 2, 2, 2, 0, 0, -3, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 2, 3, 0 };

  static const int8_T b_1[13]{ 1, 1, 1, 2, 3, 3, 3, 3, 2, 1, 2, 3, 1 };

  real_T outputMFCache[303];
  real_T rtb_aggregatedOutputs[101];
  real_T tmp[101];
  real_T tmp_0[101];
  real_T rtb_antecedentOutputs[13];
  real_T inputMFCache[12];
  real_T area;
  real_T mVal;
  real_T sumAntecedentOutputs;
  int32_T mfIndex;
  int32_T ruleID;
  int8_T b;

  // Outputs for Atomic SubSystem: '<Root>/Fuzzy Logic  Controller'
  // MATLAB Function: '<S1>/Evaluate Rule Antecedents' incorporates:
  //   Inport: '<Root>/Consider_Time'
  //   Inport: '<Root>/Light_Diff'
  //   Inport: '<Root>/Sun_Strength'
  //   Inport: '<Root>/Time_From_Last'

  sumAntecedentOutputs = 0.0;
  inputMFCache[0] = codegen_simulink_trapmf(codegen_simulink_U.Time_From_Last, o);
  inputMFCache[1] = codegen_simulink_trimf(codegen_simulink_U.Time_From_Last, n);
  inputMFCache[2] = codegen_simulink_trapmf(codegen_simulink_U.Time_From_Last, m);
  inputMFCache[3] = codegen_simulink_trapmf(codegen_simulink_U.Light_Diff, l);
  inputMFCache[4] = codegen_simulink_trimf(codegen_simulink_U.Light_Diff, k);
  inputMFCache[5] = codegen_simulink_trapmf(codegen_simulink_U.Light_Diff, j);
  inputMFCache[6] = codegen_simulink_trapmf(codegen_simulink_U.Sun_Strength, i);
  inputMFCache[7] = codegen_simulink_trimf(codegen_simulink_U.Sun_Strength, h);
  inputMFCache[8] = codegen_simulink_trapmf(codegen_simulink_U.Sun_Strength, g);
  inputMFCache[9] = codegen_simulink_trapmf(codegen_simulink_U.Consider_Time, f);
  inputMFCache[10] = codegen_simulink_trimf(codegen_simulink_U.Consider_Time, e);
  inputMFCache[11] = codegen_simulink_trapmf(codegen_simulink_U.Consider_Time, d);
  for (ruleID = 0; ruleID < 13; ruleID++) {
    area = 1.0;
    b = b_0[ruleID];
    mfIndex = static_cast<int32_T>(std::abs(static_cast<real_T>(b)));
    if (mfIndex != 0) {
      mVal = inputMFCache[mfIndex - 1];
      if (b < 0) {
        mVal = 1.0 - mVal;
      }

      if (mVal < 1.0) {
        area = mVal;
      }
    }

    b = b_0[ruleID + 13];
    mfIndex = static_cast<int32_T>(std::abs(static_cast<real_T>(b)));
    if (mfIndex != 0) {
      mVal = inputMFCache[mfIndex + 2];
      if (b < 0) {
        mVal = 1.0 - mVal;
      }

      if (area > mVal) {
        area = mVal;
      }
    }

    b = b_0[ruleID + 26];
    mfIndex = static_cast<int32_T>(std::abs(static_cast<real_T>(b)));
    if (mfIndex != 0) {
      mVal = inputMFCache[mfIndex + 5];
      if (b < 0) {
        mVal = 1.0 - mVal;
      }

      if (area > mVal) {
        area = mVal;
      }
    }

    b = b_0[ruleID + 39];
    mfIndex = static_cast<int32_T>(std::abs(static_cast<real_T>(b)));
    if (mfIndex != 0) {
      mVal = inputMFCache[mfIndex + 8];
      if (b < 0) {
        mVal = 1.0 - mVal;
      }

      if (area > mVal) {
        area = mVal;
      }
    }

    area *= c_0[ruleID];
    rtb_antecedentOutputs[ruleID] = area;
    sumAntecedentOutputs += area;
  }

  // MATLAB Function: '<S1>/Evaluate Rule Consequents' incorporates:
  //   Constant: '<S1>/Output Sample Points'

  codegen_simulink_trapmf_e(codegen_simulink_ConstP.OutputSamplePoints_Value,
    d_0, tmp);
  codegen_simulink_trapmf_e(codegen_simulink_ConstP.OutputSamplePoints_Value, c,
    tmp_0);
  for (ruleID = 0; ruleID < 101; ruleID++) {
    rtb_aggregatedOutputs[ruleID] = 0.0;
    outputMFCache[3 * ruleID] = tmp[ruleID];
    mfIndex = 3 * ruleID + 1;
    outputMFCache[mfIndex] = 0.0;
    area = codegen_simulink_ConstP.OutputSamplePoints_Value[ruleID];
    if ((area > 0.4) && (area < 0.5)) {
      outputMFCache[mfIndex] = (area - 0.4) * 10.000000000000002;
    }

    if ((area > 0.5) && (area < 0.6)) {
      outputMFCache[mfIndex] = (0.6 - area) * 10.000000000000002;
    }

    if (area == 0.5) {
      outputMFCache[mfIndex] = 1.0;
    }

    outputMFCache[3 * ruleID + 2] = tmp_0[ruleID];
  }

  for (ruleID = 0; ruleID < 13; ruleID++) {
    for (mfIndex = 0; mfIndex < 101; mfIndex++) {
      area = outputMFCache[(3 * mfIndex + b_1[ruleID]) - 1];
      mVal = rtb_antecedentOutputs[ruleID];
      if (!(area > mVal)) {
        if (std::isnan(area)) {
          if (std::isnan(mVal)) {
            mVal = area;
          }
        } else {
          mVal = area;
        }
      }

      if (rtb_aggregatedOutputs[mfIndex] < mVal) {
        rtb_aggregatedOutputs[mfIndex] = mVal;
      }
    }
  }

  // End of MATLAB Function: '<S1>/Evaluate Rule Consequents'

  // MATLAB Function: '<S1>/Defuzzify Outputs' incorporates:
  //   Constant: '<S1>/Output Sample Points'
  //   MATLAB Function: '<S1>/Evaluate Rule Antecedents'

  if (sumAntecedentOutputs == 0.0) {
    // Outport: '<Root>/Move_Decision'
    codegen_simulink_Y.Move_Decision = 0.5;
  } else {
    sumAntecedentOutputs = 0.0;
    area = 0.0;
    for (ruleID = 0; ruleID < 101; ruleID++) {
      area += rtb_aggregatedOutputs[ruleID];
    }

    if (area == 0.0) {
      // Outport: '<Root>/Move_Decision'
      codegen_simulink_Y.Move_Decision = 0.5;
    } else {
      for (ruleID = 0; ruleID < 101; ruleID++) {
        sumAntecedentOutputs +=
          codegen_simulink_ConstP.OutputSamplePoints_Value[ruleID] *
          rtb_aggregatedOutputs[ruleID];
      }

      // Outport: '<Root>/Move_Decision' incorporates:
      //   Constant: '<S1>/Output Sample Points'

      codegen_simulink_Y.Move_Decision = 1.0 / area * sumAntecedentOutputs;
    }
  }

  // End of MATLAB Function: '<S1>/Defuzzify Outputs'
  // End of Outputs for SubSystem: '<Root>/Fuzzy Logic  Controller'
}

// Model initialize function
void codegen_simulink_initialize(void)
{
  // (no initialization code required)
}

// Model terminate function
void codegen_simulink_terminate(void)
{
  // (no terminate code required)
}

//
// File trailer for generated code.
//
// [EOF]
//
