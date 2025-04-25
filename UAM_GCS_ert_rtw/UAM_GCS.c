/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_GCS.c
 *
 * Code generated for Simulink model 'UAM_GCS'.
 *
 * Model version                  : 1.20
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Fri Apr 25 10:34:23 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "UAM_GCS.h"
#include "rtwtypes.h"

/* Block states (default storage) */
DW_UAM_GCS_T UAM_GCS_DW;

/* External outputs (root outports fed by signals with default storage) */
ExtY_UAM_GCS_T UAM_GCS_Y;

/* Real-time model */
static RT_MODEL_UAM_GCS_T UAM_GCS_M_;
RT_MODEL_UAM_GCS_T *const UAM_GCS_M = &UAM_GCS_M_;

/* Model step function for TID0 */
void UAM_GCS_step0(void)               /* Sample time: [0.005s, 0.0s] */
{
  int32_T i;
  int32_T tmp;

  /* RateTransition: '<Root>/Rate Transition' */
  tmp = UAM_GCS_DW.RateTransition_ActiveBufIdx * 9;
  for (i = 0; i < 9; i++) {
    UAM_GCS_Y.GlobalMission[i] = UAM_GCS_DW.RateTransition_Buffer[i + tmp];
  }

  /* End of RateTransition: '<Root>/Rate Transition' */
}

/* Model step function for TID1 */
void UAM_GCS_step1(void)               /* Sample time: [0.08s, 0.0s] */
{
  int32_T i;

  /* RateTransition: '<Root>/Rate Transition' incorporates:
   *  Constant: '<S2>/Waypoints2'
   */
  for (i = 0; i < 9; i++) {
    UAM_GCS_DW.RateTransition_Buffer[i + (UAM_GCS_DW.RateTransition_ActiveBufIdx
      == 0) * 9] = UAM_GCS_ConstP.Waypoints2_Value[i];
  }

  UAM_GCS_DW.RateTransition_ActiveBufIdx = (int8_T)
    (UAM_GCS_DW.RateTransition_ActiveBufIdx == 0);

  /* End of RateTransition: '<Root>/Rate Transition' */
}

/* Model initialize function */
void UAM_GCS_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void UAM_GCS_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
