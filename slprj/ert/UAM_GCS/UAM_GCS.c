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
 * C/C++ source code generated on : Fri Apr 25 10:09:23 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "UAM_GCS.h"
#include "UAM_GCS_types.h"
#include "rtwtypes.h"
#include "UAM_GCS_private.h"

/* Output and update for referenced model: 'UAM_GCS' */
void UAM_GCSTID0(UAVPathManagerBus rty_GlobalMission[9], DW_UAM_GCS_f_T *localDW)
{
  int32_T i;
  int32_T tmp;

  /* RateTransition: '<Root>/Rate Transition' */
  tmp = localDW->RateTransition_ActiveBufIdx * 9;
  for (i = 0; i < 9; i++) {
    rty_GlobalMission[i] = localDW->RateTransition_Buffer[i + tmp];
  }

  /* End of RateTransition: '<Root>/Rate Transition' */
}

/* Output and update for referenced model: 'UAM_GCS' */
void UAM_GCSTID1(DW_UAM_GCS_f_T *localDW)
{
  int32_T i;

  /* RateTransition: '<Root>/Rate Transition' incorporates:
   *  Constant: '<S2>/Waypoints2'
   */
  for (i = 0; i < 9; i++) {
    localDW->RateTransition_Buffer[i + (localDW->RateTransition_ActiveBufIdx ==
      0) * 9] = UAM_GCS_ConstP.Waypoints2_Value[i];
  }

  localDW->RateTransition_ActiveBufIdx = (int8_T)
    (localDW->RateTransition_ActiveBufIdx == 0);

  /* End of RateTransition: '<Root>/Rate Transition' */
}

/* Model initialize function */
void UAM_GCS_initialize(const char_T **rt_errorStatus, RT_MODEL_UAM_GCS_T *const
  UAM_GCS_M)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatusPointer(UAM_GCS_M, rt_errorStatus);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
