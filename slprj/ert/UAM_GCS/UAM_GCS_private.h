/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_GCS_private.h
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

#ifndef UAM_GCS_private_h_
#define UAM_GCS_private_h_
#include "rtwtypes.h"
#include "UAM_GCS_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         (*((rtm)->errorStatus))
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    (*((rtm)->errorStatus) = (val))
#endif

#ifndef rtmGetErrorStatusPointer
#define rtmGetErrorStatusPointer(rtm)  (rtm)->errorStatus
#endif

#ifndef rtmSetErrorStatusPointer
#define rtmSetErrorStatusPointer(rtm, val) ((rtm)->errorStatus = (val))
#endif

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: Waypoints2_Value
   * Referenced by: '<S2>/Waypoints2'
   */
  UAVPathManagerBus Waypoints2_Value[9];
} ConstP_UAM_GCS_T;

/* Constant parameters (default storage) */
extern const ConstP_UAM_GCS_T UAM_GCS_ConstP;

#endif                                 /* UAM_GCS_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
