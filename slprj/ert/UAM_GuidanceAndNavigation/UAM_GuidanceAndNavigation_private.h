/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_GuidanceAndNavigation_private.h
 *
 * Code generated for Simulink model 'UAM_GuidanceAndNavigation'.
 *
 * Model version                  : 1.80
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Fri Apr 25 10:13:17 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef UAM_GuidanceAndNavigation_private_h_
#define UAM_GuidanceAndNavigation_private_h_
#include "rtwtypes.h"
#include "UAM_GuidanceAndNavigation.h"
#include "UAM_GuidanceAndNavigation_types.h"

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

/* Invariant block signals (default storage) */
extern const ConstB_UAM_GuidanceAndNavig_h_T UAM_GuidanceAndNavigatio_ConstB;

#endif                                /* UAM_GuidanceAndNavigation_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
