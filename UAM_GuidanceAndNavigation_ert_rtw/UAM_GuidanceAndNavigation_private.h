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
 * C/C++ source code generated on : Fri Apr 25 10:33:43 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef UAM_GuidanceAndNavigation_private_h_
#define UAM_GuidanceAndNavigation_private_h_
#include "rtwtypes.h"
#include "UAM_GuidanceAndNavigation_types.h"
#include "UAM_GuidanceAndNavigation.h"

extern real_T rt_modd_snf(real_T u0, real_T u1);
extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern void UAM_GuidanceA_IfActionSubsystem(real_T rtu_u, real_T rtu_absu,
  real_T *rty_signu);
extern void UAM_Guidance_IfActionSubsystem1(real_T *rty_zero);

#endif                                /* UAM_GuidanceAndNavigation_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
