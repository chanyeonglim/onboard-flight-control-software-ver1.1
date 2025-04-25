/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_FlightMode_private.h
 *
 * Code generated for Simulink model 'UAM_FlightMode'.
 *
 * Model version                  : 1.128
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Fri Apr 25 10:34:19 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef UAM_FlightMode_private_h_
#define UAM_FlightMode_private_h_
#include "rtwtypes.h"
#include "zero_crossing_types.h"
#include "UAM_FlightMode.h"
#include "UAM_FlightMode_types.h"

extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern void UAM_Fligh_WaypointFollower_Init(DW_WaypointFollower_UAM_Fligh_T
  *localDW);
extern void UAM_FlightMode_WaypointFollower(const real_T rtu_0[4], const real_T
  rtu_1[8], real_T rtu_2, B_WaypointFollower_UAM_Flight_T *localB,
  DW_WaypointFollower_UAM_Fligh_T *localDW);

#endif                                 /* UAM_FlightMode_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
