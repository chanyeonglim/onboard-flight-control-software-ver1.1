/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_GCS_types.h
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

#ifndef UAM_GCS_types_h_
#define UAM_GCS_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_UAVPathManagerBus_
#define DEFINED_TYPEDEF_FOR_UAVPathManagerBus_

typedef struct {
  uint8_T mode;
  real_T position[3];
  real_T params[4];
} UAVPathManagerBus;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_UAM_GCS_T RT_MODEL_UAM_GCS_T;

#endif                                 /* UAM_GCS_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
