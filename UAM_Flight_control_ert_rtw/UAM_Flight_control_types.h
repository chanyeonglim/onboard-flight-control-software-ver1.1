/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_Flight_control_types.h
 *
 * Code generated for Simulink model 'UAM_Flight_control'.
 *
 * Model version                  : 1.88
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Fri Apr 25 10:14:16 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef UAM_Flight_control_types_h_
#define UAM_Flight_control_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_UAVPathManagerBus_
#define DEFINED_TYPEDEF_FOR_UAVPathManagerBus_

typedef struct {
  uint8_T mode;
  real_T position[3];
  real_T params[4];
} UAVPathManagerBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_GuidanceStates_
#define DEFINED_TYPEDEF_FOR_GuidanceStates_

typedef struct {
  real_T Euler[3];
  real_T pqr[3];
  real_T Ve[3];
  real_T Xe[3];
  real_T course;
  real_T c1;
} GuidanceStates;

#endif

#ifndef DEFINED_TYPEDEF_FOR_flightState_
#define DEFINED_TYPEDEF_FOR_flightState_

typedef enum {
  Hover = 0,                           /* Default value */
  Transition,
  FixedWing,
  BackTransition
} flightState;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_UAM_Flight_control_T RT_MODEL_UAM_Flight_control_T;

#endif                                 /* UAM_Flight_control_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
