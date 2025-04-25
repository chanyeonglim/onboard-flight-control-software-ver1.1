/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_Status_types.h
 *
 * Code generated for Simulink model 'UAM_Status'.
 *
 * Model version                  : 1.57
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Fri Apr 25 10:33:25 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef UAM_Status_types_h_
#define UAM_Status_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_RotorParameters_
#define DEFINED_TYPEDEF_FOR_RotorParameters_

typedef struct {
  real_T w1;
  real_T w2;
  real_T w3;
  real_T w4;
  real_T c1;
  real_T c2;
} RotorParameters;

#endif

#ifndef DEFINED_TYPEDEF_FOR_sensorData_
#define DEFINED_TYPEDEF_FOR_sensorData_

typedef struct {
  real_T GPSCourse;
  real_T gndspeed;
  real_T DiffPress;
  real_T rho;
  real_T LLA[3];
  real_T Abb[3];
  real_T Gyro[3];
  real_T Mag[3];
  real_T GPSVelocity[3];
  real_T Abe[3];
  RotorParameters RotorParameters;
} sensorData;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_UAM_Status_T RT_MODEL_UAM_Status_T;

#endif                                 /* UAM_Status_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
