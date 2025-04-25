/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_Status.c
 *
 * Code generated for Simulink model 'UAM_Status'.
 *
 * Model version                  : 1.57
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Fri Apr 25 10:14:12 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "UAM_Status.h"
#include "rtwtypes.h"
#include "UAM_Status_private.h"

/* Output and update for referenced model: 'UAM_Status' */
void UAM_Status(const real_T *rtu_Inport_w1, const real_T *rtu_Inport_w2, const
                real_T *rtu_Inport_w3, const real_T *rtu_Inport_w4, const real_T
                *rtu_Inport_c1, const real_T *rtu_Inport_c2, real_T
                *rty_sensorData_GPSCourse, real_T *rty_sensorData_gndspeed,
                real_T *rty_sensorData_DiffPress, real_T *rty_sensorData_rho,
                real_T rty_sensorData_LLA[3], real_T rty_sensorData_Abb[3],
                real_T rty_sensorData_Gyro[3], real_T rty_sensorData_Mag[3],
                real_T rty_sensorData_GPSVelocity[3], real_T rty_sensorData_Abe
                [3], real_T *rty_sensorData_RotorParameters_, real_T
                *rty_sensorData_RotorParameter_e, real_T
                *rty_sensorData_RotorParameter_c, real_T
                *rty_sensorData_RotorParamete_ez, real_T
                *rty_sensorData_RotorParameter_o, real_T
                *rty_sensorData_RotorParameter_p)
{
  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   *  Constant: '<Root>/Constant'
   */
  rty_sensorData_Abe[0] = UAM_Status_ConstB.Abe[0];

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   *  Constant: '<Root>/LLA'
   */
  rty_sensorData_LLA[0] = UAM_Status_ConstB.LLA[0];

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   *  Constant: '<Root>/Abb'
   */
  rty_sensorData_Abb[0] = UAM_Status_ConstB.Abb[0];

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   *  Constant: '<Root>/Gyro'
   */
  rty_sensorData_Gyro[0] = UAM_Status_ConstB.Gyro[0];

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   *  Constant: '<Root>/Mag'
   */
  rty_sensorData_Mag[0] = UAM_Status_ConstB.Mag[0];

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   *  Constant: '<Root>/GPSVelocity'
   */
  rty_sensorData_GPSVelocity[0] = UAM_Status_ConstB.GPSVelocity[0];

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   *  Constant: '<Root>/Constant'
   */
  rty_sensorData_Abe[1] = UAM_Status_ConstB.Abe[1];

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   *  Constant: '<Root>/LLA'
   */
  rty_sensorData_LLA[1] = UAM_Status_ConstB.LLA[1];

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   *  Constant: '<Root>/Abb'
   */
  rty_sensorData_Abb[1] = UAM_Status_ConstB.Abb[1];

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   *  Constant: '<Root>/Gyro'
   */
  rty_sensorData_Gyro[1] = UAM_Status_ConstB.Gyro[1];

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   *  Constant: '<Root>/Mag'
   */
  rty_sensorData_Mag[1] = UAM_Status_ConstB.Mag[1];

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   *  Constant: '<Root>/GPSVelocity'
   */
  rty_sensorData_GPSVelocity[1] = UAM_Status_ConstB.GPSVelocity[1];

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   *  Constant: '<Root>/Constant'
   */
  rty_sensorData_Abe[2] = UAM_Status_ConstB.Abe[2];

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   *  Constant: '<Root>/LLA'
   */
  rty_sensorData_LLA[2] = UAM_Status_ConstB.LLA[2];

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   *  Constant: '<Root>/Abb'
   */
  rty_sensorData_Abb[2] = UAM_Status_ConstB.Abb[2];

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   *  Constant: '<Root>/Gyro'
   */
  rty_sensorData_Gyro[2] = UAM_Status_ConstB.Gyro[2];

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   *  Constant: '<Root>/Mag'
   */
  rty_sensorData_Mag[2] = UAM_Status_ConstB.Mag[2];

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   *  Constant: '<Root>/GPSVelocity'
   */
  rty_sensorData_GPSVelocity[2] = UAM_Status_ConstB.GPSVelocity[2];

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   */
  *rty_sensorData_GPSCourse = UAM_Status_ConstB.GPSCourse;

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   */
  *rty_sensorData_gndspeed = UAM_Status_ConstB.gndspeed;

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   */
  *rty_sensorData_DiffPress = UAM_Status_ConstB.DiffPress;

  /* SignalConversion generated from: '<Root>/sensorData' incorporates:
   *  BusCreator: '<Root>/Bus Creator'
   */
  *rty_sensorData_rho = UAM_Status_ConstB.rho;

  /* SignalConversion generated from: '<Root>/sensorData' */
  *rty_sensorData_RotorParameters_ = *rtu_Inport_w1;

  /* SignalConversion generated from: '<Root>/sensorData' */
  *rty_sensorData_RotorParameter_e = *rtu_Inport_w2;

  /* SignalConversion generated from: '<Root>/sensorData' */
  *rty_sensorData_RotorParameter_c = *rtu_Inport_w3;

  /* SignalConversion generated from: '<Root>/sensorData' */
  *rty_sensorData_RotorParamete_ez = *rtu_Inport_w4;

  /* SignalConversion generated from: '<Root>/sensorData' */
  *rty_sensorData_RotorParameter_o = *rtu_Inport_c1;

  /* SignalConversion generated from: '<Root>/sensorData' */
  *rty_sensorData_RotorParameter_p = *rtu_Inport_c2;
}

/* Model initialize function */
void UAM_Status_initialize(const char_T **rt_errorStatus, RT_MODEL_UAM_Status_T *
  const UAM_Status_M)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatusPointer(UAM_Status_M, rt_errorStatus);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
