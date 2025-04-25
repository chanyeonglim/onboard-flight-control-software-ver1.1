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
 * C/C++ source code generated on : Fri Apr 25 10:33:25 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "UAM_Status.h"

/* External inputs (root inport signals with default storage) */
ExtU_UAM_Status_T UAM_Status_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_UAM_Status_T UAM_Status_Y;

/* Real-time model */
static RT_MODEL_UAM_Status_T UAM_Status_M_;
RT_MODEL_UAM_Status_T *const UAM_Status_M = &UAM_Status_M_;

/* Model step function */
void UAM_Status_step(void)
{
  /* BusCreator generated from: '<Root>/Bus Creator' incorporates:
   *  Inport: '<Root>/Inport'
   */
  UAM_Status_Y.sensorData_l.RotorParameters = UAM_Status_U.RotorParameters_f;

  /* BusCreator: '<Root>/Bus Creator' incorporates:
   *  Constant: '<Root>/Abb'
   *  Constant: '<Root>/Constant'
   *  Constant: '<Root>/DiffPress'
   *  Constant: '<Root>/GPSCourse'
   *  Constant: '<Root>/GPSGroundSpeed'
   *  Constant: '<Root>/GPSVelocity'
   *  Constant: '<Root>/Gyro'
   *  Constant: '<Root>/LLA'
   *  Constant: '<Root>/Mag'
   *  Constant: '<Root>/rho'
   *  Outport: '<Root>/sensorData'
   */
  UAM_Status_Y.sensorData_l.GPSCourse = 0.0;
  UAM_Status_Y.sensorData_l.gndspeed = 0.0;
  UAM_Status_Y.sensorData_l.DiffPress = 0.0;
  UAM_Status_Y.sensorData_l.rho = 1.225;
  UAM_Status_Y.sensorData_l.LLA[0] = 37.0;
  UAM_Status_Y.sensorData_l.Abb[0] = 0.0;
  UAM_Status_Y.sensorData_l.Gyro[0] = 0.0;
  UAM_Status_Y.sensorData_l.Mag[0] = 1.0;
  UAM_Status_Y.sensorData_l.GPSVelocity[0] = 0.0;
  UAM_Status_Y.sensorData_l.Abe[0] = 0.0;
  UAM_Status_Y.sensorData_l.LLA[1] = 127.0;
  UAM_Status_Y.sensorData_l.Abb[1] = 0.0;
  UAM_Status_Y.sensorData_l.Gyro[1] = 0.0;
  UAM_Status_Y.sensorData_l.Mag[1] = 0.0;
  UAM_Status_Y.sensorData_l.GPSVelocity[1] = 0.0;
  UAM_Status_Y.sensorData_l.Abe[1] = 0.0;
  UAM_Status_Y.sensorData_l.LLA[2] = 100.0;
  UAM_Status_Y.sensorData_l.Abb[2] = 9.81;
  UAM_Status_Y.sensorData_l.Gyro[2] = 0.0;
  UAM_Status_Y.sensorData_l.Mag[2] = 0.0;
  UAM_Status_Y.sensorData_l.GPSVelocity[2] = 0.0;
  UAM_Status_Y.sensorData_l.Abe[2] = 0.0;
}

/* Model initialize function */
void UAM_Status_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void UAM_Status_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
