/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_Status.h
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

#ifndef UAM_Status_h_
#define UAM_Status_h_
#ifndef UAM_Status_COMMON_INCLUDES_
#define UAM_Status_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* UAM_Status_COMMON_INCLUDES_ */

#include "UAM_Status_types.h"

/* Invariant block signals for model 'UAM_Status' */
typedef struct {
  const real_T Abb[3];                 /* '<Root>/Abb' */
  const real_T Abe[3];                 /* '<Root>/Constant' */
  const real_T DiffPress;              /* '<Root>/DiffPress' */
  const real_T GPSCourse;              /* '<Root>/GPSCourse' */
  const real_T gndspeed;               /* '<Root>/GPSGroundSpeed' */
  const real_T GPSVelocity[3];         /* '<Root>/GPSVelocity' */
  const real_T Gyro[3];                /* '<Root>/Gyro' */
  const real_T LLA[3];                 /* '<Root>/LLA' */
  const real_T Mag[3];                 /* '<Root>/Mag' */
  const real_T rho;                    /* '<Root>/rho' */
} ConstB_UAM_Status_h_T;

/* Real-time Model Data Structure */
struct tag_RTM_UAM_Status_T {
  const char_T **errorStatus;
};

typedef struct {
  RT_MODEL_UAM_Status_T rtm;
} MdlrefDW_UAM_Status_T;

/* Model reference registration function */
extern void UAM_Status_initialize(const char_T **rt_errorStatus,
  RT_MODEL_UAM_Status_T *const UAM_Status_M);
extern void UAM_Status(const real_T *rtu_Inport_w1, const real_T *rtu_Inport_w2,
  const real_T *rtu_Inport_w3, const real_T *rtu_Inport_w4, const real_T
  *rtu_Inport_c1, const real_T *rtu_Inport_c2, real_T *rty_sensorData_GPSCourse,
  real_T *rty_sensorData_gndspeed, real_T *rty_sensorData_DiffPress, real_T
  *rty_sensorData_rho, real_T rty_sensorData_LLA[3], real_T rty_sensorData_Abb[3],
  real_T rty_sensorData_Gyro[3], real_T rty_sensorData_Mag[3], real_T
  rty_sensorData_GPSVelocity[3], real_T rty_sensorData_Abe[3], real_T
  *rty_sensorData_RotorParameters_, real_T *rty_sensorData_RotorParameter_e,
  real_T *rty_sensorData_RotorParameter_c, real_T
  *rty_sensorData_RotorParamete_ez, real_T *rty_sensorData_RotorParameter_o,
  real_T *rty_sensorData_RotorParameter_p);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'UAM_Status'
 */
#endif                                 /* UAM_Status_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
