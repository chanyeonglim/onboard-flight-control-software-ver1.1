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
 * C/C++ source code generated on : Fri Apr 25 10:33:25 2025
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

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* External inputs (root inport signals with default storage) */
typedef struct {
  RotorParameters RotorParameters_f;   /* '<Root>/Inport' */
} ExtU_UAM_Status_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  sensorData sensorData_l;             /* '<Root>/sensorData' */
} ExtY_UAM_Status_T;

/* Real-time Model Data Structure */
struct tag_RTM_UAM_Status_T {
  const char_T * volatile errorStatus;
};

/* External inputs (root inport signals with default storage) */
extern ExtU_UAM_Status_T UAM_Status_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_UAM_Status_T UAM_Status_Y;

/* Model entry point functions */
extern void UAM_Status_initialize(void);
extern void UAM_Status_step(void);
extern void UAM_Status_terminate(void);

/* Real-time Model object */
extern RT_MODEL_UAM_Status_T *const UAM_Status_M;

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
