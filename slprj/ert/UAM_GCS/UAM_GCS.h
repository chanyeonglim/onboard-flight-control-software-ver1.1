/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_GCS.h
 *
 * Code generated for Simulink model 'UAM_GCS'.
 *
 * Model version                  : 1.20
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Fri Apr 25 10:09:23 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef UAM_GCS_h_
#define UAM_GCS_h_
#ifndef UAM_GCS_COMMON_INCLUDES_
#define UAM_GCS_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* UAM_GCS_COMMON_INCLUDES_ */

#include "UAM_GCS_types.h"

/* Block states (default storage) for model 'UAM_GCS' */
typedef struct {
  volatile UAVPathManagerBus RateTransition_Buffer[18];/* '<Root>/Rate Transition' */
  volatile int8_T RateTransition_ActiveBufIdx;/* '<Root>/Rate Transition' */
} DW_UAM_GCS_f_T;

/* Real-time Model Data Structure */
struct tag_RTM_UAM_GCS_T {
  const char_T **errorStatus;
};

typedef struct {
  DW_UAM_GCS_f_T rtdw;
  RT_MODEL_UAM_GCS_T rtm;
} MdlrefDW_UAM_GCS_T;

/* Model reference registration function */
extern void UAM_GCS_initialize(const char_T **rt_errorStatus, RT_MODEL_UAM_GCS_T
  *const UAM_GCS_M);
extern void UAM_GCSTID0(UAVPathManagerBus rty_GlobalMission[9], DW_UAM_GCS_f_T
  *localDW);
extern void UAM_GCSTID1(DW_UAM_GCS_f_T *localDW);

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
 * '<Root>' : 'UAM_GCS'
 * '<S1>'   : 'UAM_GCS/Get Flight Mission'
 * '<S2>'   : 'UAM_GCS/Get Flight Mission/noQGC'
 */
#endif                                 /* UAM_GCS_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
