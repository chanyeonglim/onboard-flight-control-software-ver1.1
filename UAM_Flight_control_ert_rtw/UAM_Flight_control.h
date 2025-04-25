/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_Flight_control.h
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

#ifndef UAM_Flight_control_h_
#define UAM_Flight_control_h_
#ifndef UAM_Flight_control_COMMON_INCLUDES_
#define UAM_Flight_control_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* UAM_Flight_control_COMMON_INCLUDES_ */

#include "UAM_Flight_control_types.h"
#include "UAM_Status.h"
#include "UAM_GCS.h"
#include "UAM_GuidanceAndNavigation.h"
#include "UAM_FlightMode.h"
#include "zero_crossing_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmCounterLimit
#define rtmCounterLimit(rtm, idx)      ((rtm)->Timing.TaskCounters.cLimit[(idx)])
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetErrorStatusPointer
#define rtmGetErrorStatusPointer(rtm)  ((const char_T **)(&((rtm)->errorStatus)))
#endif

#ifndef rtmStepTask
#define rtmStepTask(rtm, idx)          ((rtm)->Timing.TaskCounters.TID[(idx)] == 0)
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmTaskCounter
#define rtmTaskCounter(rtm, idx)       ((rtm)->Timing.TaskCounters.TID[(idx)])
#endif

/* Block signals (default storage) */
typedef struct {
  GuidanceStates BusConversion_InsertedFor_Model;
  UAVPathManagerBus UAM_GCS_l[9];      /* '<Root>/UAM_GCS' */
  UAVPathManagerBus BusConversion_InsertedFor_Mod_d;
  UAVPathManagerBus BusConversion_InsertedFor_Mod_m;
  real_T params[4];                    /* '<Root>/UAM_GuidanceAndNavigation' */
  real_T params_a[4];                  /* '<Root>/UAM_GuidanceAndNavigation' */
  real_T UAM_GuidanceAndNavigation_o[4];/* '<Root>/UAM_GuidanceAndNavigation' */
  real_T Euler[3];                     /* '<Root>/UAM_GuidanceAndNavigation' */
  real_T pqr[3];                       /* '<Root>/UAM_GuidanceAndNavigation' */
  real_T Ve[3];                        /* '<Root>/UAM_GuidanceAndNavigation' */
  real_T Xe[3];                        /* '<Root>/UAM_GuidanceAndNavigation' */
  real_T position[3];                  /* '<Root>/UAM_GuidanceAndNavigation' */
  real_T position_i[3];                /* '<Root>/UAM_GuidanceAndNavigation' */
  real_T LLA[3];                       /* '<Root>/UAM_Status' */
  real_T Abb[3];                       /* '<Root>/UAM_Status' */
  real_T Gyro[3];                      /* '<Root>/UAM_Status' */
  real_T Mag[3];                       /* '<Root>/UAM_Status' */
  real_T GPSVelocity[3];               /* '<Root>/UAM_Status' */
  real_T Abe[3];                       /* '<Root>/UAM_Status' */
  real_T course;                       /* '<Root>/UAM_GuidanceAndNavigation' */
  real_T c1;                           /* '<Root>/UAM_GuidanceAndNavigation' */
  real_T TmpRTBAtModelInport4[4];
  real_T X;                            /* '<Root>/Model' */
  real_T Y;                            /* '<Root>/Model' */
  real_T Z;                            /* '<Root>/Model' */
  real_T Yaw;                          /* '<Root>/Model' */
  real_T UAM_GuidanceAndNavigation_o15;/* '<Root>/UAM_GuidanceAndNavigation' */
  real_T GPSCourse;                    /* '<Root>/UAM_Status' */
  real_T gndspeed;                     /* '<Root>/UAM_Status' */
  real_T DiffPress;                    /* '<Root>/UAM_Status' */
  real_T rho;                          /* '<Root>/UAM_Status' */
  real_T w1;                           /* '<Root>/UAM_Status' */
  real_T w2;                           /* '<Root>/UAM_Status' */
  real_T w3;                           /* '<Root>/UAM_Status' */
  real_T w4;                           /* '<Root>/UAM_Status' */
  real_T c1_m;                         /* '<Root>/UAM_Status' */
  real_T c2;                           /* '<Root>/UAM_Status' */
  flightState Model_o21;               /* '<Root>/Model' */
  uint8_T RateTransition;              /* '<Root>/Rate Transition' */
  uint8_T UAM_GuidanceAndNavigation_o1;/* '<Root>/UAM_GuidanceAndNavigation' */
  uint8_T TmpRTBAtModelInport1;
  uint8_T mode_h;                      /* '<Root>/UAM_GuidanceAndNavigation' */
  uint8_T mode_b;                      /* '<Root>/UAM_GuidanceAndNavigation' */
  boolean_T DataTypeConversion;        /* '<Root>/Data Type Conversion' */
  boolean_T b;
} B_UAM_Flight_control_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T params_Buffer[4];             /* synthesized block */
  real_T params_Buffer_a[4];           /* synthesized block */
  real_T TmpRTBAtModelInport4_Buffer[4];/* synthesized block */
  real_T position_Buffer[3];           /* synthesized block */
  real_T position_Buffer_b[3];         /* synthesized block */
  real_T Euler_Buffer[3];              /* synthesized block */
  real_T Ve_Buffer[3];                 /* synthesized block */
  real_T Xe_Buffer[3];                 /* synthesized block */
  real_T pqr_Buffer[3];                /* synthesized block */
  real_T c1_Buffer;                    /* synthesized block */
  real_T course_Buffer;                /* synthesized block */
  uint8_T RateTransition_Buffer0;      /* '<Root>/Rate Transition' */
  uint8_T mode_Buffer;                 /* synthesized block */
  uint8_T mode_Buffer_k;               /* synthesized block */
  uint8_T TmpRTBAtModelInport1_Buffer; /* synthesized block */
  MdlrefDW_UAM_Status_T UAM_Status_InstanceData;/* '<Root>/UAM_Status' */
  MdlrefDW_UAM_GCS_T UAM_GCS_InstanceData;/* '<Root>/UAM_GCS' */
  MdlrefDW_UAM_GuidanceAndNavig_T UAM_GuidanceAndNavigation_Insta;/* '<Root>/UAM_GuidanceAndNavigation' */
  MdlrefDW_UAM_FlightMode_T Model_InstanceData;/* '<Root>/Model' */
} DW_UAM_Flight_control_T;

/* Real-time Model Data Structure */
struct tag_RTM_UAM_Flight_control_T {
  const char_T *errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    struct {
      uint32_T TID[2];
      uint32_T cLimit[2];
    } TaskCounters;

    struct {
      uint32_T TID0_1;
      boolean_T b_TID0_1;
    } RateInteraction;

    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block signals (default storage) */
extern B_UAM_Flight_control_T UAM_Flight_control_B;

/* Block states (default storage) */
extern DW_UAM_Flight_control_T UAM_Flight_control_DW;

/* External data declarations for dependent source files */
extern const real_T UAM_Flight_control_RGND;/* real_T ground */

/* Model entry point functions */
extern void UAM_Flight_control_initialize(void);
extern void UAM_Flight_control_step0(void);/* Sample time: [0.005s, 0.0s] */
extern void UAM_Flight_control_step1(void);/* Sample time: [0.08s, 0.0s] */
extern void UAM_Flight_control_terminate(void);

/* Real-time Model object */
extern RT_MODEL_UAM_Flight_control_T *const UAM_Flight_control_M;

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
 * '<Root>' : 'UAM_Flight_control'
 */
#endif                                 /* UAM_Flight_control_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
