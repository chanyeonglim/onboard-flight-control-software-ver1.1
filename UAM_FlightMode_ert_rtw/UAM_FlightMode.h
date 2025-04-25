/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_FlightMode.h
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

#ifndef UAM_FlightMode_h_
#define UAM_FlightMode_h_
#ifndef UAM_FlightMode_COMMON_INCLUDES_
#define UAM_FlightMode_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* UAM_FlightMode_COMMON_INCLUDES_ */

#include "UAM_FlightMode_types.h"
#include "rt_nonfinite.h"
#include "rtGetNaN.h"
#include "zero_crossing_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
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

/* Block signals for system '<S8>/Waypoint Follower' */
typedef struct {
  real_T LAP[3];                       /* '<S8>/Waypoint Follower' */
  real_T HeadingCmd;                   /* '<S8>/Waypoint Follower' */
  real_T YawCmd;                       /* '<S8>/Waypoint Follower' */
} B_WaypointFollower_UAM_Flight_T;

/* Block states (default storage) for system '<S8>/Waypoint Follower' */
typedef struct {
  uav_sluav_internal_system_Way_T obj; /* '<S8>/Waypoint Follower' */
  boolean_T objisempty;                /* '<S8>/Waypoint Follower' */
} DW_WaypointFollower_UAM_Fligh_T;

/* Block signals (default storage) */
typedef struct {
  innerLoopCmdsBus InnerLoopCmds;      /* '<Root>/Guidance Mode Selector' */
  AAC aacSP;                           /* '<Root>/Guidance Mode Selector' */
  uint8_T Status;                      /* '<Root>/Guidance Mode Selector' */
  B_WaypointFollower_UAM_Flight_T WaypointFollower_c2;/* '<S8>/Waypoint Follower' */
  B_WaypointFollower_UAM_Flight_T WaypointFollower_c;/* '<S8>/Waypoint Follower' */
} B_UAM_FlightMode_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  uav_sluav_internal_system__de_T obj; /* '<S12>/Waypoint Follower' */
  uav_sluav_internal_system_W_d_T obj_f;/* '<S6>/Waypoint Follower' */
  uav_sluav_internal_system_Orb_T obj_l;/* '<S10>/UAV Orbit Follower' */
  uav_sluav_internal_system_Orb_T obj_b;/* '<S5>/UAV Orbit Follower' */
  uint8_T is_active_c14_UAM_FlightMode;/* '<Root>/Guidance Mode Selector' */
  uint8_T is_GuidanceLogic;            /* '<Root>/Guidance Mode Selector' */
  uint8_T is_FIXED_WING_ENTRY;         /* '<Root>/Guidance Mode Selector' */
  uint8_T is_Land;                     /* '<Root>/Guidance Mode Selector' */
  uint8_T temporalCounter_i1;          /* '<Root>/Guidance Mode Selector' */
  uint8_T mode_start;                  /* '<Root>/Guidance Mode Selector' */
  DW_WaypointFollower_UAM_Fligh_T WaypointFollower_c2;/* '<S8>/Waypoint Follower' */
  DW_WaypointFollower_UAM_Fligh_T WaypointFollower_c;/* '<S8>/Waypoint Follower' */
} DW_UAM_FlightMode_T;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState TriggeredSubsystem_Trig_ZCE;/* '<S17>/Triggered Subsystem' */
} PrevZCX_UAM_FlightMode_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real_T YawCmd;                 /* '<S11>/Yaw Cmd Sat' */
} ConstB_UAM_FlightMode_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  uint8_T mode;                        /* '<Root>/mode' */
  UAVPathManagerBus ToWP;              /* '<Root>/ToWP' */
  UAVPathManagerBus FromWP;            /* '<Root>/FromWP' */
  real_T Pose[4];                      /* '<Root>/Pose' */
  GuidanceStates States;               /* '<Root>/States' */
  real_T Ground;                       /* '<Root>/Ground' */
} ExtU_UAM_FlightMode_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  HoverSPBus HoverSP;                  /* '<Root>/HoverSP' */
  AAC aacSP;                           /* '<Root>/aacSP' */
  FixedWingCommandBus FixedWingSP;     /* '<Root>/FixedWingSP' */
  controlMode controlMode_m;           /* '<Root>/controlMode' */
  flightState FlightMode;              /* '<Root>/FlightMode' */
  uint8_T Status;                      /* '<Root>/Status' */
} ExtY_UAM_FlightMode_T;

/* Real-time Model Data Structure */
struct tag_RTM_UAM_FlightMode_T {
  const char_T * volatile errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block signals (default storage) */
extern B_UAM_FlightMode_T UAM_FlightMode_B;

/* Block states (default storage) */
extern DW_UAM_FlightMode_T UAM_FlightMode_DW;

/* Zero-crossing (trigger) state */
extern PrevZCX_UAM_FlightMode_T UAM_FlightMode_PrevZCX;

/* External inputs (root inport signals with default storage) */
extern ExtU_UAM_FlightMode_T UAM_FlightMode_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_UAM_FlightMode_T UAM_FlightMode_Y;
extern const ConstB_UAM_FlightMode_T UAM_FlightMode_ConstB;/* constant block i/o */

/* Model entry point functions */
extern void UAM_FlightMode_initialize(void);
extern void UAM_FlightMode_step(void);
extern void UAM_FlightMode_terminate(void);

/* Real-time Model object */
extern RT_MODEL_UAM_FlightMode_T *const UAM_FlightMode_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S6>/Scope' : Unused code path elimination
 * Block '<S9>/Scope' : Unused code path elimination
 * Block '<S20>/Reshape' : Unused code path elimination
 * Block '<S12>/Scope' : Unused code path elimination
 * Block '<S4>/Gain' : Eliminated nontunable gain of 1
 * Block '<Root>/Rate Transition1' : Eliminated since input and output rates are identical
 * Block '<Root>/Rate Transition2' : Eliminated since input and output rates are identical
 * Block '<Root>/Rate Transition3' : Eliminated since input and output rates are identical
 * Block '<Root>/Rate Transition4' : Eliminated since input and output rates are identical
 */

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
 * '<Root>' : 'UAM_FlightMode'
 * '<S1>'   : 'UAM_FlightMode/Compare To Constant'
 * '<S2>'   : 'UAM_FlightMode/Compare To Constant1'
 * '<S3>'   : 'UAM_FlightMode/Guidance Mode Selector'
 * '<S4>'   : 'UAM_FlightMode/Guidance Mode Selector/GuidanceLogic.BackTransition.BackTransition'
 * '<S5>'   : 'UAM_FlightMode/Guidance Mode Selector/GuidanceLogic.FIXED_WING_ORBIT.ORBIT'
 * '<S6>'   : 'UAM_FlightMode/Guidance Mode Selector/GuidanceLogic.FIXED_WING_WAYPOINT.WAYPOINT'
 * '<S7>'   : 'UAM_FlightMode/Guidance Mode Selector/GuidanceLogic.ForwardTransition'
 * '<S8>'   : 'UAM_FlightMode/Guidance Mode Selector/GuidanceLogic.Land.Descend'
 * '<S9>'   : 'UAM_FlightMode/Guidance Mode Selector/GuidanceLogic.Land.ToLand'
 * '<S10>'  : 'UAM_FlightMode/Guidance Mode Selector/GuidanceLogic.Orbit'
 * '<S11>'  : 'UAM_FlightMode/Guidance Mode Selector/GuidanceLogic.Takeoff'
 * '<S12>'  : 'UAM_FlightMode/Guidance Mode Selector/GuidanceLogic.WP'
 * '<S13>'  : 'UAM_FlightMode/Guidance Mode Selector/GuidanceLogic.BackTransition.BackTransition/MATLAB Function'
 * '<S14>'  : 'UAM_FlightMode/Guidance Mode Selector/GuidanceLogic.FIXED_WING_WAYPOINT.WAYPOINT/MATLAB Function'
 * '<S15>'  : 'UAM_FlightMode/Guidance Mode Selector/GuidanceLogic.Land.Descend/MATLAB Function'
 * '<S16>'  : 'UAM_FlightMode/Guidance Mode Selector/GuidanceLogic.Land.ToLand/MATLAB Function'
 * '<S17>'  : 'UAM_FlightMode/Guidance Mode Selector/GuidanceLogic.WP/Back Transition Guard'
 * '<S18>'  : 'UAM_FlightMode/Guidance Mode Selector/GuidanceLogic.WP/Dist2WP'
 * '<S19>'  : 'UAM_FlightMode/Guidance Mode Selector/GuidanceLogic.WP/MATLAB Function'
 * '<S20>'  : 'UAM_FlightMode/Guidance Mode Selector/GuidanceLogic.WP/Back Transition Guard/Triggered Subsystem'
 * '<S21>'  : 'UAM_FlightMode/Guidance Mode Selector/GuidanceLogic.WP/Dist2WP/Subsystem'
 */
#endif                                 /* UAM_FlightMode_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
