/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_FlightMode.h
 *
 * Code generated for Simulink model 'UAM_FlightMode'.
 *
 * Model version                  : 1.127
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Fri Apr 25 09:58:43 2025
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

/* Block signals for model 'UAM_FlightMode' */
typedef struct {
  innerLoopCmdsBus InnerLoopCmds;      /* '<Root>/Guidance Mode Selector' */
  AAC aacSP;                           /* '<Root>/Guidance Mode Selector' */
  FixedWingCommandBus FixedWingSP;     /* '<Root>/Guidance Mode Selector' */
  controlMode controlMode_m;           /* '<Root>/Guidance Mode Selector' */
  flightState FlightMode;              /* '<Root>/Guidance Mode Selector' */
  uint8_T Status;                      /* '<Root>/Guidance Mode Selector' */
  B_WaypointFollower_UAM_Flight_T WaypointFollower_c2;/* '<S8>/Waypoint Follower' */
  B_WaypointFollower_UAM_Flight_T WaypointFollower_c;/* '<S8>/Waypoint Follower' */
} B_UAM_FlightMode_c_T;

/* Block states (default storage) for model 'UAM_FlightMode' */
typedef struct {
  uav_sluav_internal_system__cc_T obj; /* '<S12>/Waypoint Follower' */
  uav_sluav_internal_system_W_c_T obj_f;/* '<S6>/Waypoint Follower' */
  uav_sluav_internal_system_Orb_T obj_l;/* '<S10>/UAV Orbit Follower' */
  uav_sluav_internal_system_Orb_T obj_b;/* '<S5>/UAV Orbit Follower' */
  uint8_T is_active_c14_UAM_FlightMode;/* '<Root>/Guidance Mode Selector' */
  uint8_T is_GuidanceLogic;            /* '<Root>/Guidance Mode Selector' */
  uint8_T is_BackTransition;           /* '<Root>/Guidance Mode Selector' */
  uint8_T is_FIXED_WING_ENTRY;         /* '<Root>/Guidance Mode Selector' */
  uint8_T is_FIXED_WING_ORBIT;         /* '<Root>/Guidance Mode Selector' */
  uint8_T is_FIXED_WING_WAYPOINT;      /* '<Root>/Guidance Mode Selector' */
  uint8_T is_Land;                     /* '<Root>/Guidance Mode Selector' */
  uint8_T temporalCounter_i1;          /* '<Root>/Guidance Mode Selector' */
  uint8_T mode_prev;                   /* '<Root>/Guidance Mode Selector' */
  uint8_T mode_start;                  /* '<Root>/Guidance Mode Selector' */
  boolean_T objisempty;                /* '<S12>/Waypoint Follower' */
  boolean_T objisempty_n;              /* '<S10>/UAV Orbit Follower' */
  boolean_T objisempty_m;              /* '<S6>/Waypoint Follower' */
  boolean_T objisempty_b;              /* '<S5>/UAV Orbit Follower' */
  DW_WaypointFollower_UAM_Fligh_T WaypointFollower_c2;/* '<S8>/Waypoint Follower' */
  DW_WaypointFollower_UAM_Fligh_T WaypointFollower_c;/* '<S8>/Waypoint Follower' */
} DW_UAM_FlightMode_f_T;

/* Zero-crossing (trigger) state for model 'UAM_FlightMode' */
typedef struct {
  ZCSigState TriggeredSubsystem_Trig_ZCE;/* '<S17>/Triggered Subsystem' */
} ZCE_UAM_FlightMode_T;

/* Invariant block signals for model 'UAM_FlightMode' */
typedef struct {
  const real_T YawCmd;                 /* '<S11>/Yaw Cmd Sat' */
} ConstB_UAM_FlightMode_h_T;

/* Real-time Model Data Structure */
struct tag_RTM_UAM_FlightMode_T {
  const char_T **errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    boolean_T *stopRequestedFlag;
  } Timing;
};

typedef struct {
  B_UAM_FlightMode_c_T rtb;
  DW_UAM_FlightMode_f_T rtdw;
  RT_MODEL_UAM_FlightMode_T rtm;
  ZCE_UAM_FlightMode_T rtzce;
} MdlrefDW_UAM_FlightMode_T;

/* Model reference registration function */
extern void UAM_FlightMode_initialize(const char_T **rt_errorStatus, boolean_T
  *rt_stopRequested, RT_MODEL_UAM_FlightMode_T *const UAM_FlightMode_M,
  ZCE_UAM_FlightMode_T *localZCE);
extern void UAM_Fligh_WaypointFollower_Init(DW_WaypointFollower_UAM_Fligh_T
  *localDW);
extern void UAM_FlightMode_WaypointFollower(const real_T rtu_0[4], const real_T
  rtu_1[8], real_T rtu_2, B_WaypointFollower_UAM_Flight_T *localB,
  DW_WaypointFollower_UAM_Fligh_T *localDW);
extern void UAM_FlightMode_Init(DW_UAM_FlightMode_f_T *localDW);
extern void UAM_FlightMode(RT_MODEL_UAM_FlightMode_T * const UAM_FlightMode_M,
  const uint8_T *rtu_mode, const UAVPathManagerBus *rtu_ToWP, const
  UAVPathManagerBus *rtu_FromWP, const real_T rtu_Pose[4], const GuidanceStates *
  rtu_States, const real_T *rtu_Ground, real_T *rty_HoverSP_X, real_T
  *rty_HoverSP_Y, real_T *rty_HoverSP_Z, real_T *rty_HoverSP_Yaw, real_T
  *rty_aacSP_airspeed, real_T *rty_aacSP_altitude, real_T *rty_aacSP_course,
  real_T *rty_aacSP_L1, real_T *rty_aacSP_climbrate, real_T
  *rty_FixedWingSP_roll, real_T *rty_FixedWingSP_pitch, real_T
  *rty_FixedWingSP_yaw, real_T *rty_FixedWingSP_airspeed, uint8_T
  *rty_controlMode_lateralGuidance, uint8_T *rty_controlMode_airspeedAltitud,
  uint8_T *rty_controlMode_attitude, uint8_T *rty_controlMode_manual, uint8_T
  *rty_controlMode_armed, uint8_T *rty_controlMode_inTransition, uint8_T
  *rty_controlMode_TransitionCondi, flightState *rty_FlightMode, uint8_T
  *rty_Status, B_UAM_FlightMode_c_T *localB, DW_UAM_FlightMode_f_T *localDW);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S6>/Scope' : Unused code path elimination
 * Block '<S9>/Scope' : Unused code path elimination
 * Block '<S20>/Reshape' : Unused code path elimination
 * Block '<S12>/Scope' : Unused code path elimination
 * Block '<S4>/Gain' : Eliminated nontunable gain of 1
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
