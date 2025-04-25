/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_FlightMode_types.h
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

#ifndef UAM_FlightMode_types_h_
#define UAM_FlightMode_types_h_
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

#ifndef DEFINED_TYPEDEF_FOR_HoverSPBus_
#define DEFINED_TYPEDEF_FOR_HoverSPBus_

typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
  real_T Yaw;
} HoverSPBus;

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

#ifndef DEFINED_TYPEDEF_FOR_GdncModeType_
#define DEFINED_TYPEDEF_FOR_GdncModeType_

typedef enum {
  GdncModeType_None = 0,               /* Default value */
  GdncModeType_Start,
  GdncModeType_PreTransition,
  GdncModeType_BackTransition,
  GdncModeType_WAYPOINT,
  GdncModeType_Stabilize,
  GdncModeType_FIXEDWINGFLIGHT,
  GdncModeType_ORBIT,
  GdncModeType_HOVER_ENTRY,
  GdncModeType_WP,
  GdncModeType_Orbit,
  GdncModeType_ForwardTransition,
  GdncModeType_FWCOMPLETE,
  GdncModeType_ToLand,
  GdncModeType_Descend,
  GdncModeType_Takeoff
} GdncModeType;

#endif

#ifndef DEFINED_TYPEDEF_FOR_AAC_
#define DEFINED_TYPEDEF_FOR_AAC_

typedef struct {
  real_T airspeed;
  real_T altitude;
  real_T course;
  real_T L1;
  real_T climbrate;
} AAC;

#endif

#ifndef DEFINED_TYPEDEF_FOR_FixedWingCommandBus_
#define DEFINED_TYPEDEF_FOR_FixedWingCommandBus_

typedef struct {
  real_T roll;
  real_T pitch;
  real_T yaw;
  real_T airspeed;
} FixedWingCommandBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_controlMode_
#define DEFINED_TYPEDEF_FOR_controlMode_

typedef struct {
  uint8_T lateralGuidance;
  uint8_T airspeedAltitude;
  uint8_T attitude;
  uint8_T manual;
  uint8_T armed;
  uint8_T inTransition;
  uint8_T TransitionCondition;
} controlMode;

#endif

#ifndef DEFINED_TYPEDEF_FOR_innerLoopCmdsBus_
#define DEFINED_TYPEDEF_FOR_innerLoopCmdsBus_

typedef struct {
  real_T LAP[3];
  real_T HeadingCmd;
  real_T YawCmd;
} innerLoopCmdsBus;

#endif

#ifndef struct_tag_BlgwLpgj2bjudmbmVKWwDE
#define struct_tag_BlgwLpgj2bjudmbmVKWwDE

struct tag_BlgwLpgj2bjudmbmVKWwDE
{
  uint32_T f1[8];
};

#endif                                 /* struct_tag_BlgwLpgj2bjudmbmVKWwDE */

#ifndef typedef_cell_wrap_UAM_FlightMode_T
#define typedef_cell_wrap_UAM_FlightMode_T

typedef struct tag_BlgwLpgj2bjudmbmVKWwDE cell_wrap_UAM_FlightMode_T;

#endif                                 /* typedef_cell_wrap_UAM_FlightMode_T */

#ifndef struct_tag_vxlth3EMPLG7CAzUleegd
#define struct_tag_vxlth3EMPLG7CAzUleegd

struct tag_vxlth3EMPLG7CAzUleegd
{
  int32_T isInitialized;
  cell_wrap_UAM_FlightMode_T inputVarSize[3];
  real_T LookaheadDistance;
  real_T WaypointIndex;
  real_T InitYaw;
  real_T FinalYaw;
  real_T NumWaypoints;
  real_T WaypointsInternal[8];
  boolean_T LastWaypointFlag;
  boolean_T StartFlag;
  real_T InitialPose[4];
  real_T LookaheadFactor;
  uint8_T LookaheadDistFlag;
};

#endif                                 /* struct_tag_vxlth3EMPLG7CAzUleegd */

#ifndef typedef_uav_sluav_internal_system_Way_T
#define typedef_uav_sluav_internal_system_Way_T

typedef struct tag_vxlth3EMPLG7CAzUleegd uav_sluav_internal_system_Way_T;

#endif                             /* typedef_uav_sluav_internal_system_Way_T */

#ifndef struct_tag_wDrDFcl7TOMfHPqLOhcgc
#define struct_tag_wDrDFcl7TOMfHPqLOhcgc

struct tag_wDrDFcl7TOMfHPqLOhcgc
{
  int32_T isInitialized;
  real_T NumCircles;
  real_T PrevPosition[3];
  boolean_T StartFlag;
  real_T LookaheadDistance;
  boolean_T SelectTurnDirectionFlag;
  real_T TurnDirectionInternal;
  real_T OrbitCenterInternal[3];
  real_T OrbitRadiusInternal;
  uint8_T OrbitRadiusFlag;
  uint8_T LookaheadDistFlag;
  real_T PrevResetSignal;
};

#endif                                 /* struct_tag_wDrDFcl7TOMfHPqLOhcgc */

#ifndef typedef_uav_sluav_internal_system_Orb_T
#define typedef_uav_sluav_internal_system_Orb_T

typedef struct tag_wDrDFcl7TOMfHPqLOhcgc uav_sluav_internal_system_Orb_T;

#endif                             /* typedef_uav_sluav_internal_system_Orb_T */

#ifndef struct_tag_rjIxem9ZNbPEw52KCgcImD
#define struct_tag_rjIxem9ZNbPEw52KCgcImD

struct tag_rjIxem9ZNbPEw52KCgcImD
{
  int32_T isInitialized;
  cell_wrap_UAM_FlightMode_T inputVarSize[3];
  real_T LookaheadDistance;
  real_T WaypointIndex;
  real_T NumWaypoints;
  real_T WaypointsInternal[6];
  boolean_T LastWaypointFlag;
  boolean_T StartFlag;
  real_T InitialPose[4];
  real_T LookaheadFactor;
  uint8_T LookaheadDistFlag;
};

#endif                                 /* struct_tag_rjIxem9ZNbPEw52KCgcImD */

#ifndef typedef_uav_sluav_internal_system_W_d_T
#define typedef_uav_sluav_internal_system_W_d_T

typedef struct tag_rjIxem9ZNbPEw52KCgcImD uav_sluav_internal_system_W_d_T;

#endif                             /* typedef_uav_sluav_internal_system_W_d_T */

#ifndef struct_tag_YMR6rFOhSCamgU15cjwXe
#define struct_tag_YMR6rFOhSCamgU15cjwXe

struct tag_YMR6rFOhSCamgU15cjwXe
{
  int32_T isInitialized;
  cell_wrap_UAM_FlightMode_T inputVarSize[3];
  real_T LookaheadDistance;
  real_T WaypointIndex;
  real_T InitYaw;
  real_T FinalYaw;
  real_T NumWaypoints;
  real_T WaypointsInternal[8];
  boolean_T LastWaypointFlag;
  boolean_T StartFlag;
  real_T InitialPose[4];
  real_T LookaheadFactor;
  boolean_T SearchFlag;
  uint8_T LookaheadDistFlag;
};

#endif                                 /* struct_tag_YMR6rFOhSCamgU15cjwXe */

#ifndef typedef_uav_sluav_internal_system__de_T
#define typedef_uav_sluav_internal_system__de_T

typedef struct tag_YMR6rFOhSCamgU15cjwXe uav_sluav_internal_system__de_T;

#endif                             /* typedef_uav_sluav_internal_system__de_T */

/* Forward declaration for rtModel */
typedef struct tag_RTM_UAM_FlightMode_T RT_MODEL_UAM_FlightMode_T;

#endif                                 /* UAM_FlightMode_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
