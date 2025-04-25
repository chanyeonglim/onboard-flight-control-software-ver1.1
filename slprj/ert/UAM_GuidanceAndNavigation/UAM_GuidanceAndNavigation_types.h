/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_GuidanceAndNavigation_types.h
 *
 * Code generated for Simulink model 'UAM_GuidanceAndNavigation'.
 *
 * Model version                  : 1.80
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Fri Apr 25 10:13:17 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef UAM_GuidanceAndNavigation_types_h_
#define UAM_GuidanceAndNavigation_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_UAVPathManagerBus_
#define DEFINED_TYPEDEF_FOR_UAVPathManagerBus_

typedef struct {
  uint8_T mode;
  real_T position[3];
  real_T params[4];
} UAVPathManagerBus;

#endif

#ifndef struct_tag_PNHAOeXMeBDLyeZaJKzdBF
#define struct_tag_PNHAOeXMeBDLyeZaJKzdBF

struct tag_PNHAOeXMeBDLyeZaJKzdBF
{
  real_T a;
  real_T b;
  real_T c;
  real_T d;
};

#endif                                 /* struct_tag_PNHAOeXMeBDLyeZaJKzdBF */

#ifndef typedef_d_quaternion_UAM_GuidanceAndN_T
#define typedef_d_quaternion_UAM_GuidanceAndN_T

typedef struct tag_PNHAOeXMeBDLyeZaJKzdBF d_quaternion_UAM_GuidanceAndN_T;

#endif                             /* typedef_d_quaternion_UAM_GuidanceAndN_T */

#ifndef struct_tag_rE0p9RRszDGqA6Mm8JUyIH
#define struct_tag_rE0p9RRszDGqA6Mm8JUyIH

struct tag_rE0p9RRszDGqA6Mm8JUyIH
{
  int32_T __dummy;
};

#endif                                 /* struct_tag_rE0p9RRszDGqA6Mm8JUyIH */

#ifndef typedef_c_fusion_internal_frames_NED__T
#define typedef_c_fusion_internal_frames_NED__T

typedef struct tag_rE0p9RRszDGqA6Mm8JUyIH c_fusion_internal_frames_NED__T;

#endif                             /* typedef_c_fusion_internal_frames_NED__T */

#ifndef struct_tag_PnnaIwx6QfMRJiuany93oF
#define struct_tag_PnnaIwx6QfMRJiuany93oF

struct tag_PnnaIwx6QfMRJiuany93oF
{
  int32_T isInitialized;
};

#endif                                 /* struct_tag_PnnaIwx6QfMRJiuany93oF */

#ifndef typedef_robotics_slcore_internal_bloc_T
#define typedef_robotics_slcore_internal_bloc_T

typedef struct tag_PnnaIwx6QfMRJiuany93oF robotics_slcore_internal_bloc_T;

#endif                             /* typedef_robotics_slcore_internal_bloc_T */

#ifndef struct_tag_ca7u59acYki0muEU4gGASF
#define struct_tag_ca7u59acYki0muEU4gGASF

struct tag_ca7u59acYki0muEU4gGASF
{
  int32_T isInitialized;
  real_T MissionPointCounter;
  UAVPathManagerBus MissionParams[2];
  UAVPathManagerBus PrevMissionPoint;
  boolean_T MissionStart;
  boolean_T PrevModeStatus;
  boolean_T PoseHoldFlag;
  boolean_T PoseRTLFlag;
  real_T HoldPose[4];
  boolean_T EnableRepeat;
  boolean_T FinalMissionPoint;
};

#endif                                 /* struct_tag_ca7u59acYki0muEU4gGASF */

#ifndef typedef_uav_sluav_internal_system_Pat_T
#define typedef_uav_sluav_internal_system_Pat_T

typedef struct tag_ca7u59acYki0muEU4gGASF uav_sluav_internal_system_Pat_T;

#endif                             /* typedef_uav_sluav_internal_system_Pat_T */

#ifndef struct_tag_Eze2x1Z3d8nruYygLKeJzB
#define struct_tag_Eze2x1Z3d8nruYygLKeJzB

struct tag_Eze2x1Z3d8nruYygLKeJzB
{
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  real_T AccelerometerNoise;
  real_T GyroscopeNoise;
  real_T GyroscopeDriftNoise;
  real_T LinearAccelerationNoise;
  real_T LinearAccelerationDecayFactor;
  real_T pQw[144];
  real_T pQv[36];
  d_quaternion_UAM_GuidanceAndN_T pOrientPost;
  boolean_T pFirstTime;
  c_fusion_internal_frames_NED__T pRefSys;
  real_T pSensorPeriod;
  real_T pKalmanPeriod;
  real_T pGyroOffset[3];
  real_T pLinAccelPost[3];
  real_T pInputPrototype[3];
  real_T MagnetometerNoise;
  real_T MagneticDisturbanceNoise;
  real_T MagneticDisturbanceDecayFactor;
  real_T ExpectedMagneticFieldStrength;
  real_T pMagVec[3];
  real_T pInclinationLimit;
};

#endif                                 /* struct_tag_Eze2x1Z3d8nruYygLKeJzB */

#ifndef typedef_fusion_simulink_ahrsfilter_UA_T
#define typedef_fusion_simulink_ahrsfilter_UA_T

typedef struct tag_Eze2x1Z3d8nruYygLKeJzB fusion_simulink_ahrsfilter_UA_T;

#endif                             /* typedef_fusion_simulink_ahrsfilter_UA_T */

/* Forward declaration for rtModel */
typedef struct tag_RTM_UAM_GuidanceAndNaviga_T RT_MODEL_UAM_GuidanceAndNavig_T;

#endif                                 /* UAM_GuidanceAndNavigation_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
