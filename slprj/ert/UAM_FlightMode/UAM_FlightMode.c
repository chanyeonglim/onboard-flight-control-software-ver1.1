/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_FlightMode.c
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

#include "UAM_FlightMode.h"
#include "rtwtypes.h"
#include "UAM_FlightMode_types.h"
#include <string.h>
#include <emmintrin.h>
#include "rt_nonfinite.h"
#include <math.h>
#include "rt_atan2d_snf.h"
#include "UAM_FlightMode_private.h"

/* Named constants for Chart: '<Root>/Guidance Mode Selector' */
#define UAM_Flig_IN_FIXED_WING_WAYPOINT ((uint8_T)4U)
#define UAM_FlightM_IN_FIXED_WING_ENTRY ((uint8_T)2U)
#define UAM_FlightM_IN_FIXED_WING_ORBIT ((uint8_T)3U)
#define UAM_FlightMo_IN_FIXEDWINGFLIGHT ((uint8_T)1U)
#define UAM_FlightMo_IN_NO_ACTIVE_CHILD ((uint8_T)0U)
#define UAM_FlightMod_IN_BackTransition ((uint8_T)1U)
#define UAM_FlightMode_IN_Descend      ((uint8_T)1U)
#define UAM_FlightMode_IN_FWCOMPLETE   ((uint8_T)5U)
#define UAM_FlightMode_IN_HOVER_ENTRY  ((uint8_T)7U)
#define UAM_FlightMode_IN_Land         ((uint8_T)8U)
#define UAM_FlightMode_IN_ORBIT        ((uint8_T)1U)
#define UAM_FlightMode_IN_Orbit        ((uint8_T)9U)
#define UAM_FlightMode_IN_PreTransition ((uint8_T)10U)
#define UAM_FlightMode_IN_Stabilize    ((uint8_T)2U)
#define UAM_FlightMode_IN_Start        ((uint8_T)11U)
#define UAM_FlightMode_IN_Takeoff      ((uint8_T)12U)
#define UAM_FlightMode_IN_ToLand       ((uint8_T)2U)
#define UAM_FlightMode_IN_WAYPOINT     ((uint8_T)1U)
#define UAM_FlightMode_IN_WP           ((uint8_T)13U)
#define UAM_Flight_IN_ForwardTransition ((uint8_T)6U)

/* Forward declaration for local functions */
static real_T UAM_FlightMode_norm(const real_T x[3]);
static real_T UAM_FlightMode_wrapToPi(real_T theta);

/* Forward declaration for local functions */
static void UAM_exit_internal_GuidanceLogic(DW_UAM_FlightMode_f_T *localDW);
static real_T UAM_FlightMode_norm_p(const real_T x[2]);
static real_T UAM_FlightMode_norm_pv(const real_T x[3]);
static real_T UAM_FlightMode_angdiff(real_T x, real_T y);
static void UAM_FlightMode_FIXED_WING_ENTRY(const uint8_T *rtu_mode, const
  UAVPathManagerBus *rtu_ToWP, const UAVPathManagerBus *rtu_FromWP, const
  GuidanceStates *rtu_States, B_UAM_FlightMode_c_T *localB,
  DW_UAM_FlightMode_f_T *localDW);
static boolean_T UAM_Flig_transitionConditionMet(const GuidanceStates
  *rtu_States);
static void enter_internal_FIXED_WING_ENTRY(B_UAM_FlightMode_c_T *localB,
  DW_UAM_FlightMode_f_T *localDW);
static real_T UAM_FlightMode_wrapToPi_p(real_T theta);
static void UAM_FlightMode_SystemCore_step(uav_sluav_internal_system__cc_T *obj,
  const real_T varargin_1[4], const real_T varargin_2[8], real_T varargin_3,
  real_T varargout_1[3], real_T *varargout_2, real_T *varargout_3, uint8_T
  *varargout_4);
static void UAM_FlightMode_Start_d(const uint8_T *rtu_mode, const
  UAVPathManagerBus *rtu_ToWP, const UAVPathManagerBus *rtu_FromWP, const real_T
  rtu_Pose[4], const GuidanceStates *rtu_States, B_UAM_FlightMode_c_T *localB,
  DW_UAM_FlightMode_f_T *localDW);
static void UAM_FlightMode_GuidanceLogic(const uint8_T *rtu_mode, const
  UAVPathManagerBus *rtu_ToWP, const UAVPathManagerBus *rtu_FromWP, const real_T
  rtu_Pose[4], const GuidanceStates *rtu_States, const real_T *rtu_Ground,
  B_UAM_FlightMode_c_T *localB, DW_UAM_FlightMode_f_T *localDW);
static real_T UAM_FlightMode_norm(const real_T x[3])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;

  /* Start for MATLABSystem: '<S8>/Waypoint Follower' */
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  /* Start for MATLABSystem: '<S8>/Waypoint Follower' */
  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  /* Start for MATLABSystem: '<S8>/Waypoint Follower' */
  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

static real_T UAM_FlightMode_wrapToPi(real_T theta)
{
  real_T b_theta;
  real_T q;
  real_T thetaWrap;
  boolean_T rEQ0;
  b_theta = theta;

  /* Start for MATLABSystem: '<S8>/Waypoint Follower' */
  if (fabs(theta) > 3.1415926535897931) {
    if (rtIsNaN(theta + 3.1415926535897931) || rtIsInf(theta +
         3.1415926535897931)) {
      thetaWrap = (rtNaN);
    } else if (theta + 3.1415926535897931 == 0.0) {
      thetaWrap = 0.0;
    } else {
      thetaWrap = fmod(theta + 3.1415926535897931, 6.2831853071795862);
      rEQ0 = (thetaWrap == 0.0);
      if (!rEQ0) {
        q = fabs((theta + 3.1415926535897931) / 6.2831853071795862);
        rEQ0 = !(fabs(q - floor(q + 0.5)) > 2.2204460492503131E-16 * q);
      }

      if (rEQ0) {
        thetaWrap = 0.0;
      } else if (thetaWrap < 0.0) {
        thetaWrap += 6.2831853071795862;
      }
    }

    if ((thetaWrap == 0.0) && (theta + 3.1415926535897931 > 0.0)) {
      thetaWrap = 6.2831853071795862;
    }

    b_theta = thetaWrap - 3.1415926535897931;
  }

  /* End of Start for MATLABSystem: '<S8>/Waypoint Follower' */
  return b_theta;
}

/* System initialize for atomic system: */
void UAM_Fligh_WaypointFollower_Init(DW_WaypointFollower_UAM_Fligh_T *localDW)
{
  /* Start for MATLABSystem: '<S8>/Waypoint Follower' */
  localDW->obj.LastWaypointFlag = false;
  localDW->obj.StartFlag = true;
  localDW->obj.LookaheadFactor = 1.01;
  localDW->objisempty = true;
  localDW->obj.isInitialized = 1;
  localDW->obj.NumWaypoints = 0.0;

  /* InitializeConditions for MATLABSystem: '<S8>/Waypoint Follower' */
  localDW->obj.WaypointIndex = 1.0;
  memset(&localDW->obj.WaypointsInternal[0], 0, sizeof(real_T) << 3U);
}

/* Output and update for atomic system: */
void UAM_FlightMode_WaypointFollower(const real_T rtu_0[4], const real_T rtu_1[8],
  real_T rtu_2, B_WaypointFollower_UAM_Flight_T *localB,
  DW_WaypointFollower_UAM_Fligh_T *localDW)
{
  __m128d tmp;
  real_T b_waypointsIn_data[8];
  real_T waypoints_data[8];
  real_T b_waypointsIn[3];
  real_T rtu_0_0[3];
  real_T unitVectorUtoV_tmp[3];
  real_T absx_tmp;
  real_T b_endWaypoint_idx_0_tmp;
  real_T b_endWaypoint_idx_1_tmp;
  real_T b_endWaypoint_idx_2_tmp;
  real_T b_startWaypoint_idx_1_tmp;
  real_T b_startWaypoint_idx_2_tmp;
  real_T currentPosition;
  real_T dist;
  real_T lookaheadDist;
  real_T unitVectorUtoV_tmp_0;
  real_T unitVectorUtoV_tmp_1;
  real_T y;
  int32_T b_exponent;
  int32_T b_exponent_0;
  int32_T b_k;
  int32_T i;
  int32_T tmp_size_idx_1;
  int32_T waypoints_size_idx_0;
  int8_T tmp_data[2];
  boolean_T x[3];
  boolean_T distinctWptsIdx[2];
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T guard2;
  boolean_T p;
  boolean_T p_0;

  /* MATLABSystem: '<S8>/Waypoint Follower' */
  lookaheadDist = rtu_2;
  localDW->obj.LookaheadDistFlag = 0U;
  if (rtu_2 < 0.1) {
    lookaheadDist = 0.1;
    localDW->obj.LookaheadDistFlag = 1U;
  }

  localDW->obj.InitialPose[0] = 0.0;
  localDW->obj.InitialPose[1] = 0.0;
  localDW->obj.InitialPose[2] = 0.0;
  localDW->obj.InitialPose[3] = 0.0;
  localDW->obj.NumWaypoints = 2.0;
  p = false;
  p_0 = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k <= 7)) {
    i = ((b_k / 2) << 1) + b_k % 2;
    if (!(localDW->obj.WaypointsInternal[i] == rtu_1[i])) {
      p_0 = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  if (p_0) {
    p = true;
  }

  if (!p) {
    memcpy(&localDW->obj.WaypointsInternal[0], &rtu_1[0], sizeof(real_T) << 3U);
    localDW->obj.WaypointIndex = 1.0;
  }

  distinctWptsIdx[1] = true;
  x[0] = (rtu_1[0] != rtu_1[1]);
  x[1] = (rtu_1[2] != rtu_1[3]);
  x[2] = (rtu_1[4] != rtu_1[5]);
  distinctWptsIdx[0] = false;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 3)) {
    if (x[b_k]) {
      distinctWptsIdx[0] = true;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  b_k = 0;
  for (i = 0; i < 2; i++) {
    /* MATLABSystem: '<S8>/Waypoint Follower' */
    if (distinctWptsIdx[i]) {
      b_k++;
    }
  }

  tmp_size_idx_1 = b_k;
  b_k = 0;
  for (i = 0; i < 2; i++) {
    /* MATLABSystem: '<S8>/Waypoint Follower' */
    if (distinctWptsIdx[i]) {
      /* Start for MATLABSystem: '<S8>/Waypoint Follower' */
      tmp_data[b_k] = (int8_T)i;
      b_k++;
    }
  }

  /* MATLABSystem: '<S8>/Waypoint Follower' */
  for (i = 0; i < 4; i++) {
    for (b_k = 0; b_k < tmp_size_idx_1; b_k++) {
      b_waypointsIn_data[b_k + tmp_size_idx_1 * i] = rtu_1[(i << 1) +
        tmp_data[b_k]];
    }
  }

  localDW->obj.LookaheadDistance = lookaheadDist;
  if (tmp_size_idx_1 == 0) {
    localB->LAP[0] = rtu_0[0];
    localB->LAP[1] = rtu_0[1];
    localB->LAP[2] = rtu_0[2];

    /* MATLABSystem: '<S8>/Waypoint Follower' */
    localB->HeadingCmd = rtu_0[3];
    localB->YawCmd = rtu_0[3];
  } else {
    guard1 = false;
    if (tmp_size_idx_1 == 1) {
      if (localDW->obj.StartFlag) {
        localDW->obj.InitialPose[0] = rtu_0[0];
        localDW->obj.InitialPose[1] = rtu_0[1];
        localDW->obj.InitialPose[2] = rtu_0[2];
        localDW->obj.InitialPose[3] = rtu_0[3];
      }

      tmp = _mm_sub_pd(_mm_loadu_pd(&b_waypointsIn_data[0]), _mm_loadu_pd
                       (&rtu_0[0]));
      _mm_storeu_pd(&b_waypointsIn[0], tmp);
      b_waypointsIn[2] = b_waypointsIn_data[2] - rtu_0[2];
      if (UAM_FlightMode_norm(b_waypointsIn) < 1.4901161193847656E-8) {
        localB->LAP[0] = rtu_0[0];
        localB->LAP[1] = rtu_0[1];
        localB->LAP[2] = rtu_0[2];

        /* MATLABSystem: '<S8>/Waypoint Follower' */
        localB->HeadingCmd = rtu_0[3];
        localB->YawCmd = rtu_0[3];
        localDW->obj.StartFlag = false;
      } else {
        localDW->obj.StartFlag = false;
        localDW->obj.NumWaypoints = 2.0;
        waypoints_size_idx_0 = tmp_size_idx_1 + 1;
        waypoints_data[0] = localDW->obj.InitialPose[0];
        waypoints_data[tmp_size_idx_1 + 1] = localDW->obj.InitialPose[1];
        waypoints_data[(tmp_size_idx_1 + 1) << 1] = localDW->obj.InitialPose[2];
        waypoints_data[(tmp_size_idx_1 + 1) * 3] = rtu_1[tmp_data[0] + 6];
        for (i = 0; i < 4; i++) {
          for (b_k = 0; b_k < tmp_size_idx_1; b_k++) {
            waypoints_data[(b_k + (tmp_size_idx_1 + 1) * i) + 1] =
              b_waypointsIn_data[tmp_size_idx_1 * i + b_k];
          }
        }

        guard1 = true;
      }
    } else {
      waypoints_size_idx_0 = tmp_size_idx_1;
      b_k = tmp_size_idx_1 << 2;
      if (b_k - 1 >= 0) {
        memcpy(&waypoints_data[0], &b_waypointsIn_data[0], (uint32_T)b_k *
               sizeof(real_T));
      }

      guard1 = true;
    }

    if (guard1) {
      p = false;
      if (localDW->obj.WaypointIndex == 2.0) {
        p = true;
      }

      if (p) {
        localDW->obj.LastWaypointFlag = true;
        localDW->obj.WaypointIndex--;
      }

      localDW->obj.InitYaw = waypoints_data[(waypoints_size_idx_0 * 3 + (int32_T)
        localDW->obj.WaypointIndex) - 1];
      localDW->obj.FinalYaw = waypoints_data[((int32_T)
        (localDW->obj.WaypointIndex + 1.0) + waypoints_size_idx_0 * 3) - 1];
      lookaheadDist = waypoints_data[(int32_T)localDW->obj.WaypointIndex - 1];
      b_endWaypoint_idx_0_tmp = waypoints_data[(int32_T)
        (localDW->obj.WaypointIndex + 1.0) - 1];
      b_waypointsIn[0] = rtu_0[0] - b_endWaypoint_idx_0_tmp;
      b_startWaypoint_idx_1_tmp = waypoints_data[((int32_T)
        localDW->obj.WaypointIndex + waypoints_size_idx_0) - 1];
      b_endWaypoint_idx_1_tmp = waypoints_data[((int32_T)
        (localDW->obj.WaypointIndex + 1.0) + waypoints_size_idx_0) - 1];
      b_waypointsIn[1] = rtu_0[1] - b_endWaypoint_idx_1_tmp;
      b_startWaypoint_idx_2_tmp = waypoints_data[((waypoints_size_idx_0 << 1) +
        (int32_T)localDW->obj.WaypointIndex) - 1];
      b_endWaypoint_idx_2_tmp = waypoints_data[((int32_T)
        (localDW->obj.WaypointIndex + 1.0) + (waypoints_size_idx_0 << 1)) - 1];
      b_waypointsIn[2] = rtu_0[2] - b_endWaypoint_idx_2_tmp;
      y = UAM_FlightMode_norm(b_waypointsIn);
      guard2 = false;
      if (y <= 1.0) {
        guard2 = true;
      } else {
        _mm_storeu_pd(&unitVectorUtoV_tmp[0], _mm_sub_pd(_mm_set_pd
          (b_endWaypoint_idx_1_tmp, b_endWaypoint_idx_0_tmp), _mm_set_pd
          (b_startWaypoint_idx_1_tmp, lookaheadDist)));
        unitVectorUtoV_tmp[2] = b_endWaypoint_idx_2_tmp -
          b_startWaypoint_idx_2_tmp;
        currentPosition = UAM_FlightMode_norm(unitVectorUtoV_tmp);
        unitVectorUtoV_tmp_0 = (unitVectorUtoV_tmp[0] / currentPosition *
          (b_waypointsIn[0] / y) + unitVectorUtoV_tmp[1] / currentPosition *
          (b_waypointsIn[1] / y)) + unitVectorUtoV_tmp[2] / currentPosition *
          (b_waypointsIn[2] / y);
        if (rtIsNaN(unitVectorUtoV_tmp_0) || (unitVectorUtoV_tmp_0 < 0.0)) {
        } else {
          guard2 = true;
        }
      }

      if (guard2) {
        localDW->obj.WaypointIndex++;
        p = false;
        if (localDW->obj.WaypointIndex == 2.0) {
          p = true;
        }

        if (p) {
          localDW->obj.LastWaypointFlag = true;
          localDW->obj.WaypointIndex--;
        }

        lookaheadDist = waypoints_data[(int32_T)localDW->obj.WaypointIndex - 1];
        b_endWaypoint_idx_0_tmp = waypoints_data[(int32_T)
          (localDW->obj.WaypointIndex + 1.0) - 1];
        b_startWaypoint_idx_1_tmp = waypoints_data[((int32_T)
          localDW->obj.WaypointIndex + waypoints_size_idx_0) - 1];
        b_endWaypoint_idx_1_tmp = waypoints_data[((int32_T)
          (localDW->obj.WaypointIndex + 1.0) + waypoints_size_idx_0) - 1];
        b_startWaypoint_idx_2_tmp = waypoints_data[((waypoints_size_idx_0 << 1)
          + (int32_T)localDW->obj.WaypointIndex) - 1];
        b_endWaypoint_idx_2_tmp = waypoints_data[((int32_T)
          (localDW->obj.WaypointIndex + 1.0) + (waypoints_size_idx_0 << 1)) - 1];
        localDW->obj.InitYaw = waypoints_data[(waypoints_size_idx_0 * 3 +
          (int32_T)localDW->obj.WaypointIndex) - 1];
        localDW->obj.FinalYaw = waypoints_data[((int32_T)
          (localDW->obj.WaypointIndex + 1.0) + waypoints_size_idx_0 * 3) - 1];
      }

      unitVectorUtoV_tmp_0 = rtu_0[0] - lookaheadDist;
      unitVectorUtoV_tmp[0] = unitVectorUtoV_tmp_0;
      currentPosition = b_endWaypoint_idx_0_tmp - lookaheadDist;
      b_waypointsIn[0] = currentPosition;
      unitVectorUtoV_tmp_1 = unitVectorUtoV_tmp_0 * currentPosition;
      y = currentPosition * currentPosition;
      unitVectorUtoV_tmp_0 = rtu_0[1] - b_startWaypoint_idx_1_tmp;
      unitVectorUtoV_tmp[1] = unitVectorUtoV_tmp_0;
      currentPosition = b_endWaypoint_idx_1_tmp - b_startWaypoint_idx_1_tmp;
      b_waypointsIn[1] = currentPosition;
      unitVectorUtoV_tmp_1 += unitVectorUtoV_tmp_0 * currentPosition;
      y += currentPosition * currentPosition;
      unitVectorUtoV_tmp_0 = rtu_0[2] - b_startWaypoint_idx_2_tmp;
      unitVectorUtoV_tmp[2] = unitVectorUtoV_tmp_0;
      currentPosition = b_endWaypoint_idx_2_tmp - b_startWaypoint_idx_2_tmp;
      y += currentPosition * currentPosition;
      unitVectorUtoV_tmp_0 = (unitVectorUtoV_tmp_0 * currentPosition +
        unitVectorUtoV_tmp_1) / y;
      if (unitVectorUtoV_tmp_0 < 0.0) {
        dist = UAM_FlightMode_norm(unitVectorUtoV_tmp);
      } else if (unitVectorUtoV_tmp_0 > 1.0) {
        tmp = _mm_sub_pd(_mm_loadu_pd(&rtu_0[0]), _mm_set_pd
                         (b_endWaypoint_idx_1_tmp, b_endWaypoint_idx_0_tmp));
        _mm_storeu_pd(&rtu_0_0[0], tmp);
        rtu_0_0[2] = rtu_0[2] - b_endWaypoint_idx_2_tmp;
        dist = UAM_FlightMode_norm(rtu_0_0);
      } else {
        tmp = _mm_sub_pd(_mm_loadu_pd(&rtu_0[0]), _mm_add_pd(_mm_mul_pd
          (_mm_set1_pd(unitVectorUtoV_tmp_0), _mm_loadu_pd(&b_waypointsIn[0])),
          _mm_set_pd(b_startWaypoint_idx_1_tmp, lookaheadDist)));
        _mm_storeu_pd(&rtu_0_0[0], tmp);
        rtu_0_0[2] = rtu_0[2] - (unitVectorUtoV_tmp_0 * currentPosition +
          b_startWaypoint_idx_2_tmp);
        dist = UAM_FlightMode_norm(rtu_0_0);
      }

      if (localDW->obj.LastWaypointFlag) {
        tmp = _mm_sub_pd(_mm_loadu_pd(&rtu_0[0]), _mm_add_pd(_mm_mul_pd
          (_mm_set1_pd(unitVectorUtoV_tmp_0), _mm_loadu_pd(&b_waypointsIn[0])),
          _mm_set_pd(b_startWaypoint_idx_1_tmp, lookaheadDist)));
        _mm_storeu_pd(&rtu_0_0[0], tmp);
        rtu_0_0[2] = rtu_0[2] - (unitVectorUtoV_tmp_0 * currentPosition +
          b_startWaypoint_idx_2_tmp);
        dist = UAM_FlightMode_norm(rtu_0_0);
      }

      absx_tmp = fabs(dist);
      if (rtIsInf(absx_tmp) || rtIsNaN(absx_tmp)) {
        unitVectorUtoV_tmp_1 = (rtNaN);
        absx_tmp = (rtNaN);
      } else if (absx_tmp < 4.4501477170144028E-308) {
        unitVectorUtoV_tmp_1 = 4.94065645841247E-324;
        absx_tmp = 4.94065645841247E-324;
      } else {
        frexp(absx_tmp, &b_exponent);
        unitVectorUtoV_tmp_1 = ldexp(1.0, b_exponent - 53);
        frexp(absx_tmp, &b_exponent_0);
        absx_tmp = ldexp(1.0, b_exponent_0 - 53);
      }

      unitVectorUtoV_tmp_1 = sqrt(unitVectorUtoV_tmp_1);
      absx_tmp *= 5.0;
      if ((unitVectorUtoV_tmp_1 >= absx_tmp) || rtIsNaN(absx_tmp)) {
        absx_tmp = unitVectorUtoV_tmp_1;
      }

      if (localDW->obj.LookaheadDistance <= dist + absx_tmp) {
        localDW->obj.LookaheadDistance = localDW->obj.LookaheadFactor * dist;
      }

      if (localDW->obj.LastWaypointFlag) {
        localB->LAP[0] = b_endWaypoint_idx_0_tmp;
        localB->LAP[1] = b_endWaypoint_idx_1_tmp;
        localB->LAP[2] = b_endWaypoint_idx_2_tmp;
      } else {
        tmp = _mm_sub_pd(_mm_set_pd(b_startWaypoint_idx_1_tmp, lookaheadDist),
                         _mm_loadu_pd(&rtu_0[0]));
        _mm_storeu_pd(&unitVectorUtoV_tmp[0], tmp);
        unitVectorUtoV_tmp[2] = b_startWaypoint_idx_2_tmp - rtu_0[2];
        dist = ((b_waypointsIn[0] * unitVectorUtoV_tmp[0] + b_waypointsIn[1] *
                 unitVectorUtoV_tmp[1]) + currentPosition * unitVectorUtoV_tmp[2])
          * 2.0;
        absx_tmp = sqrt(dist * dist - (((unitVectorUtoV_tmp[0] *
          unitVectorUtoV_tmp[0] + unitVectorUtoV_tmp[1] * unitVectorUtoV_tmp[1])
          + unitVectorUtoV_tmp[2] * unitVectorUtoV_tmp[2]) -
          localDW->obj.LookaheadDistance * localDW->obj.LookaheadDistance) *
                        (4.0 * y));
        unitVectorUtoV_tmp_1 = (-dist + absx_tmp) / 2.0 / y;
        y = (-dist - absx_tmp) / 2.0 / y;
        if ((unitVectorUtoV_tmp_1 >= y) || rtIsNaN(y)) {
          y = unitVectorUtoV_tmp_1;
        }

        localB->LAP[0] = (1.0 - y) * lookaheadDist + y * b_endWaypoint_idx_0_tmp;
        localB->LAP[1] = (1.0 - y) * b_startWaypoint_idx_1_tmp + y *
          b_endWaypoint_idx_1_tmp;
        localB->LAP[2] = (1.0 - y) * b_startWaypoint_idx_2_tmp + y *
          b_endWaypoint_idx_2_tmp;
      }

      /* MATLABSystem: '<S8>/Waypoint Follower' */
      localB->HeadingCmd = rt_atan2d_snf(localB->LAP[1] - rtu_0[1], localB->LAP
        [0] - rtu_0[0]);
      p = (unitVectorUtoV_tmp_0 < 0.0);
      p_0 = (unitVectorUtoV_tmp_0 > 1.0);
      if (p) {
        y = lookaheadDist;
      } else if (p_0) {
        y = b_endWaypoint_idx_0_tmp;
      } else {
        y = unitVectorUtoV_tmp_0 * b_waypointsIn[0] + lookaheadDist;
      }

      unitVectorUtoV_tmp[0] = y - lookaheadDist;
      rtu_0_0[0] = lookaheadDist - b_endWaypoint_idx_0_tmp;
      if (p) {
        y = b_startWaypoint_idx_1_tmp;
      } else if (p_0) {
        y = b_endWaypoint_idx_1_tmp;
      } else {
        y = unitVectorUtoV_tmp_0 * b_waypointsIn[1] + b_startWaypoint_idx_1_tmp;
      }

      unitVectorUtoV_tmp[1] = y - b_startWaypoint_idx_1_tmp;
      rtu_0_0[1] = b_startWaypoint_idx_1_tmp - b_endWaypoint_idx_1_tmp;
      if (p) {
        y = b_startWaypoint_idx_2_tmp;
      } else if (p_0) {
        y = b_endWaypoint_idx_2_tmp;
      } else {
        y = unitVectorUtoV_tmp_0 * currentPosition + b_startWaypoint_idx_2_tmp;
      }

      unitVectorUtoV_tmp[2] = y - b_startWaypoint_idx_2_tmp;
      rtu_0_0[2] = b_startWaypoint_idx_2_tmp - b_endWaypoint_idx_2_tmp;
      localB->YawCmd = UAM_FlightMode_wrapToPi(UAM_FlightMode_norm
        (unitVectorUtoV_tmp) / UAM_FlightMode_norm(rtu_0_0) *
        UAM_FlightMode_wrapToPi(localDW->obj.FinalYaw - localDW->obj.InitYaw) +
        localDW->obj.InitYaw);
      if (fabs(localB->YawCmd - -3.1415926535897931) < 1.4901161193847656E-8) {
        localB->YawCmd = 3.1415926535897931;
      }

      localDW->obj.LastWaypointFlag = false;
    }
  }
}

/* Function for Chart: '<Root>/Guidance Mode Selector' */
static void UAM_exit_internal_GuidanceLogic(DW_UAM_FlightMode_f_T *localDW)
{
  switch (localDW->is_GuidanceLogic) {
   case UAM_FlightMod_IN_BackTransition:
    localDW->is_BackTransition = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    localDW->is_GuidanceLogic = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    break;

   case UAM_FlightM_IN_FIXED_WING_ORBIT:
    localDW->is_FIXED_WING_ORBIT = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    localDW->is_GuidanceLogic = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    break;

   case UAM_Flig_IN_FIXED_WING_WAYPOINT:
    localDW->is_FIXED_WING_WAYPOINT = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    localDW->is_GuidanceLogic = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    break;

   case UAM_Flight_IN_ForwardTransition:
    localDW->is_GuidanceLogic = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    break;

   case UAM_FlightMode_IN_Land:
    switch (localDW->is_Land) {
     case UAM_FlightMode_IN_Descend:
      localDW->is_Land = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
      break;

     case UAM_FlightMode_IN_ToLand:
      localDW->is_Land = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
      break;
    }

    localDW->is_GuidanceLogic = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    break;

   case UAM_FlightMode_IN_Orbit:
    localDW->is_GuidanceLogic = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    break;

   case UAM_FlightMode_IN_Takeoff:
    localDW->is_GuidanceLogic = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    break;

   case UAM_FlightMode_IN_WP:
    localDW->is_GuidanceLogic = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    break;

   default:
    localDW->is_FIXED_WING_ENTRY = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    localDW->is_GuidanceLogic = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    break;
  }
}

static real_T UAM_FlightMode_norm_p(const real_T x[2])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;

  /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
   *  MATLABSystem: '<S10>/UAV Orbit Follower'
   */
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
   *  MATLABSystem: '<S10>/UAV Orbit Follower'
   */
  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

static real_T UAM_FlightMode_norm_pv(const real_T x[3])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;

  /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
   *  MATLABSystem: '<S10>/UAV Orbit Follower'
   *  MATLABSystem: '<S12>/Waypoint Follower'
   *  MATLABSystem: '<S6>/Waypoint Follower'
   */
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
   *  MATLABSystem: '<S10>/UAV Orbit Follower'
   *  MATLABSystem: '<S12>/Waypoint Follower'
   *  MATLABSystem: '<S6>/Waypoint Follower'
   */
  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
   *  MATLABSystem: '<S10>/UAV Orbit Follower'
   *  MATLABSystem: '<S12>/Waypoint Follower'
   *  MATLABSystem: '<S6>/Waypoint Follower'
   */
  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

static real_T UAM_FlightMode_angdiff(real_T x, real_T y)
{
  real_T delta;
  real_T q;
  real_T thetaWrap;
  boolean_T rEQ0;

  /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
   *  MATLABSystem: '<S10>/UAV Orbit Follower'
   */
  delta = y - x;
  if (fabs(delta) > 3.1415926535897931) {
    if (rtIsNaN(delta + 3.1415926535897931) || rtIsInf(delta +
         3.1415926535897931)) {
      thetaWrap = (rtNaN);
    } else if (delta + 3.1415926535897931 == 0.0) {
      thetaWrap = 0.0;
    } else {
      thetaWrap = fmod(delta + 3.1415926535897931, 6.2831853071795862);
      rEQ0 = (thetaWrap == 0.0);
      if (!rEQ0) {
        q = fabs((delta + 3.1415926535897931) / 6.2831853071795862);
        rEQ0 = !(fabs(q - floor(q + 0.5)) > 2.2204460492503131E-16 * q);
      }

      if (rEQ0) {
        thetaWrap = 0.0;
      } else if (thetaWrap < 0.0) {
        thetaWrap += 6.2831853071795862;
      }
    }

    if ((thetaWrap == 0.0) && (delta + 3.1415926535897931 > 0.0)) {
      thetaWrap = 6.2831853071795862;
    }

    delta = thetaWrap - 3.1415926535897931;
  }

  /* End of Start for MATLABSystem: '<S5>/UAV Orbit Follower' */
  return delta;
}

/* Function for Chart: '<Root>/Guidance Mode Selector' */
static void UAM_FlightMode_FIXED_WING_ENTRY(const uint8_T *rtu_mode, const
  UAVPathManagerBus *rtu_ToWP, const UAVPathManagerBus *rtu_FromWP, const
  GuidanceStates *rtu_States, B_UAM_FlightMode_c_T *localB,
  DW_UAM_FlightMode_f_T *localDW)
{
  __m128d tmp_1;
  __m128d tmp_2;
  __m128d tmp_3;
  real_T b_waypointsIn_data[6];
  real_T rtb_wps[6];
  real_T rtb_TmpSignalConversionAtUAVO_0[3];
  real_T turnVector[3];
  real_T u[3];
  real_T unitVectorUtoV_tmp[3];
  real_T v[3];
  real_T b_tmp[2];
  real_T tmp[2];
  real_T tmp_0[2];
  real_T xyPose[2];
  real_T a;
  real_T absx_tmp_tmp;
  real_T d;
  real_T distToCenter;
  real_T turnInternal;
  real_T y2;
  int32_T b_exponent;
  int32_T b_exponent_0;
  int32_T b_k;
  int32_T b_waypointsIn_data_tmp;
  int32_T b_waypointsIn_tmp;
  int32_T i;
  int32_T scalarLB;
  int32_T tmp_size_idx_1;
  int32_T vectorUB;
  int8_T tmp_data[2];
  boolean_T x[3];
  boolean_T distinctWptsIdx[2];
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T guard2;
  boolean_T p;
  boolean_T p_0;
  if (localDW->is_FIXED_WING_ENTRY == UAM_FlightMo_IN_FIXEDWINGFLIGHT) {
    switch (*rtu_mode) {
     case 3U:
      localDW->is_FIXED_WING_ENTRY = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
      localDW->is_GuidanceLogic = UAM_FlightM_IN_FIXED_WING_ORBIT;
      localB->controlMode_m.airspeedAltitude = 1U;
      localB->controlMode_m.attitude = 1U;
      localB->controlMode_m.lateralGuidance = 1U;
      localDW->is_FIXED_WING_ORBIT = UAM_FlightMode_IN_ORBIT;

      /* MATLABSystem: '<S5>/UAV Orbit Follower' */
      a = rtu_ToWP->params[0];
      localDW->obj_b.OrbitRadiusFlag = 0U;
      if (rtu_ToWP->params[0] <= 50.0) {
        a = 50.0;
        localDW->obj_b.OrbitRadiusFlag = 1U;
      }

      localDW->obj_b.LookaheadDistFlag = 0U;

      /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' */
      tmp_2 = _mm_loadu_pd(&rtu_ToWP->position[0]);

      /* SignalConversion generated from: '<S5>/UAV Orbit Follower' */
      tmp_3 = _mm_loadu_pd(&rtu_States->Xe[0]);

      /* MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
       *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
       */
      _mm_storeu_pd(&tmp[0], _mm_sub_pd(tmp_3, tmp_2));
      if (UAM_FlightMode_norm_p(tmp) < 2.47032822920623E-323) {
        /* BusCreator: '<S5>/Bus Creator' incorporates:
         *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
         */
        localB->aacSP.course = rtu_States->course;
        distToCenter = localDW->obj_b.NumCircles;
      } else {
        p = false;
        p_0 = true;
        b_k = 0;
        exitg1 = false;
        while ((!exitg1) && (b_k < 3)) {
          if (!(localDW->obj_b.OrbitCenterInternal[b_k] == rtu_ToWP->
                position[b_k])) {
            p_0 = false;
            exitg1 = true;
          } else {
            b_k++;
          }
        }

        if (p_0) {
          p = true;
        }

        guard1 = false;
        if (!p) {
          guard1 = true;
        } else {
          p = false;
          if (localDW->obj_b.OrbitRadiusInternal == a) {
            p = true;
          }

          if (!p) {
            guard1 = true;
          }
        }

        if (guard1) {
          localDW->obj_b.NumCircles = 0.0;
          localDW->obj_b.OrbitCenterInternal[0] = rtu_ToWP->position[0];
          localDW->obj_b.OrbitCenterInternal[1] = rtu_ToWP->position[1];
          localDW->obj_b.OrbitCenterInternal[2] = rtu_ToWP->position[2];
          localDW->obj_b.OrbitRadiusInternal = a;
          localDW->obj_b.SelectTurnDirectionFlag = true;
        }

        if (a <= 30.0) {
          localDW->obj_b.LookaheadDistance = 0.9 * a;
        } else {
          localDW->obj_b.LookaheadDistance = 30.0;
        }

        /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
         *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
         */
        y2 = rtu_States->Xe[0] - rtu_ToWP->position[0];
        b_tmp[0] = y2;
        turnInternal = y2 * y2;
        y2 = rtu_States->Xe[1] - rtu_ToWP->position[1];
        b_tmp[1] = y2;
        distToCenter = sqrt(y2 * y2 + turnInternal);

        /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' */
        absx_tmp_tmp = a + localDW->obj_b.LookaheadDistance;
        d = fabs(absx_tmp_tmp);

        /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' */
        p = (rtIsInf(d) || rtIsNaN(d));
        if (p) {
          turnInternal = (rtNaN);
        } else if (d < 4.4501477170144028E-308) {
          turnInternal = 4.94065645841247E-324;
        } else {
          frexp(d, &b_exponent);
          turnInternal = ldexp(1.0, b_exponent - 53);
        }

        guard1 = false;
        if (distToCenter >= absx_tmp_tmp - 5.0 * turnInternal) {
          guard1 = true;
        } else {
          if (p) {
            turnInternal = (rtNaN);
          } else if (d < 4.4501477170144028E-308) {
            turnInternal = 4.94065645841247E-324;
          } else {
            frexp(d, &b_exponent_0);
            turnInternal = ldexp(1.0, b_exponent_0 - 53);
          }

          if (distToCenter <= (a - localDW->obj_b.LookaheadDistance) + 5.0 *
              turnInternal) {
            guard1 = true;
          } else {
            if (localDW->obj_b.StartFlag) {
              localDW->obj_b.PrevPosition[0] = rtu_States->Xe[0];
              localDW->obj_b.PrevPosition[1] = rtu_States->Xe[1];
              localDW->obj_b.StartFlag = false;
            }

            if ((rtu_ToWP->params[1] == 0.0) &&
                (!localDW->obj_b.SelectTurnDirectionFlag)) {
              turnInternal = localDW->obj_b.TurnDirectionInternal;
            } else {
              turnInternal = rtu_ToWP->params[1];
            }

            _mm_storeu_pd(&tmp_0[0], _mm_sub_pd(_mm_set_pd
              (localDW->obj_b.PrevPosition[0], rtu_States->Xe[0]), _mm_set1_pd
              (rtu_ToWP->position[0])));
            xyPose[0] = tmp_0[0];
            turnVector[0] = tmp_0[1];
            _mm_storeu_pd(&tmp_0[0], _mm_sub_pd(_mm_set_pd
              (localDW->obj_b.PrevPosition[1], rtu_States->Xe[1]), _mm_set1_pd
              (rtu_ToWP->position[1])));
            xyPose[1] = tmp_0[0];
            turnVector[1] = tmp_0[1];
            d = UAM_FlightMode_norm_p(xyPose);
            distToCenter = localDW->obj_b.LookaheadDistance *
              localDW->obj_b.LookaheadDistance;
            a = ((distToCenter - a * a) + d * d) / (2.0 * d);
            tmp_1 = _mm_set1_pd(d);
            tmp_2 = _mm_sub_pd(tmp_2, tmp_3);
            _mm_storeu_pd(&tmp_0[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(tmp_2,
              _mm_set1_pd(a)), tmp_1), tmp_3));
            y2 = tmp_0[1];
            distToCenter = sqrt(distToCenter - a * a);
            b_tmp[0] = tmp_0[0] - (rtu_ToWP->position[1] - rtu_States->Xe[1]) *
              distToCenter / d;
            _mm_storeu_pd(&tmp_0[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd
              (_mm_set_pd(rtu_ToWP->position[0], rtu_ToWP->position[1]),
               _mm_set_pd(rtu_States->Xe[0], rtu_States->Xe[1])), _mm_set1_pd
              (distToCenter)), tmp_1), _mm_set_pd(tmp_0[1], tmp_0[0])));
            b_tmp[1] = tmp_0[0];
            a = tmp_0[1];
            y2 -= (rtu_ToWP->position[0] - rtu_States->Xe[0]) * distToCenter / d;
            u[0] = turnVector[0];
            u[1] = turnVector[1];
            u[2] = 0.0;
            v[0] = tmp[0];
            v[1] = tmp[1];
            v[2] = 0.0;
            if (turnInternal < 0.0) {
              u[0] = tmp[0];
              v[0] = turnVector[0];
              u[1] = tmp[1];
              v[1] = turnVector[1];
              u[2] = 0.0;
              v[2] = 0.0;
            }

            d = UAM_FlightMode_norm_pv(u);
            distToCenter = UAM_FlightMode_norm_pv(v);
            tmp_1 = _mm_set_pd(distToCenter, d);
            _mm_storeu_pd(&tmp_0[0], _mm_div_pd(_mm_set_pd(v[0], u[0]), tmp_1));
            u[0] = tmp_0[0];
            v[0] = tmp_0[1];
            _mm_storeu_pd(&tmp_0[0], _mm_div_pd(_mm_set_pd(v[1], u[1]), tmp_1));
            v[1] = tmp_0[1];
            turnVector[2] = u[0] * tmp_0[1] - v[0] * tmp_0[0];
            localDW->obj_b.PrevPosition[0] = rtu_States->Xe[0];
            localDW->obj_b.PrevPosition[1] = rtu_States->Xe[1];
            localDW->obj_b.PrevPosition[2] = rtu_States->Xe[2];
            localDW->obj_b.NumCircles += rt_atan2d_snf(turnVector[2], (u[0] * v
              [0] + tmp_0[0] * tmp_0[1]) + 0.0 / d * (0.0 / distToCenter)) / 2.0
              / 3.1415926535897931;
            distToCenter = localDW->obj_b.NumCircles;
            _mm_storeu_pd(&v[0], tmp_2);
            if (rtIsNaN(turnInternal)) {
              d = (rtNaN);
            } else if (turnInternal < 0.0) {
              d = -1.0;
            } else {
              d = (turnInternal > 0.0);
            }

            switch ((int32_T)d) {
             case 1:
              if ((b_tmp[0] - rtu_States->Xe[0]) * v[1] - (a - rtu_States->Xe[1])
                  * v[0] > 0.0) {
                turnInternal = b_tmp[0];
                y2 = a;
              } else {
                turnInternal = b_tmp[1];
              }
              break;

             case -1:
              if ((b_tmp[0] - rtu_States->Xe[0]) * v[1] - (a - rtu_States->Xe[1])
                  * v[0] < 0.0) {
                turnInternal = b_tmp[0];
                y2 = a;
              } else {
                turnInternal = b_tmp[1];
              }
              break;

             default:
              if (fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(a - rtu_States->Xe[1],
                     b_tmp[0] - rtu_States->Xe[0]), rtu_States->course)) < fabs
                  (UAM_FlightMode_angdiff(rt_atan2d_snf(y2 - rtu_States->Xe[1],
                     b_tmp[1] - rtu_States->Xe[0]), rtu_States->course))) {
                turnInternal = b_tmp[0];
                y2 = a;
              } else {
                turnInternal = b_tmp[1];
              }

              if ((turnInternal - rtu_States->Xe[0]) * v[1] - (y2 -
                   rtu_States->Xe[1]) * v[0] > 0.0) {
                localDW->obj_b.TurnDirectionInternal = 1.0;
              } else {
                localDW->obj_b.TurnDirectionInternal = -1.0;
              }

              localDW->obj_b.SelectTurnDirectionFlag = false;
              break;
            }
          }
        }

        if (guard1) {
          _mm_storeu_pd(&tmp_0[0], _mm_add_pd(_mm_mul_pd(_mm_div_pd(_mm_set_pd
            (y2, b_tmp[0]), _mm_set1_pd(UAM_FlightMode_norm_p(b_tmp))),
            _mm_set1_pd(a)), tmp_2));
          turnInternal = tmp_0[0];
          y2 = tmp_0[1];
          distToCenter = localDW->obj_b.NumCircles;
        }

        /* BusCreator: '<S5>/Bus Creator' incorporates:
         *  MATLABSystem: '<S5>/UAV Orbit Follower'
         *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
         */
        localB->aacSP.course = rt_atan2d_snf(y2 - rtu_States->Xe[1],
          turnInternal - rtu_States->Xe[0]);
      }

      /* BusCreator: '<S5>/Bus Creator' incorporates:
       *  Constant: '<S5>/Constant'
       *  Constant: '<S5>/Constant1'
       *  Constant: '<S5>/Lookahead Distance'
       *  UnaryMinus: '<S5>/Unary Minus'
       */
      localB->aacSP.airspeed = 15.0;
      localB->aacSP.altitude = -rtu_ToWP->position[2];
      localB->aacSP.L1 = 30.0;
      localB->aacSP.climbrate = 0.0;

      /* Merge: '<S3>/ Merge ' incorporates:
       *  Abs: '<S5>/Abs1'
       *  DataTypeConversion: '<S5>/Data Type Conversion'
       *  MATLABSystem: '<S5>/UAV Orbit Follower'
       *  RelationalOperator: '<S5>/Relational Operator'
       * */
      localB->Status = (uint8_T)(fabs(distToCenter) > rtu_ToWP->params[2]);
      break;

     case 2U:
      localDW->is_FIXED_WING_ENTRY = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
      localDW->is_GuidanceLogic = UAM_Flig_IN_FIXED_WING_WAYPOINT;
      localB->controlMode_m.airspeedAltitude = 1U;
      localB->controlMode_m.attitude = 1U;
      localB->controlMode_m.lateralGuidance = 1U;
      localDW->is_FIXED_WING_WAYPOINT = UAM_FlightMode_IN_WAYPOINT;

      /* MATLAB Function: '<S6>/MATLAB Function' */
      rtb_wps[0] = rtu_FromWP->position[0];
      rtb_wps[1] = rtu_ToWP->position[0];
      rtb_wps[2] = rtu_FromWP->position[1];
      rtb_wps[3] = rtu_ToWP->position[1];
      rtb_wps[4] = rtu_FromWP->position[2];
      rtb_wps[5] = rtu_ToWP->position[2];

      /* MATLABSystem: '<S6>/Waypoint Follower' incorporates:
       *  MATLAB Function: '<S6>/MATLAB Function'
       */
      localDW->obj_f.LookaheadDistFlag = 0U;
      localDW->obj_f.InitialPose[0] = 0.0;
      localDW->obj_f.InitialPose[1] = 0.0;
      localDW->obj_f.InitialPose[2] = 0.0;
      localDW->obj_f.InitialPose[3] = 0.0;
      localDW->obj_f.NumWaypoints = 2.0;
      p = false;
      p_0 = true;
      b_k = 0;
      exitg1 = false;
      while ((!exitg1) && (b_k <= 5)) {
        i = ((b_k / 2) << 1) + b_k % 2;
        if (!(localDW->obj_f.WaypointsInternal[i] == rtb_wps[i])) {
          p_0 = false;
          exitg1 = true;
        } else {
          b_k++;
        }
      }

      if (p_0) {
        p = true;
      }

      if (!p) {
        for (i = 0; i < 6; i++) {
          localDW->obj_f.WaypointsInternal[i] = rtb_wps[i];
        }

        localDW->obj_f.WaypointIndex = 1.0;
      }

      distinctWptsIdx[1] = true;
      x[0] = (rtu_FromWP->position[0] != rtu_ToWP->position[0]);
      x[1] = (rtu_FromWP->position[1] != rtu_ToWP->position[1]);
      x[2] = (rtu_FromWP->position[2] != rtu_ToWP->position[2]);
      distinctWptsIdx[0] = false;
      b_k = 0;
      exitg1 = false;
      while ((!exitg1) && (b_k < 3)) {
        if (x[b_k]) {
          distinctWptsIdx[0] = true;
          exitg1 = true;
        } else {
          b_k++;
        }
      }

      b_k = 0;
      for (i = 0; i < 2; i++) {
        /* MATLABSystem: '<S6>/Waypoint Follower' */
        if (distinctWptsIdx[i]) {
          b_k++;
        }
      }

      tmp_size_idx_1 = b_k;
      b_k = 0;
      for (i = 0; i < 2; i++) {
        /* MATLABSystem: '<S6>/Waypoint Follower' */
        if (distinctWptsIdx[i]) {
          /* Start for MATLABSystem: '<S6>/Waypoint Follower' */
          tmp_data[b_k] = (int8_T)i;
          b_k++;
        }
      }

      /* MATLABSystem: '<S6>/Waypoint Follower' incorporates:
       *  SignalConversion generated from: '<S6>/Waypoint Follower'
       */
      for (i = 0; i < 3; i++) {
        scalarLB = (tmp_size_idx_1 / 2) << 1;
        vectorUB = scalarLB - 2;
        for (b_k = 0; b_k <= vectorUB; b_k += 2) {
          b_waypointsIn_tmp = i << 1;
          b_waypointsIn_data_tmp = tmp_size_idx_1 * i;
          b_waypointsIn_data[b_k + b_waypointsIn_data_tmp] =
            rtb_wps[b_waypointsIn_tmp + tmp_data[b_k]];
          b_waypointsIn_data[(b_k + b_waypointsIn_data_tmp) + 1] =
            rtb_wps[tmp_data[b_k + 1] + b_waypointsIn_tmp];
        }

        for (b_k = scalarLB; b_k < tmp_size_idx_1; b_k++) {
          b_waypointsIn_data[b_k + tmp_size_idx_1 * i] = rtb_wps[(i << 1) +
            tmp_data[b_k]];
        }
      }

      localDW->obj_f.LookaheadDistance = 30.0;
      if (tmp_size_idx_1 == 0) {
        /* BusCreator: '<S6>/Bus Creator' incorporates:
         *  SignalConversion generated from: '<S6>/Waypoint Follower'
         */
        localB->aacSP.course = rtu_States->course;

        /* Merge: '<S3>/ Merge ' */
        localB->Status = 1U;
      } else {
        guard1 = false;
        if (tmp_size_idx_1 == 1) {
          if (localDW->obj_f.StartFlag) {
            localDW->obj_f.InitialPose[0] = rtu_States->Xe[0];
            localDW->obj_f.InitialPose[1] = rtu_States->Xe[1];
            localDW->obj_f.InitialPose[2] = rtu_States->Xe[2];
            localDW->obj_f.InitialPose[3] = rtu_States->course;
          }

          tmp_1 = _mm_sub_pd(_mm_loadu_pd(&b_waypointsIn_data[0]), _mm_loadu_pd(
            &rtu_States->Xe[0]));
          _mm_storeu_pd(&u[0], tmp_1);
          u[2] = b_waypointsIn_data[2] - rtu_States->Xe[2];
          if (UAM_FlightMode_norm_pv(u) < 1.4901161193847656E-8) {
            /* BusCreator: '<S6>/Bus Creator' incorporates:
             *  SignalConversion generated from: '<S6>/Waypoint Follower'
             */
            localB->aacSP.course = rtu_States->course;

            /* Merge: '<S3>/ Merge ' */
            localB->Status = 1U;
            localDW->obj_f.StartFlag = false;
          } else {
            localDW->obj_f.StartFlag = false;
            localDW->obj_f.NumWaypoints = 2.0;
            vectorUB = tmp_size_idx_1 + 1;
            for (i = 0; i < 3; i++) {
              scalarLB = (tmp_size_idx_1 + 1) * i;
              rtb_wps[scalarLB] = localDW->obj_f.InitialPose[i];
              for (b_k = 0; b_k < tmp_size_idx_1; b_k++) {
                rtb_wps[(b_k + scalarLB) + 1] =
                  b_waypointsIn_data[tmp_size_idx_1 * i + b_k];
              }
            }

            guard1 = true;
          }
        } else {
          vectorUB = tmp_size_idx_1;
          scalarLB = tmp_size_idx_1 * 3;
          if (scalarLB - 1 >= 0) {
            memcpy(&rtb_wps[0], &b_waypointsIn_data[0], (uint32_T)scalarLB *
                   sizeof(real_T));
          }

          guard1 = true;
        }

        if (guard1) {
          p = false;
          if (localDW->obj_f.WaypointIndex == 2.0) {
            p = true;
          }

          if (p) {
            localDW->obj_f.LastWaypointFlag = true;
            localDW->obj_f.WaypointIndex--;
          }

          turnInternal = rtb_wps[(int32_T)localDW->obj_f.WaypointIndex - 1];
          u[0] = turnInternal;
          d = rtb_wps[(int32_T)(localDW->obj_f.WaypointIndex + 1.0) - 1];
          v[0] = d;
          a = rtb_wps[((int32_T)localDW->obj_f.WaypointIndex + vectorUB) - 1];
          u[1] = a;
          absx_tmp_tmp = rtb_wps[((int32_T)(localDW->obj_f.WaypointIndex + 1.0)
            + vectorUB) - 1];
          v[1] = absx_tmp_tmp;
          y2 = rtb_wps[((vectorUB << 1) + (int32_T)localDW->obj_f.WaypointIndex)
            - 1];
          u[2] = y2;
          distToCenter = rtb_wps[((int32_T)(localDW->obj_f.WaypointIndex + 1.0)
            + (vectorUB << 1)) - 1];
          v[2] = distToCenter;
          tmp_1 = _mm_set_pd(absx_tmp_tmp, d);
          tmp_2 = _mm_loadu_pd(&rtu_States->Xe[0]);
          _mm_storeu_pd(&turnVector[0], _mm_sub_pd(tmp_2, tmp_1));
          turnVector[2] = rtu_States->Xe[2] - distToCenter;
          d = UAM_FlightMode_norm_pv(turnVector);
          guard2 = false;
          if (d <= 10.0) {
            guard2 = true;
          } else {
            _mm_storeu_pd(&unitVectorUtoV_tmp[0], _mm_sub_pd(tmp_1, _mm_set_pd(a,
              turnInternal)));
            unitVectorUtoV_tmp[2] = distToCenter - y2;
            distToCenter = UAM_FlightMode_norm_pv(unitVectorUtoV_tmp);
            turnInternal = (unitVectorUtoV_tmp[0] / distToCenter * (turnVector[0]
              / d) + unitVectorUtoV_tmp[1] / distToCenter * (turnVector[1] / d))
              + unitVectorUtoV_tmp[2] / distToCenter * (turnVector[2] / d);
            if (rtIsNaN(turnInternal) || (turnInternal < 0.0)) {
            } else {
              guard2 = true;
            }
          }

          if (guard2) {
            localDW->obj_f.WaypointIndex++;
            p = false;
            if (localDW->obj_f.WaypointIndex == 2.0) {
              p = true;
            }

            if (p) {
              localDW->obj_f.LastWaypointFlag = true;
              localDW->obj_f.WaypointIndex--;
            }

            u[0] = rtb_wps[(int32_T)localDW->obj_f.WaypointIndex - 1];
            v[0] = rtb_wps[(int32_T)(localDW->obj_f.WaypointIndex + 1.0) - 1];
            u[1] = rtb_wps[((int32_T)localDW->obj_f.WaypointIndex + vectorUB) -
              1];
            v[1] = rtb_wps[((int32_T)(localDW->obj_f.WaypointIndex + 1.0) +
                            vectorUB) - 1];
            u[2] = rtb_wps[((vectorUB << 1) + (int32_T)
                            localDW->obj_f.WaypointIndex) - 1];
            v[2] = rtb_wps[((int32_T)(localDW->obj_f.WaypointIndex + 1.0) +
                            (vectorUB << 1)) - 1];
          }

          _mm_storeu_pd(&tmp_0[0], _mm_sub_pd(_mm_set_pd(v[0], rtu_States->Xe[0]),
            _mm_set1_pd(u[0])));
          turnVector[0] = tmp_0[0];
          unitVectorUtoV_tmp[0] = tmp_0[1];
          _mm_storeu_pd(&tmp_0[0], _mm_sub_pd(_mm_set_pd(v[1], rtu_States->Xe[1]),
            _mm_set1_pd(u[1])));
          turnVector[1] = tmp_0[0];
          unitVectorUtoV_tmp[1] = tmp_0[1];
          _mm_storeu_pd(&tmp_0[0], _mm_sub_pd(_mm_set_pd(v[2], rtu_States->Xe[2]),
            _mm_set1_pd(u[2])));
          turnVector[2] = tmp_0[0];
          unitVectorUtoV_tmp[2] = tmp_0[1];
          a = unitVectorUtoV_tmp[0] * unitVectorUtoV_tmp[0] +
            unitVectorUtoV_tmp[1] * unitVectorUtoV_tmp[1];
          turnInternal = tmp_0[1] * tmp_0[1] + a;
          d = ((turnVector[0] * unitVectorUtoV_tmp[0] + turnVector[1] *
                unitVectorUtoV_tmp[1]) + tmp_0[0] * tmp_0[1]) / turnInternal;
          if (d < 0.0) {
            y2 = UAM_FlightMode_norm_pv(turnVector);
          } else if (d > 1.0) {
            tmp_1 = _mm_sub_pd(tmp_2, _mm_loadu_pd(&v[0]));
            _mm_storeu_pd(&rtb_TmpSignalConversionAtUAVO_0[0], tmp_1);
            rtb_TmpSignalConversionAtUAVO_0[2] = rtu_States->Xe[2] - v[2];
            y2 = UAM_FlightMode_norm_pv(rtb_TmpSignalConversionAtUAVO_0);
          } else {
            tmp_1 = _mm_sub_pd(tmp_2, _mm_add_pd(_mm_mul_pd(_mm_set1_pd(d),
              _mm_loadu_pd(&unitVectorUtoV_tmp[0])), _mm_loadu_pd(&u[0])));
            _mm_storeu_pd(&rtb_TmpSignalConversionAtUAVO_0[0], tmp_1);
            rtb_TmpSignalConversionAtUAVO_0[2] = rtu_States->Xe[2] - (d * tmp_0
              [1] + u[2]);
            y2 = UAM_FlightMode_norm_pv(rtb_TmpSignalConversionAtUAVO_0);
          }

          if (localDW->obj_f.LastWaypointFlag) {
            d = (((rtu_States->Xe[0] - u[0]) * unitVectorUtoV_tmp[0] +
                  (rtu_States->Xe[1] - u[1]) * unitVectorUtoV_tmp[1]) +
                 (rtu_States->Xe[2] - u[2]) * tmp_0[1]) / turnInternal;
            tmp_1 = _mm_sub_pd(tmp_2, _mm_add_pd(_mm_mul_pd(_mm_set1_pd(d),
              _mm_loadu_pd(&unitVectorUtoV_tmp[0])), _mm_loadu_pd(&u[0])));
            _mm_storeu_pd(&rtb_TmpSignalConversionAtUAVO_0[0], tmp_1);
            rtb_TmpSignalConversionAtUAVO_0[2] = rtu_States->Xe[2] - (d * tmp_0
              [1] + u[2]);
            y2 = UAM_FlightMode_norm_pv(rtb_TmpSignalConversionAtUAVO_0);
          }

          d = fabs(y2);
          if (rtIsInf(d) || rtIsNaN(d)) {
            turnInternal = (rtNaN);
            d = (rtNaN);
          } else if (d < 4.4501477170144028E-308) {
            turnInternal = 4.94065645841247E-324;
            d = 4.94065645841247E-324;
          } else {
            frexp(d, &b_exponent);
            turnInternal = ldexp(1.0, b_exponent - 53);
            frexp(d, &b_exponent_0);
            d = ldexp(1.0, b_exponent_0 - 53);
          }

          turnInternal = sqrt(turnInternal);
          d *= 5.0;
          if ((turnInternal >= d) || rtIsNaN(d)) {
            d = turnInternal;
          }

          if (y2 + d >= 30.0) {
            localDW->obj_f.LookaheadDistance = localDW->obj_f.LookaheadFactor *
              y2;
          }

          turnVector[0] = unitVectorUtoV_tmp[0];
          turnVector[1] = unitVectorUtoV_tmp[1];
          tmp_1 = _mm_sub_pd(_mm_loadu_pd(&u[0]), tmp_2);
          _mm_storeu_pd(&unitVectorUtoV_tmp[0], tmp_1);
          a += unitVectorUtoV_tmp[2] * unitVectorUtoV_tmp[2];
          unitVectorUtoV_tmp[2] = u[2] - rtu_States->Xe[2];
          d = ((turnVector[0] * unitVectorUtoV_tmp[0] + turnVector[1] *
                unitVectorUtoV_tmp[1]) + tmp_0[1] * unitVectorUtoV_tmp[2]) * 2.0;
          y2 = sqrt(d * d - (((unitVectorUtoV_tmp[0] * unitVectorUtoV_tmp[0] +
                               unitVectorUtoV_tmp[1] * unitVectorUtoV_tmp[1]) +
                              unitVectorUtoV_tmp[2] * unitVectorUtoV_tmp[2]) -
                             localDW->obj_f.LookaheadDistance *
                             localDW->obj_f.LookaheadDistance) * (4.0 * a));
          turnInternal = (-d + y2) / 2.0 / a;
          a = (-d - y2) / 2.0 / a;
          if ((turnInternal >= a) || rtIsNaN(a)) {
            a = turnInternal;
          }

          tmp_1 = _mm_set1_pd(a);
          tmp_2 = _mm_add_pd(_mm_mul_pd(_mm_sub_pd(_mm_set1_pd(1.0), tmp_1),
            _mm_loadu_pd(&u[0])), _mm_mul_pd(tmp_1, _mm_loadu_pd(&v[0])));
          _mm_storeu_pd(&v[0], tmp_2);

          /* BusCreator: '<S6>/Bus Creator' incorporates:
           *  SignalConversion generated from: '<S6>/Waypoint Follower'
           */
          localB->aacSP.course = rt_atan2d_snf(v[1] - rtu_States->Xe[1], v[0] -
            rtu_States->Xe[0]);

          /* Merge: '<S3>/ Merge ' */
          localB->Status = 0U;
          p = false;
          if (localDW->obj_f.LastWaypointFlag) {
            p = true;
          }

          if (p) {
            /* Merge: '<S3>/ Merge ' */
            localB->Status = 1U;
          }

          localDW->obj_f.LastWaypointFlag = false;
        }
      }

      /* BusCreator: '<S6>/Bus Creator' incorporates:
       *  Constant: '<S6>/Constant'
       *  Constant: '<S6>/Constant1'
       *  Constant: '<S6>/Lookahead Distance'
       *  MATLAB Function: '<S6>/MATLAB Function'
       */
      localB->aacSP.airspeed = 15.0;
      localB->aacSP.altitude = -rtu_ToWP->position[2];
      localB->aacSP.L1 = 30.0;
      localB->aacSP.climbrate = 0.0;
      break;
    }

    /* case IN_Stabilize: */
  } else if (localDW->temporalCounter_i1 >= 38) {
    localDW->is_FIXED_WING_ENTRY = UAM_FlightMo_IN_FIXEDWINGFLIGHT;
    localB->controlMode_m.airspeedAltitude = 1U;
    localB->controlMode_m.attitude = 1U;
    localB->controlMode_m.lateralGuidance = 1U;
    localB->aacSP.L1 = 25.0;
    localB->aacSP.airspeed = 14.0;
    localB->aacSP.altitude = 0.0;
    localB->aacSP.course = 0.0;
  }
}

/* Function for Chart: '<Root>/Guidance Mode Selector' */
static boolean_T UAM_Flig_transitionConditionMet(const GuidanceStates
  *rtu_States)
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  boolean_T valid;
  if (rtu_States->c1 >= 1.57) {
    scale = 3.3121686421112381E-170;
    absxk = fabs(rtu_States->Ve[0]);
    if (absxk > 3.3121686421112381E-170) {
      y = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      y = t * t;
    }

    absxk = fabs(rtu_States->Ve[1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }

    absxk = fabs(rtu_States->Ve[2]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }

    valid = (scale * sqrt(y) > 10.0);
  } else {
    valid = false;
  }

  return valid;
}

/* Function for Chart: '<Root>/Guidance Mode Selector' */
static void enter_internal_FIXED_WING_ENTRY(B_UAM_FlightMode_c_T *localB,
  DW_UAM_FlightMode_f_T *localDW)
{
  localDW->temporalCounter_i1 = 0U;
  localDW->is_FIXED_WING_ENTRY = UAM_FlightMode_IN_Stabilize;
  localB->FlightMode = FixedWing;
  localB->controlMode_m.attitude = 1U;
  localB->controlMode_m.airspeedAltitude = 0U;
  localB->controlMode_m.lateralGuidance = 0U;
  localB->FixedWingSP.yaw = 0.0;
  localB->FixedWingSP.pitch = 0.0;
  localB->FixedWingSP.roll = 0.0;
  localB->FixedWingSP.airspeed = 14.0;

  /* Merge: '<S3>/ Merge ' */
  localB->Status = 0U;
}

static real_T UAM_FlightMode_wrapToPi_p(real_T theta)
{
  real_T b_theta;
  real_T q;
  real_T thetaWrap;
  boolean_T rEQ0;
  b_theta = theta;

  /* Start for MATLABSystem: '<S12>/Waypoint Follower' */
  if (fabs(theta) > 3.1415926535897931) {
    if (rtIsNaN(theta + 3.1415926535897931) || rtIsInf(theta +
         3.1415926535897931)) {
      thetaWrap = (rtNaN);
    } else if (theta + 3.1415926535897931 == 0.0) {
      thetaWrap = 0.0;
    } else {
      thetaWrap = fmod(theta + 3.1415926535897931, 6.2831853071795862);
      rEQ0 = (thetaWrap == 0.0);
      if (!rEQ0) {
        q = fabs((theta + 3.1415926535897931) / 6.2831853071795862);
        rEQ0 = !(fabs(q - floor(q + 0.5)) > 2.2204460492503131E-16 * q);
      }

      if (rEQ0) {
        thetaWrap = 0.0;
      } else if (thetaWrap < 0.0) {
        thetaWrap += 6.2831853071795862;
      }
    }

    if ((thetaWrap == 0.0) && (theta + 3.1415926535897931 > 0.0)) {
      thetaWrap = 6.2831853071795862;
    }

    b_theta = thetaWrap - 3.1415926535897931;
  }

  /* End of Start for MATLABSystem: '<S12>/Waypoint Follower' */
  return b_theta;
}

static void UAM_FlightMode_SystemCore_step(uav_sluav_internal_system__cc_T *obj,
  const real_T varargin_1[4], const real_T varargin_2[8], real_T varargin_3,
  real_T varargout_1[3], real_T *varargout_2, real_T *varargout_3, uint8_T
  *varargout_4)
{
  __m128d tmp;
  real_T b_waypointsIn_data[8];
  real_T waypoints_data[8];
  real_T b_waypointsIn[3];
  real_T unitVectorUtoV_tmp[3];
  real_T varargin_1_0[3];
  real_T absx_tmp;
  real_T b_endWaypoint_idx_0_tmp;
  real_T b_endWaypoint_idx_1_tmp;
  real_T b_endWaypoint_idx_2_tmp;
  real_T b_startWaypoint_idx_1_tmp;
  real_T b_startWaypoint_idx_2_tmp;
  real_T currentPosition;
  real_T dist;
  real_T lookaheadDist_tmp;
  real_T unitVectorUtoV_tmp_0;
  real_T unitVectorUtoV_tmp_1;
  real_T y;
  int32_T b_exponent;
  int32_T b_exponent_0;
  int32_T b_k;
  int32_T b_waypointsIn_data_tmp;
  int32_T b_waypointsIn_tmp;
  int32_T c;
  int32_T e;
  int32_T scalarLB;
  int32_T vectorUB;
  int8_T tmp_data[2];
  boolean_T x[3];
  boolean_T distinctWptsIdx[2];
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T guard2;
  boolean_T p;
  boolean_T p_0;

  /* Start for MATLABSystem: '<S12>/Waypoint Follower' */
  lookaheadDist_tmp = varargin_3;
  obj->LookaheadDistFlag = 0U;
  if (varargin_3 < 0.1) {
    lookaheadDist_tmp = 0.1;
    obj->LookaheadDistFlag = 1U;
  }

  obj->InitialPose[0] = 0.0;
  obj->InitialPose[1] = 0.0;
  obj->InitialPose[2] = 0.0;
  obj->InitialPose[3] = 0.0;
  obj->NumWaypoints = 2.0;
  guard1 = false;
  if (obj->NumWaypoints != 2.0) {
    guard1 = true;
  } else {
    if (obj->NumWaypoints < 1.0) {
      c = 0;
    } else {
      c = (int32_T)obj->NumWaypoints;
    }

    if (obj->NumWaypoints < 1.0) {
      e = 0;
    } else {
      e = (int32_T)obj->NumWaypoints;
    }

    p = false;
    p_0 = false;
    if (c == e) {
      p_0 = true;
    }

    if (p_0 && (c != 0) && (e != 0)) {
      b_k = 0;
      exitg1 = false;
      while ((!exitg1) && (b_k <= (e << 2) - 1)) {
        if (!(obj->WaypointsInternal[((b_k / c) << 1) + b_k % c] == varargin_2
              [((b_k / e) << 1) + b_k % e])) {
          p_0 = false;
          exitg1 = true;
        } else {
          b_k++;
        }
      }
    }

    if (p_0) {
      p = true;
    }

    if (!p) {
      guard1 = true;
    }
  }

  if (guard1) {
    memcpy(&obj->WaypointsInternal[0], &varargin_2[0], sizeof(real_T) << 3U);
    obj->WaypointIndex = 1.0;
    obj->SearchFlag = true;
  }

  distinctWptsIdx[1] = true;

  /* Start for MATLABSystem: '<S12>/Waypoint Follower' */
  x[0] = (varargin_2[0] != varargin_2[1]);
  x[1] = (varargin_2[2] != varargin_2[3]);
  x[2] = (varargin_2[4] != varargin_2[5]);
  distinctWptsIdx[0] = false;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 3)) {
    if (x[b_k]) {
      distinctWptsIdx[0] = true;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  e = 0;
  for (c = 0; c < 2; c++) {
    if (distinctWptsIdx[c]) {
      e++;
    }
  }

  b_k = e;
  e = 0;
  for (c = 0; c < 2; c++) {
    if (distinctWptsIdx[c]) {
      /* Start for MATLABSystem: '<S12>/Waypoint Follower' */
      tmp_data[e] = (int8_T)c;
      e++;
    }
  }

  for (c = 0; c < 4; c++) {
    scalarLB = (b_k / 2) << 1;
    vectorUB = scalarLB - 2;
    for (e = 0; e <= vectorUB; e += 2) {
      /* Start for MATLABSystem: '<S12>/Waypoint Follower' */
      b_waypointsIn_tmp = c << 1;
      b_waypointsIn_data_tmp = b_k * c;

      /* Start for MATLABSystem: '<S12>/Waypoint Follower' */
      b_waypointsIn_data[e + b_waypointsIn_data_tmp] =
        varargin_2[b_waypointsIn_tmp + tmp_data[e]];
      b_waypointsIn_data[(e + b_waypointsIn_data_tmp) + 1] =
        varargin_2[tmp_data[e + 1] + b_waypointsIn_tmp];
    }

    for (e = scalarLB; e < b_k; e++) {
      /* Start for MATLABSystem: '<S12>/Waypoint Follower' */
      b_waypointsIn_data[e + b_k * c] = varargin_2[(c << 1) + tmp_data[e]];
    }
  }

  /* Start for MATLABSystem: '<S12>/Waypoint Follower' */
  obj->LookaheadDistance = lookaheadDist_tmp;
  if (b_k == 0) {
    varargout_1[0] = varargin_1[0];
    varargout_1[1] = varargin_1[1];
    varargout_1[2] = varargin_1[2];
    *varargout_2 = varargin_1[3];
    *varargout_3 = varargin_1[3];
  } else {
    guard1 = false;
    if (b_k == 1) {
      if (obj->StartFlag) {
        obj->InitialPose[0] = varargin_1[0];
        obj->InitialPose[1] = varargin_1[1];
        obj->InitialPose[2] = varargin_1[2];
        obj->InitialPose[3] = varargin_1[3];
      }

      tmp = _mm_sub_pd(_mm_loadu_pd(&b_waypointsIn_data[0]), _mm_loadu_pd
                       (&varargin_1[0]));
      _mm_storeu_pd(&b_waypointsIn[0], tmp);
      b_waypointsIn[2] = b_waypointsIn_data[2] - varargin_1[2];
      if (UAM_FlightMode_norm_pv(b_waypointsIn) < 1.4901161193847656E-8) {
        varargout_1[0] = varargin_1[0];
        varargout_1[1] = varargin_1[1];
        varargout_1[2] = varargin_1[2];
        *varargout_2 = varargin_1[3];
        *varargout_3 = varargin_1[3];
        obj->StartFlag = false;
      } else {
        obj->StartFlag = false;
        obj->NumWaypoints = 2.0;
        scalarLB = b_k + 1;
        waypoints_data[0] = obj->InitialPose[0];
        waypoints_data[b_k + 1] = obj->InitialPose[1];
        waypoints_data[(b_k + 1) << 1] = obj->InitialPose[2];
        waypoints_data[(b_k + 1) * 3] = varargin_2[tmp_data[0] + 6];
        for (c = 0; c < 4; c++) {
          for (e = 0; e < b_k; e++) {
            waypoints_data[(e + (b_k + 1) * c) + 1] = b_waypointsIn_data[b_k * c
              + e];
          }
        }

        guard1 = true;
      }
    } else {
      scalarLB = b_k;
      e = b_k << 2;
      if (e - 1 >= 0) {
        memcpy(&waypoints_data[0], &b_waypointsIn_data[0], (uint32_T)e * sizeof
               (real_T));
      }

      guard1 = true;
    }

    if (guard1) {
      if (obj->SearchFlag) {
        obj->WaypointIndex = 1.0;
        obj->SearchFlag = false;
      }

      p = false;
      if (obj->WaypointIndex == obj->NumWaypoints) {
        p = true;
      }

      if (p) {
        obj->LastWaypointFlag = true;
        obj->WaypointIndex--;
      }

      obj->InitYaw = waypoints_data[(scalarLB * 3 + (int32_T)obj->WaypointIndex)
        - 1];
      obj->FinalYaw = waypoints_data[((int32_T)(obj->WaypointIndex + 1.0) +
        scalarLB * 3) - 1];
      lookaheadDist_tmp = waypoints_data[(int32_T)obj->WaypointIndex - 1];
      b_endWaypoint_idx_0_tmp = waypoints_data[(int32_T)(obj->WaypointIndex +
        1.0) - 1];
      b_waypointsIn[0] = varargin_1[0] - b_endWaypoint_idx_0_tmp;
      b_startWaypoint_idx_1_tmp = waypoints_data[((int32_T)obj->WaypointIndex +
        scalarLB) - 1];
      b_endWaypoint_idx_1_tmp = waypoints_data[((int32_T)(obj->WaypointIndex +
        1.0) + scalarLB) - 1];
      b_waypointsIn[1] = varargin_1[1] - b_endWaypoint_idx_1_tmp;
      b_startWaypoint_idx_2_tmp = waypoints_data[((scalarLB << 1) + (int32_T)
        obj->WaypointIndex) - 1];
      b_endWaypoint_idx_2_tmp = waypoints_data[((int32_T)(obj->WaypointIndex +
        1.0) + (scalarLB << 1)) - 1];
      b_waypointsIn[2] = varargin_1[2] - b_endWaypoint_idx_2_tmp;
      y = UAM_FlightMode_norm_pv(b_waypointsIn);
      guard2 = false;
      if (y <= 1.0) {
        guard2 = true;
      } else {
        _mm_storeu_pd(&unitVectorUtoV_tmp[0], _mm_sub_pd(_mm_set_pd
          (b_endWaypoint_idx_1_tmp, b_endWaypoint_idx_0_tmp), _mm_set_pd
          (b_startWaypoint_idx_1_tmp, lookaheadDist_tmp)));
        unitVectorUtoV_tmp[2] = b_endWaypoint_idx_2_tmp -
          b_startWaypoint_idx_2_tmp;
        currentPosition = UAM_FlightMode_norm_pv(unitVectorUtoV_tmp);
        unitVectorUtoV_tmp_0 = (unitVectorUtoV_tmp[0] / currentPosition *
          (b_waypointsIn[0] / y) + unitVectorUtoV_tmp[1] / currentPosition *
          (b_waypointsIn[1] / y)) + unitVectorUtoV_tmp[2] / currentPosition *
          (b_waypointsIn[2] / y);
        if (rtIsNaN(unitVectorUtoV_tmp_0) || (unitVectorUtoV_tmp_0 < 0.0)) {
        } else {
          guard2 = true;
        }
      }

      if (guard2) {
        obj->WaypointIndex++;
        p = false;
        if (obj->WaypointIndex == obj->NumWaypoints) {
          p = true;
        }

        if (p) {
          obj->LastWaypointFlag = true;
          obj->WaypointIndex--;
        }

        lookaheadDist_tmp = waypoints_data[(int32_T)obj->WaypointIndex - 1];
        b_endWaypoint_idx_0_tmp = waypoints_data[(int32_T)(obj->WaypointIndex +
          1.0) - 1];
        b_startWaypoint_idx_1_tmp = waypoints_data[((int32_T)obj->WaypointIndex
          + scalarLB) - 1];
        b_endWaypoint_idx_1_tmp = waypoints_data[((int32_T)(obj->WaypointIndex +
          1.0) + scalarLB) - 1];
        b_startWaypoint_idx_2_tmp = waypoints_data[((scalarLB << 1) + (int32_T)
          obj->WaypointIndex) - 1];
        b_endWaypoint_idx_2_tmp = waypoints_data[((int32_T)(obj->WaypointIndex +
          1.0) + (scalarLB << 1)) - 1];
        obj->InitYaw = waypoints_data[(scalarLB * 3 + (int32_T)
          obj->WaypointIndex) - 1];
        obj->FinalYaw = waypoints_data[((int32_T)(obj->WaypointIndex + 1.0) +
          scalarLB * 3) - 1];
      }

      unitVectorUtoV_tmp_0 = varargin_1[0] - lookaheadDist_tmp;
      unitVectorUtoV_tmp[0] = unitVectorUtoV_tmp_0;
      currentPosition = b_endWaypoint_idx_0_tmp - lookaheadDist_tmp;
      b_waypointsIn[0] = currentPosition;
      unitVectorUtoV_tmp_1 = unitVectorUtoV_tmp_0 * currentPosition;
      y = currentPosition * currentPosition;
      unitVectorUtoV_tmp_0 = varargin_1[1] - b_startWaypoint_idx_1_tmp;
      unitVectorUtoV_tmp[1] = unitVectorUtoV_tmp_0;
      currentPosition = b_endWaypoint_idx_1_tmp - b_startWaypoint_idx_1_tmp;
      b_waypointsIn[1] = currentPosition;
      unitVectorUtoV_tmp_1 += unitVectorUtoV_tmp_0 * currentPosition;
      y += currentPosition * currentPosition;
      unitVectorUtoV_tmp_0 = varargin_1[2] - b_startWaypoint_idx_2_tmp;
      unitVectorUtoV_tmp[2] = unitVectorUtoV_tmp_0;
      currentPosition = b_endWaypoint_idx_2_tmp - b_startWaypoint_idx_2_tmp;
      y += currentPosition * currentPosition;
      unitVectorUtoV_tmp_0 = (unitVectorUtoV_tmp_0 * currentPosition +
        unitVectorUtoV_tmp_1) / y;
      if (unitVectorUtoV_tmp_0 < 0.0) {
        dist = UAM_FlightMode_norm_pv(unitVectorUtoV_tmp);
      } else if (unitVectorUtoV_tmp_0 > 1.0) {
        _mm_storeu_pd(&varargin_1_0[0], _mm_sub_pd(_mm_loadu_pd(&varargin_1[0]),
          _mm_set_pd(b_endWaypoint_idx_1_tmp, b_endWaypoint_idx_0_tmp)));
        varargin_1_0[2] = varargin_1[2] - b_endWaypoint_idx_2_tmp;
        dist = UAM_FlightMode_norm_pv(varargin_1_0);
      } else {
        tmp = _mm_sub_pd(_mm_loadu_pd(&varargin_1[0]), _mm_add_pd(_mm_mul_pd
          (_mm_set1_pd(unitVectorUtoV_tmp_0), _mm_loadu_pd(&b_waypointsIn[0])),
          _mm_set_pd(b_startWaypoint_idx_1_tmp, lookaheadDist_tmp)));
        _mm_storeu_pd(&varargin_1_0[0], tmp);
        varargin_1_0[2] = varargin_1[2] - (unitVectorUtoV_tmp_0 *
          currentPosition + b_startWaypoint_idx_2_tmp);
        dist = UAM_FlightMode_norm_pv(varargin_1_0);
      }

      if (obj->LastWaypointFlag) {
        tmp = _mm_sub_pd(_mm_loadu_pd(&varargin_1[0]), _mm_add_pd(_mm_mul_pd
          (_mm_set1_pd(unitVectorUtoV_tmp_0), _mm_loadu_pd(&b_waypointsIn[0])),
          _mm_set_pd(b_startWaypoint_idx_1_tmp, lookaheadDist_tmp)));
        _mm_storeu_pd(&varargin_1_0[0], tmp);
        varargin_1_0[2] = varargin_1[2] - (unitVectorUtoV_tmp_0 *
          currentPosition + b_startWaypoint_idx_2_tmp);
        dist = UAM_FlightMode_norm_pv(varargin_1_0);
      }

      absx_tmp = fabs(dist);
      if (rtIsInf(absx_tmp) || rtIsNaN(absx_tmp)) {
        unitVectorUtoV_tmp_1 = (rtNaN);
        absx_tmp = (rtNaN);
      } else if (absx_tmp < 4.4501477170144028E-308) {
        unitVectorUtoV_tmp_1 = 4.94065645841247E-324;
        absx_tmp = 4.94065645841247E-324;
      } else {
        frexp(absx_tmp, &b_exponent);
        unitVectorUtoV_tmp_1 = ldexp(1.0, b_exponent - 53);
        frexp(absx_tmp, &b_exponent_0);
        absx_tmp = ldexp(1.0, b_exponent_0 - 53);
      }

      unitVectorUtoV_tmp_1 = sqrt(unitVectorUtoV_tmp_1);
      absx_tmp *= 5.0;
      if ((unitVectorUtoV_tmp_1 >= absx_tmp) || rtIsNaN(absx_tmp)) {
        absx_tmp = unitVectorUtoV_tmp_1;
      }

      if (obj->LookaheadDistance <= dist + absx_tmp) {
        obj->LookaheadDistance = obj->LookaheadFactor * dist;
      }

      _mm_storeu_pd(&unitVectorUtoV_tmp[0], _mm_sub_pd(_mm_set_pd
        (b_startWaypoint_idx_1_tmp, lookaheadDist_tmp), _mm_loadu_pd
        (&varargin_1[0])));
      unitVectorUtoV_tmp[2] = b_startWaypoint_idx_2_tmp - varargin_1[2];
      dist = ((b_waypointsIn[0] * unitVectorUtoV_tmp[0] + b_waypointsIn[1] *
               unitVectorUtoV_tmp[1]) + currentPosition * unitVectorUtoV_tmp[2])
        * 2.0;
      absx_tmp = sqrt(dist * dist - (((unitVectorUtoV_tmp[0] *
        unitVectorUtoV_tmp[0] + unitVectorUtoV_tmp[1] * unitVectorUtoV_tmp[1]) +
        unitVectorUtoV_tmp[2] * unitVectorUtoV_tmp[2]) - obj->LookaheadDistance *
        obj->LookaheadDistance) * (4.0 * y));
      unitVectorUtoV_tmp_1 = (-dist + absx_tmp) / 2.0 / y;
      y = (-dist - absx_tmp) / 2.0 / y;
      if ((unitVectorUtoV_tmp_1 >= y) || rtIsNaN(y)) {
        y = unitVectorUtoV_tmp_1;
      }

      varargout_1[0] = (1.0 - y) * lookaheadDist_tmp + y *
        b_endWaypoint_idx_0_tmp;
      varargout_1[1] = (1.0 - y) * b_startWaypoint_idx_1_tmp + y *
        b_endWaypoint_idx_1_tmp;
      varargout_1[2] = (1.0 - y) * b_startWaypoint_idx_2_tmp + y *
        b_endWaypoint_idx_2_tmp;
      *varargout_2 = rt_atan2d_snf(varargout_1[1] - varargin_1[1], varargout_1[0]
        - varargin_1[0]);
      p = (unitVectorUtoV_tmp_0 < 0.0);
      p_0 = (unitVectorUtoV_tmp_0 > 1.0);
      if (p) {
        y = lookaheadDist_tmp;
      } else if (p_0) {
        y = b_endWaypoint_idx_0_tmp;
      } else {
        y = unitVectorUtoV_tmp_0 * b_waypointsIn[0] + lookaheadDist_tmp;
      }

      unitVectorUtoV_tmp[0] = y - lookaheadDist_tmp;
      varargin_1_0[0] = lookaheadDist_tmp - b_endWaypoint_idx_0_tmp;
      if (p) {
        y = b_startWaypoint_idx_1_tmp;
      } else if (p_0) {
        y = b_endWaypoint_idx_1_tmp;
      } else {
        y = unitVectorUtoV_tmp_0 * b_waypointsIn[1] + b_startWaypoint_idx_1_tmp;
      }

      unitVectorUtoV_tmp[1] = y - b_startWaypoint_idx_1_tmp;
      varargin_1_0[1] = b_startWaypoint_idx_1_tmp - b_endWaypoint_idx_1_tmp;
      if (p) {
        y = b_startWaypoint_idx_2_tmp;
      } else if (p_0) {
        y = b_endWaypoint_idx_2_tmp;
      } else {
        y = unitVectorUtoV_tmp_0 * currentPosition + b_startWaypoint_idx_2_tmp;
      }

      unitVectorUtoV_tmp[2] = y - b_startWaypoint_idx_2_tmp;
      varargin_1_0[2] = b_startWaypoint_idx_2_tmp - b_endWaypoint_idx_2_tmp;
      *varargout_3 = UAM_FlightMode_wrapToPi_p(UAM_FlightMode_norm_pv
        (unitVectorUtoV_tmp) / UAM_FlightMode_norm_pv(varargin_1_0) *
        UAM_FlightMode_wrapToPi_p(obj->FinalYaw - obj->InitYaw) + obj->InitYaw);
      if (fabs(*varargout_3 - -3.1415926535897931) < 1.4901161193847656E-8) {
        *varargout_3 = 3.1415926535897931;
      }

      obj->LastWaypointFlag = false;
    }
  }

  *varargout_4 = obj->LookaheadDistFlag;
}

/* Function for Chart: '<Root>/Guidance Mode Selector' */
static void UAM_FlightMode_Start_d(const uint8_T *rtu_mode, const
  UAVPathManagerBus *rtu_ToWP, const UAVPathManagerBus *rtu_FromWP, const real_T
  rtu_Pose[4], const GuidanceStates *rtu_States, B_UAM_FlightMode_c_T *localB,
  DW_UAM_FlightMode_f_T *localDW)
{
  /* local block i/o variables */
  real_T rtb_wps_l[8];
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_2;
  real_T rtb_wps_h[8];
  real_T b_waypointsIn_data[6];
  real_T rtb_wps[6];
  real_T rtb_Product[3];
  real_T rtb_Sum[3];
  real_T rtb_TmpSignalConversionAtUAVO_1[3];
  real_T turnVector[3];
  real_T unitVectorUtoV_tmp[3];
  real_T b_tmp[2];
  real_T rtu_Pose_0[2];
  real_T tmp[2];
  real_T xyLookaheadPoint[2];
  real_T xyPose[2];
  real_T a;
  real_T absx_tmp;
  real_T d;
  real_T distToCenter;
  real_T h;
  real_T rtb_Abs;
  real_T rtb_DotProduct;
  real_T rtb_Sign;
  real_T xyPose_idx_0;
  int32_T b_exponent;
  int32_T b_exponent_0;
  int32_T b_k;
  int32_T b_waypointsIn_data_tmp;
  int32_T b_waypointsIn_tmp;
  int32_T i;
  int32_T scalarLB;
  int32_T tmp_size_idx_1;
  int32_T vectorUB;
  int8_T tmp_data[2];
  uint8_T b_varargout_4;
  boolean_T x[3];
  boolean_T distinctWptsIdx[2];
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T guard2;
  boolean_T p;
  boolean_T p_0;
  if ((*rtu_mode == 6) && (localB->FlightMode == FixedWing)) {
    localDW->temporalCounter_i1 = 0U;
    localDW->is_GuidanceLogic = UAM_FlightMode_IN_PreTransition;
    localB->FlightMode = FixedWing;
    localB->controlMode_m.airspeedAltitude = 1U;
    localB->controlMode_m.attitude = 1U;
    localB->controlMode_m.lateralGuidance = 1U;
    localB->aacSP.L1 = 25.0;
    localB->aacSP.airspeed = 14.0;
    localB->aacSP.altitude = -rtu_States->Xe[2];
    localB->aacSP.course = rtu_States->course;
  } else if ((*rtu_mode == 2) && (localB->FlightMode == FixedWing)) {
    localDW->is_GuidanceLogic = UAM_Flig_IN_FIXED_WING_WAYPOINT;
    localB->controlMode_m.airspeedAltitude = 1U;
    localB->controlMode_m.attitude = 1U;
    localB->controlMode_m.lateralGuidance = 1U;
    localDW->is_FIXED_WING_WAYPOINT = UAM_FlightMode_IN_WAYPOINT;

    /* MATLAB Function: '<S6>/MATLAB Function' */
    rtb_wps[0] = rtu_FromWP->position[0];
    rtb_wps[1] = rtu_ToWP->position[0];
    rtb_wps[2] = rtu_FromWP->position[1];
    rtb_wps[3] = rtu_ToWP->position[1];
    rtb_wps[4] = rtu_FromWP->position[2];
    rtb_wps[5] = rtu_ToWP->position[2];

    /* MATLABSystem: '<S6>/Waypoint Follower' incorporates:
     *  MATLAB Function: '<S6>/MATLAB Function'
     */
    localDW->obj_f.LookaheadDistFlag = 0U;
    localDW->obj_f.InitialPose[0] = 0.0;
    localDW->obj_f.InitialPose[1] = 0.0;
    localDW->obj_f.InitialPose[2] = 0.0;
    localDW->obj_f.InitialPose[3] = 0.0;
    localDW->obj_f.NumWaypoints = 2.0;
    p = false;
    p_0 = true;
    b_k = 0;
    exitg1 = false;
    while ((!exitg1) && (b_k <= 5)) {
      i = ((b_k / 2) << 1) + b_k % 2;
      if (!(localDW->obj_f.WaypointsInternal[i] == rtb_wps[i])) {
        p_0 = false;
        exitg1 = true;
      } else {
        b_k++;
      }
    }

    if (p_0) {
      p = true;
    }

    if (!p) {
      for (i = 0; i < 6; i++) {
        localDW->obj_f.WaypointsInternal[i] = rtb_wps[i];
      }

      localDW->obj_f.WaypointIndex = 1.0;
    }

    distinctWptsIdx[1] = true;
    x[0] = (rtu_FromWP->position[0] != rtu_ToWP->position[0]);
    x[1] = (rtu_FromWP->position[1] != rtu_ToWP->position[1]);
    x[2] = (rtu_FromWP->position[2] != rtu_ToWP->position[2]);
    distinctWptsIdx[0] = false;
    b_k = 0;
    exitg1 = false;
    while ((!exitg1) && (b_k < 3)) {
      if (x[b_k]) {
        distinctWptsIdx[0] = true;
        exitg1 = true;
      } else {
        b_k++;
      }
    }

    b_k = 0;
    for (i = 0; i < 2; i++) {
      /* MATLABSystem: '<S6>/Waypoint Follower' */
      if (distinctWptsIdx[i]) {
        b_k++;
      }
    }

    tmp_size_idx_1 = b_k;
    b_k = 0;
    for (i = 0; i < 2; i++) {
      /* MATLABSystem: '<S6>/Waypoint Follower' */
      if (distinctWptsIdx[i]) {
        /* Start for MATLABSystem: '<S6>/Waypoint Follower' */
        tmp_data[b_k] = (int8_T)i;
        b_k++;
      }
    }

    /* MATLABSystem: '<S6>/Waypoint Follower' incorporates:
     *  SignalConversion generated from: '<S6>/Waypoint Follower'
     */
    for (i = 0; i < 3; i++) {
      scalarLB = (tmp_size_idx_1 / 2) << 1;
      vectorUB = scalarLB - 2;
      for (b_k = 0; b_k <= vectorUB; b_k += 2) {
        b_waypointsIn_tmp = i << 1;
        b_waypointsIn_data_tmp = tmp_size_idx_1 * i;
        b_waypointsIn_data[b_k + b_waypointsIn_data_tmp] =
          rtb_wps[b_waypointsIn_tmp + tmp_data[b_k]];
        b_waypointsIn_data[(b_k + b_waypointsIn_data_tmp) + 1] =
          rtb_wps[tmp_data[b_k + 1] + b_waypointsIn_tmp];
      }

      for (b_k = scalarLB; b_k < tmp_size_idx_1; b_k++) {
        b_waypointsIn_data[b_k + tmp_size_idx_1 * i] = rtb_wps[(i << 1) +
          tmp_data[b_k]];
      }
    }

    localDW->obj_f.LookaheadDistance = 30.0;
    if (tmp_size_idx_1 == 0) {
      xyPose_idx_0 = rtu_States->course;
      localB->Status = 1U;
    } else {
      guard1 = false;
      if (tmp_size_idx_1 == 1) {
        if (localDW->obj_f.StartFlag) {
          localDW->obj_f.InitialPose[0] = rtu_States->Xe[0];
          localDW->obj_f.InitialPose[1] = rtu_States->Xe[1];
          localDW->obj_f.InitialPose[2] = rtu_States->Xe[2];
          localDW->obj_f.InitialPose[3] = rtu_States->course;
        }

        tmp_0 = _mm_sub_pd(_mm_loadu_pd(&b_waypointsIn_data[0]), _mm_loadu_pd
                           (&rtu_States->Xe[0]));
        _mm_storeu_pd(&rtb_Product[0], tmp_0);
        rtb_Product[2] = b_waypointsIn_data[2] - rtu_States->Xe[2];
        if (UAM_FlightMode_norm_pv(rtb_Product) < 1.4901161193847656E-8) {
          xyPose_idx_0 = rtu_States->course;
          localB->Status = 1U;
          localDW->obj_f.StartFlag = false;
        } else {
          localDW->obj_f.StartFlag = false;
          localDW->obj_f.NumWaypoints = 2.0;
          scalarLB = tmp_size_idx_1 + 1;
          for (i = 0; i < 3; i++) {
            vectorUB = (tmp_size_idx_1 + 1) * i;
            rtb_wps[vectorUB] = localDW->obj_f.InitialPose[i];
            for (b_k = 0; b_k < tmp_size_idx_1; b_k++) {
              rtb_wps[(b_k + vectorUB) + 1] = b_waypointsIn_data[tmp_size_idx_1 *
                i + b_k];
            }
          }

          guard1 = true;
        }
      } else {
        scalarLB = tmp_size_idx_1;
        b_k = tmp_size_idx_1 * 3;
        if (b_k - 1 >= 0) {
          memcpy(&rtb_wps[0], &b_waypointsIn_data[0], (uint32_T)b_k * sizeof
                 (real_T));
        }

        guard1 = true;
      }

      if (guard1) {
        p = false;
        if (localDW->obj_f.WaypointIndex == 2.0) {
          p = true;
        }

        if (p) {
          localDW->obj_f.LastWaypointFlag = true;
          localDW->obj_f.WaypointIndex--;
        }

        xyPose_idx_0 = rtb_wps[(int32_T)localDW->obj_f.WaypointIndex - 1];
        rtb_Product[0] = xyPose_idx_0;
        rtb_Abs = rtb_wps[(int32_T)(localDW->obj_f.WaypointIndex + 1.0) - 1];
        rtb_Sum[0] = rtb_Abs;
        rtb_DotProduct = rtb_wps[((int32_T)localDW->obj_f.WaypointIndex +
          scalarLB) - 1];
        rtb_Product[1] = rtb_DotProduct;
        rtb_Sign = rtb_wps[((int32_T)(localDW->obj_f.WaypointIndex + 1.0) +
                            scalarLB) - 1];
        rtb_Sum[1] = rtb_Sign;
        rtb_Product[2] = rtb_wps[((scalarLB << 1) + (int32_T)
          localDW->obj_f.WaypointIndex) - 1];
        d = rtb_wps[((int32_T)(localDW->obj_f.WaypointIndex + 1.0) + (scalarLB <<
          1)) - 1];
        rtb_Sum[2] = d;
        tmp_0 = _mm_set_pd(rtb_Sign, rtb_Abs);
        tmp_2 = _mm_loadu_pd(&rtu_States->Xe[0]);
        _mm_storeu_pd(&turnVector[0], _mm_sub_pd(tmp_2, tmp_0));
        turnVector[2] = rtu_States->Xe[2] - d;
        h = UAM_FlightMode_norm_pv(turnVector);
        guard2 = false;
        if (h <= 10.0) {
          guard2 = true;
        } else {
          _mm_storeu_pd(&unitVectorUtoV_tmp[0], _mm_sub_pd(tmp_0, _mm_set_pd
            (rtb_DotProduct, xyPose_idx_0)));
          unitVectorUtoV_tmp[2] = rtb_wps[((int32_T)
            (localDW->obj_f.WaypointIndex + 1.0) + (scalarLB << 1)) - 1] -
            rtb_wps[((scalarLB << 1) + (int32_T)localDW->obj_f.WaypointIndex) -
            1];
          d = UAM_FlightMode_norm_pv(unitVectorUtoV_tmp);
          xyPose_idx_0 = (unitVectorUtoV_tmp[0] / d * (turnVector[0] / h) +
                          unitVectorUtoV_tmp[1] / d * (turnVector[1] / h)) +
            unitVectorUtoV_tmp[2] / d * (turnVector[2] / h);
          if (rtIsNaN(xyPose_idx_0) || (xyPose_idx_0 < 0.0)) {
          } else {
            guard2 = true;
          }
        }

        if (guard2) {
          localDW->obj_f.WaypointIndex++;
          p = false;
          if (localDW->obj_f.WaypointIndex == 2.0) {
            p = true;
          }

          if (p) {
            localDW->obj_f.LastWaypointFlag = true;
            localDW->obj_f.WaypointIndex--;
          }

          rtb_Product[0] = rtb_wps[(int32_T)localDW->obj_f.WaypointIndex - 1];
          rtb_Sum[0] = rtb_wps[(int32_T)(localDW->obj_f.WaypointIndex + 1.0) - 1];
          rtb_Product[1] = rtb_wps[((int32_T)localDW->obj_f.WaypointIndex +
            scalarLB) - 1];
          rtb_Sum[1] = rtb_wps[((int32_T)(localDW->obj_f.WaypointIndex + 1.0) +
                                scalarLB) - 1];
          rtb_Product[2] = rtb_wps[((scalarLB << 1) + (int32_T)
            localDW->obj_f.WaypointIndex) - 1];
          rtb_Sum[2] = rtb_wps[((int32_T)(localDW->obj_f.WaypointIndex + 1.0) +
                                (scalarLB << 1)) - 1];
        }

        _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(rtb_Sum[0], rtu_States->Xe
          [0]), _mm_set1_pd(rtb_Product[0])));
        turnVector[0] = tmp[0];
        unitVectorUtoV_tmp[0] = tmp[1];
        _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(rtb_Sum[1], rtu_States->Xe
          [1]), _mm_set1_pd(rtb_Product[1])));
        turnVector[1] = tmp[0];
        unitVectorUtoV_tmp[1] = tmp[1];
        _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(rtb_Sum[2], rtu_States->Xe
          [2]), _mm_set1_pd(rtb_Product[2])));
        turnVector[2] = tmp[0];
        unitVectorUtoV_tmp[2] = tmp[1];
        xyPose_idx_0 = unitVectorUtoV_tmp[0] * unitVectorUtoV_tmp[0] +
          unitVectorUtoV_tmp[1] * unitVectorUtoV_tmp[1];
        rtb_DotProduct = tmp[1] * tmp[1] + xyPose_idx_0;
        rtb_Sign = ((turnVector[0] * unitVectorUtoV_tmp[0] + turnVector[1] *
                     unitVectorUtoV_tmp[1]) + tmp[0] * tmp[1]) / rtb_DotProduct;
        if (rtb_Sign < 0.0) {
          rtb_Sign = UAM_FlightMode_norm_pv(turnVector);
        } else if (rtb_Sign > 1.0) {
          tmp_0 = _mm_sub_pd(tmp_2, _mm_loadu_pd(&rtb_Sum[0]));
          _mm_storeu_pd(&rtb_TmpSignalConversionAtUAVO_1[0], tmp_0);
          rtb_TmpSignalConversionAtUAVO_1[2] = rtu_States->Xe[2] - rtb_Sum[2];
          rtb_Sign = UAM_FlightMode_norm_pv(rtb_TmpSignalConversionAtUAVO_1);
        } else {
          tmp_0 = _mm_sub_pd(tmp_2, _mm_add_pd(_mm_mul_pd(_mm_set1_pd(rtb_Sign),
            _mm_loadu_pd(&unitVectorUtoV_tmp[0])), _mm_loadu_pd(&rtb_Product[0])));
          _mm_storeu_pd(&rtb_TmpSignalConversionAtUAVO_1[0], tmp_0);
          rtb_TmpSignalConversionAtUAVO_1[2] = rtu_States->Xe[2] - (rtb_Sign *
            tmp[1] + rtb_Product[2]);
          rtb_Sign = UAM_FlightMode_norm_pv(rtb_TmpSignalConversionAtUAVO_1);
        }

        if (localDW->obj_f.LastWaypointFlag) {
          rtb_Sign = (((rtu_States->Xe[0] - rtb_Product[0]) *
                       unitVectorUtoV_tmp[0] + (rtu_States->Xe[1] - rtb_Product
            [1]) * unitVectorUtoV_tmp[1]) + (rtu_States->Xe[2] - rtb_Product[2])
                      * tmp[1]) / rtb_DotProduct;
          tmp_0 = _mm_sub_pd(tmp_2, _mm_add_pd(_mm_mul_pd(_mm_set1_pd(rtb_Sign),
            _mm_loadu_pd(&unitVectorUtoV_tmp[0])), _mm_loadu_pd(&rtb_Product[0])));
          _mm_storeu_pd(&rtb_TmpSignalConversionAtUAVO_1[0], tmp_0);
          rtb_TmpSignalConversionAtUAVO_1[2] = rtu_States->Xe[2] - (rtb_Sign *
            tmp[1] + rtb_Product[2]);
          rtb_Sign = UAM_FlightMode_norm_pv(rtb_TmpSignalConversionAtUAVO_1);
        }

        absx_tmp = fabs(rtb_Sign);
        if (rtIsInf(absx_tmp) || rtIsNaN(absx_tmp)) {
          a = (rtNaN);
          absx_tmp = (rtNaN);
        } else if (absx_tmp < 4.4501477170144028E-308) {
          a = 4.94065645841247E-324;
          absx_tmp = 4.94065645841247E-324;
        } else {
          frexp(absx_tmp, &b_exponent);
          a = ldexp(1.0, b_exponent - 53);
          frexp(absx_tmp, &b_exponent_0);
          absx_tmp = ldexp(1.0, b_exponent_0 - 53);
        }

        rtb_DotProduct = sqrt(a);
        a = 5.0 * absx_tmp;
        if ((rtb_DotProduct >= a) || rtIsNaN(a)) {
          a = rtb_DotProduct;
        }

        if (rtb_Sign + a >= 30.0) {
          localDW->obj_f.LookaheadDistance = localDW->obj_f.LookaheadFactor *
            rtb_Sign;
        }

        turnVector[0] = unitVectorUtoV_tmp[0];
        turnVector[1] = unitVectorUtoV_tmp[1];
        tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtb_Product[0]), tmp_2);
        _mm_storeu_pd(&unitVectorUtoV_tmp[0], tmp_0);
        xyPose_idx_0 += unitVectorUtoV_tmp[2] * unitVectorUtoV_tmp[2];
        unitVectorUtoV_tmp[2] = rtb_Product[2] - rtu_States->Xe[2];
        rtb_Sign = ((turnVector[0] * unitVectorUtoV_tmp[0] + turnVector[1] *
                     unitVectorUtoV_tmp[1]) + tmp[1] * unitVectorUtoV_tmp[2]) *
          2.0;
        a = sqrt(rtb_Sign * rtb_Sign - (((unitVectorUtoV_tmp[0] *
                    unitVectorUtoV_tmp[0] + unitVectorUtoV_tmp[1] *
                    unitVectorUtoV_tmp[1]) + unitVectorUtoV_tmp[2] *
                   unitVectorUtoV_tmp[2]) - localDW->obj_f.LookaheadDistance *
                  localDW->obj_f.LookaheadDistance) * (4.0 * xyPose_idx_0));
        rtb_DotProduct = (-rtb_Sign + a) / 2.0 / xyPose_idx_0;
        xyPose_idx_0 = (-rtb_Sign - a) / 2.0 / xyPose_idx_0;
        if ((rtb_DotProduct >= xyPose_idx_0) || rtIsNaN(xyPose_idx_0)) {
          xyPose_idx_0 = rtb_DotProduct;
        }

        tmp_0 = _mm_set1_pd(xyPose_idx_0);
        tmp_0 = _mm_add_pd(_mm_mul_pd(_mm_sub_pd(_mm_set1_pd(1.0), tmp_0),
          _mm_loadu_pd(&rtb_Product[0])), _mm_mul_pd(tmp_0, _mm_loadu_pd
          (&rtb_Sum[0])));
        _mm_storeu_pd(&rtb_Sum[0], tmp_0);
        xyPose_idx_0 = rt_atan2d_snf(rtb_Sum[1] - rtu_States->Xe[1], rtb_Sum[0]
          - rtu_States->Xe[0]);
        localB->Status = 0U;
        p = false;
        if (localDW->obj_f.LastWaypointFlag) {
          p = true;
        }

        if (p) {
          localB->Status = 1U;
        }

        localDW->obj_f.LastWaypointFlag = false;
      }
    }

    /* BusCreator: '<S6>/Bus Creator' incorporates:
     *  Constant: '<S6>/Constant'
     *  Constant: '<S6>/Constant1'
     *  Constant: '<S6>/Lookahead Distance'
     *  MATLAB Function: '<S6>/MATLAB Function'
     *  MATLABSystem: '<S6>/Waypoint Follower'
     * */
    localB->aacSP.airspeed = 15.0;
    localB->aacSP.altitude = -rtu_ToWP->position[2];
    localB->aacSP.course = xyPose_idx_0;
    localB->aacSP.L1 = 30.0;
    localB->aacSP.climbrate = 0.0;
  } else {
    switch (localB->FlightMode) {
     case Transition:
      localB->controlMode_m.inTransition = 0U;
      localDW->is_GuidanceLogic = UAM_FlightM_IN_FIXED_WING_ENTRY;
      enter_internal_FIXED_WING_ENTRY(localB, localDW);
      break;

     case BackTransition:
      localDW->is_GuidanceLogic = UAM_FlightMode_IN_HOVER_ENTRY;
      localB->FlightMode = Hover;
      localB->controlMode_m.inTransition = 0U;
      break;

     default:
      if ((*rtu_mode == 1) && (localB->FlightMode == Hover)) {
        localB->controlMode_m.inTransition = 0U;
        localDW->is_GuidanceLogic = UAM_FlightMode_IN_Takeoff;

        /* SignalConversion generated from: '<S11>/Bus Creator' */
        localB->InnerLoopCmds.LAP[0] = rtu_Pose[0];
        localB->InnerLoopCmds.LAP[1] = rtu_Pose[1];
        localB->InnerLoopCmds.LAP[2] = rtu_ToWP->position[2];

        /* Saturate: '<S11>/Hdg. Cmd Sat' */
        if (rtu_ToWP->params[3] > 3.1415926535897931) {
          /* BusCreator: '<S11>/Bus Creator' incorporates:
           *  Merge: '<S3>/ Merge 2'
           */
          localB->InnerLoopCmds.HeadingCmd = 3.1415926535897931;
        } else if (rtu_ToWP->params[3] < -3.1415926535897931) {
          /* BusCreator: '<S11>/Bus Creator' incorporates:
           *  Merge: '<S3>/ Merge 2'
           */
          localB->InnerLoopCmds.HeadingCmd = -3.1415926535897931;
        } else {
          /* BusCreator: '<S11>/Bus Creator' incorporates:
           *  Merge: '<S3>/ Merge 2'
           */
          localB->InnerLoopCmds.HeadingCmd = rtu_ToWP->params[3];
        }

        /* End of Saturate: '<S11>/Hdg. Cmd Sat' */

        /* BusCreator: '<S11>/Bus Creator' incorporates:
         *  Merge: '<S3>/ Merge 2'
         */
        localB->InnerLoopCmds.YawCmd = UAM_FlightMode_ConstB.YawCmd;

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S11>/Constant'
         *  SignalConversion generated from: '<S11>/Status'
         */
        localB->Status = 0U;
      } else if ((*rtu_mode == 3) && (localB->FlightMode == FixedWing)) {
        localDW->is_GuidanceLogic = UAM_FlightM_IN_FIXED_WING_ORBIT;
        localB->controlMode_m.airspeedAltitude = 1U;
        localB->controlMode_m.attitude = 1U;
        localB->controlMode_m.lateralGuidance = 1U;
        localDW->is_FIXED_WING_ORBIT = UAM_FlightMode_IN_ORBIT;

        /* MATLABSystem: '<S5>/UAV Orbit Follower' */
        rtb_DotProduct = rtu_ToWP->params[0];
        localDW->obj_b.OrbitRadiusFlag = 0U;
        if (rtu_ToWP->params[0] <= 50.0) {
          rtb_DotProduct = 50.0;
          localDW->obj_b.OrbitRadiusFlag = 1U;
        }

        localDW->obj_b.LookaheadDistFlag = 0U;

        /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' */
        tmp_2 = _mm_loadu_pd(&rtu_ToWP->position[0]);

        /* SignalConversion generated from: '<S5>/UAV Orbit Follower' */
        tmp_1 = _mm_loadu_pd(&rtu_States->Xe[0]);

        /* MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
         *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
         */
        _mm_storeu_pd(&xyLookaheadPoint[0], _mm_sub_pd(tmp_1, tmp_2));
        if (UAM_FlightMode_norm_p(xyLookaheadPoint) < 2.47032822920623E-323) {
          xyPose_idx_0 = rtu_States->course;
          d = localDW->obj_b.NumCircles;
        } else {
          p = false;
          p_0 = true;
          b_k = 0;
          exitg1 = false;
          while ((!exitg1) && (b_k < 3)) {
            if (!(localDW->obj_b.OrbitCenterInternal[b_k] == rtu_ToWP->
                  position[b_k])) {
              p_0 = false;
              exitg1 = true;
            } else {
              b_k++;
            }
          }

          if (p_0) {
            p = true;
          }

          guard1 = false;
          if (!p) {
            guard1 = true;
          } else {
            p = false;
            if (localDW->obj_b.OrbitRadiusInternal == rtb_DotProduct) {
              p = true;
            }

            if (!p) {
              guard1 = true;
            }
          }

          if (guard1) {
            localDW->obj_b.NumCircles = 0.0;
            localDW->obj_b.OrbitCenterInternal[0] = rtu_ToWP->position[0];
            localDW->obj_b.OrbitCenterInternal[1] = rtu_ToWP->position[1];
            localDW->obj_b.OrbitCenterInternal[2] = rtu_ToWP->position[2];
            localDW->obj_b.OrbitRadiusInternal = rtb_DotProduct;
            localDW->obj_b.SelectTurnDirectionFlag = true;
          }

          if (rtb_DotProduct <= 30.0) {
            localDW->obj_b.LookaheadDistance = 0.9 * rtb_DotProduct;
          } else {
            localDW->obj_b.LookaheadDistance = 30.0;
          }

          /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
           *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
           */
          d = rtu_States->Xe[0] - rtu_ToWP->position[0];
          b_tmp[0] = d;
          a = d * d;
          d = rtu_States->Xe[1] - rtu_ToWP->position[1];
          b_tmp[1] = d;
          distToCenter = sqrt(d * d + a);

          /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' */
          h = rtb_DotProduct + localDW->obj_b.LookaheadDistance;
          absx_tmp = fabs(h);

          /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' */
          p = (rtIsInf(absx_tmp) || rtIsNaN(absx_tmp));
          if (p) {
            a = (rtNaN);
          } else if (absx_tmp < 4.4501477170144028E-308) {
            a = 4.94065645841247E-324;
          } else {
            frexp(absx_tmp, &b_exponent);
            a = ldexp(1.0, b_exponent - 53);
          }

          guard1 = false;
          if (distToCenter >= h - 5.0 * a) {
            guard1 = true;
          } else {
            if (p) {
              a = (rtNaN);
            } else if (absx_tmp < 4.4501477170144028E-308) {
              a = 4.94065645841247E-324;
            } else {
              frexp(absx_tmp, &b_exponent_0);
              a = ldexp(1.0, b_exponent_0 - 53);
            }

            if (distToCenter <= (rtb_DotProduct -
                                 localDW->obj_b.LookaheadDistance) + 5.0 * a) {
              guard1 = true;
            } else {
              if (localDW->obj_b.StartFlag) {
                localDW->obj_b.PrevPosition[0] = rtu_States->Xe[0];
                localDW->obj_b.PrevPosition[1] = rtu_States->Xe[1];
                localDW->obj_b.StartFlag = false;
              }

              if ((rtu_ToWP->params[1] == 0.0) &&
                  (!localDW->obj_b.SelectTurnDirectionFlag)) {
                xyPose_idx_0 = localDW->obj_b.TurnDirectionInternal;
              } else {
                xyPose_idx_0 = rtu_ToWP->params[1];
              }

              _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
                (localDW->obj_b.PrevPosition[0], rtu_States->Xe[0]), _mm_set1_pd
                (rtu_ToWP->position[0])));
              xyPose[0] = tmp[0];
              turnVector[0] = tmp[1];
              _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
                (localDW->obj_b.PrevPosition[1], rtu_States->Xe[1]), _mm_set1_pd
                (rtu_ToWP->position[1])));
              xyPose[1] = tmp[0];
              turnVector[1] = tmp[1];
              d = UAM_FlightMode_norm_p(xyPose);
              h = localDW->obj_b.LookaheadDistance *
                localDW->obj_b.LookaheadDistance;
              a = ((h - rtb_DotProduct * rtb_DotProduct) + d * d) / (2.0 * d);
              tmp_0 = _mm_set1_pd(d);
              tmp_2 = _mm_sub_pd(tmp_2, tmp_1);
              _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(tmp_2,
                _mm_set1_pd(a)), tmp_0), tmp_1));
              distToCenter = tmp[1];
              h = sqrt(h - a * a);
              b_tmp[0] = tmp[0] - (rtu_ToWP->position[1] - rtu_States->Xe[1]) *
                h / d;
              _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd
                (_mm_set_pd(rtu_ToWP->position[0], rtu_ToWP->position[1]),
                 _mm_set_pd(rtu_States->Xe[0], rtu_States->Xe[1])), _mm_set1_pd
                (h)), tmp_0), _mm_set_pd(tmp[1], tmp[0])));
              b_tmp[1] = tmp[0];
              a = tmp[1];
              distToCenter -= (rtu_ToWP->position[0] - rtu_States->Xe[0]) * h /
                d;
              rtb_Product[0] = turnVector[0];
              rtb_Product[1] = turnVector[1];
              rtb_Product[2] = 0.0;
              rtb_Sum[0] = xyLookaheadPoint[0];
              rtb_Sum[1] = xyLookaheadPoint[1];
              rtb_Sum[2] = 0.0;
              if (xyPose_idx_0 < 0.0) {
                rtb_Product[0] = xyLookaheadPoint[0];
                rtb_Sum[0] = turnVector[0];
                rtb_Product[1] = xyLookaheadPoint[1];
                rtb_Sum[1] = turnVector[1];
                rtb_Product[2] = 0.0;
                rtb_Sum[2] = 0.0;
              }

              h = UAM_FlightMode_norm_pv(rtb_Product);
              d = UAM_FlightMode_norm_pv(rtb_Sum);
              tmp_0 = _mm_set_pd(d, h);
              _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(rtb_Sum[0],
                rtb_Product[0]), tmp_0));
              rtb_Product[0] = tmp[0];
              rtb_Sum[0] = tmp[1];
              _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(rtb_Sum[1],
                rtb_Product[1]), tmp_0));
              rtb_Sum[1] = tmp[1];
              turnVector[2] = rtb_Product[0] * tmp[1] - rtb_Sum[0] * tmp[0];
              localDW->obj_b.PrevPosition[0] = rtu_States->Xe[0];
              localDW->obj_b.PrevPosition[1] = rtu_States->Xe[1];
              localDW->obj_b.PrevPosition[2] = rtu_States->Xe[2];
              localDW->obj_b.NumCircles += rt_atan2d_snf(turnVector[2],
                (rtb_Product[0] * rtb_Sum[0] + tmp[0] * tmp[1]) + 0.0 / h * (0.0
                / d)) / 2.0 / 3.1415926535897931;
              d = localDW->obj_b.NumCircles;
              _mm_storeu_pd(&rtb_Sum[0], tmp_2);
              if (rtIsNaN(xyPose_idx_0)) {
                h = (rtNaN);
              } else if (xyPose_idx_0 < 0.0) {
                h = -1.0;
              } else {
                h = (xyPose_idx_0 > 0.0);
              }

              switch ((int32_T)h) {
               case 1:
                if ((b_tmp[0] - rtu_States->Xe[0]) * rtb_Sum[1] - (a -
                     rtu_States->Xe[1]) * rtb_Sum[0] > 0.0) {
                  xyPose_idx_0 = b_tmp[0];
                  distToCenter = a;
                } else {
                  xyPose_idx_0 = b_tmp[1];
                }
                break;

               case -1:
                if ((b_tmp[0] - rtu_States->Xe[0]) * rtb_Sum[1] - (a -
                     rtu_States->Xe[1]) * rtb_Sum[0] < 0.0) {
                  xyPose_idx_0 = b_tmp[0];
                  distToCenter = a;
                } else {
                  xyPose_idx_0 = b_tmp[1];
                }
                break;

               default:
                if (fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(a - rtu_States->
                       Xe[1], b_tmp[0] - rtu_States->Xe[0]), rtu_States->course))
                    < fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(distToCenter -
                       rtu_States->Xe[1], b_tmp[1] - rtu_States->Xe[0]),
                      rtu_States->course))) {
                  xyPose_idx_0 = b_tmp[0];
                  distToCenter = a;
                } else {
                  xyPose_idx_0 = b_tmp[1];
                }

                if ((xyPose_idx_0 - rtu_States->Xe[0]) * rtb_Sum[1] -
                    (distToCenter - rtu_States->Xe[1]) * rtb_Sum[0] > 0.0) {
                  localDW->obj_b.TurnDirectionInternal = 1.0;
                } else {
                  localDW->obj_b.TurnDirectionInternal = -1.0;
                }

                localDW->obj_b.SelectTurnDirectionFlag = false;
                break;
              }
            }
          }

          if (guard1) {
            _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_mul_pd(_mm_div_pd(_mm_set_pd(d,
              b_tmp[0]), _mm_set1_pd(UAM_FlightMode_norm_p(b_tmp))), _mm_set1_pd
              (rtb_DotProduct)), tmp_2));
            xyPose_idx_0 = tmp[0];
            distToCenter = tmp[1];
            d = localDW->obj_b.NumCircles;
          }

          xyPose_idx_0 = rt_atan2d_snf(distToCenter - rtu_States->Xe[1],
            xyPose_idx_0 - rtu_States->Xe[0]);
        }

        /* BusCreator: '<S5>/Bus Creator' incorporates:
         *  Constant: '<S5>/Constant'
         *  Constant: '<S5>/Constant1'
         *  Constant: '<S5>/Lookahead Distance'
         *  MATLABSystem: '<S5>/UAV Orbit Follower'
         *  UnaryMinus: '<S5>/Unary Minus'
         * */
        localB->aacSP.airspeed = 15.0;
        localB->aacSP.altitude = -rtu_ToWP->position[2];
        localB->aacSP.course = xyPose_idx_0;
        localB->aacSP.L1 = 30.0;
        localB->aacSP.climbrate = 0.0;

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Abs: '<S5>/Abs1'
         *  DataTypeConversion: '<S5>/Data Type Conversion'
         *  MATLABSystem: '<S5>/UAV Orbit Follower'
         *  RelationalOperator: '<S5>/Relational Operator'
         * */
        localB->Status = (uint8_T)(fabs(d) > rtu_ToWP->params[2]);
      } else if (localB->FlightMode == Hover) {
        localB->controlMode_m.inTransition = 0U;
        switch (*rtu_mode) {
         case 2U:
          localDW->is_GuidanceLogic = UAM_FlightMode_IN_WP;

          /* Switch: '<S17>/Switch' incorporates:
           *  Product: '<S21>/Product'
           *  RelationalOperator: '<S17>/Equal'
           */
          if (rtu_FromWP->mode == 6) {
            rtb_Product[0] = rtu_FromWP->position[0];
            rtb_Product[1] = rtu_FromWP->position[1];
            rtb_Product[2] = rtu_FromWP->position[2];
          } else {
            rtb_Product[0] = 0.0;
            rtb_Product[1] = 0.0;
            rtb_Product[2] = 0.0;
          }

          /* End of Switch: '<S17>/Switch' */

          /* MATLAB Function: '<S12>/MATLAB Function' incorporates:
           *  BusCreator generated from: '<S12>/MATLAB Function'
           *  Product: '<S21>/Product'
           */
          rtb_wps_h[0] = rtb_Product[0];
          rtb_wps_h[2] = rtb_Product[1];
          rtb_wps_h[4] = rtb_Product[2];
          rtb_wps_h[6] = rtu_FromWP->params[3];
          rtb_wps_h[1] = rtu_ToWP->position[0];
          rtb_wps_h[3] = rtu_ToWP->position[1];
          rtb_wps_h[5] = rtu_ToWP->position[2];
          rtb_wps_h[7] = rtu_ToWP->params[3];

          /* MATLABSystem: '<S12>/Waypoint Follower' incorporates:
           *  Constant: '<S12>/Lookahead Distance'
           */
          UAM_FlightMode_SystemCore_step(&localDW->obj, rtu_Pose, rtb_wps_h, 5.0,
            localB->InnerLoopCmds.LAP, &xyPose_idx_0, &rtb_DotProduct,
            &b_varargout_4);

          /* BusCreator: '<S12>/Bus Creator' incorporates:
           *  Constant: '<S12>/Constant'
           *  MATLABSystem: '<S12>/Waypoint Follower'
           *  Merge: '<S3>/ Merge 2'
           * */
          localB->InnerLoopCmds.HeadingCmd = xyPose_idx_0;
          localB->InnerLoopCmds.YawCmd = 0.0;

          /* Sum: '<S18>/Sum1' incorporates:
           *  Product: '<S21>/Product'
           */
          xyPose_idx_0 = rtu_ToWP->position[0] - rtb_Product[0];
          rtb_Product[0] = xyPose_idx_0;

          /* DotProduct: '<S21>/Dot Product1' incorporates:
           *  Product: '<S21>/Product'
           */
          rtb_DotProduct = xyPose_idx_0 * xyPose_idx_0;

          /* Sum: '<S18>/Sum1' incorporates:
           *  Product: '<S21>/Product'
           */
          xyPose_idx_0 = rtu_ToWP->position[1] - rtb_Product[1];
          rtb_Product[1] = xyPose_idx_0;

          /* DotProduct: '<S21>/Dot Product1' incorporates:
           *  Product: '<S21>/Product'
           */
          rtb_DotProduct += xyPose_idx_0 * xyPose_idx_0;

          /* Sum: '<S18>/Sum1' incorporates:
           *  Product: '<S21>/Product'
           */
          xyPose_idx_0 = rtu_ToWP->position[2] - rtb_Product[2];

          /* DotProduct: '<S21>/Dot Product1' incorporates:
           *  Product: '<S21>/Product'
           */
          rtb_DotProduct += xyPose_idx_0 * xyPose_idx_0;

          /* Saturate: '<S21>/Saturation' incorporates:
           *  DotProduct: '<S21>/Dot Product1'
           */
          if (rtb_DotProduct <= 1.0E-5) {
            rtb_DotProduct = 1.0E-5;
          }

          /* Sqrt: '<S21>/Sqrt' incorporates:
           *  Saturate: '<S21>/Saturation'
           */
          rtb_DotProduct = sqrt(rtb_DotProduct);

          /* Sum: '<S18>/Sum' */
          tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtu_ToWP->position[0]), _mm_loadu_pd(
            &rtu_Pose[0]));
          _mm_storeu_pd(&rtb_Sum[0], tmp_0);
          rtb_Sum[2] = rtu_ToWP->position[2] - rtu_Pose[2];

          /* Merge: '<S3>/ Merge ' incorporates:
           *  Constant: '<S12>/Lookahead Distance'
           *  DataTypeConversion: '<S12>/Data Type Conversion'
           *  DotProduct: '<S18>/Dot Product'
           *  Product: '<S21>/Product'
           *  RelationalOperator: '<S12>/Relational Operator'
           *  Sum: '<S18>/Sum'
           *  Sum: '<S18>/Sum1'
           */
          localB->Status = (uint8_T)((rtb_Product[0] / rtb_DotProduct * rtb_Sum
            [0] + rtb_Product[1] / rtb_DotProduct * rtb_Sum[1]) + xyPose_idx_0 /
            rtb_DotProduct * rtb_Sum[2] <= 5.0);
          break;

         case 3U:
          localDW->is_GuidanceLogic = UAM_FlightMode_IN_Orbit;

          /* Abs: '<S10>/Abs' */
          rtb_Abs = fabs(rtu_ToWP->params[0]);

          /* Signum: '<S10>/Sign' */
          if (rtIsNaN(rtu_ToWP->params[1])) {
            rtb_Sign = (rtNaN);
          } else if (rtu_ToWP->params[1] < 0.0) {
            rtb_Sign = -1.0;
          } else {
            rtb_Sign = (rtu_ToWP->params[1] > 0.0);
          }

          /* End of Signum: '<S10>/Sign' */

          /* MATLABSystem: '<S10>/UAV Orbit Follower' */
          localDW->obj_l.OrbitRadiusFlag = 0U;
          if (rtb_Abs <= 1.0) {
            rtb_Abs = 1.0;
            localDW->obj_l.OrbitRadiusFlag = 1U;
          }

          localDW->obj_l.LookaheadDistFlag = 0U;

          /* Start for MATLABSystem: '<S10>/UAV Orbit Follower' */
          tmp_2 = _mm_loadu_pd(&rtu_ToWP->position[0]);
          tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtu_Pose[0]), tmp_2);

          /* MATLABSystem: '<S10>/UAV Orbit Follower' */
          _mm_storeu_pd(&rtu_Pose_0[0], tmp_0);
          if (UAM_FlightMode_norm_p(rtu_Pose_0) < 2.47032822920623E-323) {
            tmp_0 = _mm_add_pd(_mm_mul_pd(_mm_set1_pd(rtb_Abs), _mm_set_pd(sin
              (rtu_Pose[3]), cos(rtu_Pose[3]))), _mm_loadu_pd(&rtu_Pose[0]));
            _mm_storeu_pd(&rtb_Product[0], tmp_0);
            rtb_Product[2] = rtu_ToWP->position[2];
            xyPose_idx_0 = rtu_Pose[3];
            rtb_Sign = rtu_Pose[3];
            d = localDW->obj_l.NumCircles;
          } else {
            p = false;
            p_0 = true;
            b_k = 0;
            exitg1 = false;
            while ((!exitg1) && (b_k < 3)) {
              if (!(localDW->obj_l.OrbitCenterInternal[b_k] ==
                    rtu_ToWP->position[b_k])) {
                p_0 = false;
                exitg1 = true;
              } else {
                b_k++;
              }
            }

            if (p_0) {
              p = true;
            }

            guard1 = false;
            if (!p) {
              guard1 = true;
            } else {
              p = false;
              if (localDW->obj_l.OrbitRadiusInternal == rtb_Abs) {
                p = true;
              }

              if (!p) {
                guard1 = true;
              }
            }

            if (guard1) {
              localDW->obj_l.NumCircles = 0.0;
              localDW->obj_l.OrbitCenterInternal[0] = rtu_ToWP->position[0];
              localDW->obj_l.OrbitCenterInternal[1] = rtu_ToWP->position[1];
              localDW->obj_l.OrbitCenterInternal[2] = rtu_ToWP->position[2];
              localDW->obj_l.OrbitRadiusInternal = rtb_Abs;
              localDW->obj_l.SelectTurnDirectionFlag = true;
            }

            if (rtb_Abs <= 5.0) {
              localDW->obj_l.LookaheadDistance = 0.9 * rtb_Abs;
            } else {
              localDW->obj_l.LookaheadDistance = 5.0;
            }

            xyPose_idx_0 = rtu_Pose[0];
            rtb_DotProduct = rtu_Pose[1];

            /* Start for MATLABSystem: '<S10>/UAV Orbit Follower' */
            d = xyPose_idx_0 - rtu_ToWP->position[0];
            b_tmp[0] = d;
            a = d * d;
            d = rtb_DotProduct - rtu_ToWP->position[1];
            b_tmp[1] = d;
            distToCenter = sqrt(d * d + a);

            /* Start for MATLABSystem: '<S10>/UAV Orbit Follower' */
            h = rtb_Abs + localDW->obj_l.LookaheadDistance;
            p = (rtIsInf(h) || rtIsNaN(h));
            if (p) {
              a = (rtNaN);
            } else if (h < 4.4501477170144028E-308) {
              a = 4.94065645841247E-324;
            } else {
              frexp(h, &b_exponent);
              a = ldexp(1.0, b_exponent - 53);
            }

            guard1 = false;
            if (distToCenter >= h - 5.0 * a) {
              guard1 = true;
            } else {
              if (p) {
                a = (rtNaN);
              } else if (h < 4.4501477170144028E-308) {
                a = 4.94065645841247E-324;
              } else {
                frexp(h, &b_exponent_0);
                a = ldexp(1.0, b_exponent_0 - 53);
              }

              if (distToCenter <= (rtb_Abs - localDW->obj_l.LookaheadDistance) +
                  5.0 * a) {
                guard1 = true;
              } else {
                if (localDW->obj_l.StartFlag) {
                  localDW->obj_l.PrevPosition[0] = rtu_Pose[0];
                  localDW->obj_l.PrevPosition[1] = rtu_Pose[1];
                  localDW->obj_l.StartFlag = false;
                }

                if ((rtb_Sign == 0.0) &&
                    (!localDW->obj_l.SelectTurnDirectionFlag)) {
                  rtb_Sign = localDW->obj_l.TurnDirectionInternal;
                }

                _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
                  (localDW->obj_l.PrevPosition[0], xyPose_idx_0), _mm_set1_pd
                  (rtu_ToWP->position[0])));
                xyPose[0] = tmp[0];
                turnVector[0] = tmp[1];
                _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
                  (localDW->obj_l.PrevPosition[1], rtb_DotProduct), _mm_set1_pd
                  (rtu_ToWP->position[1])));
                xyPose[1] = tmp[0];
                turnVector[1] = tmp[1];
                d = UAM_FlightMode_norm_p(xyPose);
                h = localDW->obj_l.LookaheadDistance *
                  localDW->obj_l.LookaheadDistance;
                a = ((h - rtb_Abs * rtb_Abs) + d * d) / (2.0 * d);
                tmp_0 = _mm_set_pd(rtb_DotProduct, xyPose_idx_0);
                tmp_1 = _mm_set1_pd(d);
                _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd
                  (_mm_sub_pd(tmp_2, tmp_0), _mm_set1_pd(a)), tmp_1), tmp_0));
                distToCenter = tmp[1];
                h = sqrt(h - a * a);
                absx_tmp = rtu_ToWP->position[1] - rtb_DotProduct;
                b_tmp[0] = tmp[0] - absx_tmp * h / d;
                _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd
                  (_mm_sub_pd(_mm_set_pd(rtu_ToWP->position[0],
                  rtu_ToWP->position[1]), _mm_set_pd(xyPose_idx_0,
                  rtb_DotProduct)), _mm_set1_pd(h)), tmp_1), _mm_set_pd(tmp[1],
                  tmp[0])));
                b_tmp[1] = tmp[0];
                a = tmp[1];
                rtb_Abs = rtu_ToWP->position[0] - xyPose_idx_0;
                distToCenter -= rtb_Abs * h / d;
                rtb_Product[0] = turnVector[0];
                rtb_Product[1] = turnVector[1];
                rtb_Product[2] = 0.0;
                tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtu_Pose[0]), _mm_loadu_pd
                                   (&rtu_ToWP->position[0]));
                _mm_storeu_pd(&rtb_Sum[0], tmp_0);
                rtb_Sum[2] = 0.0;
                if (rtb_Sign < 0.0) {
                  rtb_Product[0] = rtb_Sum[0];
                  rtb_Sum[0] = turnVector[0];
                  rtb_Product[1] = rtb_Sum[1];
                  rtb_Sum[1] = turnVector[1];
                  rtb_Product[2] = 0.0;
                  rtb_Sum[2] = 0.0;
                }

                h = UAM_FlightMode_norm_pv(rtb_Product);
                d = UAM_FlightMode_norm_pv(rtb_Sum);
                tmp_0 = _mm_set_pd(d, h);
                _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(rtb_Sum[0],
                  rtb_Product[0]), tmp_0));
                rtb_Product[0] = tmp[0];
                rtb_Sum[0] = tmp[1];
                _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(rtb_Sum[1],
                  rtb_Product[1]), tmp_0));
                rtb_Sum[1] = tmp[1];
                turnVector[2] = rtb_Product[0] * tmp[1] - rtb_Sum[0] * tmp[0];
                localDW->obj_l.NumCircles += rt_atan2d_snf(turnVector[2],
                  (rtb_Product[0] * rtb_Sum[0] + tmp[0] * tmp[1]) + 0.0 / h *
                  (0.0 / d)) / 2.0 / 3.1415926535897931;
                d = localDW->obj_l.NumCircles;
                localDW->obj_l.PrevPosition[0] = rtu_Pose[0];
                localDW->obj_l.PrevPosition[1] = rtu_Pose[1];
                localDW->obj_l.PrevPosition[2] = rtu_Pose[2];
                tmp_0 = _mm_sub_pd(_mm_set_pd(a, b_tmp[0]), _mm_loadu_pd
                                   (&rtu_Pose[0]));
                _mm_storeu_pd(&rtb_Product[0], tmp_0);
                tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtu_ToWP->position[0]),
                                   _mm_loadu_pd(&rtu_Pose[0]));
                _mm_storeu_pd(&rtb_Sum[0], tmp_0);
                if (rtIsNaN(rtb_Sign)) {
                  h = (rtNaN);
                } else if (rtb_Sign < 0.0) {
                  h = -1.0;
                } else {
                  h = (rtb_Sign > 0.0);
                }

                switch ((int32_T)h) {
                 case 1:
                  if (rtb_Product[0] * rtb_Sum[1] - rtb_Sum[0] * rtb_Product[1] >
                      0.0) {
                    xyLookaheadPoint[0] = b_tmp[0];
                    xyLookaheadPoint[1] = a;
                  } else {
                    xyLookaheadPoint[0] = b_tmp[1];
                    xyLookaheadPoint[1] = distToCenter;
                  }
                  break;

                 case -1:
                  if (rtb_Product[0] * rtb_Sum[1] - rtb_Sum[0] * rtb_Product[1] <
                      0.0) {
                    xyLookaheadPoint[0] = b_tmp[0];
                    xyLookaheadPoint[1] = a;
                  } else {
                    xyLookaheadPoint[0] = b_tmp[1];
                    xyLookaheadPoint[1] = distToCenter;
                  }
                  break;

                 default:
                  p = (fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(a - rtu_Pose[1],
                          b_tmp[0] - rtu_Pose[0]), rtu_Pose[3])) < fabs
                       (UAM_FlightMode_angdiff(rt_atan2d_snf(distToCenter -
                          rtu_Pose[1], b_tmp[1] - rtu_Pose[0]), rtu_Pose[3])));
                  if (p) {
                    xyLookaheadPoint[0] = b_tmp[0];
                    xyLookaheadPoint[1] = a;
                  } else {
                    xyLookaheadPoint[0] = b_tmp[1];
                    xyLookaheadPoint[1] = distToCenter;
                  }

                  tmp_0 = _mm_sub_pd(_mm_loadu_pd(&xyLookaheadPoint[0]),
                                     _mm_loadu_pd(&rtu_Pose[0]));
                  _mm_storeu_pd(&rtb_Product[0], tmp_0);
                  if (rtb_Product[0] * rtb_Sum[1] - rtb_Sum[0] * rtb_Product[1] >
                      0.0) {
                    localDW->obj_l.TurnDirectionInternal = 1.0;
                  } else {
                    localDW->obj_l.TurnDirectionInternal = -1.0;
                  }

                  localDW->obj_l.SelectTurnDirectionFlag = false;
                  break;
                }

                rtb_Sign = rt_atan2d_snf(absx_tmp, rtb_Abs);
              }
            }

            if (guard1) {
              _mm_storeu_pd(&xyLookaheadPoint[0], _mm_add_pd(_mm_mul_pd
                (_mm_div_pd(_mm_set_pd(d, b_tmp[0]), _mm_set1_pd
                            (UAM_FlightMode_norm_p(b_tmp))), _mm_set1_pd(rtb_Abs)),
                tmp_2));
              rtb_Sign = rt_atan2d_snf(xyLookaheadPoint[1] - rtb_DotProduct,
                xyLookaheadPoint[0] - xyPose_idx_0);
              d = localDW->obj_l.NumCircles;
            }

            rtb_Product[0] = xyLookaheadPoint[0];
            rtb_Product[1] = xyLookaheadPoint[1];
            rtb_Product[2] = rtu_ToWP->position[2];
            xyPose_idx_0 = rt_atan2d_snf(xyLookaheadPoint[1] - rtb_DotProduct,
              xyLookaheadPoint[0] - xyPose_idx_0);
          }

          /* BusCreator: '<S10>/Bus Creator' incorporates:
           *  MATLABSystem: '<S10>/UAV Orbit Follower'
           *  Merge: '<S3>/ Merge 2'
           * */
          localB->InnerLoopCmds.LAP[0] = rtb_Product[0];
          localB->InnerLoopCmds.LAP[1] = rtb_Product[1];
          localB->InnerLoopCmds.LAP[2] = rtb_Product[2];
          localB->InnerLoopCmds.HeadingCmd = xyPose_idx_0;
          localB->InnerLoopCmds.YawCmd = rtb_Sign;

          /* Merge: '<S3>/ Merge ' incorporates:
           *  DataTypeConversion: '<S10>/Data Type Conversion'
           *  MATLABSystem: '<S10>/UAV Orbit Follower'
           *  RelationalOperator: '<S10>/Relational Operator'
           * */
          localB->Status = (uint8_T)(d > rtu_ToWP->params[2]);
          break;

         case 6U:
          localB->FlightMode = Transition;
          localB->controlMode_m.inTransition = 1U;
          localB->controlMode_m.TransitionCondition = 0U;
          localDW->is_GuidanceLogic = UAM_Flight_IN_ForwardTransition;

          /* BusCreator: '<S7>/Bus Creator1' incorporates:
           *  Constant: '<S7>/Constant1'
           *  Constant: '<S7>/Constant2'
           *  Constant: '<S7>/Constant3'
           *  Merge: '<S3>/ Merge 2'
           */
          localB->InnerLoopCmds.LAP[0] = 0.0;
          localB->InnerLoopCmds.LAP[1] = 0.0;
          localB->InnerLoopCmds.LAP[2] = 0.0;
          localB->InnerLoopCmds.HeadingCmd = 0.0;
          localB->InnerLoopCmds.YawCmd = 0.0;

          /* Merge: '<S3>/ Merge ' incorporates:
           *  Constant: '<S7>/Constant'
           *  DataTypeConversion: '<S7>/Data Type Conversion'
           */
          localB->Status = 0U;
          break;

         default:
          localDW->is_GuidanceLogic = UAM_FlightMode_IN_Land;
          localDW->is_Land = UAM_FlightMode_IN_ToLand;

          /* MATLAB Function: '<S9>/MATLAB Function' */
          rtb_wps_l[0] = rtu_FromWP->position[0];
          rtb_wps_l[2] = rtu_FromWP->position[1];
          rtb_wps_l[4] = rtu_FromWP->position[2];
          rtb_wps_l[6] = rtu_FromWP->params[3];
          rtb_wps_l[1] = rtu_ToWP->position[0];
          rtb_wps_l[3] = rtu_ToWP->position[1];
          rtb_wps_l[5] = rtu_ToWP->position[2];
          rtb_wps_l[7] = rtu_ToWP->params[3];

          /* Constant: '<S9>/Lookahead Distance' */
          UAM_FlightMode_WaypointFollower(rtu_Pose, rtb_wps_l, 3.0,
            &localB->WaypointFollower_c2, &localDW->WaypointFollower_c2);

          /* BusCreator: '<S9>/Bus Creator1' incorporates:
           *  MATLABSystem: '<S9>/Waypoint Follower'
           *  Merge: '<S3>/ Merge 2'
           */
          localB->InnerLoopCmds.LAP[0] = localB->WaypointFollower_c2.LAP[0];
          localB->InnerLoopCmds.LAP[1] = localB->WaypointFollower_c2.LAP[1];
          localB->InnerLoopCmds.LAP[2] = localB->WaypointFollower_c2.LAP[2];
          localB->InnerLoopCmds.HeadingCmd =
            localB->WaypointFollower_c2.HeadingCmd;
          localB->InnerLoopCmds.YawCmd = localB->WaypointFollower_c2.YawCmd;

          /* Merge: '<S3>/ Merge ' incorporates:
           *  Constant: '<S9>/Constant'
           *  SignalConversion generated from: '<S9>/Status'
           */
          localB->Status = 0U;
          break;
        }
      }
      break;
    }
  }
}

/* Function for Chart: '<Root>/Guidance Mode Selector' */
static void UAM_FlightMode_GuidanceLogic(const uint8_T *rtu_mode, const
  UAVPathManagerBus *rtu_ToWP, const UAVPathManagerBus *rtu_FromWP, const real_T
  rtu_Pose[4], const GuidanceStates *rtu_States, const real_T *rtu_Ground,
  B_UAM_FlightMode_c_T *localB, DW_UAM_FlightMode_f_T *localDW)
{
  /* local block i/o variables */
  real_T rtb_wps_l[8];
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_2;
  __m128d tmp_3;
  real_T rtb_wps[8];
  real_T b_waypointsIn_data[6];
  real_T rtb_wps_k[6];
  real_T rtb_Product[3];
  real_T rtb_Sum[3];
  real_T turnVector[3];
  real_T turnVector_0[3];
  real_T unitVectorUtoV_tmp[3];
  real_T b_tmp[2];
  real_T rtu_Pose_0[2];
  real_T tmp[2];
  real_T xyLookaheadPoint[2];
  real_T xyPose[2];
  real_T absxk;
  real_T b_tmp_0;
  real_T distToCenter;
  real_T h;
  real_T rtb_Abs;
  real_T rtb_DotProduct;
  real_T t;
  real_T turnInternal;
  real_T y2;
  int32_T b_exponent;
  int32_T b_exponent_0;
  int32_T b_k;
  int32_T b_waypointsIn_data_tmp;
  int32_T b_waypointsIn_tmp;
  int32_T i;
  int32_T scalarLB;
  int32_T tmp_size_idx_1;
  int32_T vectorUB;
  int8_T tmp_data[2];
  uint8_T b_varargout_4;
  boolean_T x[3];
  boolean_T distinctWptsIdx[2];
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T guard2;
  boolean_T guard3;
  boolean_T p;
  boolean_T p_0;
  guard1 = false;
  if (localDW->mode_prev != localDW->mode_start) {
    if ((*rtu_mode == 6) && (localB->FlightMode == FixedWing)) {
      UAM_exit_internal_GuidanceLogic(localDW);
      localDW->temporalCounter_i1 = 0U;
      localDW->is_GuidanceLogic = UAM_FlightMode_IN_PreTransition;
      localB->FlightMode = FixedWing;
      localB->controlMode_m.airspeedAltitude = 1U;
      localB->controlMode_m.attitude = 1U;
      localB->controlMode_m.lateralGuidance = 1U;
      localB->aacSP.L1 = 25.0;
      localB->aacSP.airspeed = 14.0;

      /* Chart: '<Root>/Guidance Mode Selector' */
      localB->aacSP.altitude = -rtu_States->Xe[2];
      localB->aacSP.course = rtu_States->course;
    } else if ((*rtu_mode == 2) && (localB->FlightMode == FixedWing)) {
      UAM_exit_internal_GuidanceLogic(localDW);
      localDW->is_GuidanceLogic = UAM_Flig_IN_FIXED_WING_WAYPOINT;
      localB->controlMode_m.airspeedAltitude = 1U;
      localB->controlMode_m.attitude = 1U;
      localB->controlMode_m.lateralGuidance = 1U;
      localDW->is_FIXED_WING_WAYPOINT = UAM_FlightMode_IN_WAYPOINT;

      /* MATLAB Function: '<S6>/MATLAB Function' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       */
      rtb_wps_k[0] = rtu_FromWP->position[0];
      rtb_wps_k[1] = rtu_ToWP->position[0];
      rtb_wps_k[2] = rtu_FromWP->position[1];
      rtb_wps_k[3] = rtu_ToWP->position[1];
      rtb_wps_k[4] = rtu_FromWP->position[2];
      rtb_wps_k[5] = rtu_ToWP->position[2];

      /* MATLABSystem: '<S6>/Waypoint Follower' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       *  MATLAB Function: '<S6>/MATLAB Function'
       */
      localDW->obj_f.LookaheadDistFlag = 0U;
      localDW->obj_f.InitialPose[0] = 0.0;
      localDW->obj_f.InitialPose[1] = 0.0;
      localDW->obj_f.InitialPose[2] = 0.0;
      localDW->obj_f.InitialPose[3] = 0.0;
      localDW->obj_f.NumWaypoints = 2.0;
      p = false;
      p_0 = true;
      b_k = 0;
      exitg1 = false;
      while ((!exitg1) && (b_k <= 5)) {
        i = ((b_k / 2) << 1) + b_k % 2;
        if (!(localDW->obj_f.WaypointsInternal[i] == rtb_wps_k[i])) {
          p_0 = false;
          exitg1 = true;
        } else {
          b_k++;
        }
      }

      if (p_0) {
        p = true;
      }

      if (!p) {
        for (i = 0; i < 6; i++) {
          localDW->obj_f.WaypointsInternal[i] = rtb_wps_k[i];
        }

        localDW->obj_f.WaypointIndex = 1.0;
      }

      distinctWptsIdx[1] = true;
      x[0] = (rtu_FromWP->position[0] != rtu_ToWP->position[0]);
      x[1] = (rtu_FromWP->position[1] != rtu_ToWP->position[1]);
      x[2] = (rtu_FromWP->position[2] != rtu_ToWP->position[2]);
      distinctWptsIdx[0] = false;
      b_k = 0;
      exitg1 = false;
      while ((!exitg1) && (b_k < 3)) {
        if (x[b_k]) {
          distinctWptsIdx[0] = true;
          exitg1 = true;
        } else {
          b_k++;
        }
      }

      b_k = 0;
      for (i = 0; i < 2; i++) {
        /* MATLABSystem: '<S6>/Waypoint Follower' */
        if (distinctWptsIdx[i]) {
          b_k++;
        }
      }

      tmp_size_idx_1 = b_k;
      b_k = 0;
      for (i = 0; i < 2; i++) {
        /* MATLABSystem: '<S6>/Waypoint Follower' */
        if (distinctWptsIdx[i]) {
          /* Start for MATLABSystem: '<S6>/Waypoint Follower' */
          tmp_data[b_k] = (int8_T)i;
          b_k++;
        }
      }

      /* MATLABSystem: '<S6>/Waypoint Follower' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       *  SignalConversion generated from: '<S6>/Waypoint Follower'
       */
      for (i = 0; i < 3; i++) {
        scalarLB = (tmp_size_idx_1 / 2) << 1;
        vectorUB = scalarLB - 2;
        for (b_k = 0; b_k <= vectorUB; b_k += 2) {
          b_waypointsIn_tmp = i << 1;
          b_waypointsIn_data[b_k + tmp_size_idx_1 * i] =
            rtb_wps_k[b_waypointsIn_tmp + tmp_data[b_k]];
          b_waypointsIn_data[(b_k + tmp_size_idx_1 * i) + 1] =
            rtb_wps_k[tmp_data[b_k + 1] + b_waypointsIn_tmp];
        }

        for (b_k = scalarLB; b_k < tmp_size_idx_1; b_k++) {
          b_waypointsIn_data[b_k + tmp_size_idx_1 * i] = rtb_wps_k[(i << 1) +
            tmp_data[b_k]];
        }
      }

      localDW->obj_f.LookaheadDistance = 30.0;
      if (tmp_size_idx_1 == 0) {
        turnInternal = rtu_States->course;
        localB->Status = 1U;
      } else {
        guard2 = false;
        if (tmp_size_idx_1 == 1) {
          if (localDW->obj_f.StartFlag) {
            localDW->obj_f.InitialPose[0] = rtu_States->Xe[0];
            localDW->obj_f.InitialPose[1] = rtu_States->Xe[1];
            localDW->obj_f.InitialPose[2] = rtu_States->Xe[2];
            localDW->obj_f.InitialPose[3] = rtu_States->course;
          }

          tmp_0 = _mm_sub_pd(_mm_loadu_pd(&b_waypointsIn_data[0]), _mm_loadu_pd(
            &rtu_States->Xe[0]));
          _mm_storeu_pd(&rtb_Product[0], tmp_0);
          rtb_Product[2] = b_waypointsIn_data[2] - rtu_States->Xe[2];
          if (UAM_FlightMode_norm_pv(rtb_Product) < 1.4901161193847656E-8) {
            turnInternal = rtu_States->course;
            localB->Status = 1U;
            localDW->obj_f.StartFlag = false;
          } else {
            localDW->obj_f.StartFlag = false;
            localDW->obj_f.NumWaypoints = 2.0;
            scalarLB = tmp_size_idx_1 + 1;
            for (i = 0; i < 3; i++) {
              vectorUB = (tmp_size_idx_1 + 1) * i;
              rtb_wps_k[vectorUB] = localDW->obj_f.InitialPose[i];
              for (b_k = 0; b_k < tmp_size_idx_1; b_k++) {
                rtb_wps_k[(b_k + vectorUB) + 1] =
                  b_waypointsIn_data[tmp_size_idx_1 * i + b_k];
              }
            }

            guard2 = true;
          }
        } else {
          scalarLB = tmp_size_idx_1;
          b_k = tmp_size_idx_1 * 3;
          if (b_k - 1 >= 0) {
            memcpy(&rtb_wps_k[0], &b_waypointsIn_data[0], (uint32_T)b_k * sizeof
                   (real_T));
          }

          guard2 = true;
        }

        if (guard2) {
          p = false;
          if (localDW->obj_f.WaypointIndex == 2.0) {
            p = true;
          }

          if (p) {
            localDW->obj_f.LastWaypointFlag = true;
            localDW->obj_f.WaypointIndex--;
          }

          rtb_Product[0] = rtb_wps_k[(int32_T)localDW->obj_f.WaypointIndex - 1];
          rtb_Sum[0] = rtb_wps_k[(int32_T)(localDW->obj_f.WaypointIndex + 1.0) -
            1];
          rtb_Product[1] = rtb_wps_k[((int32_T)localDW->obj_f.WaypointIndex +
            scalarLB) - 1];
          rtb_Sum[1] = rtb_wps_k[((int32_T)(localDW->obj_f.WaypointIndex + 1.0)
            + scalarLB) - 1];
          rtb_Product[2] = rtb_wps_k[((scalarLB << 1) + (int32_T)
            localDW->obj_f.WaypointIndex) - 1];
          rtb_DotProduct = rtb_wps_k[((int32_T)(localDW->obj_f.WaypointIndex +
            1.0) + (scalarLB << 1)) - 1];
          rtb_Sum[2] = rtb_DotProduct;
          tmp_1 = _mm_loadu_pd(&rtu_States->Xe[0]);
          _mm_storeu_pd(&turnVector_0[0], _mm_sub_pd(tmp_1, _mm_set_pd
            (rtb_wps_k[((int32_T)(localDW->obj_f.WaypointIndex + 1.0) + scalarLB)
             - 1], rtb_wps_k[(int32_T)(localDW->obj_f.WaypointIndex + 1.0) - 1])));
          turnVector_0[2] = rtu_States->Xe[2] - rtb_DotProduct;
          guard3 = false;
          if (UAM_FlightMode_norm_pv(turnVector_0) <= 10.0) {
            guard3 = true;
          } else {
            turnInternal = rtb_wps_k[(int32_T)(localDW->obj_f.WaypointIndex +
              1.0) - 1];
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(rtu_States->Xe[0],
              turnInternal), _mm_set_pd(turnInternal, rtb_wps_k[(int32_T)
              localDW->obj_f.WaypointIndex - 1])));
            unitVectorUtoV_tmp[0] = tmp[0];
            turnVector[0] = tmp[1];
            turnInternal = rtb_wps_k[((int32_T)(localDW->obj_f.WaypointIndex +
              1.0) + scalarLB) - 1];
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(rtu_States->Xe[1],
              turnInternal), _mm_set_pd(turnInternal, rtb_wps_k[((int32_T)
              localDW->obj_f.WaypointIndex + scalarLB) - 1])));
            unitVectorUtoV_tmp[1] = tmp[0];
            turnVector[1] = tmp[1];
            turnInternal = rtb_wps_k[((int32_T)(localDW->obj_f.WaypointIndex +
              1.0) + (scalarLB << 1)) - 1];
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(rtu_States->Xe[2],
              turnInternal), _mm_set_pd(turnInternal, rtb_wps_k[((int32_T)
              localDW->obj_f.WaypointIndex + (scalarLB << 1)) - 1])));
            unitVectorUtoV_tmp[2] = tmp[0];
            turnVector[2] = tmp[1];
            h = UAM_FlightMode_norm_pv(unitVectorUtoV_tmp);
            distToCenter = UAM_FlightMode_norm_pv(turnVector);
            turnInternal = (unitVectorUtoV_tmp[0] / h * (turnVector[0] /
              distToCenter) + unitVectorUtoV_tmp[1] / h * (turnVector[1] /
              distToCenter)) + tmp[0] / h * (tmp[1] / distToCenter);
            if (rtIsNaN(turnInternal) || (turnInternal < 0.0)) {
            } else {
              guard3 = true;
            }
          }

          if (guard3) {
            localDW->obj_f.WaypointIndex++;
            p = false;
            if (localDW->obj_f.WaypointIndex == 2.0) {
              p = true;
            }

            if (p) {
              localDW->obj_f.LastWaypointFlag = true;
              localDW->obj_f.WaypointIndex--;
            }

            rtb_Product[0] = rtb_wps_k[(int32_T)localDW->obj_f.WaypointIndex - 1];
            rtb_Sum[0] = rtb_wps_k[(int32_T)(localDW->obj_f.WaypointIndex + 1.0)
              - 1];
            rtb_Product[1] = rtb_wps_k[((int32_T)localDW->obj_f.WaypointIndex +
              scalarLB) - 1];
            rtb_Sum[1] = rtb_wps_k[((int32_T)(localDW->obj_f.WaypointIndex + 1.0)
              + scalarLB) - 1];
            rtb_Product[2] = rtb_wps_k[((scalarLB << 1) + (int32_T)
              localDW->obj_f.WaypointIndex) - 1];
            rtb_Sum[2] = rtb_wps_k[((int32_T)(localDW->obj_f.WaypointIndex + 1.0)
              + (scalarLB << 1)) - 1];
          }

          _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(rtb_Sum[0],
            rtu_States->Xe[0]), _mm_set1_pd(rtb_Product[0])));
          turnVector[0] = tmp[0];
          unitVectorUtoV_tmp[0] = tmp[1];
          _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(rtb_Sum[1],
            rtu_States->Xe[1]), _mm_set1_pd(rtb_Product[1])));
          turnVector[1] = tmp[0];
          unitVectorUtoV_tmp[1] = tmp[1];
          _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(rtb_Sum[2],
            rtu_States->Xe[2]), _mm_set1_pd(rtb_Product[2])));
          turnVector[2] = tmp[0];
          unitVectorUtoV_tmp[2] = tmp[1];
          turnInternal = unitVectorUtoV_tmp[0] * unitVectorUtoV_tmp[0] +
            unitVectorUtoV_tmp[1] * unitVectorUtoV_tmp[1];
          rtb_DotProduct = tmp[1] * tmp[1] + turnInternal;
          absxk = ((turnVector[0] * unitVectorUtoV_tmp[0] + turnVector[1] *
                    unitVectorUtoV_tmp[1]) + tmp[0] * tmp[1]) / rtb_DotProduct;
          if (absxk < 0.0) {
            absxk = UAM_FlightMode_norm_pv(turnVector);
          } else if (absxk > 1.0) {
            tmp_0 = _mm_sub_pd(tmp_1, _mm_loadu_pd(&rtb_Sum[0]));
            _mm_storeu_pd(&turnVector_0[0], tmp_0);
            turnVector_0[2] = rtu_States->Xe[2] - rtb_Sum[2];
            absxk = UAM_FlightMode_norm_pv(turnVector_0);
          } else {
            tmp_0 = _mm_sub_pd(tmp_1, _mm_add_pd(_mm_mul_pd(_mm_set1_pd(absxk),
              _mm_loadu_pd(&unitVectorUtoV_tmp[0])), _mm_loadu_pd(&rtb_Product[0])));
            _mm_storeu_pd(&turnVector_0[0], tmp_0);
            turnVector_0[2] = rtu_States->Xe[2] - (absxk * tmp[1] + rtb_Product
              [2]);
            absxk = UAM_FlightMode_norm_pv(turnVector_0);
          }

          if (localDW->obj_f.LastWaypointFlag) {
            absxk = (((rtu_States->Xe[0] - rtb_Product[0]) * unitVectorUtoV_tmp
                      [0] + (rtu_States->Xe[1] - rtb_Product[1]) *
                      unitVectorUtoV_tmp[1]) + (rtu_States->Xe[2] - rtb_Product
                      [2]) * tmp[1]) / rtb_DotProduct;
            tmp_0 = _mm_sub_pd(tmp_1, _mm_add_pd(_mm_mul_pd(_mm_set1_pd(absxk),
              _mm_loadu_pd(&unitVectorUtoV_tmp[0])), _mm_loadu_pd(&rtb_Product[0])));
            _mm_storeu_pd(&turnVector_0[0], tmp_0);
            turnVector_0[2] = rtu_States->Xe[2] - (absxk * tmp[1] + rtb_Product
              [2]);
            absxk = UAM_FlightMode_norm_pv(turnVector_0);
          }

          rtb_DotProduct = fabs(absxk);
          if (rtIsInf(rtb_DotProduct) || rtIsNaN(rtb_DotProduct)) {
            t = (rtNaN);
            b_tmp_0 = (rtNaN);
          } else if (rtb_DotProduct < 4.4501477170144028E-308) {
            t = 4.94065645841247E-324;
            b_tmp_0 = 4.94065645841247E-324;
          } else {
            frexp(rtb_DotProduct, &b_exponent);
            t = ldexp(1.0, b_exponent - 53);
            frexp(rtb_DotProduct, &b_exponent_0);
            b_tmp_0 = ldexp(1.0, b_exponent_0 - 53);
          }

          rtb_DotProduct = sqrt(t);
          t = 5.0 * b_tmp_0;
          if ((rtb_DotProduct >= t) || rtIsNaN(t)) {
            t = rtb_DotProduct;
          }

          if (absxk + t >= 30.0) {
            localDW->obj_f.LookaheadDistance = localDW->obj_f.LookaheadFactor *
              absxk;
          }

          turnVector[0] = unitVectorUtoV_tmp[0];
          turnVector[1] = unitVectorUtoV_tmp[1];
          tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtb_Product[0]), tmp_1);
          _mm_storeu_pd(&unitVectorUtoV_tmp[0], tmp_0);
          turnInternal += unitVectorUtoV_tmp[2] * unitVectorUtoV_tmp[2];
          unitVectorUtoV_tmp[2] = rtb_Product[2] - rtu_States->Xe[2];
          absxk = ((turnVector[0] * unitVectorUtoV_tmp[0] + turnVector[1] *
                    unitVectorUtoV_tmp[1]) + tmp[1] * unitVectorUtoV_tmp[2]) *
            2.0;
          t = sqrt(absxk * absxk - (((unitVectorUtoV_tmp[0] *
                      unitVectorUtoV_tmp[0] + unitVectorUtoV_tmp[1] *
                      unitVectorUtoV_tmp[1]) + unitVectorUtoV_tmp[2] *
                     unitVectorUtoV_tmp[2]) - localDW->obj_f.LookaheadDistance *
                    localDW->obj_f.LookaheadDistance) * (4.0 * turnInternal));
          rtb_DotProduct = (-absxk + t) / 2.0 / turnInternal;
          turnInternal = (-absxk - t) / 2.0 / turnInternal;
          if ((rtb_DotProduct >= turnInternal) || rtIsNaN(turnInternal)) {
            turnInternal = rtb_DotProduct;
          }

          tmp_0 = _mm_set1_pd(turnInternal);
          tmp_0 = _mm_add_pd(_mm_mul_pd(_mm_sub_pd(_mm_set1_pd(1.0), tmp_0),
            _mm_loadu_pd(&rtb_Product[0])), _mm_mul_pd(tmp_0, _mm_loadu_pd
            (&rtb_Sum[0])));
          _mm_storeu_pd(&rtb_Sum[0], tmp_0);
          turnInternal = rt_atan2d_snf(rtb_Sum[1] - rtu_States->Xe[1], rtb_Sum[0]
            - rtu_States->Xe[0]);
          localB->Status = 0U;
          p = false;
          if (localDW->obj_f.LastWaypointFlag) {
            p = true;
          }

          if (p) {
            localB->Status = 1U;
          }

          localDW->obj_f.LastWaypointFlag = false;
        }
      }

      /* BusCreator: '<S6>/Bus Creator' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       *  Constant: '<S6>/Constant'
       *  Constant: '<S6>/Constant1'
       *  Constant: '<S6>/Lookahead Distance'
       *  MATLAB Function: '<S6>/MATLAB Function'
       *  MATLABSystem: '<S6>/Waypoint Follower'
       * */
      localB->aacSP.airspeed = 15.0;
      localB->aacSP.altitude = -rtu_ToWP->position[2];
      localB->aacSP.course = turnInternal;
      localB->aacSP.L1 = 30.0;
      localB->aacSP.climbrate = 0.0;
    } else if (localB->FlightMode == Transition) {
      localB->controlMode_m.inTransition = 0U;
      UAM_exit_internal_GuidanceLogic(localDW);
      localDW->is_GuidanceLogic = UAM_FlightM_IN_FIXED_WING_ENTRY;
      enter_internal_FIXED_WING_ENTRY(localB, localDW);
    } else if (localB->FlightMode == BackTransition) {
      UAM_exit_internal_GuidanceLogic(localDW);
      localDW->is_GuidanceLogic = UAM_FlightMode_IN_HOVER_ENTRY;
      localB->FlightMode = Hover;
      localB->controlMode_m.inTransition = 0U;
    } else if ((*rtu_mode == 1) && (localB->FlightMode == Hover)) {
      localB->controlMode_m.inTransition = 0U;
      UAM_exit_internal_GuidanceLogic(localDW);
      localDW->is_GuidanceLogic = UAM_FlightMode_IN_Takeoff;

      /* SignalConversion generated from: '<S11>/Bus Creator' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       */
      localB->InnerLoopCmds.LAP[0] = rtu_Pose[0];
      localB->InnerLoopCmds.LAP[1] = rtu_Pose[1];
      localB->InnerLoopCmds.LAP[2] = rtu_ToWP->position[2];

      /* Saturate: '<S11>/Hdg. Cmd Sat' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       */
      if (rtu_ToWP->params[3] > 3.1415926535897931) {
        /* BusCreator: '<S11>/Bus Creator' incorporates:
         *  Merge: '<S3>/ Merge 2'
         */
        localB->InnerLoopCmds.HeadingCmd = 3.1415926535897931;
      } else if (rtu_ToWP->params[3] < -3.1415926535897931) {
        /* BusCreator: '<S11>/Bus Creator' incorporates:
         *  Merge: '<S3>/ Merge 2'
         */
        localB->InnerLoopCmds.HeadingCmd = -3.1415926535897931;
      } else {
        /* BusCreator: '<S11>/Bus Creator' incorporates:
         *  Merge: '<S3>/ Merge 2'
         */
        localB->InnerLoopCmds.HeadingCmd = rtu_ToWP->params[3];
      }

      /* BusCreator: '<S11>/Bus Creator' incorporates:
       *  Merge: '<S3>/ Merge 2'
       */
      localB->InnerLoopCmds.YawCmd = UAM_FlightMode_ConstB.YawCmd;

      /* Merge: '<S3>/ Merge ' incorporates:
       *  Constant: '<S11>/Constant'
       *  SignalConversion generated from: '<S11>/Status'
       */
      localB->Status = 0U;
    } else if ((*rtu_mode == 3) && (localB->FlightMode == FixedWing)) {
      UAM_exit_internal_GuidanceLogic(localDW);
      localDW->is_GuidanceLogic = UAM_FlightM_IN_FIXED_WING_ORBIT;
      localB->controlMode_m.airspeedAltitude = 1U;
      localB->controlMode_m.attitude = 1U;
      localB->controlMode_m.lateralGuidance = 1U;
      localDW->is_FIXED_WING_ORBIT = UAM_FlightMode_IN_ORBIT;

      /* MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
       */
      rtb_DotProduct = rtu_ToWP->params[0];
      localDW->obj_b.OrbitRadiusFlag = 0U;
      if (rtu_ToWP->params[0] <= 50.0) {
        rtb_DotProduct = 50.0;
        localDW->obj_b.OrbitRadiusFlag = 1U;
      }

      localDW->obj_b.LookaheadDistFlag = 0U;
      _mm_storeu_pd(&xyLookaheadPoint[0], _mm_sub_pd(_mm_loadu_pd
        (&rtu_States->Xe[0]), _mm_loadu_pd(&rtu_ToWP->position[0])));
      if (UAM_FlightMode_norm_p(xyLookaheadPoint) < 2.47032822920623E-323) {
        turnInternal = rtu_States->course;
        distToCenter = localDW->obj_b.NumCircles;
      } else {
        p = false;
        p_0 = true;
        b_k = 0;
        exitg1 = false;
        while ((!exitg1) && (b_k < 3)) {
          if (!(localDW->obj_b.OrbitCenterInternal[b_k] == rtu_ToWP->
                position[b_k])) {
            p_0 = false;
            exitg1 = true;
          } else {
            b_k++;
          }
        }

        if (p_0) {
          p = true;
        }

        guard2 = false;
        if (!p) {
          guard2 = true;
        } else {
          p = false;
          if (localDW->obj_b.OrbitRadiusInternal == rtb_DotProduct) {
            p = true;
          }

          if (!p) {
            guard2 = true;
          }
        }

        if (guard2) {
          localDW->obj_b.NumCircles = 0.0;
          localDW->obj_b.OrbitCenterInternal[0] = rtu_ToWP->position[0];
          localDW->obj_b.OrbitCenterInternal[1] = rtu_ToWP->position[1];
          localDW->obj_b.OrbitCenterInternal[2] = rtu_ToWP->position[2];
          localDW->obj_b.OrbitRadiusInternal = rtb_DotProduct;
          localDW->obj_b.SelectTurnDirectionFlag = true;
        }

        if (rtb_DotProduct <= 30.0) {
          localDW->obj_b.LookaheadDistance = 0.9 * rtb_DotProduct;
        } else {
          localDW->obj_b.LookaheadDistance = 30.0;
        }

        b_tmp_0 = rtu_States->Xe[0] - rtu_ToWP->position[0];
        b_tmp[0] = b_tmp_0;
        t = b_tmp_0 * b_tmp_0;
        b_tmp_0 = rtu_States->Xe[1] - rtu_ToWP->position[1];
        b_tmp[1] = b_tmp_0;
        distToCenter = sqrt(b_tmp_0 * b_tmp_0 + t);
        t = fabs(rtb_DotProduct + localDW->obj_b.LookaheadDistance);
        if (rtIsInf(t) || rtIsNaN(t)) {
          t = (rtNaN);
        } else if (t < 4.4501477170144028E-308) {
          t = 4.94065645841247E-324;
        } else {
          frexp(t, &b_exponent);
          t = ldexp(1.0, b_exponent - 53);
        }

        guard2 = false;
        if (distToCenter >= (rtb_DotProduct + localDW->obj_b.LookaheadDistance)
            - 5.0 * t) {
          guard2 = true;
        } else {
          t = fabs(rtb_DotProduct + localDW->obj_b.LookaheadDistance);
          if (rtIsInf(t) || rtIsNaN(t)) {
            t = (rtNaN);
          } else if (t < 4.4501477170144028E-308) {
            t = 4.94065645841247E-324;
          } else {
            frexp(t, &b_exponent_0);
            t = ldexp(1.0, b_exponent_0 - 53);
          }

          if (distToCenter <= (rtb_DotProduct - localDW->obj_b.LookaheadDistance)
              + 5.0 * t) {
            guard2 = true;
          } else {
            if (localDW->obj_b.StartFlag) {
              localDW->obj_b.PrevPosition[0] = rtu_States->Xe[0];
              localDW->obj_b.PrevPosition[1] = rtu_States->Xe[1];
              localDW->obj_b.PrevPosition[2] = rtu_States->Xe[2];
              localDW->obj_b.StartFlag = false;
            }

            if ((rtu_ToWP->params[1] == 0.0) &&
                (!localDW->obj_b.SelectTurnDirectionFlag)) {
              turnInternal = localDW->obj_b.TurnDirectionInternal;
            } else {
              turnInternal = rtu_ToWP->params[1];
            }

            rtb_Product[2] = 0.0;
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
              (localDW->obj_b.PrevPosition[0], rtu_States->Xe[0]), _mm_set1_pd
              (rtu_ToWP->position[0])));
            xyPose[0] = tmp[0];
            rtb_Product[0] = tmp[1];
            rtb_Sum[0] = xyLookaheadPoint[0];
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
              (localDW->obj_b.PrevPosition[1], rtu_States->Xe[1]), _mm_set1_pd
              (rtu_ToWP->position[1])));
            xyPose[1] = tmp[0];
            rtb_Product[1] = tmp[1];
            rtb_Sum[1] = xyLookaheadPoint[1];
            distToCenter = UAM_FlightMode_norm_p(xyPose);
            h = localDW->obj_b.LookaheadDistance *
              localDW->obj_b.LookaheadDistance;
            t = ((h - rtb_DotProduct * rtb_DotProduct) + distToCenter *
                 distToCenter) / (2.0 * distToCenter);
            tmp_0 = _mm_set1_pd(distToCenter);
            tmp_1 = _mm_sub_pd(_mm_loadu_pd(&rtu_ToWP->position[0]),
                               _mm_loadu_pd(&rtu_States->Xe[0]));
            _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(tmp_1,
              _mm_set1_pd(t)), tmp_0), _mm_loadu_pd(&rtu_States->Xe[0])));
            y2 = tmp[1];
            h = sqrt(h - t * t);
            b_tmp[0] = tmp[0] - (rtu_ToWP->position[1] - rtu_States->Xe[1]) * h /
              distToCenter;
            _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd
              (_mm_set_pd(rtu_ToWP->position[0], rtu_ToWP->position[1]),
               _mm_set_pd(rtu_States->Xe[0], rtu_States->Xe[1])), _mm_set1_pd(h)),
              tmp_0), _mm_set_pd(tmp[1], tmp[0])));
            b_tmp[1] = tmp[0];
            t = tmp[1];
            y2 -= (rtu_ToWP->position[0] - rtu_States->Xe[0]) * h / distToCenter;
            rtb_Sum[2] = 0.0;
            if (turnInternal < 0.0) {
              rtb_Product[0] = xyLookaheadPoint[0];
              rtb_Product[1] = xyLookaheadPoint[1];
              rtb_Product[2] = 0.0;
              tmp_0 = _mm_sub_pd(_mm_loadu_pd(&localDW->obj_b.PrevPosition[0]),
                                 _mm_loadu_pd(&rtu_ToWP->position[0]));
              _mm_storeu_pd(&rtb_Sum[0], tmp_0);
              rtb_Sum[2] = 0.0;
            }

            h = UAM_FlightMode_norm_pv(rtb_Product);
            distToCenter = UAM_FlightMode_norm_pv(rtb_Sum);
            tmp_0 = _mm_set_pd(distToCenter, h);
            _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(rtb_Sum[0],
              rtb_Product[0]), tmp_0));
            rtb_Product[0] = tmp[0];
            rtb_Sum[0] = tmp[1];
            _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(rtb_Sum[1],
              rtb_Product[1]), tmp_0));
            rtb_Sum[1] = tmp[1];
            turnVector[2] = rtb_Product[0] * tmp[1] - rtb_Sum[0] * tmp[0];
            localDW->obj_b.PrevPosition[0] = rtu_States->Xe[0];
            localDW->obj_b.PrevPosition[1] = rtu_States->Xe[1];
            localDW->obj_b.PrevPosition[2] = rtu_States->Xe[2];
            localDW->obj_b.NumCircles += rt_atan2d_snf(turnVector[2],
              (rtb_Product[0] * rtb_Sum[0] + tmp[0] * tmp[1]) + 0.0 / h * (0.0 /
              distToCenter)) / 2.0 / 3.1415926535897931;
            distToCenter = localDW->obj_b.NumCircles;
            _mm_storeu_pd(&rtb_Sum[0], tmp_1);
            if (rtIsNaN(turnInternal)) {
              h = (rtNaN);
            } else if (turnInternal < 0.0) {
              h = -1.0;
            } else {
              h = (turnInternal > 0.0);
            }

            switch ((int32_T)h) {
             case 1:
              if ((b_tmp[0] - rtu_States->Xe[0]) * rtb_Sum[1] - (t -
                   rtu_States->Xe[1]) * rtb_Sum[0] > 0.0) {
                turnInternal = b_tmp[0];
                y2 = t;
              } else {
                turnInternal = b_tmp[1];
              }
              break;

             case -1:
              if ((b_tmp[0] - rtu_States->Xe[0]) * rtb_Sum[1] - (t -
                   rtu_States->Xe[1]) * rtb_Sum[0] < 0.0) {
                turnInternal = b_tmp[0];
                y2 = t;
              } else {
                turnInternal = b_tmp[1];
              }
              break;

             default:
              if (fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(t - rtu_States->Xe[1],
                     b_tmp[0] - rtu_States->Xe[0]), rtu_States->course)) < fabs
                  (UAM_FlightMode_angdiff(rt_atan2d_snf(y2 - rtu_States->Xe[1],
                     b_tmp[1] - rtu_States->Xe[0]), rtu_States->course))) {
                turnInternal = b_tmp[0];
                y2 = t;
              } else {
                turnInternal = b_tmp[1];
              }

              if ((turnInternal - rtu_States->Xe[0]) * rtb_Sum[1] - (y2 -
                   rtu_States->Xe[1]) * rtb_Sum[0] > 0.0) {
                localDW->obj_b.TurnDirectionInternal = 1.0;
              } else {
                localDW->obj_b.TurnDirectionInternal = -1.0;
              }

              localDW->obj_b.SelectTurnDirectionFlag = false;
              break;
            }
          }
        }

        if (guard2) {
          _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_mul_pd(_mm_div_pd(_mm_set_pd
            (b_tmp_0, b_tmp[0]), _mm_set1_pd(UAM_FlightMode_norm_p(b_tmp))),
            _mm_set1_pd(rtb_DotProduct)), _mm_loadu_pd(&rtu_ToWP->position[0])));
          turnInternal = tmp[0];
          y2 = tmp[1];
          distToCenter = localDW->obj_b.NumCircles;
        }

        turnInternal = rt_atan2d_snf(y2 - rtu_States->Xe[1], turnInternal -
          rtu_States->Xe[0]);
      }

      /* BusCreator: '<S5>/Bus Creator' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       *  Constant: '<S5>/Constant'
       *  Constant: '<S5>/Constant1'
       *  Constant: '<S5>/Lookahead Distance'
       *  MATLABSystem: '<S5>/UAV Orbit Follower'
       *  UnaryMinus: '<S5>/Unary Minus'
       * */
      localB->aacSP.airspeed = 15.0;
      localB->aacSP.altitude = -rtu_ToWP->position[2];
      localB->aacSP.course = turnInternal;
      localB->aacSP.L1 = 30.0;
      localB->aacSP.climbrate = 0.0;

      /* Merge: '<S3>/ Merge ' incorporates:
       *  Abs: '<S5>/Abs1'
       *  Chart: '<Root>/Guidance Mode Selector'
       *  DataTypeConversion: '<S5>/Data Type Conversion'
       *  MATLABSystem: '<S5>/UAV Orbit Follower'
       *  RelationalOperator: '<S5>/Relational Operator'
       * */
      localB->Status = (uint8_T)(fabs(distToCenter) > rtu_ToWP->params[2]);
    } else if (localB->FlightMode == Hover) {
      localB->controlMode_m.inTransition = 0U;

      /* Chart: '<Root>/Guidance Mode Selector' incorporates:
       *  Constant: '<S9>/Lookahead Distance'
       *  MATLABSystem: '<S10>/UAV Orbit Follower'
       *  Sum: '<S18>/Sum'
       * */
      switch (*rtu_mode) {
       case 2U:
        UAM_exit_internal_GuidanceLogic(localDW);
        localDW->is_GuidanceLogic = UAM_FlightMode_IN_WP;

        /* Switch: '<S17>/Switch' incorporates:
         *  Product: '<S21>/Product'
         *  RelationalOperator: '<S17>/Equal'
         */
        if (rtu_FromWP->mode == 6) {
          rtb_Product[0] = rtu_FromWP->position[0];
          rtb_Product[1] = rtu_FromWP->position[1];
          rtb_Product[2] = rtu_FromWP->position[2];
        } else {
          rtb_Product[0] = 0.0;
          rtb_Product[1] = 0.0;
          rtb_Product[2] = 0.0;
        }

        /* MATLAB Function: '<S12>/MATLAB Function' incorporates:
         *  BusCreator generated from: '<S12>/MATLAB Function'
         *  Product: '<S21>/Product'
         */
        rtb_wps[0] = rtb_Product[0];
        rtb_wps[2] = rtb_Product[1];
        rtb_wps[4] = rtb_Product[2];
        rtb_wps[6] = rtu_FromWP->params[3];
        rtb_wps[1] = rtu_ToWP->position[0];
        rtb_wps[3] = rtu_ToWP->position[1];
        rtb_wps[5] = rtu_ToWP->position[2];
        rtb_wps[7] = rtu_ToWP->params[3];

        /* MATLABSystem: '<S12>/Waypoint Follower' incorporates:
         *  Constant: '<S12>/Lookahead Distance'
         */
        UAM_FlightMode_SystemCore_step(&localDW->obj, rtu_Pose, rtb_wps, 5.0,
          localB->InnerLoopCmds.LAP, &turnInternal, &rtb_DotProduct,
          &b_varargout_4);

        /* BusCreator: '<S12>/Bus Creator' incorporates:
         *  Constant: '<S12>/Constant'
         *  MATLABSystem: '<S12>/Waypoint Follower'
         *  Merge: '<S3>/ Merge 2'
         * */
        localB->InnerLoopCmds.HeadingCmd = turnInternal;
        localB->InnerLoopCmds.YawCmd = 0.0;

        /* Sum: '<S18>/Sum1' incorporates:
         *  Product: '<S21>/Product'
         */
        turnInternal = rtu_ToWP->position[0] - rtb_Product[0];
        rtb_Product[0] = turnInternal;

        /* DotProduct: '<S21>/Dot Product1' incorporates:
         *  Product: '<S21>/Product'
         */
        rtb_DotProduct = turnInternal * turnInternal;

        /* Sum: '<S18>/Sum1' incorporates:
         *  Product: '<S21>/Product'
         */
        turnInternal = rtu_ToWP->position[1] - rtb_Product[1];
        rtb_Product[1] = turnInternal;

        /* DotProduct: '<S21>/Dot Product1' incorporates:
         *  Product: '<S21>/Product'
         */
        rtb_DotProduct += turnInternal * turnInternal;

        /* Sum: '<S18>/Sum1' incorporates:
         *  Product: '<S21>/Product'
         */
        turnInternal = rtu_ToWP->position[2] - rtb_Product[2];

        /* DotProduct: '<S21>/Dot Product1' incorporates:
         *  Product: '<S21>/Product'
         */
        rtb_DotProduct += turnInternal * turnInternal;

        /* Saturate: '<S21>/Saturation' incorporates:
         *  DotProduct: '<S21>/Dot Product1'
         */
        if (rtb_DotProduct <= 1.0E-5) {
          rtb_DotProduct = 1.0E-5;
        }

        /* Sqrt: '<S21>/Sqrt' incorporates:
         *  Saturate: '<S21>/Saturation'
         */
        rtb_DotProduct = sqrt(rtb_DotProduct);
        tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtu_ToWP->position[0]), _mm_loadu_pd
                           (&rtu_Pose[0]));
        _mm_storeu_pd(&rtb_Sum[0], tmp_0);

        /* Sum: '<S18>/Sum' */
        rtb_Sum[2] = rtu_ToWP->position[2] - rtu_Pose[2];

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S12>/Lookahead Distance'
         *  DataTypeConversion: '<S12>/Data Type Conversion'
         *  DotProduct: '<S18>/Dot Product'
         *  Product: '<S21>/Product'
         *  RelationalOperator: '<S12>/Relational Operator'
         *  Sum: '<S18>/Sum'
         *  Sum: '<S18>/Sum1'
         */
        localB->Status = (uint8_T)((rtb_Product[0] / rtb_DotProduct * rtb_Sum[0]
          + rtb_Product[1] / rtb_DotProduct * rtb_Sum[1]) + turnInternal /
          rtb_DotProduct * rtb_Sum[2] <= 5.0);
        break;

       case 3U:
        UAM_exit_internal_GuidanceLogic(localDW);
        localDW->is_GuidanceLogic = UAM_FlightMode_IN_Orbit;

        /* Abs: '<S10>/Abs' */
        rtb_Abs = fabs(rtu_ToWP->params[0]);

        /* Signum: '<S10>/Sign' */
        if (rtIsNaN(rtu_ToWP->params[1])) {
          absxk = (rtNaN);
        } else if (rtu_ToWP->params[1] < 0.0) {
          absxk = -1.0;
        } else {
          absxk = (rtu_ToWP->params[1] > 0.0);
        }

        /* MATLABSystem: '<S10>/UAV Orbit Follower' */
        localDW->obj_l.OrbitRadiusFlag = 0U;
        if (rtb_Abs <= 1.0) {
          rtb_Abs = 1.0;
          localDW->obj_l.OrbitRadiusFlag = 1U;
        }

        localDW->obj_l.LookaheadDistFlag = 0U;
        tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtu_Pose[0]), _mm_loadu_pd
                           (&rtu_ToWP->position[0]));
        _mm_storeu_pd(&rtu_Pose_0[0], tmp_0);

        /* MATLABSystem: '<S10>/UAV Orbit Follower' */
        if (UAM_FlightMode_norm_p(rtu_Pose_0) < 2.47032822920623E-323) {
          tmp_0 = _mm_add_pd(_mm_mul_pd(_mm_set1_pd(rtb_Abs), _mm_set_pd(sin
            (rtu_Pose[3]), cos(rtu_Pose[3]))), _mm_loadu_pd(&rtu_Pose[0]));
          _mm_storeu_pd(&rtb_Product[0], tmp_0);
          rtb_Product[2] = rtu_ToWP->position[2];
          turnInternal = rtu_Pose[3];
          absxk = rtu_Pose[3];
          distToCenter = localDW->obj_l.NumCircles;
        } else {
          p = false;
          p_0 = true;
          b_k = 0;
          exitg1 = false;
          while ((!exitg1) && (b_k < 3)) {
            if (!(localDW->obj_l.OrbitCenterInternal[b_k] == rtu_ToWP->
                  position[b_k])) {
              p_0 = false;
              exitg1 = true;
            } else {
              b_k++;
            }
          }

          if (p_0) {
            p = true;
          }

          guard2 = false;
          if (!p) {
            guard2 = true;
          } else {
            p = false;
            if (localDW->obj_l.OrbitRadiusInternal == rtb_Abs) {
              p = true;
            }

            if (!p) {
              guard2 = true;
            }
          }

          if (guard2) {
            localDW->obj_l.NumCircles = 0.0;
            localDW->obj_l.OrbitCenterInternal[0] = rtu_ToWP->position[0];
            localDW->obj_l.OrbitCenterInternal[1] = rtu_ToWP->position[1];
            localDW->obj_l.OrbitCenterInternal[2] = rtu_ToWP->position[2];
            localDW->obj_l.OrbitRadiusInternal = rtb_Abs;
            localDW->obj_l.SelectTurnDirectionFlag = true;
          }

          if (rtb_Abs <= 5.0) {
            localDW->obj_l.LookaheadDistance = 0.9 * rtb_Abs;
          } else {
            localDW->obj_l.LookaheadDistance = 5.0;
          }

          turnInternal = rtu_Pose[0];
          rtb_DotProduct = rtu_Pose[1];
          b_tmp_0 = turnInternal - rtu_ToWP->position[0];
          b_tmp[0] = b_tmp_0;
          t = b_tmp_0 * b_tmp_0;
          b_tmp_0 = rtb_DotProduct - rtu_ToWP->position[1];
          b_tmp[1] = b_tmp_0;
          distToCenter = sqrt(b_tmp_0 * b_tmp_0 + t);
          t = rtb_Abs + localDW->obj_l.LookaheadDistance;
          if (rtIsInf(t) || rtIsNaN(t)) {
            t = (rtNaN);
          } else if (t < 4.4501477170144028E-308) {
            t = 4.94065645841247E-324;
          } else {
            frexp(t, &b_exponent);
            t = ldexp(1.0, b_exponent - 53);
          }

          guard2 = false;
          if (distToCenter >= (rtb_Abs + localDW->obj_l.LookaheadDistance) - 5.0
              * t) {
            guard2 = true;
          } else {
            t = rtb_Abs + localDW->obj_l.LookaheadDistance;
            if (rtIsInf(t) || rtIsNaN(t)) {
              t = (rtNaN);
            } else if (t < 4.4501477170144028E-308) {
              t = 4.94065645841247E-324;
            } else {
              frexp(t, &b_exponent_0);
              t = ldexp(1.0, b_exponent_0 - 53);
            }

            if (distToCenter <= (rtb_Abs - localDW->obj_l.LookaheadDistance) +
                5.0 * t) {
              guard2 = true;
            } else {
              if (localDW->obj_l.StartFlag) {
                localDW->obj_l.PrevPosition[0] = rtu_Pose[0];
                localDW->obj_l.PrevPosition[1] = rtu_Pose[1];
                localDW->obj_l.PrevPosition[2] = rtu_Pose[2];
                localDW->obj_l.StartFlag = false;
              }

              if ((absxk == 0.0) && (!localDW->obj_l.SelectTurnDirectionFlag)) {
                absxk = localDW->obj_l.TurnDirectionInternal;
              }

              _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
                (localDW->obj_l.PrevPosition[0], turnInternal), _mm_set1_pd
                (rtu_ToWP->position[0])));
              xyPose[0] = tmp[0];
              rtb_Product[0] = tmp[1];
              _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
                (localDW->obj_l.PrevPosition[1], rtb_DotProduct), _mm_set1_pd
                (rtu_ToWP->position[1])));
              xyPose[1] = tmp[0];
              rtb_Product[1] = tmp[1];
              distToCenter = UAM_FlightMode_norm_p(xyPose);
              h = localDW->obj_l.LookaheadDistance *
                localDW->obj_l.LookaheadDistance;
              t = ((h - rtb_Abs * rtb_Abs) + distToCenter * distToCenter) / (2.0
                * distToCenter);
              tmp_0 = _mm_set_pd(rtb_DotProduct, turnInternal);
              tmp_1 = _mm_set1_pd(distToCenter);
              tmp_2 = _mm_loadu_pd(&rtu_ToWP->position[0]);
              _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd
                (tmp_2, tmp_0), _mm_set1_pd(t)), tmp_1), tmp_0));
              y2 = tmp[1];
              h = sqrt(h - t * t);
              b_tmp_0 = rtu_ToWP->position[1] - rtb_DotProduct;
              b_tmp[0] = tmp[0] - b_tmp_0 * h / distToCenter;
              _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd
                (_mm_set_pd(rtu_ToWP->position[0], rtu_ToWP->position[1]),
                 _mm_set_pd(turnInternal, rtb_DotProduct)), _mm_set1_pd(h)),
                tmp_1), _mm_set_pd(tmp[1], tmp[0])));
              b_tmp[1] = tmp[0];
              t = tmp[1];
              rtb_Abs = rtu_ToWP->position[0] - turnInternal;
              y2 -= rtb_Abs * h / distToCenter;
              rtb_Product[2] = 0.0;
              tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtu_Pose[0]), _mm_loadu_pd
                                 (&rtu_ToWP->position[0]));
              _mm_storeu_pd(&rtb_Sum[0], tmp_0);
              rtb_Sum[2] = 0.0;
              if (absxk < 0.0) {
                rtb_Product[0] = rtb_Sum[0];
                rtb_Product[1] = rtb_Sum[1];
                rtb_Product[2] = 0.0;
                tmp_0 = _mm_sub_pd(_mm_loadu_pd(&localDW->obj_l.PrevPosition[0]),
                                   tmp_2);
                _mm_storeu_pd(&rtb_Sum[0], tmp_0);
                rtb_Sum[2] = 0.0;
              }

              h = UAM_FlightMode_norm_pv(rtb_Product);
              distToCenter = UAM_FlightMode_norm_pv(rtb_Sum);
              tmp_0 = _mm_set_pd(distToCenter, h);
              _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(rtb_Sum[0],
                rtb_Product[0]), tmp_0));
              rtb_Product[0] = tmp[0];
              rtb_Sum[0] = tmp[1];
              _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(rtb_Sum[1],
                rtb_Product[1]), tmp_0));
              rtb_Sum[1] = tmp[1];
              turnVector[2] = rtb_Product[0] * tmp[1] - rtb_Sum[0] * tmp[0];
              localDW->obj_l.NumCircles += rt_atan2d_snf(turnVector[2],
                (rtb_Product[0] * rtb_Sum[0] + tmp[0] * tmp[1]) + 0.0 / h * (0.0
                / distToCenter)) / 2.0 / 3.1415926535897931;
              distToCenter = localDW->obj_l.NumCircles;
              localDW->obj_l.PrevPosition[0] = rtu_Pose[0];
              localDW->obj_l.PrevPosition[1] = rtu_Pose[1];
              localDW->obj_l.PrevPosition[2] = rtu_Pose[2];
              tmp_0 = _mm_sub_pd(_mm_set_pd(t, b_tmp[0]), _mm_loadu_pd
                                 (&rtu_Pose[0]));
              _mm_storeu_pd(&rtb_Product[0], tmp_0);
              tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtu_ToWP->position[0]),
                                 _mm_loadu_pd(&rtu_Pose[0]));
              _mm_storeu_pd(&rtb_Sum[0], tmp_0);
              if (rtIsNaN(absxk)) {
                h = (rtNaN);
              } else if (absxk < 0.0) {
                h = -1.0;
              } else {
                h = (absxk > 0.0);
              }

              switch ((int32_T)h) {
               case 1:
                if (rtb_Product[0] * rtb_Sum[1] - rtb_Sum[0] * rtb_Product[1] >
                    0.0) {
                  xyLookaheadPoint[0] = b_tmp[0];
                  xyLookaheadPoint[1] = t;
                } else {
                  xyLookaheadPoint[0] = b_tmp[1];
                  xyLookaheadPoint[1] = y2;
                }
                break;

               case -1:
                if (rtb_Product[0] * rtb_Sum[1] - rtb_Sum[0] * rtb_Product[1] <
                    0.0) {
                  xyLookaheadPoint[0] = b_tmp[0];
                  xyLookaheadPoint[1] = t;
                } else {
                  xyLookaheadPoint[0] = b_tmp[1];
                  xyLookaheadPoint[1] = y2;
                }
                break;

               default:
                p = (fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(t - rtu_Pose[1],
                        b_tmp[0] - rtu_Pose[0]), rtu_Pose[3])) < fabs
                     (UAM_FlightMode_angdiff(rt_atan2d_snf(y2 - rtu_Pose[1],
                        b_tmp[1] - rtu_Pose[0]), rtu_Pose[3])));
                if (p) {
                  xyLookaheadPoint[0] = b_tmp[0];
                  xyLookaheadPoint[1] = t;
                } else {
                  xyLookaheadPoint[0] = b_tmp[1];
                  xyLookaheadPoint[1] = y2;
                }

                tmp_0 = _mm_sub_pd(_mm_loadu_pd(&xyLookaheadPoint[0]),
                                   _mm_loadu_pd(&rtu_Pose[0]));
                _mm_storeu_pd(&rtb_Product[0], tmp_0);
                if (rtb_Product[0] * rtb_Sum[1] - rtb_Sum[0] * rtb_Product[1] >
                    0.0) {
                  localDW->obj_l.TurnDirectionInternal = 1.0;
                } else {
                  localDW->obj_l.TurnDirectionInternal = -1.0;
                }

                localDW->obj_l.SelectTurnDirectionFlag = false;
                break;
              }

              absxk = rt_atan2d_snf(b_tmp_0, rtb_Abs);
            }
          }

          if (guard2) {
            _mm_storeu_pd(&xyLookaheadPoint[0], _mm_add_pd(_mm_mul_pd(_mm_div_pd
              (_mm_set_pd(b_tmp_0, b_tmp[0]), _mm_set1_pd(UAM_FlightMode_norm_p
              (b_tmp))), _mm_set1_pd(rtb_Abs)), _mm_loadu_pd(&rtu_ToWP->
              position[0])));
            absxk = rt_atan2d_snf(xyLookaheadPoint[1] - rtb_DotProduct,
                                  xyLookaheadPoint[0] - turnInternal);
            distToCenter = localDW->obj_l.NumCircles;
          }

          rtb_Product[0] = xyLookaheadPoint[0];
          rtb_Product[1] = xyLookaheadPoint[1];
          rtb_Product[2] = rtu_ToWP->position[2];
          turnInternal = rt_atan2d_snf(xyLookaheadPoint[1] - rtb_DotProduct,
            xyLookaheadPoint[0] - turnInternal);
        }

        /* BusCreator: '<S10>/Bus Creator' incorporates:
         *  MATLABSystem: '<S10>/UAV Orbit Follower'
         *  Merge: '<S3>/ Merge 2'
         * */
        localB->InnerLoopCmds.LAP[0] = rtb_Product[0];
        localB->InnerLoopCmds.LAP[1] = rtb_Product[1];
        localB->InnerLoopCmds.LAP[2] = rtb_Product[2];
        localB->InnerLoopCmds.HeadingCmd = turnInternal;
        localB->InnerLoopCmds.YawCmd = absxk;

        /* Merge: '<S3>/ Merge ' incorporates:
         *  DataTypeConversion: '<S10>/Data Type Conversion'
         *  MATLABSystem: '<S10>/UAV Orbit Follower'
         *  RelationalOperator: '<S10>/Relational Operator'
         * */
        localB->Status = (uint8_T)(distToCenter > rtu_ToWP->params[2]);
        break;

       case 6U:
        localB->FlightMode = Transition;
        localB->controlMode_m.inTransition = 1U;
        localB->controlMode_m.TransitionCondition = 0U;
        UAM_exit_internal_GuidanceLogic(localDW);
        localDW->is_GuidanceLogic = UAM_Flight_IN_ForwardTransition;

        /* BusCreator: '<S7>/Bus Creator1' incorporates:
         *  Constant: '<S7>/Constant1'
         *  Constant: '<S7>/Constant2'
         *  Constant: '<S7>/Constant3'
         *  Merge: '<S3>/ Merge 2'
         */
        localB->InnerLoopCmds.LAP[0] = 0.0;
        localB->InnerLoopCmds.LAP[1] = 0.0;
        localB->InnerLoopCmds.LAP[2] = 0.0;
        localB->InnerLoopCmds.HeadingCmd = 0.0;
        localB->InnerLoopCmds.YawCmd = 0.0;

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S7>/Constant'
         *  DataTypeConversion: '<S7>/Data Type Conversion'
         */
        localB->Status = 0U;
        break;

       default:
        UAM_exit_internal_GuidanceLogic(localDW);
        localDW->is_GuidanceLogic = UAM_FlightMode_IN_Land;
        localDW->is_Land = UAM_FlightMode_IN_ToLand;

        /* MATLAB Function: '<S9>/MATLAB Function' */
        rtb_wps_l[0] = rtu_FromWP->position[0];
        rtb_wps_l[2] = rtu_FromWP->position[1];
        rtb_wps_l[4] = rtu_FromWP->position[2];
        rtb_wps_l[6] = rtu_FromWP->params[3];
        rtb_wps_l[1] = rtu_ToWP->position[0];
        rtb_wps_l[3] = rtu_ToWP->position[1];
        rtb_wps_l[5] = rtu_ToWP->position[2];
        rtb_wps_l[7] = rtu_ToWP->params[3];
        UAM_FlightMode_WaypointFollower(rtu_Pose, rtb_wps_l, 3.0,
          &localB->WaypointFollower_c2, &localDW->WaypointFollower_c2);

        /* BusCreator: '<S9>/Bus Creator1' incorporates:
         *  Constant: '<S9>/Lookahead Distance'
         *  MATLABSystem: '<S9>/Waypoint Follower'
         *  Merge: '<S3>/ Merge 2'
         */
        localB->InnerLoopCmds.LAP[0] = localB->WaypointFollower_c2.LAP[0];
        localB->InnerLoopCmds.LAP[1] = localB->WaypointFollower_c2.LAP[1];
        localB->InnerLoopCmds.LAP[2] = localB->WaypointFollower_c2.LAP[2];
        localB->InnerLoopCmds.HeadingCmd =
          localB->WaypointFollower_c2.HeadingCmd;
        localB->InnerLoopCmds.YawCmd = localB->WaypointFollower_c2.YawCmd;

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S9>/Constant'
         *  SignalConversion generated from: '<S9>/Status'
         */
        localB->Status = 0U;
        break;
      }
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    switch (localDW->is_GuidanceLogic) {
     case UAM_FlightMod_IN_BackTransition:
      /* BusCreator: '<S4>/Bus Creator1' incorporates:
       *  Constant: '<S4>/Constant1'
       *  Constant: '<S4>/Constant2'
       *  Merge: '<S3>/ Merge 2'
       */
      localB->InnerLoopCmds.HeadingCmd = 0.0;
      localB->InnerLoopCmds.YawCmd = 0.0;

      /* MATLAB Function: '<S4>/MATLAB Function' */
      rtb_DotProduct = 3.3121686421112381E-170;

      /* BusCreator: '<S4>/Bus Creator1' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       *  Merge: '<S3>/ Merge 2'
       */
      localB->InnerLoopCmds.LAP[0] = rtu_States->Xe[0];

      /* MATLAB Function: '<S4>/MATLAB Function' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       */
      absxk = fabs(rtu_States->Ve[0]);
      if (absxk > 3.3121686421112381E-170) {
        turnInternal = 1.0;
        rtb_DotProduct = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        turnInternal = t * t;
      }

      /* BusCreator: '<S4>/Bus Creator1' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       *  Merge: '<S3>/ Merge 2'
       */
      localB->InnerLoopCmds.LAP[1] = rtu_States->Xe[1];
      localB->InnerLoopCmds.LAP[2] = rtu_States->Xe[2];

      /* Merge: '<S3>/ Merge ' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       *  DataTypeConversion: '<S4>/Data Type Conversion'
       *  MATLAB Function: '<S4>/MATLAB Function'
       */
      localB->Status = (uint8_T)((fabs(rtu_States->c1) < 0.1) && (rtb_DotProduct
        * sqrt(turnInternal) < 4.0));
      break;

     case UAM_FlightM_IN_FIXED_WING_ENTRY:
      /* Chart: '<Root>/Guidance Mode Selector' */
      UAM_FlightMode_FIXED_WING_ENTRY(rtu_mode, rtu_ToWP, rtu_FromWP, rtu_States,
        localB, localDW);
      break;

     case UAM_FlightM_IN_FIXED_WING_ORBIT:
      /* MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       */
      rtb_DotProduct = rtu_ToWP->params[0];
      localDW->obj_b.OrbitRadiusFlag = 0U;
      if (rtu_ToWP->params[0] <= 50.0) {
        rtb_DotProduct = 50.0;
        localDW->obj_b.OrbitRadiusFlag = 1U;
      }

      localDW->obj_b.LookaheadDistFlag = 0U;

      /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       */
      tmp_1 = _mm_loadu_pd(&rtu_ToWP->position[0]);

      /* SignalConversion generated from: '<S5>/UAV Orbit Follower' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       */
      tmp_2 = _mm_loadu_pd(&rtu_States->Xe[0]);

      /* MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
       */
      _mm_storeu_pd(&xyLookaheadPoint[0], _mm_sub_pd(tmp_2, tmp_1));
      if (UAM_FlightMode_norm_p(xyLookaheadPoint) < 2.47032822920623E-323) {
        turnInternal = rtu_States->course;
        distToCenter = localDW->obj_b.NumCircles;
      } else {
        p = false;
        p_0 = true;
        b_k = 0;
        exitg1 = false;
        while ((!exitg1) && (b_k < 3)) {
          if (!(localDW->obj_b.OrbitCenterInternal[b_k] == rtu_ToWP->
                position[b_k])) {
            p_0 = false;
            exitg1 = true;
          } else {
            b_k++;
          }
        }

        if (p_0) {
          p = true;
        }

        guard2 = false;
        if (!p) {
          guard2 = true;
        } else {
          p = false;
          if (localDW->obj_b.OrbitRadiusInternal == rtb_DotProduct) {
            p = true;
          }

          if (!p) {
            guard2 = true;
          }
        }

        if (guard2) {
          localDW->obj_b.NumCircles = 0.0;
          localDW->obj_b.OrbitCenterInternal[0] = rtu_ToWP->position[0];
          localDW->obj_b.OrbitCenterInternal[1] = rtu_ToWP->position[1];
          localDW->obj_b.OrbitCenterInternal[2] = rtu_ToWP->position[2];
          localDW->obj_b.OrbitRadiusInternal = rtb_DotProduct;
          localDW->obj_b.SelectTurnDirectionFlag = true;
        }

        if (rtb_DotProduct <= 30.0) {
          localDW->obj_b.LookaheadDistance = 0.9 * rtb_DotProduct;
        } else {
          localDW->obj_b.LookaheadDistance = 30.0;
        }

        /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
         *  Chart: '<Root>/Guidance Mode Selector'
         *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
         */
        b_tmp_0 = rtu_States->Xe[0] - rtu_ToWP->position[0];
        b_tmp[0] = b_tmp_0;
        t = b_tmp_0 * b_tmp_0;
        b_tmp_0 = rtu_States->Xe[1] - rtu_ToWP->position[1];
        b_tmp[1] = b_tmp_0;
        distToCenter = sqrt(b_tmp_0 * b_tmp_0 + t);
        t = fabs(rtb_DotProduct + localDW->obj_b.LookaheadDistance);
        if (rtIsInf(t) || rtIsNaN(t)) {
          t = (rtNaN);
        } else if (t < 4.4501477170144028E-308) {
          t = 4.94065645841247E-324;
        } else {
          frexp(t, &b_exponent);
          t = ldexp(1.0, b_exponent - 53);
        }

        /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' */
        turnInternal = rtb_DotProduct + localDW->obj_b.LookaheadDistance;
        guard2 = false;
        if (distToCenter >= turnInternal - 5.0 * t) {
          guard2 = true;
        } else {
          t = fabs(turnInternal);
          if (rtIsInf(t) || rtIsNaN(t)) {
            t = (rtNaN);
          } else if (t < 4.4501477170144028E-308) {
            t = 4.94065645841247E-324;
          } else {
            frexp(t, &b_exponent_0);
            t = ldexp(1.0, b_exponent_0 - 53);
          }

          if (distToCenter <= (rtb_DotProduct - localDW->obj_b.LookaheadDistance)
              + 5.0 * t) {
            guard2 = true;
          } else {
            if (localDW->obj_b.StartFlag) {
              localDW->obj_b.PrevPosition[0] = rtu_States->Xe[0];
              localDW->obj_b.PrevPosition[1] = rtu_States->Xe[1];
              localDW->obj_b.PrevPosition[2] = rtu_States->Xe[2];
              localDW->obj_b.StartFlag = false;
            }

            if ((rtu_ToWP->params[1] == 0.0) &&
                (!localDW->obj_b.SelectTurnDirectionFlag)) {
              turnInternal = localDW->obj_b.TurnDirectionInternal;
            } else {
              turnInternal = rtu_ToWP->params[1];
            }

            rtb_Product[2] = 0.0;
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
              (localDW->obj_b.PrevPosition[0], rtu_States->Xe[0]), _mm_set1_pd
              (rtu_ToWP->position[0])));
            xyPose[0] = tmp[0];
            rtb_Product[0] = tmp[1];
            rtb_Sum[0] = xyLookaheadPoint[0];
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
              (localDW->obj_b.PrevPosition[1], rtu_States->Xe[1]), _mm_set1_pd
              (rtu_ToWP->position[1])));
            xyPose[1] = tmp[0];
            rtb_Product[1] = tmp[1];
            rtb_Sum[1] = xyLookaheadPoint[1];
            distToCenter = UAM_FlightMode_norm_p(xyPose);
            h = localDW->obj_b.LookaheadDistance *
              localDW->obj_b.LookaheadDistance;
            t = ((h - rtb_DotProduct * rtb_DotProduct) + distToCenter *
                 distToCenter) / (2.0 * distToCenter);
            tmp_0 = _mm_set1_pd(distToCenter);
            tmp_3 = _mm_sub_pd(tmp_1, tmp_2);
            _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(tmp_3,
              _mm_set1_pd(t)), tmp_0), tmp_2));
            y2 = tmp[1];
            h = sqrt(h - t * t);
            b_tmp[0] = tmp[0] - (rtu_ToWP->position[1] - rtu_States->Xe[1]) * h /
              distToCenter;
            _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd
              (_mm_set_pd(rtu_ToWP->position[0], rtu_ToWP->position[1]),
               _mm_set_pd(rtu_States->Xe[0], rtu_States->Xe[1])), _mm_set1_pd(h)),
              tmp_0), _mm_set_pd(tmp[1], tmp[0])));
            b_tmp[1] = tmp[0];
            t = tmp[1];
            y2 -= (rtu_ToWP->position[0] - rtu_States->Xe[0]) * h / distToCenter;
            rtb_Sum[2] = 0.0;
            if (turnInternal < 0.0) {
              rtb_Product[0] = xyLookaheadPoint[0];
              rtb_Product[1] = xyLookaheadPoint[1];
              rtb_Product[2] = 0.0;
              tmp_0 = _mm_sub_pd(_mm_loadu_pd(&localDW->obj_b.PrevPosition[0]),
                                 tmp_1);
              _mm_storeu_pd(&rtb_Sum[0], tmp_0);
              rtb_Sum[2] = 0.0;
            }

            h = UAM_FlightMode_norm_pv(rtb_Product);
            distToCenter = UAM_FlightMode_norm_pv(rtb_Sum);
            tmp_0 = _mm_set_pd(distToCenter, h);
            _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(rtb_Sum[0],
              rtb_Product[0]), tmp_0));
            rtb_Product[0] = tmp[0];
            rtb_Sum[0] = tmp[1];
            _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(rtb_Sum[1],
              rtb_Product[1]), tmp_0));
            rtb_Sum[1] = tmp[1];
            turnVector[2] = rtb_Product[0] * tmp[1] - rtb_Sum[0] * tmp[0];
            localDW->obj_b.PrevPosition[0] = rtu_States->Xe[0];
            localDW->obj_b.PrevPosition[1] = rtu_States->Xe[1];
            localDW->obj_b.PrevPosition[2] = rtu_States->Xe[2];
            localDW->obj_b.NumCircles += rt_atan2d_snf(turnVector[2],
              (rtb_Product[0] * rtb_Sum[0] + tmp[0] * tmp[1]) + 0.0 / h * (0.0 /
              distToCenter)) / 2.0 / 3.1415926535897931;
            distToCenter = localDW->obj_b.NumCircles;
            _mm_storeu_pd(&rtb_Sum[0], tmp_3);
            if (rtIsNaN(turnInternal)) {
              h = (rtNaN);
            } else if (turnInternal < 0.0) {
              h = -1.0;
            } else {
              h = (turnInternal > 0.0);
            }

            switch ((int32_T)h) {
             case 1:
              if ((b_tmp[0] - rtu_States->Xe[0]) * rtb_Sum[1] - (t -
                   rtu_States->Xe[1]) * rtb_Sum[0] > 0.0) {
                turnInternal = b_tmp[0];
                y2 = t;
              } else {
                turnInternal = b_tmp[1];
              }
              break;

             case -1:
              if ((b_tmp[0] - rtu_States->Xe[0]) * rtb_Sum[1] - (t -
                   rtu_States->Xe[1]) * rtb_Sum[0] < 0.0) {
                turnInternal = b_tmp[0];
                y2 = t;
              } else {
                turnInternal = b_tmp[1];
              }
              break;

             default:
              if (fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(t - rtu_States->Xe[1],
                     b_tmp[0] - rtu_States->Xe[0]), rtu_States->course)) < fabs
                  (UAM_FlightMode_angdiff(rt_atan2d_snf(y2 - rtu_States->Xe[1],
                     b_tmp[1] - rtu_States->Xe[0]), rtu_States->course))) {
                turnInternal = b_tmp[0];
                y2 = t;
              } else {
                turnInternal = b_tmp[1];
              }

              if ((turnInternal - rtu_States->Xe[0]) * rtb_Sum[1] - (y2 -
                   rtu_States->Xe[1]) * rtb_Sum[0] > 0.0) {
                localDW->obj_b.TurnDirectionInternal = 1.0;
              } else {
                localDW->obj_b.TurnDirectionInternal = -1.0;
              }

              localDW->obj_b.SelectTurnDirectionFlag = false;
              break;
            }
          }
        }

        if (guard2) {
          _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_mul_pd(_mm_div_pd(_mm_set_pd
            (b_tmp_0, b_tmp[0]), _mm_set1_pd(UAM_FlightMode_norm_p(b_tmp))),
            _mm_set1_pd(rtb_DotProduct)), tmp_1));
          turnInternal = tmp[0];
          y2 = tmp[1];
          distToCenter = localDW->obj_b.NumCircles;
        }

        turnInternal = rt_atan2d_snf(y2 - rtu_States->Xe[1], turnInternal -
          rtu_States->Xe[0]);
      }

      /* BusCreator: '<S5>/Bus Creator' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       *  Constant: '<S5>/Constant'
       *  Constant: '<S5>/Constant1'
       *  Constant: '<S5>/Lookahead Distance'
       *  MATLABSystem: '<S5>/UAV Orbit Follower'
       *  UnaryMinus: '<S5>/Unary Minus'
       * */
      localB->aacSP.airspeed = 15.0;
      localB->aacSP.altitude = -rtu_ToWP->position[2];
      localB->aacSP.course = turnInternal;
      localB->aacSP.L1 = 30.0;
      localB->aacSP.climbrate = 0.0;

      /* Merge: '<S3>/ Merge ' incorporates:
       *  Abs: '<S5>/Abs1'
       *  Chart: '<Root>/Guidance Mode Selector'
       *  DataTypeConversion: '<S5>/Data Type Conversion'
       *  MATLABSystem: '<S5>/UAV Orbit Follower'
       *  RelationalOperator: '<S5>/Relational Operator'
       * */
      localB->Status = (uint8_T)(fabs(distToCenter) > rtu_ToWP->params[2]);
      break;

     case UAM_Flig_IN_FIXED_WING_WAYPOINT:
      /* MATLAB Function: '<S6>/MATLAB Function' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       */
      rtb_wps_k[0] = rtu_FromWP->position[0];
      rtb_wps_k[1] = rtu_ToWP->position[0];
      rtb_wps_k[2] = rtu_FromWP->position[1];
      rtb_wps_k[3] = rtu_ToWP->position[1];
      rtb_wps_k[4] = rtu_FromWP->position[2];
      rtb_wps_k[5] = rtu_ToWP->position[2];

      /* MATLABSystem: '<S6>/Waypoint Follower' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       *  MATLAB Function: '<S6>/MATLAB Function'
       */
      localDW->obj_f.LookaheadDistFlag = 0U;
      localDW->obj_f.InitialPose[0] = 0.0;
      localDW->obj_f.InitialPose[1] = 0.0;
      localDW->obj_f.InitialPose[2] = 0.0;
      localDW->obj_f.InitialPose[3] = 0.0;
      localDW->obj_f.NumWaypoints = 2.0;
      p = false;
      p_0 = true;
      b_k = 0;
      exitg1 = false;
      while ((!exitg1) && (b_k <= 5)) {
        i = ((b_k / 2) << 1) + b_k % 2;
        if (!(localDW->obj_f.WaypointsInternal[i] == rtb_wps_k[i])) {
          p_0 = false;
          exitg1 = true;
        } else {
          b_k++;
        }
      }

      if (p_0) {
        p = true;
      }

      if (!p) {
        for (i = 0; i < 6; i++) {
          localDW->obj_f.WaypointsInternal[i] = rtb_wps_k[i];
        }

        localDW->obj_f.WaypointIndex = 1.0;
      }

      distinctWptsIdx[1] = true;
      x[0] = (rtu_FromWP->position[0] != rtu_ToWP->position[0]);
      x[1] = (rtu_FromWP->position[1] != rtu_ToWP->position[1]);
      x[2] = (rtu_FromWP->position[2] != rtu_ToWP->position[2]);
      distinctWptsIdx[0] = false;
      b_k = 0;
      exitg1 = false;
      while ((!exitg1) && (b_k < 3)) {
        if (x[b_k]) {
          distinctWptsIdx[0] = true;
          exitg1 = true;
        } else {
          b_k++;
        }
      }

      b_k = 0;
      for (i = 0; i < 2; i++) {
        /* MATLABSystem: '<S6>/Waypoint Follower' */
        if (distinctWptsIdx[i]) {
          b_k++;
        }
      }

      tmp_size_idx_1 = b_k;
      b_k = 0;
      for (i = 0; i < 2; i++) {
        /* MATLABSystem: '<S6>/Waypoint Follower' */
        if (distinctWptsIdx[i]) {
          /* Start for MATLABSystem: '<S6>/Waypoint Follower' */
          tmp_data[b_k] = (int8_T)i;
          b_k++;
        }
      }

      /* MATLABSystem: '<S6>/Waypoint Follower' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       *  SignalConversion generated from: '<S6>/Waypoint Follower'
       */
      for (i = 0; i < 3; i++) {
        scalarLB = (tmp_size_idx_1 / 2) << 1;
        vectorUB = scalarLB - 2;
        for (b_k = 0; b_k <= vectorUB; b_k += 2) {
          b_waypointsIn_tmp = i << 1;
          b_waypointsIn_data_tmp = tmp_size_idx_1 * i;
          b_waypointsIn_data[b_k + b_waypointsIn_data_tmp] =
            rtb_wps_k[b_waypointsIn_tmp + tmp_data[b_k]];
          b_waypointsIn_data[(b_k + b_waypointsIn_data_tmp) + 1] =
            rtb_wps_k[tmp_data[b_k + 1] + b_waypointsIn_tmp];
        }

        for (b_k = scalarLB; b_k < tmp_size_idx_1; b_k++) {
          b_waypointsIn_data[b_k + tmp_size_idx_1 * i] = rtb_wps_k[(i << 1) +
            tmp_data[b_k]];
        }
      }

      localDW->obj_f.LookaheadDistance = 30.0;
      if (tmp_size_idx_1 == 0) {
        turnInternal = rtu_States->course;
        localB->Status = 1U;
      } else {
        guard2 = false;
        if (tmp_size_idx_1 == 1) {
          if (localDW->obj_f.StartFlag) {
            localDW->obj_f.InitialPose[0] = rtu_States->Xe[0];
            localDW->obj_f.InitialPose[1] = rtu_States->Xe[1];
            localDW->obj_f.InitialPose[2] = rtu_States->Xe[2];
            localDW->obj_f.InitialPose[3] = rtu_States->course;
          }

          tmp_0 = _mm_sub_pd(_mm_loadu_pd(&b_waypointsIn_data[0]), _mm_loadu_pd(
            &rtu_States->Xe[0]));
          _mm_storeu_pd(&rtb_Product[0], tmp_0);
          rtb_Product[2] = b_waypointsIn_data[2] - rtu_States->Xe[2];
          if (UAM_FlightMode_norm_pv(rtb_Product) < 1.4901161193847656E-8) {
            turnInternal = rtu_States->course;
            localB->Status = 1U;
            localDW->obj_f.StartFlag = false;
          } else {
            localDW->obj_f.StartFlag = false;
            localDW->obj_f.NumWaypoints = 2.0;
            scalarLB = tmp_size_idx_1 + 1;
            for (i = 0; i < 3; i++) {
              vectorUB = (tmp_size_idx_1 + 1) * i;
              rtb_wps_k[vectorUB] = localDW->obj_f.InitialPose[i];
              for (b_k = 0; b_k < tmp_size_idx_1; b_k++) {
                rtb_wps_k[(b_k + vectorUB) + 1] =
                  b_waypointsIn_data[tmp_size_idx_1 * i + b_k];
              }
            }

            guard2 = true;
          }
        } else {
          scalarLB = tmp_size_idx_1;
          b_k = tmp_size_idx_1 * 3;
          if (b_k - 1 >= 0) {
            memcpy(&rtb_wps_k[0], &b_waypointsIn_data[0], (uint32_T)b_k * sizeof
                   (real_T));
          }

          guard2 = true;
        }

        if (guard2) {
          p = false;
          if (localDW->obj_f.WaypointIndex == 2.0) {
            p = true;
          }

          if (p) {
            localDW->obj_f.LastWaypointFlag = true;
            localDW->obj_f.WaypointIndex--;
          }

          rtb_Product[0] = rtb_wps_k[(int32_T)localDW->obj_f.WaypointIndex - 1];
          rtb_Sum[0] = rtb_wps_k[(int32_T)(localDW->obj_f.WaypointIndex + 1.0) -
            1];
          rtb_Product[1] = rtb_wps_k[((int32_T)localDW->obj_f.WaypointIndex +
            scalarLB) - 1];
          rtb_Sum[1] = rtb_wps_k[((int32_T)(localDW->obj_f.WaypointIndex + 1.0)
            + scalarLB) - 1];
          rtb_Product[2] = rtb_wps_k[((scalarLB << 1) + (int32_T)
            localDW->obj_f.WaypointIndex) - 1];
          rtb_DotProduct = rtb_wps_k[((int32_T)(localDW->obj_f.WaypointIndex +
            1.0) + (scalarLB << 1)) - 1];
          rtb_Sum[2] = rtb_DotProduct;
          turnInternal = rtb_wps_k[(int32_T)(localDW->obj_f.WaypointIndex + 1.0)
            - 1];
          tmp_1 = _mm_loadu_pd(&rtu_States->Xe[0]);
          _mm_storeu_pd(&turnVector_0[0], _mm_sub_pd(tmp_1, _mm_set_pd
            (rtb_wps_k[((int32_T)(localDW->obj_f.WaypointIndex + 1.0) + scalarLB)
             - 1], turnInternal)));
          turnVector_0[2] = rtu_States->Xe[2] - rtb_DotProduct;
          guard3 = false;
          if (UAM_FlightMode_norm_pv(turnVector_0) <= 10.0) {
            guard3 = true;
          } else {
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(rtu_States->Xe[0],
              turnInternal), _mm_set_pd(turnInternal, rtb_wps_k[(int32_T)
              localDW->obj_f.WaypointIndex - 1])));
            unitVectorUtoV_tmp[0] = tmp[0];
            turnVector[0] = tmp[1];
            turnInternal = rtb_wps_k[((int32_T)(localDW->obj_f.WaypointIndex +
              1.0) + scalarLB) - 1];
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(rtu_States->Xe[1],
              turnInternal), _mm_set_pd(turnInternal, rtb_wps_k[((int32_T)
              localDW->obj_f.WaypointIndex + scalarLB) - 1])));
            unitVectorUtoV_tmp[1] = tmp[0];
            turnVector[1] = tmp[1];
            turnInternal = rtb_wps_k[((int32_T)(localDW->obj_f.WaypointIndex +
              1.0) + (scalarLB << 1)) - 1];
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(rtu_States->Xe[2],
              turnInternal), _mm_set_pd(turnInternal, rtb_wps_k[((int32_T)
              localDW->obj_f.WaypointIndex + (scalarLB << 1)) - 1])));
            unitVectorUtoV_tmp[2] = tmp[0];
            turnVector[2] = tmp[1];
            h = UAM_FlightMode_norm_pv(unitVectorUtoV_tmp);
            distToCenter = UAM_FlightMode_norm_pv(turnVector);
            turnInternal = (unitVectorUtoV_tmp[0] / h * (turnVector[0] /
              distToCenter) + unitVectorUtoV_tmp[1] / h * (turnVector[1] /
              distToCenter)) + tmp[0] / h * (tmp[1] / distToCenter);
            if (rtIsNaN(turnInternal) || (turnInternal < 0.0)) {
            } else {
              guard3 = true;
            }
          }

          if (guard3) {
            localDW->obj_f.WaypointIndex++;
            p = false;
            if (localDW->obj_f.WaypointIndex == 2.0) {
              p = true;
            }

            if (p) {
              localDW->obj_f.LastWaypointFlag = true;
              localDW->obj_f.WaypointIndex--;
            }

            rtb_Product[0] = rtb_wps_k[(int32_T)localDW->obj_f.WaypointIndex - 1];
            rtb_Sum[0] = rtb_wps_k[(int32_T)(localDW->obj_f.WaypointIndex + 1.0)
              - 1];
            rtb_Product[1] = rtb_wps_k[((int32_T)localDW->obj_f.WaypointIndex +
              scalarLB) - 1];
            rtb_Sum[1] = rtb_wps_k[((int32_T)(localDW->obj_f.WaypointIndex + 1.0)
              + scalarLB) - 1];
            rtb_Product[2] = rtb_wps_k[((scalarLB << 1) + (int32_T)
              localDW->obj_f.WaypointIndex) - 1];
            rtb_Sum[2] = rtb_wps_k[((int32_T)(localDW->obj_f.WaypointIndex + 1.0)
              + (scalarLB << 1)) - 1];
          }

          _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(rtb_Sum[0],
            rtu_States->Xe[0]), _mm_set1_pd(rtb_Product[0])));
          turnVector[0] = tmp[0];
          unitVectorUtoV_tmp[0] = tmp[1];
          _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(rtb_Sum[1],
            rtu_States->Xe[1]), _mm_set1_pd(rtb_Product[1])));
          turnVector[1] = tmp[0];
          unitVectorUtoV_tmp[1] = tmp[1];
          _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(rtb_Sum[2],
            rtu_States->Xe[2]), _mm_set1_pd(rtb_Product[2])));
          turnVector[2] = tmp[0];
          unitVectorUtoV_tmp[2] = tmp[1];
          turnInternal = unitVectorUtoV_tmp[0] * unitVectorUtoV_tmp[0] +
            unitVectorUtoV_tmp[1] * unitVectorUtoV_tmp[1];
          rtb_DotProduct = tmp[1] * tmp[1] + turnInternal;
          absxk = ((turnVector[0] * unitVectorUtoV_tmp[0] + turnVector[1] *
                    unitVectorUtoV_tmp[1]) + tmp[0] * tmp[1]) / rtb_DotProduct;
          if (absxk < 0.0) {
            absxk = UAM_FlightMode_norm_pv(turnVector);
          } else if (absxk > 1.0) {
            tmp_0 = _mm_sub_pd(tmp_1, _mm_loadu_pd(&rtb_Sum[0]));
            _mm_storeu_pd(&turnVector_0[0], tmp_0);
            turnVector_0[2] = rtu_States->Xe[2] - rtb_Sum[2];
            absxk = UAM_FlightMode_norm_pv(turnVector_0);
          } else {
            tmp_0 = _mm_sub_pd(tmp_1, _mm_add_pd(_mm_mul_pd(_mm_set1_pd(absxk),
              _mm_loadu_pd(&unitVectorUtoV_tmp[0])), _mm_loadu_pd(&rtb_Product[0])));
            _mm_storeu_pd(&turnVector_0[0], tmp_0);
            turnVector_0[2] = rtu_States->Xe[2] - (absxk * tmp[1] + rtb_Product
              [2]);
            absxk = UAM_FlightMode_norm_pv(turnVector_0);
          }

          if (localDW->obj_f.LastWaypointFlag) {
            absxk = (((rtu_States->Xe[0] - rtb_Product[0]) * unitVectorUtoV_tmp
                      [0] + (rtu_States->Xe[1] - rtb_Product[1]) *
                      unitVectorUtoV_tmp[1]) + (rtu_States->Xe[2] - rtb_Product
                      [2]) * tmp[1]) / rtb_DotProduct;
            tmp_0 = _mm_sub_pd(tmp_1, _mm_add_pd(_mm_mul_pd(_mm_set1_pd(absxk),
              _mm_loadu_pd(&unitVectorUtoV_tmp[0])), _mm_loadu_pd(&rtb_Product[0])));
            _mm_storeu_pd(&turnVector_0[0], tmp_0);
            turnVector_0[2] = rtu_States->Xe[2] - (absxk * tmp[1] + rtb_Product
              [2]);
            absxk = UAM_FlightMode_norm_pv(turnVector_0);
          }

          rtb_DotProduct = fabs(absxk);
          if (rtIsInf(rtb_DotProduct) || rtIsNaN(rtb_DotProduct)) {
            t = (rtNaN);
            b_tmp_0 = (rtNaN);
          } else if (rtb_DotProduct < 4.4501477170144028E-308) {
            t = 4.94065645841247E-324;
            b_tmp_0 = 4.94065645841247E-324;
          } else {
            frexp(rtb_DotProduct, &b_exponent);
            t = ldexp(1.0, b_exponent - 53);
            frexp(rtb_DotProduct, &b_exponent_0);
            b_tmp_0 = ldexp(1.0, b_exponent_0 - 53);
          }

          rtb_DotProduct = sqrt(t);
          t = 5.0 * b_tmp_0;
          if ((rtb_DotProduct >= t) || rtIsNaN(t)) {
            t = rtb_DotProduct;
          }

          if (absxk + t >= 30.0) {
            localDW->obj_f.LookaheadDistance = localDW->obj_f.LookaheadFactor *
              absxk;
          }

          turnVector[0] = unitVectorUtoV_tmp[0];
          turnVector[1] = unitVectorUtoV_tmp[1];
          tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtb_Product[0]), tmp_1);
          _mm_storeu_pd(&unitVectorUtoV_tmp[0], tmp_0);
          turnInternal += unitVectorUtoV_tmp[2] * unitVectorUtoV_tmp[2];
          unitVectorUtoV_tmp[2] = rtb_Product[2] - rtu_States->Xe[2];
          absxk = ((turnVector[0] * unitVectorUtoV_tmp[0] + turnVector[1] *
                    unitVectorUtoV_tmp[1]) + tmp[1] * unitVectorUtoV_tmp[2]) *
            2.0;
          t = sqrt(absxk * absxk - (((unitVectorUtoV_tmp[0] *
                      unitVectorUtoV_tmp[0] + unitVectorUtoV_tmp[1] *
                      unitVectorUtoV_tmp[1]) + unitVectorUtoV_tmp[2] *
                     unitVectorUtoV_tmp[2]) - localDW->obj_f.LookaheadDistance *
                    localDW->obj_f.LookaheadDistance) * (4.0 * turnInternal));
          rtb_DotProduct = (-absxk + t) / 2.0 / turnInternal;
          turnInternal = (-absxk - t) / 2.0 / turnInternal;
          if ((rtb_DotProduct >= turnInternal) || rtIsNaN(turnInternal)) {
            turnInternal = rtb_DotProduct;
          }

          tmp_0 = _mm_set1_pd(turnInternal);
          tmp_0 = _mm_add_pd(_mm_mul_pd(_mm_sub_pd(_mm_set1_pd(1.0), tmp_0),
            _mm_loadu_pd(&rtb_Product[0])), _mm_mul_pd(tmp_0, _mm_loadu_pd
            (&rtb_Sum[0])));
          _mm_storeu_pd(&rtb_Sum[0], tmp_0);
          turnInternal = rt_atan2d_snf(rtb_Sum[1] - rtu_States->Xe[1], rtb_Sum[0]
            - rtu_States->Xe[0]);
          localB->Status = 0U;
          p = false;
          if (localDW->obj_f.LastWaypointFlag) {
            p = true;
          }

          if (p) {
            localB->Status = 1U;
          }

          localDW->obj_f.LastWaypointFlag = false;
        }
      }

      /* BusCreator: '<S6>/Bus Creator' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       *  Constant: '<S6>/Constant'
       *  Constant: '<S6>/Constant1'
       *  Constant: '<S6>/Lookahead Distance'
       *  MATLAB Function: '<S6>/MATLAB Function'
       *  MATLABSystem: '<S6>/Waypoint Follower'
       * */
      localB->aacSP.airspeed = 15.0;
      localB->aacSP.altitude = -rtu_ToWP->position[2];
      localB->aacSP.course = turnInternal;
      localB->aacSP.L1 = 30.0;
      localB->aacSP.climbrate = 0.0;
      break;

     case UAM_FlightMode_IN_FWCOMPLETE:
      /* Merge: '<S3>/ Merge ' */
      localB->Status = 1U;
      localB->controlMode_m.TransitionCondition = 1U;
      break;

     case UAM_Flight_IN_ForwardTransition:
      /* Chart: '<Root>/Guidance Mode Selector' incorporates:
       *  Merge: '<S3>/ Merge '
       */
      if (UAM_Flig_transitionConditionMet(rtu_States)) {
        localDW->is_GuidanceLogic = UAM_FlightMode_IN_FWCOMPLETE;
        localB->Status = 1U;
        localB->controlMode_m.TransitionCondition = 1U;
      } else {
        /* BusCreator: '<S7>/Bus Creator1' incorporates:
         *  Constant: '<S7>/Constant1'
         *  Constant: '<S7>/Constant2'
         *  Constant: '<S7>/Constant3'
         *  Merge: '<S3>/ Merge 2'
         */
        localB->InnerLoopCmds.LAP[0] = 0.0;
        localB->InnerLoopCmds.LAP[1] = 0.0;
        localB->InnerLoopCmds.LAP[2] = 0.0;
        localB->InnerLoopCmds.HeadingCmd = 0.0;
        localB->InnerLoopCmds.YawCmd = 0.0;

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S7>/Constant'
         *  DataTypeConversion: '<S7>/Data Type Conversion'
         */
        localB->Status = 0U;
      }
      break;

     case UAM_FlightMode_IN_HOVER_ENTRY:
      /* Chart: '<Root>/Guidance Mode Selector' incorporates:
       *  Constant: '<S9>/Lookahead Distance'
       *  MATLABSystem: '<S10>/UAV Orbit Follower'
       *  Sum: '<S18>/Sum'
       * */
      switch (*rtu_mode) {
       case 2U:
        localDW->is_GuidanceLogic = UAM_FlightMode_IN_WP;

        /* Switch: '<S17>/Switch' incorporates:
         *  Product: '<S21>/Product'
         *  RelationalOperator: '<S17>/Equal'
         */
        if (rtu_FromWP->mode == 6) {
          rtb_Product[0] = rtu_FromWP->position[0];
          rtb_Product[1] = rtu_FromWP->position[1];
          rtb_Product[2] = rtu_FromWP->position[2];
        } else {
          rtb_Product[0] = 0.0;
          rtb_Product[1] = 0.0;
          rtb_Product[2] = 0.0;
        }

        /* MATLAB Function: '<S12>/MATLAB Function' incorporates:
         *  BusCreator generated from: '<S12>/MATLAB Function'
         *  Product: '<S21>/Product'
         */
        rtb_wps[0] = rtb_Product[0];
        rtb_wps[2] = rtb_Product[1];
        rtb_wps[4] = rtb_Product[2];
        rtb_wps[6] = rtu_FromWP->params[3];
        rtb_wps[1] = rtu_ToWP->position[0];
        rtb_wps[3] = rtu_ToWP->position[1];
        rtb_wps[5] = rtu_ToWP->position[2];
        rtb_wps[7] = rtu_ToWP->params[3];

        /* MATLABSystem: '<S12>/Waypoint Follower' incorporates:
         *  Constant: '<S12>/Lookahead Distance'
         */
        UAM_FlightMode_SystemCore_step(&localDW->obj, rtu_Pose, rtb_wps, 5.0,
          localB->InnerLoopCmds.LAP, &turnInternal, &rtb_DotProduct,
          &b_varargout_4);

        /* BusCreator: '<S12>/Bus Creator' incorporates:
         *  Constant: '<S12>/Constant'
         *  MATLABSystem: '<S12>/Waypoint Follower'
         *  Merge: '<S3>/ Merge 2'
         * */
        localB->InnerLoopCmds.HeadingCmd = turnInternal;
        localB->InnerLoopCmds.YawCmd = 0.0;

        /* Sum: '<S18>/Sum1' incorporates:
         *  Product: '<S21>/Product'
         */
        turnInternal = rtu_ToWP->position[0] - rtb_Product[0];
        rtb_Product[0] = turnInternal;

        /* DotProduct: '<S21>/Dot Product1' incorporates:
         *  Product: '<S21>/Product'
         */
        rtb_DotProduct = turnInternal * turnInternal;

        /* Sum: '<S18>/Sum1' incorporates:
         *  Product: '<S21>/Product'
         */
        turnInternal = rtu_ToWP->position[1] - rtb_Product[1];
        rtb_Product[1] = turnInternal;

        /* DotProduct: '<S21>/Dot Product1' incorporates:
         *  Product: '<S21>/Product'
         */
        rtb_DotProduct += turnInternal * turnInternal;

        /* Sum: '<S18>/Sum1' incorporates:
         *  Product: '<S21>/Product'
         */
        turnInternal = rtu_ToWP->position[2] - rtb_Product[2];

        /* DotProduct: '<S21>/Dot Product1' incorporates:
         *  Product: '<S21>/Product'
         */
        rtb_DotProduct += turnInternal * turnInternal;

        /* Saturate: '<S21>/Saturation' incorporates:
         *  DotProduct: '<S21>/Dot Product1'
         */
        if (rtb_DotProduct <= 1.0E-5) {
          rtb_DotProduct = 1.0E-5;
        }

        /* Sqrt: '<S21>/Sqrt' incorporates:
         *  Saturate: '<S21>/Saturation'
         */
        rtb_DotProduct = sqrt(rtb_DotProduct);
        tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtu_ToWP->position[0]), _mm_loadu_pd
                           (&rtu_Pose[0]));
        _mm_storeu_pd(&rtb_Sum[0], tmp_0);

        /* Sum: '<S18>/Sum' */
        rtb_Sum[2] = rtu_ToWP->position[2] - rtu_Pose[2];

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S12>/Lookahead Distance'
         *  DataTypeConversion: '<S12>/Data Type Conversion'
         *  DotProduct: '<S18>/Dot Product'
         *  Product: '<S21>/Product'
         *  RelationalOperator: '<S12>/Relational Operator'
         *  Sum: '<S18>/Sum'
         *  Sum: '<S18>/Sum1'
         */
        localB->Status = (uint8_T)((rtb_Product[0] / rtb_DotProduct * rtb_Sum[0]
          + rtb_Product[1] / rtb_DotProduct * rtb_Sum[1]) + turnInternal /
          rtb_DotProduct * rtb_Sum[2] <= 5.0);
        break;

       case 3U:
        localDW->is_GuidanceLogic = UAM_FlightMode_IN_Orbit;

        /* Abs: '<S10>/Abs' */
        rtb_Abs = fabs(rtu_ToWP->params[0]);

        /* Signum: '<S10>/Sign' */
        if (rtIsNaN(rtu_ToWP->params[1])) {
          absxk = (rtNaN);
        } else if (rtu_ToWP->params[1] < 0.0) {
          absxk = -1.0;
        } else {
          absxk = (rtu_ToWP->params[1] > 0.0);
        }

        /* MATLABSystem: '<S10>/UAV Orbit Follower' */
        localDW->obj_l.OrbitRadiusFlag = 0U;
        if (rtb_Abs <= 1.0) {
          rtb_Abs = 1.0;
          localDW->obj_l.OrbitRadiusFlag = 1U;
        }

        localDW->obj_l.LookaheadDistFlag = 0U;
        tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtu_Pose[0]), _mm_loadu_pd
                           (&rtu_ToWP->position[0]));
        _mm_storeu_pd(&rtu_Pose_0[0], tmp_0);

        /* MATLABSystem: '<S10>/UAV Orbit Follower' */
        if (UAM_FlightMode_norm_p(rtu_Pose_0) < 2.47032822920623E-323) {
          tmp_0 = _mm_add_pd(_mm_mul_pd(_mm_set1_pd(rtb_Abs), _mm_set_pd(sin
            (rtu_Pose[3]), cos(rtu_Pose[3]))), _mm_loadu_pd(&rtu_Pose[0]));
          _mm_storeu_pd(&rtb_Product[0], tmp_0);
          rtb_Product[2] = rtu_ToWP->position[2];
          turnInternal = rtu_Pose[3];
          absxk = rtu_Pose[3];
          distToCenter = localDW->obj_l.NumCircles;
        } else {
          p = false;
          p_0 = true;
          b_k = 0;
          exitg1 = false;
          while ((!exitg1) && (b_k < 3)) {
            if (!(localDW->obj_l.OrbitCenterInternal[b_k] == rtu_ToWP->
                  position[b_k])) {
              p_0 = false;
              exitg1 = true;
            } else {
              b_k++;
            }
          }

          if (p_0) {
            p = true;
          }

          guard2 = false;
          if (!p) {
            guard2 = true;
          } else {
            p = false;
            if (localDW->obj_l.OrbitRadiusInternal == rtb_Abs) {
              p = true;
            }

            if (!p) {
              guard2 = true;
            }
          }

          if (guard2) {
            localDW->obj_l.NumCircles = 0.0;
            localDW->obj_l.OrbitCenterInternal[0] = rtu_ToWP->position[0];
            localDW->obj_l.OrbitCenterInternal[1] = rtu_ToWP->position[1];
            localDW->obj_l.OrbitCenterInternal[2] = rtu_ToWP->position[2];
            localDW->obj_l.OrbitRadiusInternal = rtb_Abs;
            localDW->obj_l.SelectTurnDirectionFlag = true;
          }

          if (rtb_Abs <= 5.0) {
            localDW->obj_l.LookaheadDistance = 0.9 * rtb_Abs;
          } else {
            localDW->obj_l.LookaheadDistance = 5.0;
          }

          turnInternal = rtu_Pose[0];
          rtb_DotProduct = rtu_Pose[1];
          b_tmp_0 = turnInternal - rtu_ToWP->position[0];
          b_tmp[0] = b_tmp_0;
          t = b_tmp_0 * b_tmp_0;
          b_tmp_0 = rtb_DotProduct - rtu_ToWP->position[1];
          b_tmp[1] = b_tmp_0;
          distToCenter = sqrt(b_tmp_0 * b_tmp_0 + t);
          h = rtb_Abs + localDW->obj_l.LookaheadDistance;
          if (rtIsInf(h) || rtIsNaN(h)) {
            t = (rtNaN);
          } else if (h < 4.4501477170144028E-308) {
            t = 4.94065645841247E-324;
          } else {
            frexp(h, &b_exponent);
            t = ldexp(1.0, b_exponent - 53);
          }

          guard2 = false;
          if (distToCenter >= h - 5.0 * t) {
            guard2 = true;
          } else {
            if (rtIsInf(h) || rtIsNaN(h)) {
              t = (rtNaN);
            } else if (h < 4.4501477170144028E-308) {
              t = 4.94065645841247E-324;
            } else {
              frexp(h, &b_exponent_0);
              t = ldexp(1.0, b_exponent_0 - 53);
            }

            if (distToCenter <= (rtb_Abs - localDW->obj_l.LookaheadDistance) +
                5.0 * t) {
              guard2 = true;
            } else {
              if (localDW->obj_l.StartFlag) {
                localDW->obj_l.PrevPosition[0] = rtu_Pose[0];
                localDW->obj_l.PrevPosition[1] = rtu_Pose[1];
                localDW->obj_l.StartFlag = false;
              }

              if ((absxk == 0.0) && (!localDW->obj_l.SelectTurnDirectionFlag)) {
                absxk = localDW->obj_l.TurnDirectionInternal;
              }

              _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
                (localDW->obj_l.PrevPosition[0], turnInternal), _mm_set1_pd
                (rtu_ToWP->position[0])));
              xyPose[0] = tmp[0];
              turnVector[0] = tmp[1];
              _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
                (localDW->obj_l.PrevPosition[1], rtb_DotProduct), _mm_set1_pd
                (rtu_ToWP->position[1])));
              xyPose[1] = tmp[0];
              turnVector[1] = tmp[1];
              distToCenter = UAM_FlightMode_norm_p(xyPose);
              t = ((localDW->obj_l.LookaheadDistance *
                    localDW->obj_l.LookaheadDistance - rtb_Abs * rtb_Abs) +
                   distToCenter * distToCenter) / (2.0 * distToCenter);
              tmp_0 = _mm_set_pd(rtb_DotProduct, turnInternal);
              tmp_1 = _mm_set1_pd(distToCenter);
              _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd
                (_mm_loadu_pd(&rtu_ToWP->position[0]), tmp_0), _mm_set1_pd(t)),
                tmp_1), tmp_0));
              y2 = tmp[1];
              h = sqrt(localDW->obj_l.LookaheadDistance *
                       localDW->obj_l.LookaheadDistance - t * t);
              b_tmp_0 = rtu_ToWP->position[1] - rtb_DotProduct;
              b_tmp[0] = tmp[0] - b_tmp_0 * h / distToCenter;
              _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd
                (_mm_set_pd(rtu_ToWP->position[0], rtu_ToWP->position[1]),
                 _mm_set_pd(turnInternal, rtb_DotProduct)), _mm_set1_pd(h)),
                tmp_1), _mm_set_pd(tmp[1], tmp[0])));
              b_tmp[1] = tmp[0];
              t = tmp[1];
              rtb_Abs = rtu_ToWP->position[0] - turnInternal;
              y2 -= rtb_Abs * h / distToCenter;
              rtb_Product[0] = turnVector[0];
              rtb_Product[1] = turnVector[1];
              rtb_Product[2] = 0.0;
              tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtu_Pose[0]), _mm_loadu_pd
                                 (&rtu_ToWP->position[0]));
              _mm_storeu_pd(&rtb_Sum[0], tmp_0);
              rtb_Sum[2] = 0.0;
              if (absxk < 0.0) {
                rtb_Product[0] = rtb_Sum[0];
                rtb_Sum[0] = turnVector[0];
                rtb_Product[1] = rtb_Sum[1];
                rtb_Sum[1] = turnVector[1];
                rtb_Product[2] = 0.0;
                rtb_Sum[2] = 0.0;
              }

              h = UAM_FlightMode_norm_pv(rtb_Product);
              distToCenter = UAM_FlightMode_norm_pv(rtb_Sum);
              tmp_0 = _mm_set_pd(distToCenter, h);
              _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(rtb_Sum[0],
                rtb_Product[0]), tmp_0));
              rtb_Product[0] = tmp[0];
              rtb_Sum[0] = tmp[1];
              _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(rtb_Sum[1],
                rtb_Product[1]), tmp_0));
              rtb_Sum[1] = tmp[1];
              turnVector[2] = rtb_Product[0] * tmp[1] - rtb_Sum[0] * tmp[0];
              localDW->obj_l.NumCircles += rt_atan2d_snf(turnVector[2],
                (rtb_Product[0] * rtb_Sum[0] + tmp[0] * tmp[1]) + 0.0 / h * (0.0
                / distToCenter)) / 2.0 / 3.1415926535897931;
              distToCenter = localDW->obj_l.NumCircles;
              localDW->obj_l.PrevPosition[0] = rtu_Pose[0];
              localDW->obj_l.PrevPosition[1] = rtu_Pose[1];
              localDW->obj_l.PrevPosition[2] = rtu_Pose[2];
              tmp_0 = _mm_sub_pd(_mm_set_pd(t, b_tmp[0]), _mm_loadu_pd
                                 (&rtu_Pose[0]));
              _mm_storeu_pd(&rtb_Product[0], tmp_0);
              tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtu_ToWP->position[0]),
                                 _mm_loadu_pd(&rtu_Pose[0]));
              _mm_storeu_pd(&rtb_Sum[0], tmp_0);
              if (rtIsNaN(absxk)) {
                h = (rtNaN);
              } else if (absxk < 0.0) {
                h = -1.0;
              } else {
                h = (absxk > 0.0);
              }

              switch ((int32_T)h) {
               case 1:
                if (rtb_Product[0] * rtb_Sum[1] - rtb_Sum[0] * rtb_Product[1] >
                    0.0) {
                  xyLookaheadPoint[0] = b_tmp[0];
                  xyLookaheadPoint[1] = t;
                } else {
                  xyLookaheadPoint[0] = b_tmp[1];
                  xyLookaheadPoint[1] = y2;
                }
                break;

               case -1:
                if (rtb_Product[0] * rtb_Sum[1] - rtb_Sum[0] * rtb_Product[1] <
                    0.0) {
                  xyLookaheadPoint[0] = b_tmp[0];
                  xyLookaheadPoint[1] = t;
                } else {
                  xyLookaheadPoint[0] = b_tmp[1];
                  xyLookaheadPoint[1] = y2;
                }
                break;

               default:
                p = (fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(t - rtu_Pose[1],
                        b_tmp[0] - rtu_Pose[0]), rtu_Pose[3])) < fabs
                     (UAM_FlightMode_angdiff(rt_atan2d_snf(y2 - rtu_Pose[1],
                        b_tmp[1] - rtu_Pose[0]), rtu_Pose[3])));
                if (p) {
                  xyLookaheadPoint[0] = b_tmp[0];
                  xyLookaheadPoint[1] = t;
                } else {
                  xyLookaheadPoint[0] = b_tmp[1];
                  xyLookaheadPoint[1] = y2;
                }

                tmp_0 = _mm_sub_pd(_mm_loadu_pd(&xyLookaheadPoint[0]),
                                   _mm_loadu_pd(&rtu_Pose[0]));
                _mm_storeu_pd(&rtb_Product[0], tmp_0);
                if (rtb_Product[0] * rtb_Sum[1] - rtb_Sum[0] * rtb_Product[1] >
                    0.0) {
                  localDW->obj_l.TurnDirectionInternal = 1.0;
                } else {
                  localDW->obj_l.TurnDirectionInternal = -1.0;
                }

                localDW->obj_l.SelectTurnDirectionFlag = false;
                break;
              }

              absxk = rt_atan2d_snf(b_tmp_0, rtb_Abs);
            }
          }

          if (guard2) {
            _mm_storeu_pd(&xyLookaheadPoint[0], _mm_add_pd(_mm_mul_pd(_mm_div_pd
              (_mm_set_pd(b_tmp_0, b_tmp[0]), _mm_set1_pd(UAM_FlightMode_norm_p
              (b_tmp))), _mm_set1_pd(rtb_Abs)), _mm_loadu_pd(&rtu_ToWP->
              position[0])));
            absxk = rt_atan2d_snf(xyLookaheadPoint[1] - rtb_DotProduct,
                                  xyLookaheadPoint[0] - turnInternal);
            distToCenter = localDW->obj_l.NumCircles;
          }

          rtb_Product[0] = xyLookaheadPoint[0];
          rtb_Product[1] = xyLookaheadPoint[1];
          rtb_Product[2] = rtu_ToWP->position[2];
          turnInternal = rt_atan2d_snf(xyLookaheadPoint[1] - rtb_DotProduct,
            xyLookaheadPoint[0] - turnInternal);
        }

        /* BusCreator: '<S10>/Bus Creator' incorporates:
         *  MATLABSystem: '<S10>/UAV Orbit Follower'
         *  Merge: '<S3>/ Merge 2'
         * */
        localB->InnerLoopCmds.LAP[0] = rtb_Product[0];
        localB->InnerLoopCmds.LAP[1] = rtb_Product[1];
        localB->InnerLoopCmds.LAP[2] = rtb_Product[2];
        localB->InnerLoopCmds.HeadingCmd = turnInternal;
        localB->InnerLoopCmds.YawCmd = absxk;

        /* Merge: '<S3>/ Merge ' incorporates:
         *  DataTypeConversion: '<S10>/Data Type Conversion'
         *  MATLABSystem: '<S10>/UAV Orbit Follower'
         *  RelationalOperator: '<S10>/Relational Operator'
         * */
        localB->Status = (uint8_T)(distToCenter > rtu_ToWP->params[2]);
        break;

       case 6U:
        localB->FlightMode = Transition;
        localB->controlMode_m.inTransition = 1U;
        localB->controlMode_m.TransitionCondition = 0U;
        localDW->is_GuidanceLogic = UAM_Flight_IN_ForwardTransition;

        /* BusCreator: '<S7>/Bus Creator1' incorporates:
         *  Constant: '<S7>/Constant1'
         *  Constant: '<S7>/Constant2'
         *  Constant: '<S7>/Constant3'
         *  Merge: '<S3>/ Merge 2'
         */
        localB->InnerLoopCmds.LAP[0] = 0.0;
        localB->InnerLoopCmds.LAP[1] = 0.0;
        localB->InnerLoopCmds.LAP[2] = 0.0;
        localB->InnerLoopCmds.HeadingCmd = 0.0;
        localB->InnerLoopCmds.YawCmd = 0.0;

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S7>/Constant'
         *  DataTypeConversion: '<S7>/Data Type Conversion'
         */
        localB->Status = 0U;
        break;

       default:
        localDW->is_GuidanceLogic = UAM_FlightMode_IN_Land;
        localDW->is_Land = UAM_FlightMode_IN_ToLand;

        /* MATLAB Function: '<S9>/MATLAB Function' */
        rtb_wps_l[0] = rtu_FromWP->position[0];
        rtb_wps_l[2] = rtu_FromWP->position[1];
        rtb_wps_l[4] = rtu_FromWP->position[2];
        rtb_wps_l[6] = rtu_FromWP->params[3];
        rtb_wps_l[1] = rtu_ToWP->position[0];
        rtb_wps_l[3] = rtu_ToWP->position[1];
        rtb_wps_l[5] = rtu_ToWP->position[2];
        rtb_wps_l[7] = rtu_ToWP->params[3];
        UAM_FlightMode_WaypointFollower(rtu_Pose, rtb_wps_l, 3.0,
          &localB->WaypointFollower_c2, &localDW->WaypointFollower_c2);

        /* BusCreator: '<S9>/Bus Creator1' incorporates:
         *  Constant: '<S9>/Lookahead Distance'
         *  MATLABSystem: '<S9>/Waypoint Follower'
         *  Merge: '<S3>/ Merge 2'
         */
        localB->InnerLoopCmds.LAP[0] = localB->WaypointFollower_c2.LAP[0];
        localB->InnerLoopCmds.LAP[1] = localB->WaypointFollower_c2.LAP[1];
        localB->InnerLoopCmds.LAP[2] = localB->WaypointFollower_c2.LAP[2];
        localB->InnerLoopCmds.HeadingCmd =
          localB->WaypointFollower_c2.HeadingCmd;
        localB->InnerLoopCmds.YawCmd = localB->WaypointFollower_c2.YawCmd;

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S9>/Constant'
         *  SignalConversion generated from: '<S9>/Status'
         */
        localB->Status = 0U;
        break;
      }
      break;

     case UAM_FlightMode_IN_Land:
      if (localDW->is_Land == UAM_FlightMode_IN_Descend) {
        /* MATLAB Function: '<S8>/MATLAB Function' incorporates:
         *  Chart: '<Root>/Guidance Mode Selector'
         *  SignalConversion generated from: '<S15>/ SFunction '
         */
        rtb_wps[0] = rtu_FromWP->position[0];
        rtb_wps[2] = rtu_FromWP->position[1];
        rtb_wps[4] = rtu_FromWP->position[2];
        rtb_wps[6] = rtu_FromWP->params[3];
        rtb_wps[1] = rtu_ToWP->position[0];
        rtb_wps[3] = rtu_ToWP->position[1];
        rtb_wps[5] = *rtu_Ground;
        rtb_wps[7] = rtu_ToWP->params[3];

        /* Chart: '<Root>/Guidance Mode Selector' incorporates:
         *  Constant: '<S8>/Constant'
         */
        UAM_FlightMode_WaypointFollower(rtu_Pose, rtb_wps, 3.0,
          &localB->WaypointFollower_c, &localDW->WaypointFollower_c);

        /* BusCreator: '<S8>/Bus Creator' incorporates:
         *  MATLABSystem: '<S8>/Waypoint Follower'
         *  Merge: '<S3>/ Merge 2'
         */
        localB->InnerLoopCmds.LAP[0] = localB->WaypointFollower_c.LAP[0];
        localB->InnerLoopCmds.LAP[1] = localB->WaypointFollower_c.LAP[1];
        localB->InnerLoopCmds.LAP[2] = localB->WaypointFollower_c.LAP[2];

        /* Saturate: '<S8>/Hdg Cmd Sat' incorporates:
         *  Chart: '<Root>/Guidance Mode Selector'
         *  Saturate: '<S8>/Yaw Cmd Sat'
         */
        if (rtu_ToWP->params[3] > 3.1415926535897931) {
          /* BusCreator: '<S8>/Bus Creator' incorporates:
           *  Merge: '<S3>/ Merge 2'
           */
          localB->InnerLoopCmds.HeadingCmd = 3.1415926535897931;
          localB->InnerLoopCmds.YawCmd = 3.1415926535897931;
        } else if (rtu_ToWP->params[3] < -3.1415926535897931) {
          /* BusCreator: '<S8>/Bus Creator' incorporates:
           *  Merge: '<S3>/ Merge 2'
           */
          localB->InnerLoopCmds.HeadingCmd = -3.1415926535897931;
          localB->InnerLoopCmds.YawCmd = -3.1415926535897931;
        } else {
          /* BusCreator: '<S8>/Bus Creator' incorporates:
           *  Merge: '<S3>/ Merge 2'
           */
          localB->InnerLoopCmds.HeadingCmd = rtu_ToWP->params[3];
          localB->InnerLoopCmds.YawCmd = rtu_ToWP->params[3];
        }

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S8>/Constant3'
         *  SignalConversion generated from: '<S8>/Status'
         */
        localB->Status = 0U;
      } else {
        /* Chart: '<Root>/Guidance Mode Selector' */
        /* case IN_ToLand: */
        tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtu_Pose[0]), _mm_loadu_pd
                           (&rtu_ToWP->position[0]));
        _mm_storeu_pd(&tmp[0], tmp_0);
        rtb_DotProduct = 3.3121686421112381E-170;
        absxk = fabs(tmp[0]);
        if (absxk > 3.3121686421112381E-170) {
          turnInternal = 1.0;
          rtb_DotProduct = absxk;
        } else {
          t = absxk / 3.3121686421112381E-170;
          turnInternal = t * t;
        }

        absxk = fabs(tmp[1]);
        if (absxk > rtb_DotProduct) {
          t = rtb_DotProduct / absxk;
          turnInternal = turnInternal * t * t + 1.0;
          rtb_DotProduct = absxk;
        } else {
          t = absxk / rtb_DotProduct;
          turnInternal += t * t;
        }

        turnInternal = rtb_DotProduct * sqrt(turnInternal);
        if (turnInternal <= 3.0) {
          localDW->is_Land = UAM_FlightMode_IN_Descend;

          /* MATLAB Function: '<S8>/MATLAB Function' incorporates:
           *  Chart: '<Root>/Guidance Mode Selector'
           *  SignalConversion generated from: '<S15>/ SFunction '
           */
          rtb_wps[0] = rtu_FromWP->position[0];
          rtb_wps[2] = rtu_FromWP->position[1];
          rtb_wps[4] = rtu_FromWP->position[2];
          rtb_wps[6] = rtu_FromWP->params[3];
          rtb_wps[1] = rtu_ToWP->position[0];
          rtb_wps[3] = rtu_ToWP->position[1];
          rtb_wps[5] = *rtu_Ground;
          rtb_wps[7] = rtu_ToWP->params[3];

          /* Chart: '<Root>/Guidance Mode Selector' incorporates:
           *  Constant: '<S8>/Constant'
           */
          UAM_FlightMode_WaypointFollower(rtu_Pose, rtb_wps, 3.0,
            &localB->WaypointFollower_c, &localDW->WaypointFollower_c);

          /* BusCreator: '<S8>/Bus Creator' incorporates:
           *  MATLABSystem: '<S8>/Waypoint Follower'
           *  Merge: '<S3>/ Merge 2'
           */
          localB->InnerLoopCmds.LAP[0] = localB->WaypointFollower_c.LAP[0];
          localB->InnerLoopCmds.LAP[1] = localB->WaypointFollower_c.LAP[1];
          localB->InnerLoopCmds.LAP[2] = localB->WaypointFollower_c.LAP[2];

          /* Saturate: '<S8>/Hdg Cmd Sat' incorporates:
           *  Chart: '<Root>/Guidance Mode Selector'
           *  Saturate: '<S8>/Yaw Cmd Sat'
           */
          if (rtu_ToWP->params[3] > 3.1415926535897931) {
            /* BusCreator: '<S8>/Bus Creator' incorporates:
             *  Merge: '<S3>/ Merge 2'
             */
            localB->InnerLoopCmds.HeadingCmd = 3.1415926535897931;
            localB->InnerLoopCmds.YawCmd = 3.1415926535897931;
          } else if (rtu_ToWP->params[3] < -3.1415926535897931) {
            /* BusCreator: '<S8>/Bus Creator' incorporates:
             *  Merge: '<S3>/ Merge 2'
             */
            localB->InnerLoopCmds.HeadingCmd = -3.1415926535897931;
            localB->InnerLoopCmds.YawCmd = -3.1415926535897931;
          } else {
            /* BusCreator: '<S8>/Bus Creator' incorporates:
             *  Merge: '<S3>/ Merge 2'
             */
            localB->InnerLoopCmds.HeadingCmd = rtu_ToWP->params[3];
            localB->InnerLoopCmds.YawCmd = rtu_ToWP->params[3];
          }

          /* Merge: '<S3>/ Merge ' incorporates:
           *  Constant: '<S8>/Constant3'
           *  SignalConversion generated from: '<S8>/Status'
           */
          localB->Status = 0U;
        } else {
          /* MATLAB Function: '<S9>/MATLAB Function' incorporates:
           *  Chart: '<Root>/Guidance Mode Selector'
           */
          rtb_wps_l[0] = rtu_FromWP->position[0];
          rtb_wps_l[2] = rtu_FromWP->position[1];
          rtb_wps_l[4] = rtu_FromWP->position[2];
          rtb_wps_l[6] = rtu_FromWP->params[3];
          rtb_wps_l[1] = rtu_ToWP->position[0];
          rtb_wps_l[3] = rtu_ToWP->position[1];
          rtb_wps_l[5] = rtu_ToWP->position[2];
          rtb_wps_l[7] = rtu_ToWP->params[3];

          /* Chart: '<Root>/Guidance Mode Selector' incorporates:
           *  Constant: '<S9>/Lookahead Distance'
           */
          UAM_FlightMode_WaypointFollower(rtu_Pose, rtb_wps_l, 3.0,
            &localB->WaypointFollower_c2, &localDW->WaypointFollower_c2);

          /* BusCreator: '<S9>/Bus Creator1' incorporates:
           *  MATLABSystem: '<S9>/Waypoint Follower'
           *  Merge: '<S3>/ Merge 2'
           */
          localB->InnerLoopCmds.LAP[0] = localB->WaypointFollower_c2.LAP[0];
          localB->InnerLoopCmds.LAP[1] = localB->WaypointFollower_c2.LAP[1];
          localB->InnerLoopCmds.LAP[2] = localB->WaypointFollower_c2.LAP[2];
          localB->InnerLoopCmds.HeadingCmd =
            localB->WaypointFollower_c2.HeadingCmd;
          localB->InnerLoopCmds.YawCmd = localB->WaypointFollower_c2.YawCmd;

          /* Merge: '<S3>/ Merge ' incorporates:
           *  Constant: '<S9>/Constant'
           *  SignalConversion generated from: '<S9>/Status'
           */
          localB->Status = 0U;
        }
      }
      break;

     case UAM_FlightMode_IN_Orbit:
      /* Abs: '<S10>/Abs' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       */
      rtb_Abs = fabs(rtu_ToWP->params[0]);

      /* Signum: '<S10>/Sign' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       */
      if (rtIsNaN(rtu_ToWP->params[1])) {
        absxk = (rtNaN);
      } else if (rtu_ToWP->params[1] < 0.0) {
        absxk = -1.0;
      } else {
        absxk = (rtu_ToWP->params[1] > 0.0);
      }

      /* MATLABSystem: '<S10>/UAV Orbit Follower' */
      localDW->obj_l.OrbitRadiusFlag = 0U;
      if (rtb_Abs <= 1.0) {
        rtb_Abs = 1.0;
        localDW->obj_l.OrbitRadiusFlag = 1U;
      }

      localDW->obj_l.LookaheadDistFlag = 0U;

      /* Start for MATLABSystem: '<S10>/UAV Orbit Follower' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       */
      tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtu_Pose[0]), _mm_loadu_pd
                         (&rtu_ToWP->position[0]));

      /* MATLABSystem: '<S10>/UAV Orbit Follower' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       */
      _mm_storeu_pd(&rtu_Pose_0[0], tmp_0);
      if (UAM_FlightMode_norm_p(rtu_Pose_0) < 2.47032822920623E-323) {
        tmp_0 = _mm_add_pd(_mm_mul_pd(_mm_set1_pd(rtb_Abs), _mm_set_pd(sin
          (rtu_Pose[3]), cos(rtu_Pose[3]))), _mm_loadu_pd(&rtu_Pose[0]));
        _mm_storeu_pd(&rtb_Product[0], tmp_0);
        rtb_Product[2] = rtu_ToWP->position[2];
        turnInternal = rtu_Pose[3];
        absxk = rtu_Pose[3];
        distToCenter = localDW->obj_l.NumCircles;
      } else {
        p = false;
        p_0 = true;
        b_k = 0;
        exitg1 = false;
        while ((!exitg1) && (b_k < 3)) {
          if (!(localDW->obj_l.OrbitCenterInternal[b_k] == rtu_ToWP->
                position[b_k])) {
            p_0 = false;
            exitg1 = true;
          } else {
            b_k++;
          }
        }

        if (p_0) {
          p = true;
        }

        guard2 = false;
        if (!p) {
          guard2 = true;
        } else {
          p = false;
          if (localDW->obj_l.OrbitRadiusInternal == rtb_Abs) {
            p = true;
          }

          if (!p) {
            guard2 = true;
          }
        }

        if (guard2) {
          localDW->obj_l.NumCircles = 0.0;
          localDW->obj_l.OrbitCenterInternal[0] = rtu_ToWP->position[0];
          localDW->obj_l.OrbitCenterInternal[1] = rtu_ToWP->position[1];
          localDW->obj_l.OrbitCenterInternal[2] = rtu_ToWP->position[2];
          localDW->obj_l.OrbitRadiusInternal = rtb_Abs;
          localDW->obj_l.SelectTurnDirectionFlag = true;
        }

        if (rtb_Abs <= 5.0) {
          localDW->obj_l.LookaheadDistance = 0.9 * rtb_Abs;
        } else {
          localDW->obj_l.LookaheadDistance = 5.0;
        }

        turnInternal = rtu_Pose[0];
        rtb_DotProduct = rtu_Pose[1];

        /* Start for MATLABSystem: '<S10>/UAV Orbit Follower' incorporates:
         *  Chart: '<Root>/Guidance Mode Selector'
         */
        b_tmp_0 = turnInternal - rtu_ToWP->position[0];
        b_tmp[0] = b_tmp_0;
        t = b_tmp_0 * b_tmp_0;
        b_tmp_0 = rtb_DotProduct - rtu_ToWP->position[1];
        b_tmp[1] = b_tmp_0;
        distToCenter = sqrt(b_tmp_0 * b_tmp_0 + t);

        /* Start for MATLABSystem: '<S10>/UAV Orbit Follower' */
        h = rtb_Abs + localDW->obj_l.LookaheadDistance;
        if (rtIsInf(h) || rtIsNaN(h)) {
          t = (rtNaN);
        } else if (h < 4.4501477170144028E-308) {
          t = 4.94065645841247E-324;
        } else {
          frexp(h, &b_exponent);
          t = ldexp(1.0, b_exponent - 53);
        }

        guard2 = false;
        if (distToCenter >= h - 5.0 * t) {
          guard2 = true;
        } else {
          if (rtIsInf(h) || rtIsNaN(h)) {
            t = (rtNaN);
          } else if (h < 4.4501477170144028E-308) {
            t = 4.94065645841247E-324;
          } else {
            frexp(h, &b_exponent_0);
            t = ldexp(1.0, b_exponent_0 - 53);
          }

          if (distToCenter <= (rtb_Abs - localDW->obj_l.LookaheadDistance) + 5.0
              * t) {
            guard2 = true;
          } else {
            if (localDW->obj_l.StartFlag) {
              localDW->obj_l.PrevPosition[0] = rtu_Pose[0];
              localDW->obj_l.PrevPosition[1] = rtu_Pose[1];
              localDW->obj_l.PrevPosition[2] = rtu_Pose[2];
              localDW->obj_l.StartFlag = false;
            }

            if ((absxk == 0.0) && (!localDW->obj_l.SelectTurnDirectionFlag)) {
              absxk = localDW->obj_l.TurnDirectionInternal;
            }

            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
              (localDW->obj_l.PrevPosition[0], turnInternal), _mm_set1_pd
              (rtu_ToWP->position[0])));
            xyPose[0] = tmp[0];
            rtb_Product[0] = tmp[1];
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
              (localDW->obj_l.PrevPosition[1], rtb_DotProduct), _mm_set1_pd
              (rtu_ToWP->position[1])));
            xyPose[1] = tmp[0];
            rtb_Product[1] = tmp[1];
            distToCenter = UAM_FlightMode_norm_p(xyPose);
            t = ((localDW->obj_l.LookaheadDistance *
                  localDW->obj_l.LookaheadDistance - rtb_Abs * rtb_Abs) +
                 distToCenter * distToCenter) / (2.0 * distToCenter);
            tmp_0 = _mm_set_pd(rtb_DotProduct, turnInternal);
            tmp_1 = _mm_set1_pd(distToCenter);
            tmp_2 = _mm_loadu_pd(&rtu_ToWP->position[0]);
            _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd
              (tmp_2, tmp_0), _mm_set1_pd(t)), tmp_1), tmp_0));
            y2 = tmp[1];
            h = sqrt(localDW->obj_l.LookaheadDistance *
                     localDW->obj_l.LookaheadDistance - t * t);
            b_tmp_0 = rtu_ToWP->position[1] - rtb_DotProduct;
            b_tmp[0] = tmp[0] - b_tmp_0 * h / distToCenter;
            _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd
              (_mm_set_pd(rtu_ToWP->position[0], rtu_ToWP->position[1]),
               _mm_set_pd(turnInternal, rtb_DotProduct)), _mm_set1_pd(h)), tmp_1),
              _mm_set_pd(tmp[1], tmp[0])));
            b_tmp[1] = tmp[0];
            t = tmp[1];
            rtb_Abs = rtu_ToWP->position[0] - turnInternal;
            y2 -= rtb_Abs * h / distToCenter;
            rtb_Product[2] = 0.0;
            tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtu_Pose[0]), _mm_loadu_pd
                               (&rtu_ToWP->position[0]));
            _mm_storeu_pd(&rtb_Sum[0], tmp_0);
            rtb_Sum[2] = 0.0;
            if (absxk < 0.0) {
              rtb_Product[0] = rtb_Sum[0];
              rtb_Product[1] = rtb_Sum[1];
              rtb_Product[2] = 0.0;
              tmp_0 = _mm_sub_pd(_mm_loadu_pd(&localDW->obj_l.PrevPosition[0]),
                                 tmp_2);
              _mm_storeu_pd(&rtb_Sum[0], tmp_0);
              rtb_Sum[2] = 0.0;
            }

            h = UAM_FlightMode_norm_pv(rtb_Product);
            distToCenter = UAM_FlightMode_norm_pv(rtb_Sum);
            tmp_0 = _mm_set_pd(distToCenter, h);
            _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(rtb_Sum[0],
              rtb_Product[0]), tmp_0));
            rtb_Product[0] = tmp[0];
            rtb_Sum[0] = tmp[1];
            _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(rtb_Sum[1],
              rtb_Product[1]), tmp_0));
            rtb_Sum[1] = tmp[1];
            turnVector[2] = rtb_Product[0] * tmp[1] - rtb_Sum[0] * tmp[0];
            localDW->obj_l.NumCircles += rt_atan2d_snf(turnVector[2],
              (rtb_Product[0] * rtb_Sum[0] + tmp[0] * tmp[1]) + 0.0 / h * (0.0 /
              distToCenter)) / 2.0 / 3.1415926535897931;
            distToCenter = localDW->obj_l.NumCircles;
            localDW->obj_l.PrevPosition[0] = rtu_Pose[0];
            localDW->obj_l.PrevPosition[1] = rtu_Pose[1];
            localDW->obj_l.PrevPosition[2] = rtu_Pose[2];
            tmp_0 = _mm_sub_pd(_mm_set_pd(t, b_tmp[0]), _mm_loadu_pd(&rtu_Pose[0]));
            _mm_storeu_pd(&rtb_Product[0], tmp_0);
            tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtu_ToWP->position[0]),
                               _mm_loadu_pd(&rtu_Pose[0]));
            _mm_storeu_pd(&rtb_Sum[0], tmp_0);
            if (rtIsNaN(absxk)) {
              h = (rtNaN);
            } else if (absxk < 0.0) {
              h = -1.0;
            } else {
              h = (absxk > 0.0);
            }

            switch ((int32_T)h) {
             case 1:
              if (rtb_Product[0] * rtb_Sum[1] - rtb_Sum[0] * rtb_Product[1] >
                  0.0) {
                xyLookaheadPoint[0] = b_tmp[0];
                xyLookaheadPoint[1] = t;
              } else {
                xyLookaheadPoint[0] = b_tmp[1];
                xyLookaheadPoint[1] = y2;
              }
              break;

             case -1:
              if (rtb_Product[0] * rtb_Sum[1] - rtb_Sum[0] * rtb_Product[1] <
                  0.0) {
                xyLookaheadPoint[0] = b_tmp[0];
                xyLookaheadPoint[1] = t;
              } else {
                xyLookaheadPoint[0] = b_tmp[1];
                xyLookaheadPoint[1] = y2;
              }
              break;

             default:
              p = (fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(t - rtu_Pose[1],
                      b_tmp[0] - rtu_Pose[0]), rtu_Pose[3])) < fabs
                   (UAM_FlightMode_angdiff(rt_atan2d_snf(y2 - rtu_Pose[1],
                      b_tmp[1] - rtu_Pose[0]), rtu_Pose[3])));
              if (p) {
                xyLookaheadPoint[0] = b_tmp[0];
                xyLookaheadPoint[1] = t;
              } else {
                xyLookaheadPoint[0] = b_tmp[1];
                xyLookaheadPoint[1] = y2;
              }

              tmp_0 = _mm_sub_pd(_mm_loadu_pd(&xyLookaheadPoint[0]),
                                 _mm_loadu_pd(&rtu_Pose[0]));
              _mm_storeu_pd(&rtb_Product[0], tmp_0);
              if (rtb_Product[0] * rtb_Sum[1] - rtb_Sum[0] * rtb_Product[1] >
                  0.0) {
                localDW->obj_l.TurnDirectionInternal = 1.0;
              } else {
                localDW->obj_l.TurnDirectionInternal = -1.0;
              }

              localDW->obj_l.SelectTurnDirectionFlag = false;
              break;
            }

            absxk = rt_atan2d_snf(b_tmp_0, rtb_Abs);
          }
        }

        if (guard2) {
          _mm_storeu_pd(&xyLookaheadPoint[0], _mm_add_pd(_mm_mul_pd(_mm_div_pd
            (_mm_set_pd(b_tmp_0, b_tmp[0]), _mm_set1_pd(UAM_FlightMode_norm_p
            (b_tmp))), _mm_set1_pd(rtb_Abs)), _mm_loadu_pd(&rtu_ToWP->position[0])));
          absxk = rt_atan2d_snf(xyLookaheadPoint[1] - rtb_DotProduct,
                                xyLookaheadPoint[0] - turnInternal);
          distToCenter = localDW->obj_l.NumCircles;
        }

        rtb_Product[0] = xyLookaheadPoint[0];
        rtb_Product[1] = xyLookaheadPoint[1];
        rtb_Product[2] = rtu_ToWP->position[2];
        turnInternal = rt_atan2d_snf(xyLookaheadPoint[1] - rtb_DotProduct,
          xyLookaheadPoint[0] - turnInternal);
      }

      /* BusCreator: '<S10>/Bus Creator' incorporates:
       *  MATLABSystem: '<S10>/UAV Orbit Follower'
       *  Merge: '<S3>/ Merge 2'
       * */
      localB->InnerLoopCmds.LAP[0] = rtb_Product[0];
      localB->InnerLoopCmds.LAP[1] = rtb_Product[1];
      localB->InnerLoopCmds.LAP[2] = rtb_Product[2];
      localB->InnerLoopCmds.HeadingCmd = turnInternal;
      localB->InnerLoopCmds.YawCmd = absxk;

      /* Merge: '<S3>/ Merge ' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       *  DataTypeConversion: '<S10>/Data Type Conversion'
       *  MATLABSystem: '<S10>/UAV Orbit Follower'
       *  RelationalOperator: '<S10>/Relational Operator'
       * */
      localB->Status = (uint8_T)(distToCenter > rtu_ToWP->params[2]);
      break;

     case UAM_FlightMode_IN_PreTransition:
      if (localDW->temporalCounter_i1 >= 63) {
        localB->FlightMode = BackTransition;
        localB->controlMode_m.inTransition = 1U;
        localDW->is_GuidanceLogic = UAM_FlightMod_IN_BackTransition;
        localDW->is_BackTransition = UAM_FlightMod_IN_BackTransition;

        /* BusCreator: '<S4>/Bus Creator1' incorporates:
         *  Constant: '<S4>/Constant1'
         *  Constant: '<S4>/Constant2'
         *  Merge: '<S3>/ Merge 2'
         */
        localB->InnerLoopCmds.HeadingCmd = 0.0;
        localB->InnerLoopCmds.YawCmd = 0.0;

        /* MATLAB Function: '<S4>/MATLAB Function' */
        rtb_DotProduct = 3.3121686421112381E-170;

        /* BusCreator: '<S4>/Bus Creator1' incorporates:
         *  Chart: '<Root>/Guidance Mode Selector'
         *  Merge: '<S3>/ Merge 2'
         */
        localB->InnerLoopCmds.LAP[0] = rtu_States->Xe[0];

        /* MATLAB Function: '<S4>/MATLAB Function' incorporates:
         *  Chart: '<Root>/Guidance Mode Selector'
         */
        absxk = fabs(rtu_States->Ve[0]);
        if (absxk > 3.3121686421112381E-170) {
          turnInternal = 1.0;
          rtb_DotProduct = absxk;
        } else {
          t = absxk / 3.3121686421112381E-170;
          turnInternal = t * t;
        }

        /* BusCreator: '<S4>/Bus Creator1' incorporates:
         *  Chart: '<Root>/Guidance Mode Selector'
         *  Merge: '<S3>/ Merge 2'
         */
        localB->InnerLoopCmds.LAP[1] = rtu_States->Xe[1];
        localB->InnerLoopCmds.LAP[2] = rtu_States->Xe[2];

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Chart: '<Root>/Guidance Mode Selector'
         *  DataTypeConversion: '<S4>/Data Type Conversion'
         *  MATLAB Function: '<S4>/MATLAB Function'
         */
        localB->Status = (uint8_T)((fabs(rtu_States->c1) < 0.1) &&
          (rtb_DotProduct * sqrt(turnInternal) < 4.0));
      }
      break;

     case UAM_FlightMode_IN_Start:
      /* Chart: '<Root>/Guidance Mode Selector' */
      UAM_FlightMode_Start_d(rtu_mode, rtu_ToWP, rtu_FromWP, rtu_Pose,
        rtu_States, localB, localDW);
      break;

     case UAM_FlightMode_IN_Takeoff:
      /* Chart: '<Root>/Guidance Mode Selector' incorporates:
       *  Sum: '<S18>/Sum'
       */
      if (rtu_Pose[2] <= rtu_ToWP->position[2]) {
        localDW->is_GuidanceLogic = UAM_FlightMode_IN_WP;

        /* Switch: '<S17>/Switch' incorporates:
         *  Product: '<S21>/Product'
         *  RelationalOperator: '<S17>/Equal'
         */
        if (rtu_FromWP->mode == 6) {
          rtb_Product[0] = rtu_FromWP->position[0];
          rtb_Product[1] = rtu_FromWP->position[1];
          rtb_Product[2] = rtu_FromWP->position[2];
        } else {
          rtb_Product[0] = 0.0;
          rtb_Product[1] = 0.0;
          rtb_Product[2] = 0.0;
        }

        /* MATLAB Function: '<S12>/MATLAB Function' incorporates:
         *  BusCreator generated from: '<S12>/MATLAB Function'
         *  Product: '<S21>/Product'
         */
        rtb_wps[0] = rtb_Product[0];
        rtb_wps[2] = rtb_Product[1];
        rtb_wps[4] = rtb_Product[2];
        rtb_wps[6] = rtu_FromWP->params[3];
        rtb_wps[1] = rtu_ToWP->position[0];
        rtb_wps[3] = rtu_ToWP->position[1];
        rtb_wps[5] = rtu_ToWP->position[2];
        rtb_wps[7] = rtu_ToWP->params[3];

        /* MATLABSystem: '<S12>/Waypoint Follower' incorporates:
         *  Constant: '<S12>/Lookahead Distance'
         */
        UAM_FlightMode_SystemCore_step(&localDW->obj, rtu_Pose, rtb_wps, 5.0,
          localB->InnerLoopCmds.LAP, &turnInternal, &rtb_DotProduct,
          &b_varargout_4);

        /* BusCreator: '<S12>/Bus Creator' incorporates:
         *  Constant: '<S12>/Constant'
         *  MATLABSystem: '<S12>/Waypoint Follower'
         *  Merge: '<S3>/ Merge 2'
         * */
        localB->InnerLoopCmds.HeadingCmd = turnInternal;
        localB->InnerLoopCmds.YawCmd = 0.0;

        /* Sum: '<S18>/Sum1' incorporates:
         *  Product: '<S21>/Product'
         */
        turnInternal = rtu_ToWP->position[0] - rtb_Product[0];
        rtb_Product[0] = turnInternal;

        /* DotProduct: '<S21>/Dot Product1' incorporates:
         *  Product: '<S21>/Product'
         */
        rtb_DotProduct = turnInternal * turnInternal;

        /* Sum: '<S18>/Sum1' incorporates:
         *  Product: '<S21>/Product'
         */
        turnInternal = rtu_ToWP->position[1] - rtb_Product[1];
        rtb_Product[1] = turnInternal;

        /* DotProduct: '<S21>/Dot Product1' incorporates:
         *  Product: '<S21>/Product'
         */
        rtb_DotProduct += turnInternal * turnInternal;

        /* Sum: '<S18>/Sum1' incorporates:
         *  Product: '<S21>/Product'
         */
        turnInternal = rtu_ToWP->position[2] - rtb_Product[2];

        /* DotProduct: '<S21>/Dot Product1' incorporates:
         *  Product: '<S21>/Product'
         */
        rtb_DotProduct += turnInternal * turnInternal;

        /* Saturate: '<S21>/Saturation' incorporates:
         *  DotProduct: '<S21>/Dot Product1'
         */
        if (rtb_DotProduct <= 1.0E-5) {
          rtb_DotProduct = 1.0E-5;
        }

        /* Sqrt: '<S21>/Sqrt' incorporates:
         *  Saturate: '<S21>/Saturation'
         */
        rtb_DotProduct = sqrt(rtb_DotProduct);
        tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtu_ToWP->position[0]), _mm_loadu_pd
                           (&rtu_Pose[0]));
        _mm_storeu_pd(&rtb_Sum[0], tmp_0);

        /* Sum: '<S18>/Sum' */
        rtb_Sum[2] = rtu_ToWP->position[2] - rtu_Pose[2];

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S12>/Lookahead Distance'
         *  DataTypeConversion: '<S12>/Data Type Conversion'
         *  DotProduct: '<S18>/Dot Product'
         *  Product: '<S21>/Product'
         *  RelationalOperator: '<S12>/Relational Operator'
         *  Sum: '<S18>/Sum'
         *  Sum: '<S18>/Sum1'
         */
        localB->Status = (uint8_T)((rtb_Product[0] / rtb_DotProduct * rtb_Sum[0]
          + rtb_Product[1] / rtb_DotProduct * rtb_Sum[1]) + turnInternal /
          rtb_DotProduct * rtb_Sum[2] <= 5.0);
      } else {
        /* SignalConversion generated from: '<S11>/Bus Creator' */
        localB->InnerLoopCmds.LAP[0] = rtu_Pose[0];
        localB->InnerLoopCmds.LAP[1] = rtu_Pose[1];
        localB->InnerLoopCmds.LAP[2] = rtu_ToWP->position[2];

        /* Saturate: '<S11>/Hdg. Cmd Sat' */
        if (rtu_ToWP->params[3] > 3.1415926535897931) {
          /* BusCreator: '<S11>/Bus Creator' incorporates:
           *  Merge: '<S3>/ Merge 2'
           */
          localB->InnerLoopCmds.HeadingCmd = 3.1415926535897931;
        } else if (rtu_ToWP->params[3] < -3.1415926535897931) {
          /* BusCreator: '<S11>/Bus Creator' incorporates:
           *  Merge: '<S3>/ Merge 2'
           */
          localB->InnerLoopCmds.HeadingCmd = -3.1415926535897931;
        } else {
          /* BusCreator: '<S11>/Bus Creator' incorporates:
           *  Merge: '<S3>/ Merge 2'
           */
          localB->InnerLoopCmds.HeadingCmd = rtu_ToWP->params[3];
        }

        /* BusCreator: '<S11>/Bus Creator' incorporates:
         *  Merge: '<S3>/ Merge 2'
         */
        localB->InnerLoopCmds.YawCmd = UAM_FlightMode_ConstB.YawCmd;

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S11>/Constant'
         *  SignalConversion generated from: '<S11>/Status'
         */
        localB->Status = 0U;
      }
      break;

     default:
      /* Switch: '<S17>/Switch' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       *  Product: '<S21>/Product'
       *  RelationalOperator: '<S17>/Equal'
       */
      /* case IN_WP: */
      if (rtu_FromWP->mode == 6) {
        rtb_Product[0] = rtu_FromWP->position[0];
        rtb_Product[1] = rtu_FromWP->position[1];
        rtb_Product[2] = rtu_FromWP->position[2];
      } else {
        rtb_Product[0] = 0.0;
        rtb_Product[1] = 0.0;
        rtb_Product[2] = 0.0;
      }

      /* MATLAB Function: '<S12>/MATLAB Function' incorporates:
       *  BusCreator generated from: '<S12>/MATLAB Function'
       *  Chart: '<Root>/Guidance Mode Selector'
       *  Product: '<S21>/Product'
       */
      rtb_wps[0] = rtb_Product[0];
      rtb_wps[2] = rtb_Product[1];
      rtb_wps[4] = rtb_Product[2];
      rtb_wps[6] = rtu_FromWP->params[3];
      rtb_wps[1] = rtu_ToWP->position[0];
      rtb_wps[3] = rtu_ToWP->position[1];
      rtb_wps[5] = rtu_ToWP->position[2];
      rtb_wps[7] = rtu_ToWP->params[3];

      /* MATLABSystem: '<S12>/Waypoint Follower' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       *  Constant: '<S12>/Lookahead Distance'
       */
      UAM_FlightMode_SystemCore_step(&localDW->obj, rtu_Pose, rtb_wps, 5.0,
        localB->InnerLoopCmds.LAP, &turnInternal, &rtb_DotProduct,
        &b_varargout_4);

      /* BusCreator: '<S12>/Bus Creator' incorporates:
       *  Constant: '<S12>/Constant'
       *  MATLABSystem: '<S12>/Waypoint Follower'
       *  Merge: '<S3>/ Merge 2'
       * */
      localB->InnerLoopCmds.HeadingCmd = turnInternal;
      localB->InnerLoopCmds.YawCmd = 0.0;

      /* Sum: '<S18>/Sum1' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       *  Product: '<S21>/Product'
       */
      turnInternal = rtu_ToWP->position[0] - rtb_Product[0];
      rtb_Product[0] = turnInternal;

      /* DotProduct: '<S21>/Dot Product1' incorporates:
       *  Product: '<S21>/Product'
       */
      rtb_DotProduct = turnInternal * turnInternal;

      /* Sum: '<S18>/Sum1' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       *  Product: '<S21>/Product'
       */
      turnInternal = rtu_ToWP->position[1] - rtb_Product[1];
      rtb_Product[1] = turnInternal;

      /* DotProduct: '<S21>/Dot Product1' incorporates:
       *  Product: '<S21>/Product'
       */
      rtb_DotProduct += turnInternal * turnInternal;

      /* Sum: '<S18>/Sum1' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       *  Product: '<S21>/Product'
       */
      turnInternal = rtu_ToWP->position[2] - rtb_Product[2];

      /* DotProduct: '<S21>/Dot Product1' incorporates:
       *  Product: '<S21>/Product'
       */
      rtb_DotProduct += turnInternal * turnInternal;

      /* Saturate: '<S21>/Saturation' incorporates:
       *  DotProduct: '<S21>/Dot Product1'
       */
      if (rtb_DotProduct <= 1.0E-5) {
        rtb_DotProduct = 1.0E-5;
      }

      /* Sqrt: '<S21>/Sqrt' incorporates:
       *  Saturate: '<S21>/Saturation'
       */
      rtb_DotProduct = sqrt(rtb_DotProduct);

      /* Sum: '<S18>/Sum' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       */
      tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtu_ToWP->position[0]), _mm_loadu_pd
                         (&rtu_Pose[0]));
      _mm_storeu_pd(&rtb_Sum[0], tmp_0);
      rtb_Sum[2] = rtu_ToWP->position[2] - rtu_Pose[2];

      /* Merge: '<S3>/ Merge ' incorporates:
       *  Constant: '<S12>/Lookahead Distance'
       *  DataTypeConversion: '<S12>/Data Type Conversion'
       *  DotProduct: '<S18>/Dot Product'
       *  Product: '<S21>/Product'
       *  RelationalOperator: '<S12>/Relational Operator'
       *  Sum: '<S18>/Sum'
       *  Sum: '<S18>/Sum1'
       */
      localB->Status = (uint8_T)((rtb_Product[0] / rtb_DotProduct * rtb_Sum[0] +
        rtb_Product[1] / rtb_DotProduct * rtb_Sum[1]) + turnInternal /
        rtb_DotProduct * rtb_Sum[2] <= 5.0);
      break;
    }
  }
}

/* System initialize for referenced model: 'UAM_FlightMode' */
void UAM_FlightMode_Init(DW_UAM_FlightMode_f_T *localDW)
{
  int32_T i;

  /* SystemInitialize for IfAction SubSystem: '<S3>/GuidanceLogic.FIXED_WING_WAYPOINT.WAYPOINT' */
  /* Start for MATLABSystem: '<S6>/Waypoint Follower' incorporates:
   *  Chart: '<Root>/Guidance Mode Selector'
   */
  localDW->obj_f.LastWaypointFlag = false;
  localDW->obj_f.StartFlag = true;
  localDW->obj_f.LookaheadFactor = 1.01;
  localDW->objisempty_m = true;
  localDW->obj_f.isInitialized = 1;
  localDW->obj_f.NumWaypoints = 0.0;

  /* InitializeConditions for MATLABSystem: '<S6>/Waypoint Follower' incorporates:
   *  Chart: '<Root>/Guidance Mode Selector'
   */
  localDW->obj_f.WaypointIndex = 1.0;
  for (i = 0; i < 6; i++) {
    /* InitializeConditions for MATLABSystem: '<S6>/Waypoint Follower' incorporates:
     *  Chart: '<Root>/Guidance Mode Selector'
     */
    localDW->obj_f.WaypointsInternal[i] = 0.0;
  }

  /* End of SystemInitialize for SubSystem: '<S3>/GuidanceLogic.FIXED_WING_WAYPOINT.WAYPOINT' */

  /* SystemInitialize for IfAction SubSystem: '<S3>/GuidanceLogic.FIXED_WING_ORBIT.ORBIT' */
  /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
   *  Chart: '<Root>/Guidance Mode Selector'
   */
  localDW->objisempty_b = true;
  localDW->obj_b.isInitialized = 1;

  /* InitializeConditions for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
   *  Chart: '<Root>/Guidance Mode Selector'
   */
  localDW->obj_b.OrbitRadiusInternal = 0.0;
  localDW->obj_b.PrevResetSignal = 0.0;
  localDW->obj_b.NumCircles = 0.0;
  localDW->obj_b.OrbitCenterInternal[0] = 0.0;
  localDW->obj_b.PrevPosition[0] = 0.0;
  localDW->obj_b.OrbitCenterInternal[1] = 0.0;
  localDW->obj_b.PrevPosition[1] = 0.0;
  localDW->obj_b.OrbitCenterInternal[2] = 0.0;
  localDW->obj_b.PrevPosition[2] = 0.0;
  localDW->obj_b.StartFlag = true;
  localDW->obj_b.SelectTurnDirectionFlag = true;
  localDW->obj_b.TurnDirectionInternal = 1.0;
  localDW->obj_b.OrbitRadiusFlag = 0U;
  localDW->obj_b.LookaheadDistFlag = 0U;

  /* End of SystemInitialize for SubSystem: '<S3>/GuidanceLogic.FIXED_WING_ORBIT.ORBIT' */

  /* SystemInitialize for IfAction SubSystem: '<S3>/GuidanceLogic.WP' */
  /* Start for MATLABSystem: '<S12>/Waypoint Follower' incorporates:
   *  Chart: '<Root>/Guidance Mode Selector'
   */
  localDW->obj.LastWaypointFlag = false;
  localDW->obj.StartFlag = true;
  localDW->obj.LookaheadFactor = 1.01;
  localDW->obj.SearchFlag = true;
  localDW->objisempty = true;
  localDW->obj.isInitialized = 1;
  localDW->obj.NumWaypoints = 0.0;

  /* InitializeConditions for MATLABSystem: '<S12>/Waypoint Follower' incorporates:
   *  Chart: '<Root>/Guidance Mode Selector'
   */
  localDW->obj.WaypointIndex = 1.0;
  memset(&localDW->obj.WaypointsInternal[0], 0, sizeof(real_T) << 3U);

  /* End of SystemInitialize for SubSystem: '<S3>/GuidanceLogic.WP' */

  /* SystemInitialize for IfAction SubSystem: '<S3>/GuidanceLogic.Orbit' */
  /* Start for MATLABSystem: '<S10>/UAV Orbit Follower' incorporates:
   *  Chart: '<Root>/Guidance Mode Selector'
   */
  localDW->objisempty_n = true;
  localDW->obj_l.isInitialized = 1;

  /* InitializeConditions for MATLABSystem: '<S10>/UAV Orbit Follower' incorporates:
   *  Chart: '<Root>/Guidance Mode Selector'
   */
  localDW->obj_l.OrbitRadiusInternal = 0.0;
  localDW->obj_l.PrevResetSignal = 0.0;
  localDW->obj_l.NumCircles = 0.0;
  localDW->obj_l.OrbitCenterInternal[0] = 0.0;
  localDW->obj_l.PrevPosition[0] = 0.0;
  localDW->obj_l.OrbitCenterInternal[1] = 0.0;
  localDW->obj_l.PrevPosition[1] = 0.0;
  localDW->obj_l.OrbitCenterInternal[2] = 0.0;
  localDW->obj_l.PrevPosition[2] = 0.0;
  localDW->obj_l.StartFlag = true;
  localDW->obj_l.SelectTurnDirectionFlag = true;
  localDW->obj_l.TurnDirectionInternal = 1.0;
  localDW->obj_l.OrbitRadiusFlag = 0U;
  localDW->obj_l.LookaheadDistFlag = 0U;

  /* End of SystemInitialize for SubSystem: '<S3>/GuidanceLogic.Orbit' */

  /* SystemInitialize for IfAction SubSystem: '<S3>/GuidanceLogic.Land.Descend' */
  /* SystemInitialize for Chart: '<Root>/Guidance Mode Selector' */
  UAM_Fligh_WaypointFollower_Init(&localDW->WaypointFollower_c);

  /* End of SystemInitialize for SubSystem: '<S3>/GuidanceLogic.Land.Descend' */

  /* SystemInitialize for IfAction SubSystem: '<S3>/GuidanceLogic.Land.ToLand' */
  UAM_Fligh_WaypointFollower_Init(&localDW->WaypointFollower_c2);

  /* End of SystemInitialize for SubSystem: '<S3>/GuidanceLogic.Land.ToLand' */
}

/* Output and update for referenced model: 'UAM_FlightMode' */
void UAM_FlightMode(RT_MODEL_UAM_FlightMode_T * const UAM_FlightMode_M, const
                    uint8_T *rtu_mode, const UAVPathManagerBus *rtu_ToWP, const
                    UAVPathManagerBus *rtu_FromWP, const real_T rtu_Pose[4],
                    const GuidanceStates *rtu_States, const real_T *rtu_Ground,
                    real_T *rty_HoverSP_X, real_T *rty_HoverSP_Y, real_T
                    *rty_HoverSP_Z, real_T *rty_HoverSP_Yaw, real_T
                    *rty_aacSP_airspeed, real_T *rty_aacSP_altitude, real_T
                    *rty_aacSP_course, real_T *rty_aacSP_L1, real_T
                    *rty_aacSP_climbrate, real_T *rty_FixedWingSP_roll, real_T
                    *rty_FixedWingSP_pitch, real_T *rty_FixedWingSP_yaw, real_T *
                    rty_FixedWingSP_airspeed, uint8_T
                    *rty_controlMode_lateralGuidance, uint8_T
                    *rty_controlMode_airspeedAltitud, uint8_T
                    *rty_controlMode_attitude, uint8_T *rty_controlMode_manual,
                    uint8_T *rty_controlMode_armed, uint8_T
                    *rty_controlMode_inTransition, uint8_T
                    *rty_controlMode_TransitionCondi, flightState
                    *rty_FlightMode, uint8_T *rty_Status, B_UAM_FlightMode_c_T
                    *localB, DW_UAM_FlightMode_f_T *localDW)
{
  /* Chart: '<Root>/Guidance Mode Selector' incorporates:
   *  Constant: '<Root>/Constant2'
   *  Merge: '<S3>/ Merge 2'
   */
  if (localDW->temporalCounter_i1 < 63) {
    localDW->temporalCounter_i1++;
  }

  localDW->mode_prev = localDW->mode_start;
  localDW->mode_start = *rtu_mode;
  if (localDW->is_active_c14_UAM_FlightMode == 0) {
    localDW->mode_prev = *rtu_mode;
    localDW->is_active_c14_UAM_FlightMode = 1U;
    localDW->is_GuidanceLogic = UAM_FlightMode_IN_Start;
    localB->FlightMode = Hover;
    localB->controlMode_m.airspeedAltitude = 0U;
    localB->controlMode_m.attitude = 0U;
    localB->controlMode_m.lateralGuidance = 0U;
    localB->controlMode_m.TransitionCondition = 0U;
    localB->FixedWingSP.yaw = 0.0;
    localB->FixedWingSP.pitch = 0.0;
    localB->FixedWingSP.roll = 0.0;
    localB->FixedWingSP.airspeed = 14.0;
    localB->aacSP.L1 = 25.0;
    localB->aacSP.airspeed = 14.0;
    localB->aacSP.altitude = 0.0;
    localB->aacSP.course = 0.0;
    localB->InnerLoopCmds.YawCmd = 0.0;
  } else {
    UAM_FlightMode_GuidanceLogic(rtu_mode, rtu_ToWP, rtu_FromWP, rtu_Pose,
      rtu_States, rtu_Ground, localB, localDW);
  }

  /* End of Chart: '<Root>/Guidance Mode Selector' */

  /* Stop: '<Root>/Stop Simulation' incorporates:
   *  Constant: '<S1>/Constant'
   *  RelationalOperator: '<S1>/Compare'
   */
  if (localB->Status == 2) {
    rtmSetStopRequested(UAM_FlightMode_M, 1);
  }

  /* End of Stop: '<Root>/Stop Simulation' */

  /* RelationalOperator: '<S2>/Compare' incorporates:
   *  Constant: '<S2>/Constant'
   */
  *rty_Status = (uint8_T)(localB->Status == 1);

  /* RateTransition generated from: '<Root>/Rate Transition' */
  *rty_HoverSP_X = localB->InnerLoopCmds.LAP[0];

  /* RateTransition generated from: '<Root>/Rate Transition' */
  *rty_HoverSP_Y = localB->InnerLoopCmds.LAP[1];

  /* RateTransition generated from: '<Root>/Rate Transition' */
  *rty_HoverSP_Z = localB->InnerLoopCmds.LAP[2];

  /* RateTransition generated from: '<Root>/Rate Transition' */
  *rty_HoverSP_Yaw = localB->InnerLoopCmds.YawCmd;

  /* RateTransition: '<Root>/Rate Transition4' */
  *rty_FlightMode = localB->FlightMode;

  /* SignalConversion generated from: '<Root>/controlMode' incorporates:
   *  RateTransition: '<Root>/Rate Transition3'
   */
  *rty_controlMode_lateralGuidance = localB->controlMode_m.lateralGuidance;

  /* SignalConversion generated from: '<Root>/controlMode' incorporates:
   *  RateTransition: '<Root>/Rate Transition3'
   */
  *rty_controlMode_airspeedAltitud = localB->controlMode_m.airspeedAltitude;

  /* SignalConversion generated from: '<Root>/controlMode' incorporates:
   *  RateTransition: '<Root>/Rate Transition3'
   */
  *rty_controlMode_attitude = localB->controlMode_m.attitude;

  /* SignalConversion generated from: '<Root>/controlMode' incorporates:
   *  RateTransition: '<Root>/Rate Transition3'
   */
  *rty_controlMode_manual = localB->controlMode_m.manual;

  /* SignalConversion generated from: '<Root>/controlMode' incorporates:
   *  RateTransition: '<Root>/Rate Transition3'
   */
  *rty_controlMode_armed = localB->controlMode_m.armed;

  /* SignalConversion generated from: '<Root>/controlMode' incorporates:
   *  RateTransition: '<Root>/Rate Transition3'
   */
  *rty_controlMode_inTransition = localB->controlMode_m.inTransition;

  /* SignalConversion generated from: '<Root>/controlMode' incorporates:
   *  RateTransition: '<Root>/Rate Transition3'
   */
  *rty_controlMode_TransitionCondi = localB->controlMode_m.TransitionCondition;

  /* SignalConversion generated from: '<Root>/FixedWingSP' incorporates:
   *  RateTransition: '<Root>/Rate Transition2'
   */
  *rty_FixedWingSP_roll = localB->FixedWingSP.roll;

  /* SignalConversion generated from: '<Root>/FixedWingSP' incorporates:
   *  RateTransition: '<Root>/Rate Transition2'
   */
  *rty_FixedWingSP_pitch = localB->FixedWingSP.pitch;

  /* SignalConversion generated from: '<Root>/FixedWingSP' incorporates:
   *  RateTransition: '<Root>/Rate Transition2'
   */
  *rty_FixedWingSP_yaw = localB->FixedWingSP.yaw;

  /* SignalConversion generated from: '<Root>/FixedWingSP' incorporates:
   *  RateTransition: '<Root>/Rate Transition2'
   */
  *rty_FixedWingSP_airspeed = localB->FixedWingSP.airspeed;

  /* SignalConversion generated from: '<Root>/aacSP' incorporates:
   *  RateTransition: '<Root>/Rate Transition1'
   */
  *rty_aacSP_airspeed = localB->aacSP.airspeed;

  /* SignalConversion generated from: '<Root>/aacSP' incorporates:
   *  RateTransition: '<Root>/Rate Transition1'
   */
  *rty_aacSP_altitude = localB->aacSP.altitude;

  /* SignalConversion generated from: '<Root>/aacSP' incorporates:
   *  RateTransition: '<Root>/Rate Transition1'
   */
  *rty_aacSP_course = localB->aacSP.course;

  /* SignalConversion generated from: '<Root>/aacSP' incorporates:
   *  RateTransition: '<Root>/Rate Transition1'
   */
  *rty_aacSP_L1 = localB->aacSP.L1;

  /* SignalConversion generated from: '<Root>/aacSP' incorporates:
   *  RateTransition: '<Root>/Rate Transition1'
   */
  *rty_aacSP_climbrate = localB->aacSP.climbrate;
}

/* Model initialize function */
void UAM_FlightMode_initialize(const char_T **rt_errorStatus, boolean_T
  *rt_stopRequested, RT_MODEL_UAM_FlightMode_T *const UAM_FlightMode_M,
  ZCE_UAM_FlightMode_T *localZCE)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize error status */
  rtmSetErrorStatusPointer(UAM_FlightMode_M, rt_errorStatus);

  /* initialize stop requested flag */
  rtmSetStopRequestedPtr(UAM_FlightMode_M, rt_stopRequested);
  localZCE->TriggeredSubsystem_Trig_ZCE = POS_ZCSIG;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
