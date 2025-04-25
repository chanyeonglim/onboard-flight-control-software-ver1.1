/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_FlightMode.c
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

#include "UAM_FlightMode.h"
#include "rtwtypes.h"
#include "UAM_FlightMode_types.h"
#include "UAM_FlightMode_private.h"
#include <string.h>
#include <emmintrin.h>
#include "rt_nonfinite.h"
#include <math.h>
#include "rt_defines.h"

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
#define UAM_FlightMode_IN_Orbit        ((uint8_T)9U)
#define UAM_FlightMode_IN_PreTransition ((uint8_T)10U)
#define UAM_FlightMode_IN_Stabilize    ((uint8_T)2U)
#define UAM_FlightMode_IN_Start        ((uint8_T)11U)
#define UAM_FlightMode_IN_Takeoff      ((uint8_T)12U)
#define UAM_FlightMode_IN_ToLand       ((uint8_T)2U)
#define UAM_FlightMode_IN_WP           ((uint8_T)13U)
#define UAM_Flight_IN_ForwardTransition ((uint8_T)6U)

/* Block signals (default storage) */
B_UAM_FlightMode_T UAM_FlightMode_B;

/* Block states (default storage) */
DW_UAM_FlightMode_T UAM_FlightMode_DW;

/* Previous zero-crossings (trigger) states */
PrevZCX_UAM_FlightMode_T UAM_FlightMode_PrevZCX;

/* External inputs (root inport signals with default storage) */
ExtU_UAM_FlightMode_T UAM_FlightMode_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_UAM_FlightMode_T UAM_FlightMode_Y;

/* Real-time model */
static RT_MODEL_UAM_FlightMode_T UAM_FlightMode_M_;
RT_MODEL_UAM_FlightMode_T *const UAM_FlightMode_M = &UAM_FlightMode_M_;

/* Forward declaration for local functions */
static real_T UAM_FlightMode_norm(const real_T x[3]);
static real_T UAM_FlightMode_wrapToPi(real_T theta);

/* Forward declaration for local functions */
static real_T UAM_FlightMode_norm_de(const real_T x[3]);
static real_T UAM_FlightMode_wrapToPi_d(real_T theta);
static void UAM_FlightMode_SystemCore_step(uav_sluav_internal_system__de_T *obj,
  const real_T varargin_1[4], const real_T varargin_2[8], real_T varargin_3,
  real_T varargout_1[3], real_T *varargout_2, real_T *varargout_3, uint8_T
  *varargout_4);
static real_T UAM_FlightMode_norm_d(const real_T x[2]);
static real_T UAM_FlightMode_angdiff(real_T x, real_T y);
static void UAM_FlightMode_FIXED_WING_ENTRY(void);
static boolean_T UAM_Flig_transitionConditionMet(void);
static void enter_internal_FIXED_WING_ENTRY(void);
static void UAM_FlightMode_Start(real_T rtb_wps_l_0[8]);
static void UAM_exit_internal_GuidanceLogic(void);
static void UAM_FlightMode_GuidanceLogic(const uint8_T *mode_prev);
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

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T tmp;
  int32_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp, tmp_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
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

static real_T UAM_FlightMode_norm_de(const real_T x[3])
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

static real_T UAM_FlightMode_wrapToPi_d(real_T theta)
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

static void UAM_FlightMode_SystemCore_step(uav_sluav_internal_system__de_T *obj,
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
      if (UAM_FlightMode_norm_de(b_waypointsIn) < 1.4901161193847656E-8) {
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
      y = UAM_FlightMode_norm_de(b_waypointsIn);
      guard2 = false;
      if (y <= 1.0) {
        guard2 = true;
      } else {
        _mm_storeu_pd(&unitVectorUtoV_tmp[0], _mm_sub_pd(_mm_set_pd
          (b_endWaypoint_idx_1_tmp, b_endWaypoint_idx_0_tmp), _mm_set_pd
          (b_startWaypoint_idx_1_tmp, lookaheadDist_tmp)));
        unitVectorUtoV_tmp[2] = b_endWaypoint_idx_2_tmp -
          b_startWaypoint_idx_2_tmp;
        currentPosition = UAM_FlightMode_norm_de(unitVectorUtoV_tmp);
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
        dist = UAM_FlightMode_norm_de(unitVectorUtoV_tmp);
      } else if (unitVectorUtoV_tmp_0 > 1.0) {
        _mm_storeu_pd(&varargin_1_0[0], _mm_sub_pd(_mm_loadu_pd(&varargin_1[0]),
          _mm_set_pd(b_endWaypoint_idx_1_tmp, b_endWaypoint_idx_0_tmp)));
        varargin_1_0[2] = varargin_1[2] - b_endWaypoint_idx_2_tmp;
        dist = UAM_FlightMode_norm_de(varargin_1_0);
      } else {
        tmp = _mm_sub_pd(_mm_loadu_pd(&varargin_1[0]), _mm_add_pd(_mm_mul_pd
          (_mm_set1_pd(unitVectorUtoV_tmp_0), _mm_loadu_pd(&b_waypointsIn[0])),
          _mm_set_pd(b_startWaypoint_idx_1_tmp, lookaheadDist_tmp)));
        _mm_storeu_pd(&varargin_1_0[0], tmp);
        varargin_1_0[2] = varargin_1[2] - (unitVectorUtoV_tmp_0 *
          currentPosition + b_startWaypoint_idx_2_tmp);
        dist = UAM_FlightMode_norm_de(varargin_1_0);
      }

      if (obj->LastWaypointFlag) {
        tmp = _mm_sub_pd(_mm_loadu_pd(&varargin_1[0]), _mm_add_pd(_mm_mul_pd
          (_mm_set1_pd(unitVectorUtoV_tmp_0), _mm_loadu_pd(&b_waypointsIn[0])),
          _mm_set_pd(b_startWaypoint_idx_1_tmp, lookaheadDist_tmp)));
        _mm_storeu_pd(&varargin_1_0[0], tmp);
        varargin_1_0[2] = varargin_1[2] - (unitVectorUtoV_tmp_0 *
          currentPosition + b_startWaypoint_idx_2_tmp);
        dist = UAM_FlightMode_norm_de(varargin_1_0);
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
      *varargout_3 = UAM_FlightMode_wrapToPi_d(UAM_FlightMode_norm_de
        (unitVectorUtoV_tmp) / UAM_FlightMode_norm_de(varargin_1_0) *
        UAM_FlightMode_wrapToPi_d(obj->FinalYaw - obj->InitYaw) + obj->InitYaw);
      if (fabs(*varargout_3 - -3.1415926535897931) < 1.4901161193847656E-8) {
        *varargout_3 = 3.1415926535897931;
      }

      obj->LastWaypointFlag = false;
    }
  }

  *varargout_4 = obj->LookaheadDistFlag;
}

static real_T UAM_FlightMode_norm_d(const real_T x[2])
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
static void UAM_FlightMode_FIXED_WING_ENTRY(void)
{
  __m128d tmp_1;
  __m128d tmp_2;
  __m128d tmp_3;
  real_T b_waypointsIn_data[6];
  real_T rtb_wps[6];
  real_T rtb_TmpSignalConversionAtUAVO_1[3];
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
  if (UAM_FlightMode_DW.is_FIXED_WING_ENTRY == UAM_FlightMo_IN_FIXEDWINGFLIGHT)
  {
    /* Inport: '<Root>/mode' */
    switch (UAM_FlightMode_U.mode) {
     case 3U:
      UAM_FlightMode_DW.is_FIXED_WING_ENTRY = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
      UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightM_IN_FIXED_WING_ORBIT;

      /* Outport: '<Root>/controlMode' */
      UAM_FlightMode_Y.controlMode_m.airspeedAltitude = 1U;
      UAM_FlightMode_Y.controlMode_m.attitude = 1U;
      UAM_FlightMode_Y.controlMode_m.lateralGuidance = 1U;

      /* MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
       *  Inport: '<Root>/ToWP'
       */
      a = UAM_FlightMode_U.ToWP.params[0];
      UAM_FlightMode_DW.obj_b.OrbitRadiusFlag = 0U;
      if (UAM_FlightMode_U.ToWP.params[0] <= 50.0) {
        a = 50.0;
        UAM_FlightMode_DW.obj_b.OrbitRadiusFlag = 1U;
      }

      UAM_FlightMode_DW.obj_b.LookaheadDistFlag = 0U;

      /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
       *  Inport: '<Root>/ToWP'
       */
      tmp_2 = _mm_loadu_pd(&UAM_FlightMode_U.ToWP.position[0]);

      /* SignalConversion generated from: '<S5>/UAV Orbit Follower' incorporates:
       *  Inport: '<Root>/States'
       */
      tmp_3 = _mm_loadu_pd(&UAM_FlightMode_U.States.Xe[0]);

      /* MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
       *  Inport: '<Root>/States'
       *  Inport: '<Root>/ToWP'
       *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
       */
      _mm_storeu_pd(&tmp[0], _mm_sub_pd(tmp_3, tmp_2));
      if (UAM_FlightMode_norm_d(tmp) < 2.47032822920623E-323) {
        /* BusCreator: '<S5>/Bus Creator' incorporates:
         *  Inport: '<Root>/States'
         *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
         */
        UAM_FlightMode_B.aacSP.course = UAM_FlightMode_U.States.course;
        distToCenter = UAM_FlightMode_DW.obj_b.NumCircles;
      } else {
        p = false;
        p_0 = true;
        b_k = 0;
        exitg1 = false;
        while ((!exitg1) && (b_k < 3)) {
          if (!(UAM_FlightMode_DW.obj_b.OrbitCenterInternal[b_k] ==
                UAM_FlightMode_U.ToWP.position[b_k])) {
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
          if (UAM_FlightMode_DW.obj_b.OrbitRadiusInternal == a) {
            p = true;
          }

          if (!p) {
            guard1 = true;
          }
        }

        if (guard1) {
          UAM_FlightMode_DW.obj_b.NumCircles = 0.0;
          UAM_FlightMode_DW.obj_b.OrbitCenterInternal[0] =
            UAM_FlightMode_U.ToWP.position[0];
          UAM_FlightMode_DW.obj_b.OrbitCenterInternal[1] =
            UAM_FlightMode_U.ToWP.position[1];
          UAM_FlightMode_DW.obj_b.OrbitCenterInternal[2] =
            UAM_FlightMode_U.ToWP.position[2];
          UAM_FlightMode_DW.obj_b.OrbitRadiusInternal = a;
          UAM_FlightMode_DW.obj_b.SelectTurnDirectionFlag = true;
        }

        if (a <= 30.0) {
          UAM_FlightMode_DW.obj_b.LookaheadDistance = 0.9 * a;
        } else {
          UAM_FlightMode_DW.obj_b.LookaheadDistance = 30.0;
        }

        /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
         *  Inport: '<Root>/States'
         *  Inport: '<Root>/ToWP'
         *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
         */
        y2 = UAM_FlightMode_U.States.Xe[0] - UAM_FlightMode_U.ToWP.position[0];
        b_tmp[0] = y2;
        turnInternal = y2 * y2;
        y2 = UAM_FlightMode_U.States.Xe[1] - UAM_FlightMode_U.ToWP.position[1];
        b_tmp[1] = y2;
        distToCenter = sqrt(y2 * y2 + turnInternal);

        /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' */
        absx_tmp_tmp = a + UAM_FlightMode_DW.obj_b.LookaheadDistance;
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

          if (distToCenter <= (a - UAM_FlightMode_DW.obj_b.LookaheadDistance) +
              5.0 * turnInternal) {
            guard1 = true;
          } else {
            if (UAM_FlightMode_DW.obj_b.StartFlag) {
              UAM_FlightMode_DW.obj_b.PrevPosition[0] =
                UAM_FlightMode_U.States.Xe[0];
              UAM_FlightMode_DW.obj_b.PrevPosition[1] =
                UAM_FlightMode_U.States.Xe[1];
              UAM_FlightMode_DW.obj_b.StartFlag = false;
            }

            if ((UAM_FlightMode_U.ToWP.params[1] == 0.0) &&
                (!UAM_FlightMode_DW.obj_b.SelectTurnDirectionFlag)) {
              turnInternal = UAM_FlightMode_DW.obj_b.TurnDirectionInternal;
            } else {
              turnInternal = UAM_FlightMode_U.ToWP.params[1];
            }

            _mm_storeu_pd(&tmp_0[0], _mm_sub_pd(_mm_set_pd
              (UAM_FlightMode_DW.obj_b.PrevPosition[0],
               UAM_FlightMode_U.States.Xe[0]), _mm_set1_pd
              (UAM_FlightMode_U.ToWP.position[0])));
            xyPose[0] = tmp_0[0];
            turnVector[0] = tmp_0[1];
            _mm_storeu_pd(&tmp_0[0], _mm_sub_pd(_mm_set_pd
              (UAM_FlightMode_DW.obj_b.PrevPosition[1],
               UAM_FlightMode_U.States.Xe[1]), _mm_set1_pd
              (UAM_FlightMode_U.ToWP.position[1])));
            xyPose[1] = tmp_0[0];
            turnVector[1] = tmp_0[1];
            d = UAM_FlightMode_norm_d(xyPose);
            distToCenter = UAM_FlightMode_DW.obj_b.LookaheadDistance *
              UAM_FlightMode_DW.obj_b.LookaheadDistance;
            a = ((distToCenter - a * a) + d * d) / (2.0 * d);
            tmp_1 = _mm_set1_pd(d);
            tmp_2 = _mm_sub_pd(tmp_2, tmp_3);
            _mm_storeu_pd(&tmp_0[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(tmp_2,
              _mm_set1_pd(a)), tmp_1), tmp_3));
            y2 = tmp_0[1];
            distToCenter = sqrt(distToCenter - a * a);
            b_tmp[0] = tmp_0[0] - (UAM_FlightMode_U.ToWP.position[1] -
              UAM_FlightMode_U.States.Xe[1]) * distToCenter / d;
            _mm_storeu_pd(&tmp_0[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd
              (_mm_set_pd(UAM_FlightMode_U.ToWP.position[0],
                          UAM_FlightMode_U.ToWP.position[1]), _mm_set_pd
               (UAM_FlightMode_U.States.Xe[0], UAM_FlightMode_U.States.Xe[1])),
              _mm_set1_pd(distToCenter)), tmp_1), _mm_set_pd(tmp_0[1], tmp_0[0])));
            b_tmp[1] = tmp_0[0];
            a = tmp_0[1];
            y2 -= (UAM_FlightMode_U.ToWP.position[0] -
                   UAM_FlightMode_U.States.Xe[0]) * distToCenter / d;
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

            d = UAM_FlightMode_norm_de(u);
            distToCenter = UAM_FlightMode_norm_de(v);
            tmp_1 = _mm_set_pd(distToCenter, d);
            _mm_storeu_pd(&tmp_0[0], _mm_div_pd(_mm_set_pd(v[0], u[0]), tmp_1));
            u[0] = tmp_0[0];
            v[0] = tmp_0[1];
            _mm_storeu_pd(&tmp_0[0], _mm_div_pd(_mm_set_pd(v[1], u[1]), tmp_1));
            v[1] = tmp_0[1];
            turnVector[2] = u[0] * tmp_0[1] - v[0] * tmp_0[0];
            UAM_FlightMode_DW.obj_b.PrevPosition[0] =
              UAM_FlightMode_U.States.Xe[0];
            UAM_FlightMode_DW.obj_b.PrevPosition[1] =
              UAM_FlightMode_U.States.Xe[1];
            UAM_FlightMode_DW.obj_b.PrevPosition[2] =
              UAM_FlightMode_U.States.Xe[2];
            UAM_FlightMode_DW.obj_b.NumCircles += rt_atan2d_snf(turnVector[2],
              (u[0] * v[0] + tmp_0[0] * tmp_0[1]) + 0.0 / d * (0.0 /
              distToCenter)) / 2.0 / 3.1415926535897931;
            distToCenter = UAM_FlightMode_DW.obj_b.NumCircles;
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
              if ((b_tmp[0] - UAM_FlightMode_U.States.Xe[0]) * v[1] - (a -
                   UAM_FlightMode_U.States.Xe[1]) * v[0] > 0.0) {
                turnInternal = b_tmp[0];
                y2 = a;
              } else {
                turnInternal = b_tmp[1];
              }
              break;

             case -1:
              if ((b_tmp[0] - UAM_FlightMode_U.States.Xe[0]) * v[1] - (a -
                   UAM_FlightMode_U.States.Xe[1]) * v[0] < 0.0) {
                turnInternal = b_tmp[0];
                y2 = a;
              } else {
                turnInternal = b_tmp[1];
              }
              break;

             default:
              if (fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(a -
                     UAM_FlightMode_U.States.Xe[1], b_tmp[0] -
                     UAM_FlightMode_U.States.Xe[0]),
                    UAM_FlightMode_U.States.course)) < fabs
                  (UAM_FlightMode_angdiff(rt_atan2d_snf(y2 -
                     UAM_FlightMode_U.States.Xe[1], b_tmp[1] -
                     UAM_FlightMode_U.States.Xe[0]),
                    UAM_FlightMode_U.States.course))) {
                turnInternal = b_tmp[0];
                y2 = a;
              } else {
                turnInternal = b_tmp[1];
              }

              if ((turnInternal - UAM_FlightMode_U.States.Xe[0]) * v[1] - (y2 -
                   UAM_FlightMode_U.States.Xe[1]) * v[0] > 0.0) {
                UAM_FlightMode_DW.obj_b.TurnDirectionInternal = 1.0;
              } else {
                UAM_FlightMode_DW.obj_b.TurnDirectionInternal = -1.0;
              }

              UAM_FlightMode_DW.obj_b.SelectTurnDirectionFlag = false;
              break;
            }
          }
        }

        if (guard1) {
          _mm_storeu_pd(&tmp_0[0], _mm_add_pd(_mm_mul_pd(_mm_div_pd(_mm_set_pd
            (y2, b_tmp[0]), _mm_set1_pd(UAM_FlightMode_norm_d(b_tmp))),
            _mm_set1_pd(a)), tmp_2));
          turnInternal = tmp_0[0];
          y2 = tmp_0[1];
          distToCenter = UAM_FlightMode_DW.obj_b.NumCircles;
        }

        /* BusCreator: '<S5>/Bus Creator' incorporates:
         *  Inport: '<Root>/States'
         *  Inport: '<Root>/ToWP'
         *  MATLABSystem: '<S5>/UAV Orbit Follower'
         *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
         */
        UAM_FlightMode_B.aacSP.course = rt_atan2d_snf(y2 -
          UAM_FlightMode_U.States.Xe[1], turnInternal -
          UAM_FlightMode_U.States.Xe[0]);
      }

      /* BusCreator: '<S5>/Bus Creator' incorporates:
       *  Constant: '<S5>/Constant'
       *  Constant: '<S5>/Constant1'
       *  Constant: '<S5>/Lookahead Distance'
       *  Inport: '<Root>/ToWP'
       *  UnaryMinus: '<S5>/Unary Minus'
       */
      UAM_FlightMode_B.aacSP.airspeed = 15.0;
      UAM_FlightMode_B.aacSP.altitude = -UAM_FlightMode_U.ToWP.position[2];
      UAM_FlightMode_B.aacSP.L1 = 30.0;
      UAM_FlightMode_B.aacSP.climbrate = 0.0;

      /* Merge: '<S3>/ Merge ' incorporates:
       *  Abs: '<S5>/Abs1'
       *  DataTypeConversion: '<S5>/Data Type Conversion'
       *  Inport: '<Root>/ToWP'
       *  MATLABSystem: '<S5>/UAV Orbit Follower'
       *  RelationalOperator: '<S5>/Relational Operator'
       * */
      UAM_FlightMode_B.Status = (uint8_T)(fabs(distToCenter) >
        UAM_FlightMode_U.ToWP.params[2]);
      break;

     case 2U:
      UAM_FlightMode_DW.is_FIXED_WING_ENTRY = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
      UAM_FlightMode_DW.is_GuidanceLogic = UAM_Flig_IN_FIXED_WING_WAYPOINT;

      /* Outport: '<Root>/controlMode' */
      UAM_FlightMode_Y.controlMode_m.airspeedAltitude = 1U;
      UAM_FlightMode_Y.controlMode_m.attitude = 1U;
      UAM_FlightMode_Y.controlMode_m.lateralGuidance = 1U;

      /* MATLAB Function: '<S6>/MATLAB Function' incorporates:
       *  Inport: '<Root>/FromWP'
       *  Inport: '<Root>/ToWP'
       */
      rtb_wps[0] = UAM_FlightMode_U.FromWP.position[0];
      rtb_wps[1] = UAM_FlightMode_U.ToWP.position[0];
      rtb_wps[2] = UAM_FlightMode_U.FromWP.position[1];
      rtb_wps[3] = UAM_FlightMode_U.ToWP.position[1];
      rtb_wps[4] = UAM_FlightMode_U.FromWP.position[2];
      rtb_wps[5] = UAM_FlightMode_U.ToWP.position[2];

      /* MATLABSystem: '<S6>/Waypoint Follower' incorporates:
       *  Inport: '<Root>/FromWP'
       *  Inport: '<Root>/ToWP'
       *  MATLAB Function: '<S6>/MATLAB Function'
       */
      UAM_FlightMode_DW.obj_f.LookaheadDistFlag = 0U;
      UAM_FlightMode_DW.obj_f.InitialPose[0] = 0.0;
      UAM_FlightMode_DW.obj_f.InitialPose[1] = 0.0;
      UAM_FlightMode_DW.obj_f.InitialPose[2] = 0.0;
      UAM_FlightMode_DW.obj_f.InitialPose[3] = 0.0;
      UAM_FlightMode_DW.obj_f.NumWaypoints = 2.0;
      p = false;
      p_0 = true;
      b_k = 0;
      exitg1 = false;
      while ((!exitg1) && (b_k <= 5)) {
        i = ((b_k / 2) << 1) + b_k % 2;
        if (!(UAM_FlightMode_DW.obj_f.WaypointsInternal[i] == rtb_wps[i])) {
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
          UAM_FlightMode_DW.obj_f.WaypointsInternal[i] = rtb_wps[i];
        }

        UAM_FlightMode_DW.obj_f.WaypointIndex = 1.0;
      }

      distinctWptsIdx[1] = true;
      x[0] = (UAM_FlightMode_U.FromWP.position[0] !=
              UAM_FlightMode_U.ToWP.position[0]);
      x[1] = (UAM_FlightMode_U.FromWP.position[1] !=
              UAM_FlightMode_U.ToWP.position[1]);
      x[2] = (UAM_FlightMode_U.FromWP.position[2] !=
              UAM_FlightMode_U.ToWP.position[2]);
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
       *  Inport: '<Root>/States'
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

      UAM_FlightMode_DW.obj_f.LookaheadDistance = 30.0;
      if (tmp_size_idx_1 == 0) {
        /* BusCreator: '<S6>/Bus Creator' incorporates:
         *  Inport: '<Root>/States'
         *  SignalConversion generated from: '<S6>/Waypoint Follower'
         */
        UAM_FlightMode_B.aacSP.course = UAM_FlightMode_U.States.course;

        /* Merge: '<S3>/ Merge ' */
        UAM_FlightMode_B.Status = 1U;
      } else {
        guard1 = false;
        if (tmp_size_idx_1 == 1) {
          if (UAM_FlightMode_DW.obj_f.StartFlag) {
            UAM_FlightMode_DW.obj_f.InitialPose[0] = UAM_FlightMode_U.States.Xe
              [0];
            UAM_FlightMode_DW.obj_f.InitialPose[1] = UAM_FlightMode_U.States.Xe
              [1];
            UAM_FlightMode_DW.obj_f.InitialPose[2] = UAM_FlightMode_U.States.Xe
              [2];
            UAM_FlightMode_DW.obj_f.InitialPose[3] =
              UAM_FlightMode_U.States.course;
          }

          tmp_1 = _mm_sub_pd(_mm_loadu_pd(&b_waypointsIn_data[0]), _mm_loadu_pd(
            &UAM_FlightMode_U.States.Xe[0]));
          _mm_storeu_pd(&u[0], tmp_1);
          u[2] = b_waypointsIn_data[2] - UAM_FlightMode_U.States.Xe[2];
          if (UAM_FlightMode_norm_de(u) < 1.4901161193847656E-8) {
            /* BusCreator: '<S6>/Bus Creator' incorporates:
             *  Inport: '<Root>/States'
             *  SignalConversion generated from: '<S6>/Waypoint Follower'
             */
            UAM_FlightMode_B.aacSP.course = UAM_FlightMode_U.States.course;

            /* Merge: '<S3>/ Merge ' */
            UAM_FlightMode_B.Status = 1U;
            UAM_FlightMode_DW.obj_f.StartFlag = false;
          } else {
            UAM_FlightMode_DW.obj_f.StartFlag = false;
            UAM_FlightMode_DW.obj_f.NumWaypoints = 2.0;
            vectorUB = tmp_size_idx_1 + 1;
            for (i = 0; i < 3; i++) {
              scalarLB = (tmp_size_idx_1 + 1) * i;
              rtb_wps[scalarLB] = UAM_FlightMode_DW.obj_f.InitialPose[i];
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
          if (UAM_FlightMode_DW.obj_f.WaypointIndex == 2.0) {
            p = true;
          }

          if (p) {
            UAM_FlightMode_DW.obj_f.LastWaypointFlag = true;
            UAM_FlightMode_DW.obj_f.WaypointIndex--;
          }

          turnInternal = rtb_wps[(int32_T)UAM_FlightMode_DW.obj_f.WaypointIndex
            - 1];
          u[0] = turnInternal;
          d = rtb_wps[(int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0) - 1];
          v[0] = d;
          a = rtb_wps[((int32_T)UAM_FlightMode_DW.obj_f.WaypointIndex + vectorUB)
            - 1];
          u[1] = a;
          absx_tmp_tmp = rtb_wps[((int32_T)
            (UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0) + vectorUB) - 1];
          v[1] = absx_tmp_tmp;
          y2 = rtb_wps[((vectorUB << 1) + (int32_T)
                        UAM_FlightMode_DW.obj_f.WaypointIndex) - 1];
          u[2] = y2;
          distToCenter = rtb_wps[((int32_T)
            (UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0) + (vectorUB << 1)) - 1];
          v[2] = distToCenter;
          tmp_1 = _mm_set_pd(absx_tmp_tmp, d);
          tmp_2 = _mm_loadu_pd(&UAM_FlightMode_U.States.Xe[0]);
          _mm_storeu_pd(&turnVector[0], _mm_sub_pd(tmp_2, tmp_1));
          turnVector[2] = UAM_FlightMode_U.States.Xe[2] - distToCenter;
          d = UAM_FlightMode_norm_de(turnVector);
          guard2 = false;
          if (d <= 10.0) {
            guard2 = true;
          } else {
            _mm_storeu_pd(&unitVectorUtoV_tmp[0], _mm_sub_pd(tmp_1, _mm_set_pd(a,
              turnInternal)));
            unitVectorUtoV_tmp[2] = distToCenter - y2;
            distToCenter = UAM_FlightMode_norm_de(unitVectorUtoV_tmp);
            turnInternal = (unitVectorUtoV_tmp[0] / distToCenter * (turnVector[0]
              / d) + unitVectorUtoV_tmp[1] / distToCenter * (turnVector[1] / d))
              + unitVectorUtoV_tmp[2] / distToCenter * (turnVector[2] / d);
            if (rtIsNaN(turnInternal) || (turnInternal < 0.0)) {
            } else {
              guard2 = true;
            }
          }

          if (guard2) {
            UAM_FlightMode_DW.obj_f.WaypointIndex++;
            p = false;
            if (UAM_FlightMode_DW.obj_f.WaypointIndex == 2.0) {
              p = true;
            }

            if (p) {
              UAM_FlightMode_DW.obj_f.LastWaypointFlag = true;
              UAM_FlightMode_DW.obj_f.WaypointIndex--;
            }

            u[0] = rtb_wps[(int32_T)UAM_FlightMode_DW.obj_f.WaypointIndex - 1];
            v[0] = rtb_wps[(int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0)
              - 1];
            u[1] = rtb_wps[((int32_T)UAM_FlightMode_DW.obj_f.WaypointIndex +
                            vectorUB) - 1];
            v[1] = rtb_wps[((int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex +
              1.0) + vectorUB) - 1];
            u[2] = rtb_wps[((vectorUB << 1) + (int32_T)
                            UAM_FlightMode_DW.obj_f.WaypointIndex) - 1];
            v[2] = rtb_wps[((int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex +
              1.0) + (vectorUB << 1)) - 1];
          }

          _mm_storeu_pd(&tmp_0[0], _mm_sub_pd(_mm_set_pd(v[0],
            UAM_FlightMode_U.States.Xe[0]), _mm_set1_pd(u[0])));
          turnVector[0] = tmp_0[0];
          unitVectorUtoV_tmp[0] = tmp_0[1];
          _mm_storeu_pd(&tmp_0[0], _mm_sub_pd(_mm_set_pd(v[1],
            UAM_FlightMode_U.States.Xe[1]), _mm_set1_pd(u[1])));
          turnVector[1] = tmp_0[0];
          unitVectorUtoV_tmp[1] = tmp_0[1];
          _mm_storeu_pd(&tmp_0[0], _mm_sub_pd(_mm_set_pd(v[2],
            UAM_FlightMode_U.States.Xe[2]), _mm_set1_pd(u[2])));
          turnVector[2] = tmp_0[0];
          unitVectorUtoV_tmp[2] = tmp_0[1];
          a = unitVectorUtoV_tmp[0] * unitVectorUtoV_tmp[0] +
            unitVectorUtoV_tmp[1] * unitVectorUtoV_tmp[1];
          turnInternal = tmp_0[1] * tmp_0[1] + a;
          d = ((turnVector[0] * unitVectorUtoV_tmp[0] + turnVector[1] *
                unitVectorUtoV_tmp[1]) + tmp_0[0] * tmp_0[1]) / turnInternal;
          if (d < 0.0) {
            y2 = UAM_FlightMode_norm_de(turnVector);
          } else if (d > 1.0) {
            tmp_1 = _mm_sub_pd(tmp_2, _mm_loadu_pd(&v[0]));
            _mm_storeu_pd(&rtb_TmpSignalConversionAtUAVO_1[0], tmp_1);
            rtb_TmpSignalConversionAtUAVO_1[2] = UAM_FlightMode_U.States.Xe[2] -
              v[2];
            y2 = UAM_FlightMode_norm_de(rtb_TmpSignalConversionAtUAVO_1);
          } else {
            tmp_1 = _mm_sub_pd(tmp_2, _mm_add_pd(_mm_mul_pd(_mm_set1_pd(d),
              _mm_loadu_pd(&unitVectorUtoV_tmp[0])), _mm_loadu_pd(&u[0])));
            _mm_storeu_pd(&rtb_TmpSignalConversionAtUAVO_1[0], tmp_1);
            rtb_TmpSignalConversionAtUAVO_1[2] = UAM_FlightMode_U.States.Xe[2] -
              (d * tmp_0[1] + u[2]);
            y2 = UAM_FlightMode_norm_de(rtb_TmpSignalConversionAtUAVO_1);
          }

          if (UAM_FlightMode_DW.obj_f.LastWaypointFlag) {
            d = (((UAM_FlightMode_U.States.Xe[0] - u[0]) * unitVectorUtoV_tmp[0]
                  + (UAM_FlightMode_U.States.Xe[1] - u[1]) * unitVectorUtoV_tmp
                  [1]) + (UAM_FlightMode_U.States.Xe[2] - u[2]) * tmp_0[1]) /
              turnInternal;
            tmp_1 = _mm_sub_pd(tmp_2, _mm_add_pd(_mm_mul_pd(_mm_set1_pd(d),
              _mm_loadu_pd(&unitVectorUtoV_tmp[0])), _mm_loadu_pd(&u[0])));
            _mm_storeu_pd(&rtb_TmpSignalConversionAtUAVO_1[0], tmp_1);
            rtb_TmpSignalConversionAtUAVO_1[2] = UAM_FlightMode_U.States.Xe[2] -
              (d * tmp_0[1] + u[2]);
            y2 = UAM_FlightMode_norm_de(rtb_TmpSignalConversionAtUAVO_1);
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
            UAM_FlightMode_DW.obj_f.LookaheadDistance =
              UAM_FlightMode_DW.obj_f.LookaheadFactor * y2;
          }

          turnVector[0] = unitVectorUtoV_tmp[0];
          turnVector[1] = unitVectorUtoV_tmp[1];
          tmp_1 = _mm_sub_pd(_mm_loadu_pd(&u[0]), tmp_2);
          _mm_storeu_pd(&unitVectorUtoV_tmp[0], tmp_1);
          a += unitVectorUtoV_tmp[2] * unitVectorUtoV_tmp[2];
          unitVectorUtoV_tmp[2] = u[2] - UAM_FlightMode_U.States.Xe[2];
          d = ((turnVector[0] * unitVectorUtoV_tmp[0] + turnVector[1] *
                unitVectorUtoV_tmp[1]) + tmp_0[1] * unitVectorUtoV_tmp[2]) * 2.0;
          y2 = sqrt(d * d - (((unitVectorUtoV_tmp[0] * unitVectorUtoV_tmp[0] +
                               unitVectorUtoV_tmp[1] * unitVectorUtoV_tmp[1]) +
                              unitVectorUtoV_tmp[2] * unitVectorUtoV_tmp[2]) -
                             UAM_FlightMode_DW.obj_f.LookaheadDistance *
                             UAM_FlightMode_DW.obj_f.LookaheadDistance) * (4.0 *
                     a));
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
           *  Inport: '<Root>/States'
           *  SignalConversion generated from: '<S6>/Waypoint Follower'
           */
          UAM_FlightMode_B.aacSP.course = rt_atan2d_snf(v[1] -
            UAM_FlightMode_U.States.Xe[1], v[0] - UAM_FlightMode_U.States.Xe[0]);

          /* Merge: '<S3>/ Merge ' */
          UAM_FlightMode_B.Status = 0U;
          p = false;
          if (UAM_FlightMode_DW.obj_f.LastWaypointFlag) {
            p = true;
          }

          if (p) {
            /* Merge: '<S3>/ Merge ' */
            UAM_FlightMode_B.Status = 1U;
          }

          UAM_FlightMode_DW.obj_f.LastWaypointFlag = false;
        }
      }

      /* BusCreator: '<S6>/Bus Creator' incorporates:
       *  Constant: '<S6>/Constant'
       *  Constant: '<S6>/Constant1'
       *  Constant: '<S6>/Lookahead Distance'
       *  Inport: '<Root>/ToWP'
       *  MATLAB Function: '<S6>/MATLAB Function'
       */
      UAM_FlightMode_B.aacSP.airspeed = 15.0;
      UAM_FlightMode_B.aacSP.altitude = -UAM_FlightMode_U.ToWP.position[2];
      UAM_FlightMode_B.aacSP.L1 = 30.0;
      UAM_FlightMode_B.aacSP.climbrate = 0.0;
      break;
    }

    /* End of Inport: '<Root>/mode' */

    /* case IN_Stabilize: */
  } else if (UAM_FlightMode_DW.temporalCounter_i1 >= 38) {
    UAM_FlightMode_DW.is_FIXED_WING_ENTRY = UAM_FlightMo_IN_FIXEDWINGFLIGHT;

    /* Outport: '<Root>/controlMode' */
    UAM_FlightMode_Y.controlMode_m.airspeedAltitude = 1U;
    UAM_FlightMode_Y.controlMode_m.attitude = 1U;
    UAM_FlightMode_Y.controlMode_m.lateralGuidance = 1U;
    UAM_FlightMode_B.aacSP.L1 = 25.0;
    UAM_FlightMode_B.aacSP.airspeed = 14.0;
    UAM_FlightMode_B.aacSP.altitude = 0.0;
    UAM_FlightMode_B.aacSP.course = 0.0;
  }
}

/* Function for Chart: '<Root>/Guidance Mode Selector' */
static boolean_T UAM_Flig_transitionConditionMet(void)
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  boolean_T valid;

  /* Inport: '<Root>/States' */
  if (UAM_FlightMode_U.States.c1 >= 1.57) {
    scale = 3.3121686421112381E-170;
    absxk = fabs(UAM_FlightMode_U.States.Ve[0]);
    if (absxk > 3.3121686421112381E-170) {
      y = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      y = t * t;
    }

    absxk = fabs(UAM_FlightMode_U.States.Ve[1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }

    absxk = fabs(UAM_FlightMode_U.States.Ve[2]);
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

  /* End of Inport: '<Root>/States' */
  return valid;
}

/* Function for Chart: '<Root>/Guidance Mode Selector' */
static void enter_internal_FIXED_WING_ENTRY(void)
{
  UAM_FlightMode_DW.temporalCounter_i1 = 0U;
  UAM_FlightMode_DW.is_FIXED_WING_ENTRY = UAM_FlightMode_IN_Stabilize;

  /* Outport: '<Root>/FlightMode' */
  UAM_FlightMode_Y.FlightMode = FixedWing;

  /* Outport: '<Root>/controlMode' */
  UAM_FlightMode_Y.controlMode_m.attitude = 1U;
  UAM_FlightMode_Y.controlMode_m.airspeedAltitude = 0U;
  UAM_FlightMode_Y.controlMode_m.lateralGuidance = 0U;

  /* Outport: '<Root>/FixedWingSP' */
  UAM_FlightMode_Y.FixedWingSP.yaw = 0.0;
  UAM_FlightMode_Y.FixedWingSP.pitch = 0.0;
  UAM_FlightMode_Y.FixedWingSP.roll = 0.0;
  UAM_FlightMode_Y.FixedWingSP.airspeed = 14.0;

  /* Merge: '<S3>/ Merge ' */
  UAM_FlightMode_B.Status = 0U;
}

/* Function for Chart: '<Root>/Guidance Mode Selector' */
static void UAM_FlightMode_Start(real_T rtb_wps_l_0[8])
{
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_2;
  real_T rtb_wps_h[8];
  real_T b_waypointsIn_data[6];
  real_T rtb_wps[6];
  real_T rtb_Product[3];
  real_T rtb_TmpSignalConversionAtUAVO_2[3];
  real_T turnVector[3];
  real_T unitVectorUtoV_tmp[3];
  real_T v[3];
  real_T b_tmp[2];
  real_T tmp[2];
  real_T xyLookaheadPoint[2];
  real_T xyPose[2];
  real_T d;
  real_T d_tmp;
  real_T distToCenter;
  real_T h;
  real_T rtb_Abs;
  real_T rtb_DotProduct;
  real_T rtb_Sign;
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

  /* Inport: '<Root>/mode' incorporates:
   *  Constant: '<S9>/Lookahead Distance'
   *  Inport: '<Root>/Pose'
   *  Inport: '<Root>/States'
   *  Inport: '<Root>/ToWP'
   *  MATLABSystem: '<S10>/UAV Orbit Follower'
   *  MATLABSystem: '<S5>/UAV Orbit Follower'
   *  Outport: '<Root>/FlightMode'
   *  Outport: '<Root>/controlMode'
   *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
   * */
  if ((UAM_FlightMode_U.mode == 6) && (UAM_FlightMode_Y.FlightMode == FixedWing))
  {
    UAM_FlightMode_DW.temporalCounter_i1 = 0U;
    UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMode_IN_PreTransition;
    UAM_FlightMode_Y.FlightMode = FixedWing;
    UAM_FlightMode_Y.controlMode_m.airspeedAltitude = 1U;
    UAM_FlightMode_Y.controlMode_m.attitude = 1U;
    UAM_FlightMode_Y.controlMode_m.lateralGuidance = 1U;
    UAM_FlightMode_B.aacSP.L1 = 25.0;
    UAM_FlightMode_B.aacSP.airspeed = 14.0;
    UAM_FlightMode_B.aacSP.altitude = -UAM_FlightMode_U.States.Xe[2];
    UAM_FlightMode_B.aacSP.course = UAM_FlightMode_U.States.course;
  } else if ((UAM_FlightMode_U.mode == 2) && (UAM_FlightMode_Y.FlightMode ==
              FixedWing)) {
    UAM_FlightMode_DW.is_GuidanceLogic = UAM_Flig_IN_FIXED_WING_WAYPOINT;
    UAM_FlightMode_Y.controlMode_m.airspeedAltitude = 1U;
    UAM_FlightMode_Y.controlMode_m.attitude = 1U;
    UAM_FlightMode_Y.controlMode_m.lateralGuidance = 1U;

    /* MATLAB Function: '<S6>/MATLAB Function' incorporates:
     *  Inport: '<Root>/FromWP'
     *  Inport: '<Root>/ToWP'
     *  Outport: '<Root>/controlMode'
     */
    rtb_wps[0] = UAM_FlightMode_U.FromWP.position[0];
    rtb_wps[1] = UAM_FlightMode_U.ToWP.position[0];
    rtb_wps[2] = UAM_FlightMode_U.FromWP.position[1];
    rtb_wps[3] = UAM_FlightMode_U.ToWP.position[1];
    rtb_wps[4] = UAM_FlightMode_U.FromWP.position[2];
    rtb_wps[5] = UAM_FlightMode_U.ToWP.position[2];

    /* MATLABSystem: '<S6>/Waypoint Follower' incorporates:
     *  Inport: '<Root>/FromWP'
     *  Inport: '<Root>/ToWP'
     *  MATLAB Function: '<S6>/MATLAB Function'
     */
    UAM_FlightMode_DW.obj_f.LookaheadDistFlag = 0U;
    UAM_FlightMode_DW.obj_f.InitialPose[0] = 0.0;
    UAM_FlightMode_DW.obj_f.InitialPose[1] = 0.0;
    UAM_FlightMode_DW.obj_f.InitialPose[2] = 0.0;
    UAM_FlightMode_DW.obj_f.InitialPose[3] = 0.0;
    UAM_FlightMode_DW.obj_f.NumWaypoints = 2.0;
    p = false;
    p_0 = true;
    b_k = 0;
    exitg1 = false;
    while ((!exitg1) && (b_k <= 5)) {
      i = ((b_k / 2) << 1) + b_k % 2;
      if (!(UAM_FlightMode_DW.obj_f.WaypointsInternal[i] == rtb_wps[i])) {
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
        UAM_FlightMode_DW.obj_f.WaypointsInternal[i] = rtb_wps[i];
      }

      UAM_FlightMode_DW.obj_f.WaypointIndex = 1.0;
    }

    distinctWptsIdx[1] = true;
    x[0] = (UAM_FlightMode_U.FromWP.position[0] !=
            UAM_FlightMode_U.ToWP.position[0]);
    x[1] = (UAM_FlightMode_U.FromWP.position[1] !=
            UAM_FlightMode_U.ToWP.position[1]);
    x[2] = (UAM_FlightMode_U.FromWP.position[2] !=
            UAM_FlightMode_U.ToWP.position[2]);
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
      if (distinctWptsIdx[i]) {
        b_k++;
      }
    }

    tmp_size_idx_1 = b_k;
    b_k = 0;
    for (i = 0; i < 2; i++) {
      if (distinctWptsIdx[i]) {
        /* Start for MATLABSystem: '<S6>/Waypoint Follower' */
        tmp_data[b_k] = (int8_T)i;
        b_k++;
      }
    }

    /* MATLABSystem: '<S6>/Waypoint Follower' incorporates:
     *  Inport: '<Root>/States'
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

    UAM_FlightMode_DW.obj_f.LookaheadDistance = 30.0;
    if (tmp_size_idx_1 == 0) {
      /* BusCreator: '<S6>/Bus Creator' incorporates:
       *  Inport: '<Root>/States'
       *  SignalConversion generated from: '<S6>/Waypoint Follower'
       */
      UAM_FlightMode_B.aacSP.course = UAM_FlightMode_U.States.course;

      /* Merge: '<S3>/ Merge ' */
      UAM_FlightMode_B.Status = 1U;
    } else {
      guard1 = false;
      if (tmp_size_idx_1 == 1) {
        if (UAM_FlightMode_DW.obj_f.StartFlag) {
          UAM_FlightMode_DW.obj_f.InitialPose[0] = UAM_FlightMode_U.States.Xe[0];
          UAM_FlightMode_DW.obj_f.InitialPose[1] = UAM_FlightMode_U.States.Xe[1];
          UAM_FlightMode_DW.obj_f.InitialPose[2] = UAM_FlightMode_U.States.Xe[2];
          UAM_FlightMode_DW.obj_f.InitialPose[3] =
            UAM_FlightMode_U.States.course;
        }

        tmp_0 = _mm_sub_pd(_mm_loadu_pd(&b_waypointsIn_data[0]), _mm_loadu_pd
                           (&UAM_FlightMode_U.States.Xe[0]));
        _mm_storeu_pd(&rtb_Product[0], tmp_0);
        rtb_Product[2] = b_waypointsIn_data[2] - UAM_FlightMode_U.States.Xe[2];
        if (UAM_FlightMode_norm_de(rtb_Product) < 1.4901161193847656E-8) {
          /* BusCreator: '<S6>/Bus Creator' incorporates:
           *  Inport: '<Root>/States'
           *  SignalConversion generated from: '<S6>/Waypoint Follower'
           */
          UAM_FlightMode_B.aacSP.course = UAM_FlightMode_U.States.course;

          /* Merge: '<S3>/ Merge ' */
          UAM_FlightMode_B.Status = 1U;
          UAM_FlightMode_DW.obj_f.StartFlag = false;
        } else {
          UAM_FlightMode_DW.obj_f.StartFlag = false;
          UAM_FlightMode_DW.obj_f.NumWaypoints = 2.0;
          scalarLB = tmp_size_idx_1 + 1;
          for (i = 0; i < 3; i++) {
            vectorUB = (tmp_size_idx_1 + 1) * i;
            rtb_wps[vectorUB] = UAM_FlightMode_DW.obj_f.InitialPose[i];
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
        if (UAM_FlightMode_DW.obj_f.WaypointIndex == 2.0) {
          p = true;
        }

        if (p) {
          UAM_FlightMode_DW.obj_f.LastWaypointFlag = true;
          UAM_FlightMode_DW.obj_f.WaypointIndex--;
        }

        rtb_Sign = rtb_wps[(int32_T)UAM_FlightMode_DW.obj_f.WaypointIndex - 1];
        rtb_Product[0] = rtb_Sign;
        rtb_Abs = rtb_wps[(int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0)
          - 1];
        v[0] = rtb_Abs;
        rtb_DotProduct = rtb_wps[((int32_T)UAM_FlightMode_DW.obj_f.WaypointIndex
          + scalarLB) - 1];
        rtb_Product[1] = rtb_DotProduct;
        d = rtb_wps[((int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0) +
                     scalarLB) - 1];
        v[1] = d;
        rtb_Product[2] = rtb_wps[((scalarLB << 1) + (int32_T)
          UAM_FlightMode_DW.obj_f.WaypointIndex) - 1];
        h = rtb_wps[((int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0) +
                     (scalarLB << 1)) - 1];
        v[2] = h;
        tmp_1 = _mm_set_pd(d, rtb_Abs);
        tmp_2 = _mm_loadu_pd(&UAM_FlightMode_U.States.Xe[0]);
        _mm_storeu_pd(&turnVector[0], _mm_sub_pd(tmp_2, tmp_1));
        turnVector[2] = UAM_FlightMode_U.States.Xe[2] - h;
        rtb_Abs = UAM_FlightMode_norm_de(turnVector);
        guard2 = false;
        if (rtb_Abs <= 10.0) {
          guard2 = true;
        } else {
          _mm_storeu_pd(&unitVectorUtoV_tmp[0], _mm_sub_pd(tmp_1, _mm_set_pd
            (rtb_DotProduct, rtb_Sign)));
          unitVectorUtoV_tmp[2] = rtb_wps[((int32_T)
            (UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0) + (scalarLB << 1)) - 1]
            - rtb_wps[((scalarLB << 1) + (int32_T)
                       UAM_FlightMode_DW.obj_f.WaypointIndex) - 1];
          h = UAM_FlightMode_norm_de(unitVectorUtoV_tmp);
          rtb_Sign = (unitVectorUtoV_tmp[0] / h * (turnVector[0] / rtb_Abs) +
                      unitVectorUtoV_tmp[1] / h * (turnVector[1] / rtb_Abs)) +
            unitVectorUtoV_tmp[2] / h * (turnVector[2] / rtb_Abs);
          if (rtIsNaN(rtb_Sign) || (rtb_Sign < 0.0)) {
          } else {
            guard2 = true;
          }
        }

        if (guard2) {
          UAM_FlightMode_DW.obj_f.WaypointIndex++;
          p = false;
          if (UAM_FlightMode_DW.obj_f.WaypointIndex == 2.0) {
            p = true;
          }

          if (p) {
            UAM_FlightMode_DW.obj_f.LastWaypointFlag = true;
            UAM_FlightMode_DW.obj_f.WaypointIndex--;
          }

          rtb_Product[0] = rtb_wps[(int32_T)
            UAM_FlightMode_DW.obj_f.WaypointIndex - 1];
          v[0] = rtb_wps[(int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0)
            - 1];
          rtb_Product[1] = rtb_wps[((int32_T)
            UAM_FlightMode_DW.obj_f.WaypointIndex + scalarLB) - 1];
          v[1] = rtb_wps[((int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0)
                          + scalarLB) - 1];
          rtb_Product[2] = rtb_wps[((scalarLB << 1) + (int32_T)
            UAM_FlightMode_DW.obj_f.WaypointIndex) - 1];
          v[2] = rtb_wps[((int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0)
                          + (scalarLB << 1)) - 1];
        }

        _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(v[0],
          UAM_FlightMode_U.States.Xe[0]), _mm_set1_pd(rtb_Product[0])));
        turnVector[0] = tmp[0];
        unitVectorUtoV_tmp[0] = tmp[1];
        _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(v[1],
          UAM_FlightMode_U.States.Xe[1]), _mm_set1_pd(rtb_Product[1])));
        turnVector[1] = tmp[0];
        unitVectorUtoV_tmp[1] = tmp[1];
        _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(v[2],
          UAM_FlightMode_U.States.Xe[2]), _mm_set1_pd(rtb_Product[2])));
        turnVector[2] = tmp[0];
        unitVectorUtoV_tmp[2] = tmp[1];
        rtb_Sign = unitVectorUtoV_tmp[0] * unitVectorUtoV_tmp[0] +
          unitVectorUtoV_tmp[1] * unitVectorUtoV_tmp[1];
        rtb_DotProduct = tmp[1] * tmp[1] + rtb_Sign;
        d = ((turnVector[0] * unitVectorUtoV_tmp[0] + turnVector[1] *
              unitVectorUtoV_tmp[1]) + tmp[0] * tmp[1]) / rtb_DotProduct;
        if (d < 0.0) {
          rtb_Abs = UAM_FlightMode_norm_de(turnVector);
        } else if (d > 1.0) {
          tmp_0 = _mm_sub_pd(tmp_2, _mm_loadu_pd(&v[0]));
          _mm_storeu_pd(&rtb_TmpSignalConversionAtUAVO_2[0], tmp_0);
          rtb_TmpSignalConversionAtUAVO_2[2] = UAM_FlightMode_U.States.Xe[2] -
            v[2];
          rtb_Abs = UAM_FlightMode_norm_de(rtb_TmpSignalConversionAtUAVO_2);
        } else {
          tmp_0 = _mm_sub_pd(tmp_2, _mm_add_pd(_mm_mul_pd(_mm_set1_pd(d),
            _mm_loadu_pd(&unitVectorUtoV_tmp[0])), _mm_loadu_pd(&rtb_Product[0])));
          _mm_storeu_pd(&rtb_TmpSignalConversionAtUAVO_2[0], tmp_0);
          rtb_TmpSignalConversionAtUAVO_2[2] = UAM_FlightMode_U.States.Xe[2] -
            (d * tmp[1] + rtb_Product[2]);
          rtb_Abs = UAM_FlightMode_norm_de(rtb_TmpSignalConversionAtUAVO_2);
        }

        if (UAM_FlightMode_DW.obj_f.LastWaypointFlag) {
          d = (((UAM_FlightMode_U.States.Xe[0] - rtb_Product[0]) *
                unitVectorUtoV_tmp[0] + (UAM_FlightMode_U.States.Xe[1] -
                 rtb_Product[1]) * unitVectorUtoV_tmp[1]) +
               (UAM_FlightMode_U.States.Xe[2] - rtb_Product[2]) * tmp[1]) /
            rtb_DotProduct;
          tmp_0 = _mm_sub_pd(tmp_2, _mm_add_pd(_mm_mul_pd(_mm_set1_pd(d),
            _mm_loadu_pd(&unitVectorUtoV_tmp[0])), _mm_loadu_pd(&rtb_Product[0])));
          _mm_storeu_pd(&rtb_TmpSignalConversionAtUAVO_2[0], tmp_0);
          rtb_TmpSignalConversionAtUAVO_2[2] = UAM_FlightMode_U.States.Xe[2] -
            (d * tmp[1] + rtb_Product[2]);
          rtb_Abs = UAM_FlightMode_norm_de(rtb_TmpSignalConversionAtUAVO_2);
        }

        d = fabs(rtb_Abs);
        if (rtIsInf(d) || rtIsNaN(d)) {
          rtb_DotProduct = (rtNaN);
          d = (rtNaN);
        } else if (d < 4.4501477170144028E-308) {
          rtb_DotProduct = 4.94065645841247E-324;
          d = 4.94065645841247E-324;
        } else {
          frexp(d, &b_exponent);
          rtb_DotProduct = ldexp(1.0, b_exponent - 53);
          frexp(d, &b_exponent_0);
          d = ldexp(1.0, b_exponent_0 - 53);
        }

        rtb_DotProduct = sqrt(rtb_DotProduct);
        d *= 5.0;
        if ((rtb_DotProduct >= d) || rtIsNaN(d)) {
          d = rtb_DotProduct;
        }

        if (rtb_Abs + d >= 30.0) {
          UAM_FlightMode_DW.obj_f.LookaheadDistance =
            UAM_FlightMode_DW.obj_f.LookaheadFactor * rtb_Abs;
        }

        turnVector[0] = unitVectorUtoV_tmp[0];
        turnVector[1] = unitVectorUtoV_tmp[1];
        tmp_0 = _mm_sub_pd(_mm_loadu_pd(&rtb_Product[0]), tmp_2);
        _mm_storeu_pd(&unitVectorUtoV_tmp[0], tmp_0);
        rtb_Sign += unitVectorUtoV_tmp[2] * unitVectorUtoV_tmp[2];
        unitVectorUtoV_tmp[2] = rtb_Product[2] - UAM_FlightMode_U.States.Xe[2];
        d = ((turnVector[0] * unitVectorUtoV_tmp[0] + turnVector[1] *
              unitVectorUtoV_tmp[1]) + tmp[1] * unitVectorUtoV_tmp[2]) * 2.0;
        rtb_Abs = sqrt(d * d - (((unitVectorUtoV_tmp[0] * unitVectorUtoV_tmp[0]
          + unitVectorUtoV_tmp[1] * unitVectorUtoV_tmp[1]) + unitVectorUtoV_tmp
          [2] * unitVectorUtoV_tmp[2]) -
          UAM_FlightMode_DW.obj_f.LookaheadDistance *
          UAM_FlightMode_DW.obj_f.LookaheadDistance) * (4.0 * rtb_Sign));
        rtb_DotProduct = (-d + rtb_Abs) / 2.0 / rtb_Sign;
        rtb_Sign = (-d - rtb_Abs) / 2.0 / rtb_Sign;
        if ((rtb_DotProduct >= rtb_Sign) || rtIsNaN(rtb_Sign)) {
          rtb_Sign = rtb_DotProduct;
        }

        tmp_0 = _mm_set1_pd(rtb_Sign);
        tmp_1 = _mm_add_pd(_mm_mul_pd(_mm_sub_pd(_mm_set1_pd(1.0), tmp_0),
          _mm_loadu_pd(&rtb_Product[0])), _mm_mul_pd(tmp_0, _mm_loadu_pd(&v[0])));
        _mm_storeu_pd(&v[0], tmp_1);

        /* BusCreator: '<S6>/Bus Creator' incorporates:
         *  Inport: '<Root>/States'
         *  SignalConversion generated from: '<S6>/Waypoint Follower'
         */
        UAM_FlightMode_B.aacSP.course = rt_atan2d_snf(v[1] -
          UAM_FlightMode_U.States.Xe[1], v[0] - UAM_FlightMode_U.States.Xe[0]);

        /* Merge: '<S3>/ Merge ' */
        UAM_FlightMode_B.Status = 0U;
        p = false;
        if (UAM_FlightMode_DW.obj_f.LastWaypointFlag) {
          p = true;
        }

        if (p) {
          /* Merge: '<S3>/ Merge ' */
          UAM_FlightMode_B.Status = 1U;
        }

        UAM_FlightMode_DW.obj_f.LastWaypointFlag = false;
      }
    }

    /* BusCreator: '<S6>/Bus Creator' incorporates:
     *  Constant: '<S6>/Constant'
     *  Constant: '<S6>/Constant1'
     *  Constant: '<S6>/Lookahead Distance'
     *  Inport: '<Root>/ToWP'
     *  MATLAB Function: '<S6>/MATLAB Function'
     */
    UAM_FlightMode_B.aacSP.airspeed = 15.0;
    UAM_FlightMode_B.aacSP.altitude = -UAM_FlightMode_U.ToWP.position[2];
    UAM_FlightMode_B.aacSP.L1 = 30.0;
    UAM_FlightMode_B.aacSP.climbrate = 0.0;
  } else {
    switch (UAM_FlightMode_Y.FlightMode) {
     case Transition:
      UAM_FlightMode_Y.controlMode_m.inTransition = 0U;
      UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightM_IN_FIXED_WING_ENTRY;
      enter_internal_FIXED_WING_ENTRY();
      break;

     case BackTransition:
      UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMode_IN_HOVER_ENTRY;
      UAM_FlightMode_Y.FlightMode = Hover;
      UAM_FlightMode_Y.controlMode_m.inTransition = 0U;
      break;

     default:
      if ((UAM_FlightMode_U.mode == 1) && (UAM_FlightMode_Y.FlightMode == Hover))
      {
        UAM_FlightMode_Y.controlMode_m.inTransition = 0U;
        UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMode_IN_Takeoff;

        /* BusCreator: '<S11>/Bus Creator' incorporates:
         *  Inport: '<Root>/Pose'
         *  Inport: '<Root>/ToWP'
         *  Merge: '<S3>/ Merge 2'
         *  Outport: '<Root>/controlMode'
         *  SignalConversion generated from: '<S11>/Bus Creator'
         */
        UAM_FlightMode_B.InnerLoopCmds.LAP[0] = UAM_FlightMode_U.Pose[0];
        UAM_FlightMode_B.InnerLoopCmds.LAP[1] = UAM_FlightMode_U.Pose[1];
        UAM_FlightMode_B.InnerLoopCmds.LAP[2] = UAM_FlightMode_U.ToWP.position[2];

        /* Saturate: '<S11>/Hdg. Cmd Sat' incorporates:
         *  Inport: '<Root>/ToWP'
         */
        if (UAM_FlightMode_U.ToWP.params[3] > 3.1415926535897931) {
          /* BusCreator: '<S11>/Bus Creator' incorporates:
           *  Merge: '<S3>/ Merge 2'
           */
          UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = 3.1415926535897931;
        } else if (UAM_FlightMode_U.ToWP.params[3] < -3.1415926535897931) {
          /* BusCreator: '<S11>/Bus Creator' incorporates:
           *  Merge: '<S3>/ Merge 2'
           */
          UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = -3.1415926535897931;
        } else {
          /* BusCreator: '<S11>/Bus Creator' incorporates:
           *  Merge: '<S3>/ Merge 2'
           */
          UAM_FlightMode_B.InnerLoopCmds.HeadingCmd =
            UAM_FlightMode_U.ToWP.params[3];
        }

        /* End of Saturate: '<S11>/Hdg. Cmd Sat' */

        /* BusCreator: '<S11>/Bus Creator' incorporates:
         *  Merge: '<S3>/ Merge 2'
         */
        UAM_FlightMode_B.InnerLoopCmds.YawCmd = UAM_FlightMode_ConstB.YawCmd;

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S11>/Constant'
         *  SignalConversion generated from: '<S11>/Status'
         */
        UAM_FlightMode_B.Status = 0U;
      } else if ((UAM_FlightMode_U.mode == 3) && (UAM_FlightMode_Y.FlightMode ==
                  FixedWing)) {
        UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightM_IN_FIXED_WING_ORBIT;
        UAM_FlightMode_Y.controlMode_m.airspeedAltitude = 1U;
        UAM_FlightMode_Y.controlMode_m.attitude = 1U;
        UAM_FlightMode_Y.controlMode_m.lateralGuidance = 1U;

        /* MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
         *  Inport: '<Root>/ToWP'
         *  Outport: '<Root>/controlMode'
         */
        rtb_Abs = UAM_FlightMode_U.ToWP.params[0];
        UAM_FlightMode_DW.obj_b.OrbitRadiusFlag = 0U;
        if (UAM_FlightMode_U.ToWP.params[0] <= 50.0) {
          rtb_Abs = 50.0;
          UAM_FlightMode_DW.obj_b.OrbitRadiusFlag = 1U;
        }

        UAM_FlightMode_DW.obj_b.LookaheadDistFlag = 0U;
        tmp_1 = _mm_loadu_pd(&UAM_FlightMode_U.ToWP.position[0]);
        tmp_2 = _mm_loadu_pd(&UAM_FlightMode_U.States.Xe[0]);
        _mm_storeu_pd(&xyLookaheadPoint[0], _mm_sub_pd(tmp_2, tmp_1));

        /* MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
         *  Inport: '<Root>/States'
         *  Inport: '<Root>/ToWP'
         *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
         */
        if (UAM_FlightMode_norm_d(xyLookaheadPoint) < 2.47032822920623E-323) {
          /* BusCreator: '<S5>/Bus Creator' */
          UAM_FlightMode_B.aacSP.course = UAM_FlightMode_U.States.course;
          h = UAM_FlightMode_DW.obj_b.NumCircles;
        } else {
          p = false;
          p_0 = true;
          b_k = 0;
          exitg1 = false;
          while ((!exitg1) && (b_k < 3)) {
            if (!(UAM_FlightMode_DW.obj_b.OrbitCenterInternal[b_k] ==
                  UAM_FlightMode_U.ToWP.position[b_k])) {
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
            if (UAM_FlightMode_DW.obj_b.OrbitRadiusInternal == rtb_Abs) {
              p = true;
            }

            if (!p) {
              guard1 = true;
            }
          }

          if (guard1) {
            UAM_FlightMode_DW.obj_b.NumCircles = 0.0;
            UAM_FlightMode_DW.obj_b.OrbitCenterInternal[0] =
              UAM_FlightMode_U.ToWP.position[0];
            UAM_FlightMode_DW.obj_b.OrbitCenterInternal[1] =
              UAM_FlightMode_U.ToWP.position[1];
            UAM_FlightMode_DW.obj_b.OrbitCenterInternal[2] =
              UAM_FlightMode_U.ToWP.position[2];
            UAM_FlightMode_DW.obj_b.OrbitRadiusInternal = rtb_Abs;
            UAM_FlightMode_DW.obj_b.SelectTurnDirectionFlag = true;
          }

          if (rtb_Abs <= 30.0) {
            UAM_FlightMode_DW.obj_b.LookaheadDistance = 0.9 * rtb_Abs;
          } else {
            UAM_FlightMode_DW.obj_b.LookaheadDistance = 30.0;
          }

          h = UAM_FlightMode_U.States.Xe[0] - UAM_FlightMode_U.ToWP.position[0];
          b_tmp[0] = h;
          rtb_DotProduct = h * h;
          h = UAM_FlightMode_U.States.Xe[1] - UAM_FlightMode_U.ToWP.position[1];
          b_tmp[1] = h;
          distToCenter = sqrt(h * h + rtb_DotProduct);
          d_tmp = rtb_Abs + UAM_FlightMode_DW.obj_b.LookaheadDistance;
          d = fabs(d_tmp);
          p = (rtIsInf(d) || rtIsNaN(d));
          if (p) {
            rtb_DotProduct = (rtNaN);
          } else if (d < 4.4501477170144028E-308) {
            rtb_DotProduct = 4.94065645841247E-324;
          } else {
            frexp(d, &b_exponent);
            rtb_DotProduct = ldexp(1.0, b_exponent - 53);
          }

          guard1 = false;
          if (distToCenter >= d_tmp - 5.0 * rtb_DotProduct) {
            guard1 = true;
          } else {
            if (p) {
              rtb_DotProduct = (rtNaN);
            } else if (d < 4.4501477170144028E-308) {
              rtb_DotProduct = 4.94065645841247E-324;
            } else {
              frexp(d, &b_exponent_0);
              rtb_DotProduct = ldexp(1.0, b_exponent_0 - 53);
            }

            if (distToCenter <= (rtb_Abs -
                                 UAM_FlightMode_DW.obj_b.LookaheadDistance) +
                5.0 * rtb_DotProduct) {
              guard1 = true;
            } else {
              if (UAM_FlightMode_DW.obj_b.StartFlag) {
                UAM_FlightMode_DW.obj_b.PrevPosition[0] =
                  UAM_FlightMode_U.States.Xe[0];
                UAM_FlightMode_DW.obj_b.PrevPosition[1] =
                  UAM_FlightMode_U.States.Xe[1];
                UAM_FlightMode_DW.obj_b.StartFlag = false;
              }

              if ((UAM_FlightMode_U.ToWP.params[1] == 0.0) &&
                  (!UAM_FlightMode_DW.obj_b.SelectTurnDirectionFlag)) {
                rtb_Sign = UAM_FlightMode_DW.obj_b.TurnDirectionInternal;
              } else {
                rtb_Sign = UAM_FlightMode_U.ToWP.params[1];
              }

              _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
                (UAM_FlightMode_DW.obj_b.PrevPosition[0],
                 UAM_FlightMode_U.States.Xe[0]), _mm_set1_pd
                (UAM_FlightMode_U.ToWP.position[0])));
              xyPose[0] = tmp[0];
              turnVector[0] = tmp[1];
              _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
                (UAM_FlightMode_DW.obj_b.PrevPosition[1],
                 UAM_FlightMode_U.States.Xe[1]), _mm_set1_pd
                (UAM_FlightMode_U.ToWP.position[1])));
              xyPose[1] = tmp[0];
              turnVector[1] = tmp[1];
              d = UAM_FlightMode_norm_d(xyPose);
              h = UAM_FlightMode_DW.obj_b.LookaheadDistance *
                UAM_FlightMode_DW.obj_b.LookaheadDistance;
              rtb_DotProduct = ((h - rtb_Abs * rtb_Abs) + d * d) / (2.0 * d);
              tmp_0 = _mm_set1_pd(d);
              tmp_1 = _mm_sub_pd(tmp_1, tmp_2);
              _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(tmp_1,
                _mm_set1_pd(rtb_DotProduct)), tmp_0), tmp_2));
              rtb_Abs = tmp[1];
              h = sqrt(h - rtb_DotProduct * rtb_DotProduct);
              b_tmp[0] = tmp[0] - (UAM_FlightMode_U.ToWP.position[1] -
                                   UAM_FlightMode_U.States.Xe[1]) * h / d;
              _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd
                (_mm_set_pd(UAM_FlightMode_U.ToWP.position[0],
                            UAM_FlightMode_U.ToWP.position[1]), _mm_set_pd
                 (UAM_FlightMode_U.States.Xe[0], UAM_FlightMode_U.States.Xe[1])),
                _mm_set1_pd(h)), tmp_0), _mm_set_pd(tmp[1], tmp[0])));
              b_tmp[1] = tmp[0];
              rtb_DotProduct = tmp[1];
              d = rtb_Abs - (UAM_FlightMode_U.ToWP.position[0] -
                             UAM_FlightMode_U.States.Xe[0]) * h / d;
              rtb_Product[0] = turnVector[0];
              rtb_Product[1] = turnVector[1];
              rtb_Product[2] = 0.0;
              v[0] = xyLookaheadPoint[0];
              v[1] = xyLookaheadPoint[1];
              v[2] = 0.0;
              if (rtb_Sign < 0.0) {
                rtb_Product[0] = xyLookaheadPoint[0];
                v[0] = turnVector[0];
                rtb_Product[1] = xyLookaheadPoint[1];
                v[1] = turnVector[1];
                rtb_Product[2] = 0.0;
                v[2] = 0.0;
              }

              rtb_Abs = UAM_FlightMode_norm_de(rtb_Product);
              h = UAM_FlightMode_norm_de(v);
              tmp_0 = _mm_set_pd(h, rtb_Abs);
              _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(v[0], rtb_Product[0]),
                tmp_0));
              rtb_Product[0] = tmp[0];
              v[0] = tmp[1];
              _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(v[1], rtb_Product[1]),
                tmp_0));
              v[1] = tmp[1];
              turnVector[2] = rtb_Product[0] * tmp[1] - v[0] * tmp[0];
              UAM_FlightMode_DW.obj_b.PrevPosition[0] =
                UAM_FlightMode_U.States.Xe[0];
              UAM_FlightMode_DW.obj_b.PrevPosition[1] =
                UAM_FlightMode_U.States.Xe[1];
              UAM_FlightMode_DW.obj_b.PrevPosition[2] =
                UAM_FlightMode_U.States.Xe[2];
              UAM_FlightMode_DW.obj_b.NumCircles += rt_atan2d_snf(turnVector[2],
                (rtb_Product[0] * v[0] + tmp[0] * tmp[1]) + 0.0 / rtb_Abs * (0.0
                / h)) / 2.0 / 3.1415926535897931;
              h = UAM_FlightMode_DW.obj_b.NumCircles;
              _mm_storeu_pd(&v[0], tmp_1);
              if (rtIsNaN(rtb_Sign)) {
                rtb_Abs = (rtNaN);
              } else if (rtb_Sign < 0.0) {
                rtb_Abs = -1.0;
              } else {
                rtb_Abs = (rtb_Sign > 0.0);
              }

              switch ((int32_T)rtb_Abs) {
               case 1:
                if ((b_tmp[0] - UAM_FlightMode_U.States.Xe[0]) * v[1] -
                    (rtb_DotProduct - UAM_FlightMode_U.States.Xe[1]) * v[0] >
                    0.0) {
                  rtb_Sign = b_tmp[0];
                  d = rtb_DotProduct;
                } else {
                  rtb_Sign = b_tmp[1];
                }
                break;

               case -1:
                if ((b_tmp[0] - UAM_FlightMode_U.States.Xe[0]) * v[1] -
                    (rtb_DotProduct - UAM_FlightMode_U.States.Xe[1]) * v[0] <
                    0.0) {
                  rtb_Sign = b_tmp[0];
                  d = rtb_DotProduct;
                } else {
                  rtb_Sign = b_tmp[1];
                }
                break;

               default:
                if (fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(rtb_DotProduct -
                       UAM_FlightMode_U.States.Xe[1], b_tmp[0] -
                       UAM_FlightMode_U.States.Xe[0]),
                      UAM_FlightMode_U.States.course)) < fabs
                    (UAM_FlightMode_angdiff(rt_atan2d_snf(d -
                       UAM_FlightMode_U.States.Xe[1], b_tmp[1] -
                       UAM_FlightMode_U.States.Xe[0]),
                      UAM_FlightMode_U.States.course))) {
                  rtb_Sign = b_tmp[0];
                  d = rtb_DotProduct;
                } else {
                  rtb_Sign = b_tmp[1];
                }

                if ((rtb_Sign - UAM_FlightMode_U.States.Xe[0]) * v[1] - (d -
                     UAM_FlightMode_U.States.Xe[1]) * v[0] > 0.0) {
                  UAM_FlightMode_DW.obj_b.TurnDirectionInternal = 1.0;
                } else {
                  UAM_FlightMode_DW.obj_b.TurnDirectionInternal = -1.0;
                }

                UAM_FlightMode_DW.obj_b.SelectTurnDirectionFlag = false;
                break;
              }
            }
          }

          if (guard1) {
            _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_mul_pd(_mm_div_pd(_mm_set_pd(h,
              b_tmp[0]), _mm_set1_pd(UAM_FlightMode_norm_d(b_tmp))), _mm_set1_pd
              (rtb_Abs)), tmp_1));
            rtb_Sign = tmp[0];
            d = tmp[1];
            h = UAM_FlightMode_DW.obj_b.NumCircles;
          }

          /* BusCreator: '<S5>/Bus Creator' */
          UAM_FlightMode_B.aacSP.course = rt_atan2d_snf(d -
            UAM_FlightMode_U.States.Xe[1], rtb_Sign -
            UAM_FlightMode_U.States.Xe[0]);
        }

        /* BusCreator: '<S5>/Bus Creator' incorporates:
         *  Constant: '<S5>/Constant'
         *  Constant: '<S5>/Constant1'
         *  Constant: '<S5>/Lookahead Distance'
         *  Inport: '<Root>/ToWP'
         *  UnaryMinus: '<S5>/Unary Minus'
         */
        UAM_FlightMode_B.aacSP.airspeed = 15.0;
        UAM_FlightMode_B.aacSP.altitude = -UAM_FlightMode_U.ToWP.position[2];
        UAM_FlightMode_B.aacSP.L1 = 30.0;
        UAM_FlightMode_B.aacSP.climbrate = 0.0;

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Abs: '<S5>/Abs1'
         *  DataTypeConversion: '<S5>/Data Type Conversion'
         *  Inport: '<Root>/ToWP'
         *  MATLABSystem: '<S5>/UAV Orbit Follower'
         *  RelationalOperator: '<S5>/Relational Operator'
         * */
        UAM_FlightMode_B.Status = (uint8_T)(fabs(h) >
          UAM_FlightMode_U.ToWP.params[2]);
      } else if (UAM_FlightMode_Y.FlightMode == Hover) {
        UAM_FlightMode_Y.controlMode_m.inTransition = 0U;
        switch (UAM_FlightMode_U.mode) {
         case 2U:
          UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMode_IN_WP;

          /* Switch: '<S17>/Switch' incorporates:
           *  Inport: '<Root>/FromWP'
           *  Product: '<S21>/Product'
           *  RelationalOperator: '<S17>/Equal'
           */
          if (UAM_FlightMode_U.FromWP.mode == 6) {
            rtb_Product[0] = UAM_FlightMode_U.FromWP.position[0];
            rtb_Product[1] = UAM_FlightMode_U.FromWP.position[1];
            rtb_Product[2] = UAM_FlightMode_U.FromWP.position[2];
          } else {
            rtb_Product[0] = 0.0;
            rtb_Product[1] = 0.0;
            rtb_Product[2] = 0.0;
          }

          /* End of Switch: '<S17>/Switch' */

          /* MATLAB Function: '<S12>/MATLAB Function' incorporates:
           *  BusCreator generated from: '<S12>/MATLAB Function'
           *  Inport: '<Root>/FromWP'
           *  Inport: '<Root>/ToWP'
           *  Product: '<S21>/Product'
           */
          rtb_wps_h[0] = rtb_Product[0];
          rtb_wps_h[2] = rtb_Product[1];
          rtb_wps_h[4] = rtb_Product[2];
          rtb_wps_h[6] = UAM_FlightMode_U.FromWP.params[3];
          rtb_wps_h[1] = UAM_FlightMode_U.ToWP.position[0];
          rtb_wps_h[3] = UAM_FlightMode_U.ToWP.position[1];
          rtb_wps_h[5] = UAM_FlightMode_U.ToWP.position[2];
          rtb_wps_h[7] = UAM_FlightMode_U.ToWP.params[3];

          /* BusCreator: '<S12>/Bus Creator' incorporates:
           *  Constant: '<S12>/Constant'
           *  Constant: '<S12>/Lookahead Distance'
           *  Inport: '<Root>/Pose'
           *  MATLABSystem: '<S12>/Waypoint Follower'
           *  Merge: '<S3>/ Merge 2'
           * */
          UAM_FlightMode_SystemCore_step(&UAM_FlightMode_DW.obj,
            UAM_FlightMode_U.Pose, rtb_wps_h, 5.0,
            UAM_FlightMode_B.InnerLoopCmds.LAP,
            &UAM_FlightMode_B.InnerLoopCmds.HeadingCmd, &rtb_Sign,
            &b_varargout_4);
          UAM_FlightMode_B.InnerLoopCmds.YawCmd = 0.0;

          /* Sum: '<S18>/Sum1' incorporates:
           *  Inport: '<Root>/ToWP'
           *  Product: '<S21>/Product'
           */
          rtb_Sign = UAM_FlightMode_U.ToWP.position[0] - rtb_Product[0];
          rtb_Product[0] = rtb_Sign;

          /* DotProduct: '<S21>/Dot Product1' incorporates:
           *  Product: '<S21>/Product'
           */
          rtb_DotProduct = rtb_Sign * rtb_Sign;

          /* Sum: '<S18>/Sum1' incorporates:
           *  Inport: '<Root>/ToWP'
           *  Product: '<S21>/Product'
           */
          rtb_Sign = UAM_FlightMode_U.ToWP.position[1] - rtb_Product[1];
          rtb_Product[1] = rtb_Sign;

          /* DotProduct: '<S21>/Dot Product1' incorporates:
           *  Product: '<S21>/Product'
           */
          rtb_DotProduct += rtb_Sign * rtb_Sign;

          /* Sum: '<S18>/Sum1' incorporates:
           *  Inport: '<Root>/ToWP'
           *  Product: '<S21>/Product'
           */
          rtb_Sign = UAM_FlightMode_U.ToWP.position[2] - rtb_Product[2];

          /* DotProduct: '<S21>/Dot Product1' incorporates:
           *  Product: '<S21>/Product'
           */
          rtb_DotProduct += rtb_Sign * rtb_Sign;

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

          /* Merge: '<S3>/ Merge ' incorporates:
           *  Constant: '<S12>/Lookahead Distance'
           *  DataTypeConversion: '<S12>/Data Type Conversion'
           *  DotProduct: '<S18>/Dot Product'
           *  Inport: '<Root>/Pose'
           *  Inport: '<Root>/ToWP'
           *  Product: '<S21>/Product'
           *  RelationalOperator: '<S12>/Relational Operator'
           *  Sum: '<S18>/Sum'
           *  Sum: '<S18>/Sum1'
           */
          UAM_FlightMode_B.Status = (uint8_T)(((UAM_FlightMode_U.ToWP.position[0]
            - UAM_FlightMode_U.Pose[0]) * (rtb_Product[0] / rtb_DotProduct) +
            (UAM_FlightMode_U.ToWP.position[1] - UAM_FlightMode_U.Pose[1]) *
            (rtb_Product[1] / rtb_DotProduct)) +
            (UAM_FlightMode_U.ToWP.position[2] - UAM_FlightMode_U.Pose[2]) *
            (rtb_Sign / rtb_DotProduct) <= 5.0);
          break;

         case 3U:
          UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMode_IN_Orbit;

          /* Abs: '<S10>/Abs' incorporates:
           *  Inport: '<Root>/ToWP'
           */
          rtb_Abs = fabs(UAM_FlightMode_U.ToWP.params[0]);

          /* Signum: '<S10>/Sign' incorporates:
           *  Inport: '<Root>/ToWP'
           */
          if (rtIsNaN(UAM_FlightMode_U.ToWP.params[1])) {
            rtb_Sign = (rtNaN);
          } else if (UAM_FlightMode_U.ToWP.params[1] < 0.0) {
            rtb_Sign = -1.0;
          } else {
            rtb_Sign = (UAM_FlightMode_U.ToWP.params[1] > 0.0);
          }

          /* End of Signum: '<S10>/Sign' */

          /* MATLABSystem: '<S10>/UAV Orbit Follower' */
          UAM_FlightMode_DW.obj_l.OrbitRadiusFlag = 0U;
          if (rtb_Abs <= 1.0) {
            rtb_Abs = 1.0;
            UAM_FlightMode_DW.obj_l.OrbitRadiusFlag = 1U;
          }

          UAM_FlightMode_DW.obj_l.LookaheadDistFlag = 0U;
          tmp_1 = _mm_loadu_pd(&UAM_FlightMode_U.Pose[0]);
          tmp_2 = _mm_loadu_pd(&UAM_FlightMode_U.ToWP.position[0]);
          _mm_storeu_pd(&xyLookaheadPoint[0], _mm_sub_pd(tmp_1, tmp_2));

          /* MATLABSystem: '<S10>/UAV Orbit Follower' incorporates:
           *  BusCreator: '<S10>/Bus Creator'
           *  Inport: '<Root>/Pose'
           *  Inport: '<Root>/ToWP'
           *  Merge: '<S3>/ Merge 2'
           */
          if (UAM_FlightMode_norm_d(xyLookaheadPoint) < 2.47032822920623E-323) {
            _mm_storeu_pd(&UAM_FlightMode_B.InnerLoopCmds.LAP[0], _mm_add_pd
                          (_mm_mul_pd(_mm_set1_pd(rtb_Abs), _mm_set_pd(sin
              (UAM_FlightMode_U.Pose[3]), cos(UAM_FlightMode_U.Pose[3]))), tmp_1));

            /* BusCreator: '<S10>/Bus Creator' incorporates:
             *  Merge: '<S3>/ Merge 2'
             */
            UAM_FlightMode_B.InnerLoopCmds.LAP[2] =
              UAM_FlightMode_U.ToWP.position[2];
            UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = UAM_FlightMode_U.Pose[3];
            UAM_FlightMode_B.InnerLoopCmds.YawCmd = UAM_FlightMode_U.Pose[3];
            h = UAM_FlightMode_DW.obj_l.NumCircles;
          } else {
            p = false;
            p_0 = true;
            b_k = 0;
            exitg1 = false;
            while ((!exitg1) && (b_k < 3)) {
              if (!(UAM_FlightMode_DW.obj_l.OrbitCenterInternal[b_k] ==
                    UAM_FlightMode_U.ToWP.position[b_k])) {
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
              if (UAM_FlightMode_DW.obj_l.OrbitRadiusInternal == rtb_Abs) {
                p = true;
              }

              if (!p) {
                guard1 = true;
              }
            }

            if (guard1) {
              UAM_FlightMode_DW.obj_l.NumCircles = 0.0;
              UAM_FlightMode_DW.obj_l.OrbitCenterInternal[0] =
                UAM_FlightMode_U.ToWP.position[0];
              UAM_FlightMode_DW.obj_l.OrbitCenterInternal[1] =
                UAM_FlightMode_U.ToWP.position[1];
              UAM_FlightMode_DW.obj_l.OrbitCenterInternal[2] =
                UAM_FlightMode_U.ToWP.position[2];
              UAM_FlightMode_DW.obj_l.OrbitRadiusInternal = rtb_Abs;
              UAM_FlightMode_DW.obj_l.SelectTurnDirectionFlag = true;
            }

            if (rtb_Abs <= 5.0) {
              UAM_FlightMode_DW.obj_l.LookaheadDistance = 0.9 * rtb_Abs;
            } else {
              UAM_FlightMode_DW.obj_l.LookaheadDistance = 5.0;
            }

            h = UAM_FlightMode_U.Pose[0] - UAM_FlightMode_U.ToWP.position[0];
            b_tmp[0] = h;
            rtb_DotProduct = h * h;
            h = UAM_FlightMode_U.Pose[1] - UAM_FlightMode_U.ToWP.position[1];
            b_tmp[1] = h;
            distToCenter = sqrt(h * h + rtb_DotProduct);
            d_tmp = rtb_Abs + UAM_FlightMode_DW.obj_l.LookaheadDistance;
            p = (rtIsInf(d_tmp) || rtIsNaN(d_tmp));
            if (p) {
              rtb_DotProduct = (rtNaN);
            } else if (d_tmp < 4.4501477170144028E-308) {
              rtb_DotProduct = 4.94065645841247E-324;
            } else {
              frexp(d_tmp, &b_exponent);
              rtb_DotProduct = ldexp(1.0, b_exponent - 53);
            }

            guard1 = false;
            if (distToCenter >= d_tmp - 5.0 * rtb_DotProduct) {
              guard1 = true;
            } else {
              if (p) {
                rtb_DotProduct = (rtNaN);
              } else if (d_tmp < 4.4501477170144028E-308) {
                rtb_DotProduct = 4.94065645841247E-324;
              } else {
                frexp(d_tmp, &b_exponent_0);
                rtb_DotProduct = ldexp(1.0, b_exponent_0 - 53);
              }

              if (distToCenter <= (rtb_Abs -
                                   UAM_FlightMode_DW.obj_l.LookaheadDistance) +
                  5.0 * rtb_DotProduct) {
                guard1 = true;
              } else {
                if (UAM_FlightMode_DW.obj_l.StartFlag) {
                  UAM_FlightMode_DW.obj_l.PrevPosition[0] =
                    UAM_FlightMode_U.Pose[0];
                  UAM_FlightMode_DW.obj_l.PrevPosition[1] =
                    UAM_FlightMode_U.Pose[1];
                  UAM_FlightMode_DW.obj_l.StartFlag = false;
                }

                if ((rtb_Sign == 0.0) &&
                    (!UAM_FlightMode_DW.obj_l.SelectTurnDirectionFlag)) {
                  rtb_Sign = UAM_FlightMode_DW.obj_l.TurnDirectionInternal;
                }

                _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
                  (UAM_FlightMode_DW.obj_l.PrevPosition[0],
                   UAM_FlightMode_U.Pose[0]), _mm_set1_pd
                  (UAM_FlightMode_U.ToWP.position[0])));
                xyPose[0] = tmp[0];
                turnVector[0] = tmp[1];
                _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
                  (UAM_FlightMode_DW.obj_l.PrevPosition[1],
                   UAM_FlightMode_U.Pose[1]), _mm_set1_pd
                  (UAM_FlightMode_U.ToWP.position[1])));
                xyPose[1] = tmp[0];
                turnVector[1] = tmp[1];
                d = UAM_FlightMode_norm_d(xyPose);
                h = UAM_FlightMode_DW.obj_l.LookaheadDistance *
                  UAM_FlightMode_DW.obj_l.LookaheadDistance;
                rtb_DotProduct = ((h - rtb_Abs * rtb_Abs) + d * d) / (2.0 * d);
                tmp_0 = _mm_set1_pd(d);
                tmp_2 = _mm_sub_pd(tmp_2, tmp_1);
                _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(tmp_2,
                  _mm_set1_pd(rtb_DotProduct)), tmp_0), tmp_1));
                rtb_Abs = tmp[1];
                h = sqrt(h - rtb_DotProduct * rtb_DotProduct);
                distToCenter = UAM_FlightMode_U.ToWP.position[1] -
                  UAM_FlightMode_U.Pose[1];
                b_tmp[0] = tmp[0] - distToCenter * h / d;
                _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd
                  (_mm_sub_pd(_mm_set_pd(UAM_FlightMode_U.ToWP.position[0],
                  UAM_FlightMode_U.ToWP.position[1]), _mm_set_pd
                              (UAM_FlightMode_U.Pose[0], UAM_FlightMode_U.Pose[1])),
                   _mm_set1_pd(h)), tmp_0), _mm_set_pd(tmp[1], tmp[0])));
                b_tmp[1] = tmp[0];
                rtb_DotProduct = tmp[1];
                d_tmp = UAM_FlightMode_U.ToWP.position[0] -
                  UAM_FlightMode_U.Pose[0];
                d = rtb_Abs - d_tmp * h / d;
                rtb_Product[0] = turnVector[0];
                rtb_Product[1] = turnVector[1];
                rtb_Product[2] = 0.0;
                v[0] = xyLookaheadPoint[0];
                v[1] = xyLookaheadPoint[1];
                v[2] = 0.0;
                if (rtb_Sign < 0.0) {
                  rtb_Product[0] = xyLookaheadPoint[0];
                  v[0] = turnVector[0];
                  rtb_Product[1] = xyLookaheadPoint[1];
                  v[1] = turnVector[1];
                  rtb_Product[2] = 0.0;
                  v[2] = 0.0;
                }

                rtb_Abs = UAM_FlightMode_norm_de(rtb_Product);
                h = UAM_FlightMode_norm_de(v);
                tmp_0 = _mm_set_pd(h, rtb_Abs);
                _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(v[0], rtb_Product[0]),
                  tmp_0));
                rtb_Product[0] = tmp[0];
                v[0] = tmp[1];
                _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(v[1], rtb_Product[1]),
                  tmp_0));
                v[1] = tmp[1];
                turnVector[2] = rtb_Product[0] * tmp[1] - v[0] * tmp[0];
                UAM_FlightMode_DW.obj_l.PrevPosition[0] = UAM_FlightMode_U.Pose
                  [0];
                UAM_FlightMode_DW.obj_l.PrevPosition[1] = UAM_FlightMode_U.Pose
                  [1];
                UAM_FlightMode_DW.obj_l.PrevPosition[2] = UAM_FlightMode_U.Pose
                  [2];
                UAM_FlightMode_DW.obj_l.NumCircles += rt_atan2d_snf(turnVector[2],
                  (rtb_Product[0] * v[0] + tmp[0] * tmp[1]) + 0.0 / rtb_Abs *
                  (0.0 / h)) / 2.0 / 3.1415926535897931;
                h = UAM_FlightMode_DW.obj_l.NumCircles;
                _mm_storeu_pd(&v[0], tmp_2);
                if (rtIsNaN(rtb_Sign)) {
                  rtb_Abs = (rtNaN);
                } else if (rtb_Sign < 0.0) {
                  rtb_Abs = -1.0;
                } else {
                  rtb_Abs = (rtb_Sign > 0.0);
                }

                switch ((int32_T)rtb_Abs) {
                 case 1:
                  if ((b_tmp[0] - UAM_FlightMode_U.Pose[0]) * v[1] -
                      (rtb_DotProduct - UAM_FlightMode_U.Pose[1]) * v[0] > 0.0)
                  {
                    xyLookaheadPoint[0] = b_tmp[0];
                    xyLookaheadPoint[1] = rtb_DotProduct;
                  } else {
                    xyLookaheadPoint[0] = b_tmp[1];
                    xyLookaheadPoint[1] = d;
                  }
                  break;

                 case -1:
                  if ((b_tmp[0] - UAM_FlightMode_U.Pose[0]) * v[1] -
                      (rtb_DotProduct - UAM_FlightMode_U.Pose[1]) * v[0] < 0.0)
                  {
                    xyLookaheadPoint[0] = b_tmp[0];
                    xyLookaheadPoint[1] = rtb_DotProduct;
                  } else {
                    xyLookaheadPoint[0] = b_tmp[1];
                    xyLookaheadPoint[1] = d;
                  }
                  break;

                 default:
                  if (fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(rtb_DotProduct -
                         UAM_FlightMode_U.Pose[1], b_tmp[0] -
                         UAM_FlightMode_U.Pose[0]), UAM_FlightMode_U.Pose[3])) <
                      fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(d -
                         UAM_FlightMode_U.Pose[1], b_tmp[1] -
                         UAM_FlightMode_U.Pose[0]), UAM_FlightMode_U.Pose[3])))
                  {
                    xyLookaheadPoint[0] = b_tmp[0];
                    xyLookaheadPoint[1] = rtb_DotProduct;
                  } else {
                    xyLookaheadPoint[0] = b_tmp[1];
                    xyLookaheadPoint[1] = d;
                  }

                  if ((xyLookaheadPoint[0] - UAM_FlightMode_U.Pose[0]) * v[1] -
                      (xyLookaheadPoint[1] - UAM_FlightMode_U.Pose[1]) * v[0] >
                      0.0) {
                    UAM_FlightMode_DW.obj_l.TurnDirectionInternal = 1.0;
                  } else {
                    UAM_FlightMode_DW.obj_l.TurnDirectionInternal = -1.0;
                  }

                  UAM_FlightMode_DW.obj_l.SelectTurnDirectionFlag = false;
                  break;
                }

                /* BusCreator: '<S10>/Bus Creator' incorporates:
                 *  Merge: '<S3>/ Merge 2'
                 */
                UAM_FlightMode_B.InnerLoopCmds.YawCmd = rt_atan2d_snf
                  (distToCenter, d_tmp);
              }
            }

            if (guard1) {
              _mm_storeu_pd(&xyLookaheadPoint[0], _mm_add_pd(_mm_mul_pd
                (_mm_div_pd(_mm_set_pd(h, b_tmp[0]), _mm_set1_pd
                            (UAM_FlightMode_norm_d(b_tmp))), _mm_set1_pd(rtb_Abs)),
                tmp_2));

              /* BusCreator: '<S10>/Bus Creator' incorporates:
               *  Merge: '<S3>/ Merge 2'
               */
              UAM_FlightMode_B.InnerLoopCmds.YawCmd = rt_atan2d_snf
                (xyLookaheadPoint[1] - UAM_FlightMode_U.Pose[1],
                 xyLookaheadPoint[0] - UAM_FlightMode_U.Pose[0]);
              h = UAM_FlightMode_DW.obj_l.NumCircles;
            }

            /* BusCreator: '<S10>/Bus Creator' incorporates:
             *  Merge: '<S3>/ Merge 2'
             */
            UAM_FlightMode_B.InnerLoopCmds.LAP[0] = xyLookaheadPoint[0];
            UAM_FlightMode_B.InnerLoopCmds.LAP[1] = xyLookaheadPoint[1];
            UAM_FlightMode_B.InnerLoopCmds.LAP[2] =
              UAM_FlightMode_U.ToWP.position[2];
            UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = rt_atan2d_snf
              (xyLookaheadPoint[1] - UAM_FlightMode_U.Pose[1], xyLookaheadPoint
               [0] - UAM_FlightMode_U.Pose[0]);
          }

          /* Merge: '<S3>/ Merge ' incorporates:
           *  DataTypeConversion: '<S10>/Data Type Conversion'
           *  Inport: '<Root>/ToWP'
           *  MATLABSystem: '<S10>/UAV Orbit Follower'
           *  RelationalOperator: '<S10>/Relational Operator'
           * */
          UAM_FlightMode_B.Status = (uint8_T)(h > UAM_FlightMode_U.ToWP.params[2]);
          break;

         case 6U:
          UAM_FlightMode_Y.FlightMode = Transition;
          UAM_FlightMode_Y.controlMode_m.inTransition = 1U;
          UAM_FlightMode_Y.controlMode_m.TransitionCondition = 0U;
          UAM_FlightMode_DW.is_GuidanceLogic = UAM_Flight_IN_ForwardTransition;

          /* BusCreator: '<S7>/Bus Creator1' incorporates:
           *  Constant: '<S7>/Constant1'
           *  Constant: '<S7>/Constant2'
           *  Constant: '<S7>/Constant3'
           *  Merge: '<S3>/ Merge 2'
           */
          UAM_FlightMode_B.InnerLoopCmds.LAP[0] = 0.0;
          UAM_FlightMode_B.InnerLoopCmds.LAP[1] = 0.0;
          UAM_FlightMode_B.InnerLoopCmds.LAP[2] = 0.0;
          UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = 0.0;
          UAM_FlightMode_B.InnerLoopCmds.YawCmd = 0.0;

          /* Merge: '<S3>/ Merge ' incorporates:
           *  Constant: '<S7>/Constant'
           *  DataTypeConversion: '<S7>/Data Type Conversion'
           */
          UAM_FlightMode_B.Status = 0U;
          break;

         default:
          UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMode_IN_Land;
          UAM_FlightMode_DW.is_Land = UAM_FlightMode_IN_ToLand;

          /* MATLAB Function: '<S9>/MATLAB Function' incorporates:
           *  Inport: '<Root>/FromWP'
           *  Inport: '<Root>/ToWP'
           */
          memset(&rtb_wps_l_0[0], 0, sizeof(real_T) << 3U);
          rtb_wps_l_0[0] = UAM_FlightMode_U.FromWP.position[0];
          rtb_wps_l_0[2] = UAM_FlightMode_U.FromWP.position[1];
          rtb_wps_l_0[4] = UAM_FlightMode_U.FromWP.position[2];
          rtb_wps_l_0[6] = UAM_FlightMode_U.FromWP.params[3];
          rtb_wps_l_0[1] = UAM_FlightMode_U.ToWP.position[0];
          rtb_wps_l_0[3] = UAM_FlightMode_U.ToWP.position[1];
          rtb_wps_l_0[5] = UAM_FlightMode_U.ToWP.position[2];
          rtb_wps_l_0[7] = UAM_FlightMode_U.ToWP.params[3];
          UAM_FlightMode_WaypointFollower(UAM_FlightMode_U.Pose, rtb_wps_l_0,
            3.0, &UAM_FlightMode_B.WaypointFollower_c2,
            &UAM_FlightMode_DW.WaypointFollower_c2);

          /* BusCreator: '<S9>/Bus Creator1' incorporates:
           *  Constant: '<S9>/Lookahead Distance'
           *  Inport: '<Root>/Pose'
           *  MATLABSystem: '<S9>/Waypoint Follower'
           *  Merge: '<S3>/ Merge 2'
           */
          UAM_FlightMode_B.InnerLoopCmds.LAP[0] =
            UAM_FlightMode_B.WaypointFollower_c2.LAP[0];
          UAM_FlightMode_B.InnerLoopCmds.LAP[1] =
            UAM_FlightMode_B.WaypointFollower_c2.LAP[1];
          UAM_FlightMode_B.InnerLoopCmds.LAP[2] =
            UAM_FlightMode_B.WaypointFollower_c2.LAP[2];
          UAM_FlightMode_B.InnerLoopCmds.HeadingCmd =
            UAM_FlightMode_B.WaypointFollower_c2.HeadingCmd;
          UAM_FlightMode_B.InnerLoopCmds.YawCmd =
            UAM_FlightMode_B.WaypointFollower_c2.YawCmd;

          /* Merge: '<S3>/ Merge ' incorporates:
           *  Constant: '<S9>/Constant'
           *  SignalConversion generated from: '<S9>/Status'
           */
          UAM_FlightMode_B.Status = 0U;
          break;
        }
      }
      break;
    }
  }

  /* End of Inport: '<Root>/mode' */
}

/* Function for Chart: '<Root>/Guidance Mode Selector' */
static void UAM_exit_internal_GuidanceLogic(void)
{
  switch (UAM_FlightMode_DW.is_GuidanceLogic) {
   case UAM_FlightMod_IN_BackTransition:
    UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    break;

   case UAM_FlightM_IN_FIXED_WING_ORBIT:
    UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    break;

   case UAM_Flig_IN_FIXED_WING_WAYPOINT:
    UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    break;

   case UAM_Flight_IN_ForwardTransition:
    UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    break;

   case UAM_FlightMode_IN_Land:
    switch (UAM_FlightMode_DW.is_Land) {
     case UAM_FlightMode_IN_Descend:
      UAM_FlightMode_DW.is_Land = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
      break;

     case UAM_FlightMode_IN_ToLand:
      UAM_FlightMode_DW.is_Land = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
      break;
    }

    UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    break;

   case UAM_FlightMode_IN_Orbit:
    UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    break;

   case UAM_FlightMode_IN_Takeoff:
    UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    break;

   case UAM_FlightMode_IN_WP:
    UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    break;

   default:
    UAM_FlightMode_DW.is_FIXED_WING_ENTRY = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMo_IN_NO_ACTIVE_CHILD;
    break;
  }
}

/* Function for Chart: '<Root>/Guidance Mode Selector' */
static void UAM_FlightMode_GuidanceLogic(const uint8_T *mode_prev)
{
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_2;
  __m128d tmp_3;
  real_T rtb_wps[8];
  real_T b_waypointsIn_data[6];
  real_T rtb_wps_k[6];
  real_T rtb_Product[3];
  real_T turnVector[3];
  real_T turnVector_0[3];
  real_T unitVectorUtoV_tmp[3];
  real_T v[3];
  real_T b_tmp[2];
  real_T rtb_TmpSignalConversionAtUAVO_0[2];
  real_T tmp[2];
  real_T xyPose[2];
  real_T absxk;
  real_T distToCenter;
  real_T rtb_DotProduct;
  real_T rtb_DotProduct_tmp;
  real_T t;
  real_T turnInternal;
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
  if (*mode_prev != UAM_FlightMode_DW.mode_start) {
    if ((UAM_FlightMode_U.mode == 6) && (UAM_FlightMode_Y.FlightMode ==
         FixedWing)) {
      UAM_exit_internal_GuidanceLogic();
      UAM_FlightMode_DW.temporalCounter_i1 = 0U;
      UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMode_IN_PreTransition;

      /* Outport: '<Root>/FlightMode' */
      UAM_FlightMode_Y.FlightMode = FixedWing;

      /* Outport: '<Root>/controlMode' */
      UAM_FlightMode_Y.controlMode_m.airspeedAltitude = 1U;
      UAM_FlightMode_Y.controlMode_m.attitude = 1U;
      UAM_FlightMode_Y.controlMode_m.lateralGuidance = 1U;
      UAM_FlightMode_B.aacSP.L1 = 25.0;
      UAM_FlightMode_B.aacSP.airspeed = 14.0;

      /* Inport: '<Root>/States' */
      UAM_FlightMode_B.aacSP.altitude = -UAM_FlightMode_U.States.Xe[2];
      UAM_FlightMode_B.aacSP.course = UAM_FlightMode_U.States.course;
    } else if ((UAM_FlightMode_U.mode == 2) && (UAM_FlightMode_Y.FlightMode ==
                FixedWing)) {
      /* Merge: '<S3>/ Merge ' */
      UAM_FlightMode_B.Status = 0U;
      UAM_exit_internal_GuidanceLogic();
      UAM_FlightMode_DW.is_GuidanceLogic = UAM_Flig_IN_FIXED_WING_WAYPOINT;

      /* Outport: '<Root>/controlMode' */
      UAM_FlightMode_Y.controlMode_m.airspeedAltitude = 1U;
      UAM_FlightMode_Y.controlMode_m.attitude = 1U;
      UAM_FlightMode_Y.controlMode_m.lateralGuidance = 1U;

      /* MATLAB Function: '<S6>/MATLAB Function' incorporates:
       *  Inport: '<Root>/FromWP'
       *  Inport: '<Root>/ToWP'
       */
      rtb_wps_k[0] = UAM_FlightMode_U.FromWP.position[0];
      rtb_wps_k[1] = UAM_FlightMode_U.ToWP.position[0];
      rtb_wps_k[2] = UAM_FlightMode_U.FromWP.position[1];
      rtb_wps_k[3] = UAM_FlightMode_U.ToWP.position[1];
      rtb_wps_k[4] = UAM_FlightMode_U.FromWP.position[2];
      rtb_wps_k[5] = UAM_FlightMode_U.ToWP.position[2];

      /* MATLABSystem: '<S6>/Waypoint Follower' incorporates:
       *  Inport: '<Root>/FromWP'
       *  Inport: '<Root>/ToWP'
       *  MATLAB Function: '<S6>/MATLAB Function'
       */
      UAM_FlightMode_DW.obj_f.LookaheadDistFlag = 0U;
      UAM_FlightMode_DW.obj_f.InitialPose[0] = 0.0;
      UAM_FlightMode_DW.obj_f.InitialPose[1] = 0.0;
      UAM_FlightMode_DW.obj_f.InitialPose[2] = 0.0;
      UAM_FlightMode_DW.obj_f.InitialPose[3] = 0.0;
      UAM_FlightMode_DW.obj_f.NumWaypoints = 2.0;
      p = false;
      p_0 = true;
      b_k = 0;
      exitg1 = false;
      while ((!exitg1) && (b_k <= 5)) {
        i = ((b_k / 2) << 1) + b_k % 2;
        if (!(UAM_FlightMode_DW.obj_f.WaypointsInternal[i] == rtb_wps_k[i])) {
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
          UAM_FlightMode_DW.obj_f.WaypointsInternal[i] = rtb_wps_k[i];
        }

        UAM_FlightMode_DW.obj_f.WaypointIndex = 1.0;
      }

      distinctWptsIdx[1] = true;
      x[0] = (UAM_FlightMode_U.FromWP.position[0] !=
              UAM_FlightMode_U.ToWP.position[0]);
      x[1] = (UAM_FlightMode_U.FromWP.position[1] !=
              UAM_FlightMode_U.ToWP.position[1]);
      x[2] = (UAM_FlightMode_U.FromWP.position[2] !=
              UAM_FlightMode_U.ToWP.position[2]);
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
       *  Inport: '<Root>/States'
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

      UAM_FlightMode_DW.obj_f.LookaheadDistance = 30.0;
      if (tmp_size_idx_1 == 0) {
        /* BusCreator: '<S6>/Bus Creator' incorporates:
         *  Inport: '<Root>/States'
         *  SignalConversion generated from: '<S6>/Waypoint Follower'
         */
        UAM_FlightMode_B.aacSP.course = UAM_FlightMode_U.States.course;

        /* Merge: '<S3>/ Merge ' */
        UAM_FlightMode_B.Status = 1U;
      } else {
        guard2 = false;
        if (tmp_size_idx_1 == 1) {
          if (UAM_FlightMode_DW.obj_f.StartFlag) {
            UAM_FlightMode_DW.obj_f.InitialPose[0] = UAM_FlightMode_U.States.Xe
              [0];
            UAM_FlightMode_DW.obj_f.InitialPose[1] = UAM_FlightMode_U.States.Xe
              [1];
            UAM_FlightMode_DW.obj_f.InitialPose[2] = UAM_FlightMode_U.States.Xe
              [2];
            UAM_FlightMode_DW.obj_f.InitialPose[3] =
              UAM_FlightMode_U.States.course;
          }

          tmp_1 = _mm_sub_pd(_mm_loadu_pd(&b_waypointsIn_data[0]), _mm_loadu_pd(
            &UAM_FlightMode_U.States.Xe[0]));
          _mm_storeu_pd(&rtb_Product[0], tmp_1);
          rtb_Product[2] = b_waypointsIn_data[2] - UAM_FlightMode_U.States.Xe[2];
          if (UAM_FlightMode_norm_de(rtb_Product) < 1.4901161193847656E-8) {
            /* BusCreator: '<S6>/Bus Creator' incorporates:
             *  Inport: '<Root>/States'
             *  SignalConversion generated from: '<S6>/Waypoint Follower'
             */
            UAM_FlightMode_B.aacSP.course = UAM_FlightMode_U.States.course;

            /* Merge: '<S3>/ Merge ' */
            UAM_FlightMode_B.Status = 1U;
            UAM_FlightMode_DW.obj_f.StartFlag = false;
          } else {
            UAM_FlightMode_DW.obj_f.StartFlag = false;
            UAM_FlightMode_DW.obj_f.NumWaypoints = 2.0;
            scalarLB = tmp_size_idx_1 + 1;
            for (i = 0; i < 3; i++) {
              vectorUB = (tmp_size_idx_1 + 1) * i;
              rtb_wps_k[vectorUB] = UAM_FlightMode_DW.obj_f.InitialPose[i];
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
          if (UAM_FlightMode_DW.obj_f.WaypointIndex == 2.0) {
            p = true;
          }

          if (p) {
            UAM_FlightMode_DW.obj_f.LastWaypointFlag = true;
            UAM_FlightMode_DW.obj_f.WaypointIndex--;
          }

          rtb_Product[0] = rtb_wps_k[(int32_T)
            UAM_FlightMode_DW.obj_f.WaypointIndex - 1];
          v[0] = rtb_wps_k[(int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0)
            - 1];
          rtb_Product[1] = rtb_wps_k[((int32_T)
            UAM_FlightMode_DW.obj_f.WaypointIndex + scalarLB) - 1];
          v[1] = rtb_wps_k[((int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex +
            1.0) + scalarLB) - 1];
          rtb_Product[2] = rtb_wps_k[((scalarLB << 1) + (int32_T)
            UAM_FlightMode_DW.obj_f.WaypointIndex) - 1];
          rtb_DotProduct = rtb_wps_k[((int32_T)
            (UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0) + (scalarLB << 1)) - 1];
          v[2] = rtb_DotProduct;
          tmp_0 = _mm_loadu_pd(&UAM_FlightMode_U.States.Xe[0]);
          _mm_storeu_pd(&turnVector_0[0], _mm_sub_pd(tmp_0, _mm_set_pd
            (rtb_wps_k[((int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0) +
                        scalarLB) - 1], rtb_wps_k[(int32_T)
             (UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0) - 1])));
          turnVector_0[2] = UAM_FlightMode_U.States.Xe[2] - rtb_DotProduct;
          guard3 = false;
          if (UAM_FlightMode_norm_de(turnVector_0) <= 10.0) {
            guard3 = true;
          } else {
            turnInternal = rtb_wps_k[(int32_T)
              (UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0) - 1];
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
              (UAM_FlightMode_U.States.Xe[0], turnInternal), _mm_set_pd
              (turnInternal, rtb_wps_k[(int32_T)
               UAM_FlightMode_DW.obj_f.WaypointIndex - 1])));
            unitVectorUtoV_tmp[0] = tmp[0];
            turnVector[0] = tmp[1];
            turnInternal = rtb_wps_k[((int32_T)
              (UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0) + scalarLB) - 1];
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
              (UAM_FlightMode_U.States.Xe[1], turnInternal), _mm_set_pd
              (turnInternal, rtb_wps_k[((int32_T)
              UAM_FlightMode_DW.obj_f.WaypointIndex + scalarLB) - 1])));
            unitVectorUtoV_tmp[1] = tmp[0];
            turnVector[1] = tmp[1];
            turnInternal = rtb_wps_k[((int32_T)
              (UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0) + (scalarLB << 1)) -
              1];
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
              (UAM_FlightMode_U.States.Xe[2], turnInternal), _mm_set_pd
              (turnInternal, rtb_wps_k[((int32_T)
              UAM_FlightMode_DW.obj_f.WaypointIndex + (scalarLB << 1)) - 1])));
            unitVectorUtoV_tmp[2] = tmp[0];
            turnVector[2] = tmp[1];
            distToCenter = UAM_FlightMode_norm_de(unitVectorUtoV_tmp);
            t = UAM_FlightMode_norm_de(turnVector);
            turnInternal = (unitVectorUtoV_tmp[0] / distToCenter * (turnVector[0]
              / t) + unitVectorUtoV_tmp[1] / distToCenter * (turnVector[1] / t))
              + tmp[0] / distToCenter * (tmp[1] / t);
            if (rtIsNaN(turnInternal) || (turnInternal < 0.0)) {
            } else {
              guard3 = true;
            }
          }

          if (guard3) {
            UAM_FlightMode_DW.obj_f.WaypointIndex++;
            p = false;
            if (UAM_FlightMode_DW.obj_f.WaypointIndex == 2.0) {
              p = true;
            }

            if (p) {
              UAM_FlightMode_DW.obj_f.LastWaypointFlag = true;
              UAM_FlightMode_DW.obj_f.WaypointIndex--;
            }

            rtb_Product[0] = rtb_wps_k[(int32_T)
              UAM_FlightMode_DW.obj_f.WaypointIndex - 1];
            v[0] = rtb_wps_k[(int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex +
              1.0) - 1];
            rtb_Product[1] = rtb_wps_k[((int32_T)
              UAM_FlightMode_DW.obj_f.WaypointIndex + scalarLB) - 1];
            v[1] = rtb_wps_k[((int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex +
              1.0) + scalarLB) - 1];
            rtb_Product[2] = rtb_wps_k[((scalarLB << 1) + (int32_T)
              UAM_FlightMode_DW.obj_f.WaypointIndex) - 1];
            v[2] = rtb_wps_k[((int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex +
              1.0) + (scalarLB << 1)) - 1];
          }

          _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(v[0],
            UAM_FlightMode_U.States.Xe[0]), _mm_set1_pd(rtb_Product[0])));
          turnVector[0] = tmp[0];
          unitVectorUtoV_tmp[0] = tmp[1];
          _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(v[1],
            UAM_FlightMode_U.States.Xe[1]), _mm_set1_pd(rtb_Product[1])));
          turnVector[1] = tmp[0];
          unitVectorUtoV_tmp[1] = tmp[1];
          _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(v[2],
            UAM_FlightMode_U.States.Xe[2]), _mm_set1_pd(rtb_Product[2])));
          turnVector[2] = tmp[0];
          unitVectorUtoV_tmp[2] = tmp[1];
          turnInternal = unitVectorUtoV_tmp[0] * unitVectorUtoV_tmp[0] +
            unitVectorUtoV_tmp[1] * unitVectorUtoV_tmp[1];
          rtb_DotProduct = tmp[1] * tmp[1] + turnInternal;
          absxk = ((turnVector[0] * unitVectorUtoV_tmp[0] + turnVector[1] *
                    unitVectorUtoV_tmp[1]) + tmp[0] * tmp[1]) / rtb_DotProduct;
          if (absxk < 0.0) {
            absxk = UAM_FlightMode_norm_de(turnVector);
          } else if (absxk > 1.0) {
            tmp_1 = _mm_sub_pd(tmp_0, _mm_loadu_pd(&v[0]));
            _mm_storeu_pd(&turnVector_0[0], tmp_1);
            turnVector_0[2] = UAM_FlightMode_U.States.Xe[2] - v[2];
            absxk = UAM_FlightMode_norm_de(turnVector_0);
          } else {
            tmp_1 = _mm_sub_pd(tmp_0, _mm_add_pd(_mm_mul_pd(_mm_set1_pd(absxk),
              _mm_loadu_pd(&unitVectorUtoV_tmp[0])), _mm_loadu_pd(&rtb_Product[0])));
            _mm_storeu_pd(&turnVector_0[0], tmp_1);
            turnVector_0[2] = UAM_FlightMode_U.States.Xe[2] - (absxk * tmp[1] +
              rtb_Product[2]);
            absxk = UAM_FlightMode_norm_de(turnVector_0);
          }

          if (UAM_FlightMode_DW.obj_f.LastWaypointFlag) {
            absxk = (((UAM_FlightMode_U.States.Xe[0] - rtb_Product[0]) *
                      unitVectorUtoV_tmp[0] + (UAM_FlightMode_U.States.Xe[1] -
                       rtb_Product[1]) * unitVectorUtoV_tmp[1]) +
                     (UAM_FlightMode_U.States.Xe[2] - rtb_Product[2]) * tmp[1]) /
              rtb_DotProduct;
            tmp_1 = _mm_sub_pd(tmp_0, _mm_add_pd(_mm_mul_pd(_mm_set1_pd(absxk),
              _mm_loadu_pd(&unitVectorUtoV_tmp[0])), _mm_loadu_pd(&rtb_Product[0])));
            _mm_storeu_pd(&turnVector_0[0], tmp_1);
            turnVector_0[2] = UAM_FlightMode_U.States.Xe[2] - (absxk * tmp[1] +
              rtb_Product[2]);
            absxk = UAM_FlightMode_norm_de(turnVector_0);
          }

          t = fabs(absxk);
          if (rtIsInf(t) || rtIsNaN(t)) {
            rtb_DotProduct = (rtNaN);
            t = (rtNaN);
          } else if (t < 4.4501477170144028E-308) {
            rtb_DotProduct = 4.94065645841247E-324;
            t = 4.94065645841247E-324;
          } else {
            frexp(t, &b_exponent);
            rtb_DotProduct = ldexp(1.0, b_exponent - 53);
            frexp(t, &b_exponent_0);
            t = ldexp(1.0, b_exponent_0 - 53);
          }

          rtb_DotProduct = sqrt(rtb_DotProduct);
          t *= 5.0;
          if ((rtb_DotProduct >= t) || rtIsNaN(t)) {
            t = rtb_DotProduct;
          }

          if (absxk + t >= 30.0) {
            UAM_FlightMode_DW.obj_f.LookaheadDistance =
              UAM_FlightMode_DW.obj_f.LookaheadFactor * absxk;
          }

          turnVector[0] = unitVectorUtoV_tmp[0];
          turnVector[1] = unitVectorUtoV_tmp[1];
          tmp_1 = _mm_sub_pd(_mm_loadu_pd(&rtb_Product[0]), tmp_0);
          _mm_storeu_pd(&unitVectorUtoV_tmp[0], tmp_1);
          turnInternal += unitVectorUtoV_tmp[2] * unitVectorUtoV_tmp[2];
          unitVectorUtoV_tmp[2] = rtb_Product[2] - UAM_FlightMode_U.States.Xe[2];
          absxk = ((turnVector[0] * unitVectorUtoV_tmp[0] + turnVector[1] *
                    unitVectorUtoV_tmp[1]) + tmp[1] * unitVectorUtoV_tmp[2]) *
            2.0;
          t = sqrt(absxk * absxk - (((unitVectorUtoV_tmp[0] *
                      unitVectorUtoV_tmp[0] + unitVectorUtoV_tmp[1] *
                      unitVectorUtoV_tmp[1]) + unitVectorUtoV_tmp[2] *
                     unitVectorUtoV_tmp[2]) -
                    UAM_FlightMode_DW.obj_f.LookaheadDistance *
                    UAM_FlightMode_DW.obj_f.LookaheadDistance) * (4.0 *
                    turnInternal));
          rtb_DotProduct = (-absxk + t) / 2.0 / turnInternal;
          turnInternal = (-absxk - t) / 2.0 / turnInternal;
          if ((rtb_DotProduct >= turnInternal) || rtIsNaN(turnInternal)) {
            turnInternal = rtb_DotProduct;
          }

          tmp_1 = _mm_set1_pd(turnInternal);
          tmp_0 = _mm_add_pd(_mm_mul_pd(_mm_sub_pd(_mm_set1_pd(1.0), tmp_1),
            _mm_loadu_pd(&rtb_Product[0])), _mm_mul_pd(tmp_1, _mm_loadu_pd(&v[0])));
          _mm_storeu_pd(&v[0], tmp_0);

          /* BusCreator: '<S6>/Bus Creator' incorporates:
           *  Inport: '<Root>/States'
           *  SignalConversion generated from: '<S6>/Waypoint Follower'
           */
          UAM_FlightMode_B.aacSP.course = rt_atan2d_snf(v[1] -
            UAM_FlightMode_U.States.Xe[1], v[0] - UAM_FlightMode_U.States.Xe[0]);
          p = false;
          if (UAM_FlightMode_DW.obj_f.LastWaypointFlag) {
            p = true;
          }

          if (p) {
            /* Merge: '<S3>/ Merge ' */
            UAM_FlightMode_B.Status = 1U;
          }

          UAM_FlightMode_DW.obj_f.LastWaypointFlag = false;
        }
      }

      /* BusCreator: '<S6>/Bus Creator' incorporates:
       *  Constant: '<S6>/Constant'
       *  Constant: '<S6>/Constant1'
       *  Constant: '<S6>/Lookahead Distance'
       *  Inport: '<Root>/ToWP'
       *  MATLAB Function: '<S6>/MATLAB Function'
       */
      UAM_FlightMode_B.aacSP.airspeed = 15.0;
      UAM_FlightMode_B.aacSP.altitude = -UAM_FlightMode_U.ToWP.position[2];
      UAM_FlightMode_B.aacSP.L1 = 30.0;
      UAM_FlightMode_B.aacSP.climbrate = 0.0;
    } else if (UAM_FlightMode_Y.FlightMode == Transition) {
      /* Outport: '<Root>/controlMode' */
      UAM_FlightMode_Y.controlMode_m.inTransition = 0U;
      UAM_exit_internal_GuidanceLogic();
      UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightM_IN_FIXED_WING_ENTRY;
      enter_internal_FIXED_WING_ENTRY();
    } else if (UAM_FlightMode_Y.FlightMode == BackTransition) {
      UAM_exit_internal_GuidanceLogic();
      UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMode_IN_HOVER_ENTRY;

      /* Outport: '<Root>/FlightMode' */
      UAM_FlightMode_Y.FlightMode = Hover;

      /* Outport: '<Root>/controlMode' */
      UAM_FlightMode_Y.controlMode_m.inTransition = 0U;
    } else if ((UAM_FlightMode_U.mode == 1) && (UAM_FlightMode_Y.FlightMode ==
                Hover)) {
      /* Outport: '<Root>/controlMode' */
      UAM_FlightMode_Y.controlMode_m.inTransition = 0U;
      UAM_exit_internal_GuidanceLogic();
      UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMode_IN_Takeoff;

      /* BusCreator: '<S11>/Bus Creator' incorporates:
       *  Inport: '<Root>/Pose'
       *  Inport: '<Root>/ToWP'
       *  Merge: '<S3>/ Merge 2'
       *  SignalConversion generated from: '<S11>/Bus Creator'
       */
      UAM_FlightMode_B.InnerLoopCmds.LAP[0] = UAM_FlightMode_U.Pose[0];
      UAM_FlightMode_B.InnerLoopCmds.LAP[1] = UAM_FlightMode_U.Pose[1];
      UAM_FlightMode_B.InnerLoopCmds.LAP[2] = UAM_FlightMode_U.ToWP.position[2];

      /* Saturate: '<S11>/Hdg. Cmd Sat' incorporates:
       *  Inport: '<Root>/ToWP'
       */
      if (UAM_FlightMode_U.ToWP.params[3] > 3.1415926535897931) {
        /* BusCreator: '<S11>/Bus Creator' incorporates:
         *  Merge: '<S3>/ Merge 2'
         */
        UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = 3.1415926535897931;
      } else if (UAM_FlightMode_U.ToWP.params[3] < -3.1415926535897931) {
        /* BusCreator: '<S11>/Bus Creator' incorporates:
         *  Merge: '<S3>/ Merge 2'
         */
        UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = -3.1415926535897931;
      } else {
        /* BusCreator: '<S11>/Bus Creator' incorporates:
         *  Merge: '<S3>/ Merge 2'
         */
        UAM_FlightMode_B.InnerLoopCmds.HeadingCmd =
          UAM_FlightMode_U.ToWP.params[3];
      }

      /* BusCreator: '<S11>/Bus Creator' incorporates:
       *  Merge: '<S3>/ Merge 2'
       */
      UAM_FlightMode_B.InnerLoopCmds.YawCmd = UAM_FlightMode_ConstB.YawCmd;

      /* Merge: '<S3>/ Merge ' incorporates:
       *  Constant: '<S11>/Constant'
       *  SignalConversion generated from: '<S11>/Status'
       */
      UAM_FlightMode_B.Status = 0U;
    } else if ((UAM_FlightMode_U.mode == 3) && (UAM_FlightMode_Y.FlightMode ==
                FixedWing)) {
      UAM_exit_internal_GuidanceLogic();
      UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightM_IN_FIXED_WING_ORBIT;

      /* Outport: '<Root>/controlMode' */
      UAM_FlightMode_Y.controlMode_m.airspeedAltitude = 1U;
      UAM_FlightMode_Y.controlMode_m.attitude = 1U;
      UAM_FlightMode_Y.controlMode_m.lateralGuidance = 1U;

      /* MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
       *  Inport: '<Root>/ToWP'
       */
      t = UAM_FlightMode_U.ToWP.params[0];
      UAM_FlightMode_DW.obj_b.OrbitRadiusFlag = 0U;
      if (UAM_FlightMode_U.ToWP.params[0] <= 50.0) {
        t = 50.0;
        UAM_FlightMode_DW.obj_b.OrbitRadiusFlag = 1U;
      }

      UAM_FlightMode_DW.obj_b.LookaheadDistFlag = 0U;

      /* SignalConversion generated from: '<S5>/UAV Orbit Follower' incorporates:
       *  Inport: '<Root>/States'
       *  Inport: '<Root>/ToWP'
       *  MATLABSystem: '<S5>/UAV Orbit Follower'
       * */
      tmp_0 = _mm_sub_pd(_mm_loadu_pd(&UAM_FlightMode_U.States.Xe[0]),
                         _mm_loadu_pd(&UAM_FlightMode_U.ToWP.position[0]));

      /* MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
       *  Inport: '<Root>/States'
       *  Inport: '<Root>/ToWP'
       *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
       */
      _mm_storeu_pd(&rtb_TmpSignalConversionAtUAVO_0[0], tmp_0);
      if (UAM_FlightMode_norm_d(rtb_TmpSignalConversionAtUAVO_0) <
          2.47032822920623E-323) {
        /* BusCreator: '<S5>/Bus Creator' incorporates:
         *  Inport: '<Root>/States'
         *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
         */
        UAM_FlightMode_B.aacSP.course = UAM_FlightMode_U.States.course;
        t = UAM_FlightMode_DW.obj_b.NumCircles;
      } else {
        p = false;
        p_0 = true;
        b_k = 0;
        exitg1 = false;
        while ((!exitg1) && (b_k < 3)) {
          if (!(UAM_FlightMode_DW.obj_b.OrbitCenterInternal[b_k] ==
                UAM_FlightMode_U.ToWP.position[b_k])) {
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
          if (UAM_FlightMode_DW.obj_b.OrbitRadiusInternal == t) {
            p = true;
          }

          if (!p) {
            guard2 = true;
          }
        }

        if (guard2) {
          UAM_FlightMode_DW.obj_b.NumCircles = 0.0;
          UAM_FlightMode_DW.obj_b.OrbitCenterInternal[0] =
            UAM_FlightMode_U.ToWP.position[0];
          UAM_FlightMode_DW.obj_b.OrbitCenterInternal[1] =
            UAM_FlightMode_U.ToWP.position[1];
          UAM_FlightMode_DW.obj_b.OrbitCenterInternal[2] =
            UAM_FlightMode_U.ToWP.position[2];
          UAM_FlightMode_DW.obj_b.OrbitRadiusInternal = t;
          UAM_FlightMode_DW.obj_b.SelectTurnDirectionFlag = true;
        }

        if (t <= 30.0) {
          UAM_FlightMode_DW.obj_b.LookaheadDistance = 0.9 * t;
        } else {
          UAM_FlightMode_DW.obj_b.LookaheadDistance = 30.0;
        }

        /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
         *  Inport: '<Root>/States'
         *  Inport: '<Root>/ToWP'
         *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
         */
        absxk = UAM_FlightMode_U.States.Xe[0] - UAM_FlightMode_U.ToWP.position[0];
        b_tmp[0] = absxk;
        rtb_DotProduct = absxk * absxk;
        absxk = UAM_FlightMode_U.States.Xe[1] - UAM_FlightMode_U.ToWP.position[1];
        b_tmp[1] = absxk;
        distToCenter = sqrt(absxk * absxk + rtb_DotProduct);
        rtb_DotProduct = fabs(t + UAM_FlightMode_DW.obj_b.LookaheadDistance);
        if (rtIsInf(rtb_DotProduct) || rtIsNaN(rtb_DotProduct)) {
          rtb_DotProduct = (rtNaN);
        } else if (rtb_DotProduct < 4.4501477170144028E-308) {
          rtb_DotProduct = 4.94065645841247E-324;
        } else {
          frexp(rtb_DotProduct, &b_exponent);
          rtb_DotProduct = ldexp(1.0, b_exponent - 53);
        }

        guard2 = false;
        if (distToCenter >= (t + UAM_FlightMode_DW.obj_b.LookaheadDistance) -
            5.0 * rtb_DotProduct) {
          guard2 = true;
        } else {
          rtb_DotProduct = fabs(t + UAM_FlightMode_DW.obj_b.LookaheadDistance);
          if (rtIsInf(rtb_DotProduct) || rtIsNaN(rtb_DotProduct)) {
            rtb_DotProduct = (rtNaN);
          } else if (rtb_DotProduct < 4.4501477170144028E-308) {
            rtb_DotProduct = 4.94065645841247E-324;
          } else {
            frexp(rtb_DotProduct, &b_exponent_0);
            rtb_DotProduct = ldexp(1.0, b_exponent_0 - 53);
          }

          if (distToCenter <= (t - UAM_FlightMode_DW.obj_b.LookaheadDistance) +
              5.0 * rtb_DotProduct) {
            guard2 = true;
          } else {
            if (UAM_FlightMode_DW.obj_b.StartFlag) {
              UAM_FlightMode_DW.obj_b.PrevPosition[0] =
                UAM_FlightMode_U.States.Xe[0];
              UAM_FlightMode_DW.obj_b.PrevPosition[1] =
                UAM_FlightMode_U.States.Xe[1];
              UAM_FlightMode_DW.obj_b.PrevPosition[2] =
                UAM_FlightMode_U.States.Xe[2];
              UAM_FlightMode_DW.obj_b.StartFlag = false;
            }

            if ((UAM_FlightMode_U.ToWP.params[1] == 0.0) &&
                (!UAM_FlightMode_DW.obj_b.SelectTurnDirectionFlag)) {
              turnInternal = UAM_FlightMode_DW.obj_b.TurnDirectionInternal;
            } else {
              turnInternal = UAM_FlightMode_U.ToWP.params[1];
            }

            rtb_Product[2] = 0.0;
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
              (UAM_FlightMode_DW.obj_b.PrevPosition[0],
               UAM_FlightMode_U.States.Xe[0]), _mm_set1_pd
              (UAM_FlightMode_U.ToWP.position[0])));
            xyPose[0] = tmp[0];
            rtb_Product[0] = tmp[1];
            _mm_storeu_pd(&tmp[0], tmp_0);
            v[0] = tmp[0];
            xyPose[1] = tmp[1];
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
              (UAM_FlightMode_U.States.Xe[1],
               UAM_FlightMode_DW.obj_b.PrevPosition[1]), _mm_set1_pd
              (UAM_FlightMode_U.ToWP.position[1])));
            rtb_Product[1] = tmp[0];
            v[1] = tmp[1];
            absxk = UAM_FlightMode_norm_d(xyPose);
            rtb_DotProduct_tmp = UAM_FlightMode_DW.obj_b.LookaheadDistance *
              UAM_FlightMode_DW.obj_b.LookaheadDistance;
            rtb_DotProduct = ((rtb_DotProduct_tmp - t * t) + absxk * absxk) /
              (2.0 * absxk);
            tmp_1 = _mm_set1_pd(absxk);
            tmp_0 = _mm_sub_pd(_mm_loadu_pd(&UAM_FlightMode_U.ToWP.position[0]),
                               _mm_loadu_pd(&UAM_FlightMode_U.States.Xe[0]));
            _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(tmp_0,
              _mm_set1_pd(rtb_DotProduct)), tmp_1), _mm_loadu_pd
              (&UAM_FlightMode_U.States.Xe[0])));
            t = tmp[1];
            distToCenter = sqrt(rtb_DotProduct_tmp - rtb_DotProduct *
                                rtb_DotProduct);
            b_tmp[0] = tmp[0] - (UAM_FlightMode_U.ToWP.position[1] -
                                 UAM_FlightMode_U.States.Xe[1]) * distToCenter /
              absxk;
            _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd
              (_mm_set_pd(UAM_FlightMode_U.ToWP.position[0],
                          UAM_FlightMode_U.ToWP.position[1]), _mm_set_pd
               (UAM_FlightMode_U.States.Xe[0], UAM_FlightMode_U.States.Xe[1])),
              _mm_set1_pd(distToCenter)), tmp_1), _mm_set_pd(tmp[1], tmp[0])));
            b_tmp[1] = tmp[0];
            rtb_DotProduct = tmp[1];
            absxk = t - (UAM_FlightMode_U.ToWP.position[0] -
                         UAM_FlightMode_U.States.Xe[0]) * distToCenter / absxk;
            v[2] = 0.0;
            if (turnInternal < 0.0) {
              rtb_Product[0] = v[0];
              rtb_Product[1] = v[1];
              rtb_Product[2] = 0.0;
              tmp_1 = _mm_sub_pd(_mm_loadu_pd
                                 (&UAM_FlightMode_DW.obj_b.PrevPosition[0]),
                                 _mm_loadu_pd(&UAM_FlightMode_U.ToWP.position[0]));
              _mm_storeu_pd(&v[0], tmp_1);
              v[2] = 0.0;
            }

            distToCenter = UAM_FlightMode_norm_de(rtb_Product);
            t = UAM_FlightMode_norm_de(v);
            tmp_1 = _mm_set_pd(t, distToCenter);
            _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(v[0], rtb_Product[0]),
              tmp_1));
            rtb_Product[0] = tmp[0];
            v[0] = tmp[1];
            _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(v[1], rtb_Product[1]),
              tmp_1));
            v[1] = tmp[1];
            turnVector[2] = rtb_Product[0] * tmp[1] - v[0] * tmp[0];
            UAM_FlightMode_DW.obj_b.PrevPosition[0] =
              UAM_FlightMode_U.States.Xe[0];
            UAM_FlightMode_DW.obj_b.PrevPosition[1] =
              UAM_FlightMode_U.States.Xe[1];
            UAM_FlightMode_DW.obj_b.PrevPosition[2] =
              UAM_FlightMode_U.States.Xe[2];
            UAM_FlightMode_DW.obj_b.NumCircles += rt_atan2d_snf(turnVector[2],
              (rtb_Product[0] * v[0] + tmp[0] * tmp[1]) + 0.0 / distToCenter *
              (0.0 / t)) / 2.0 / 3.1415926535897931;
            t = UAM_FlightMode_DW.obj_b.NumCircles;
            _mm_storeu_pd(&v[0], tmp_0);
            if (rtIsNaN(turnInternal)) {
              distToCenter = (rtNaN);
            } else if (turnInternal < 0.0) {
              distToCenter = -1.0;
            } else {
              distToCenter = (turnInternal > 0.0);
            }

            switch ((int32_T)distToCenter) {
             case 1:
              if ((b_tmp[0] - UAM_FlightMode_U.States.Xe[0]) * v[1] -
                  (rtb_DotProduct - UAM_FlightMode_U.States.Xe[1]) * v[0] > 0.0)
              {
                turnInternal = b_tmp[0];
                absxk = rtb_DotProduct;
              } else {
                turnInternal = b_tmp[1];
              }
              break;

             case -1:
              if ((b_tmp[0] - UAM_FlightMode_U.States.Xe[0]) * v[1] -
                  (rtb_DotProduct - UAM_FlightMode_U.States.Xe[1]) * v[0] < 0.0)
              {
                turnInternal = b_tmp[0];
                absxk = rtb_DotProduct;
              } else {
                turnInternal = b_tmp[1];
              }
              break;

             default:
              if (fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(rtb_DotProduct -
                     UAM_FlightMode_U.States.Xe[1], b_tmp[0] -
                     UAM_FlightMode_U.States.Xe[0]),
                    UAM_FlightMode_U.States.course)) < fabs
                  (UAM_FlightMode_angdiff(rt_atan2d_snf(absxk -
                     UAM_FlightMode_U.States.Xe[1], b_tmp[1] -
                     UAM_FlightMode_U.States.Xe[0]),
                    UAM_FlightMode_U.States.course))) {
                turnInternal = b_tmp[0];
                absxk = rtb_DotProduct;
              } else {
                turnInternal = b_tmp[1];
              }

              if ((turnInternal - UAM_FlightMode_U.States.Xe[0]) * v[1] - (absxk
                   - UAM_FlightMode_U.States.Xe[1]) * v[0] > 0.0) {
                UAM_FlightMode_DW.obj_b.TurnDirectionInternal = 1.0;
              } else {
                UAM_FlightMode_DW.obj_b.TurnDirectionInternal = -1.0;
              }

              UAM_FlightMode_DW.obj_b.SelectTurnDirectionFlag = false;
              break;
            }
          }
        }

        if (guard2) {
          _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_mul_pd(_mm_div_pd(_mm_set_pd
            (absxk, b_tmp[0]), _mm_set1_pd(UAM_FlightMode_norm_d(b_tmp))),
            _mm_set1_pd(t)), _mm_loadu_pd(&UAM_FlightMode_U.ToWP.position[0])));
          turnInternal = tmp[0];
          absxk = tmp[1];
          t = UAM_FlightMode_DW.obj_b.NumCircles;
        }

        /* BusCreator: '<S5>/Bus Creator' incorporates:
         *  Inport: '<Root>/States'
         *  Inport: '<Root>/ToWP'
         *  MATLABSystem: '<S5>/UAV Orbit Follower'
         *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
         */
        UAM_FlightMode_B.aacSP.course = rt_atan2d_snf(absxk -
          UAM_FlightMode_U.States.Xe[1], turnInternal -
          UAM_FlightMode_U.States.Xe[0]);
      }

      /* BusCreator: '<S5>/Bus Creator' incorporates:
       *  Constant: '<S5>/Constant'
       *  Constant: '<S5>/Constant1'
       *  Constant: '<S5>/Lookahead Distance'
       *  Inport: '<Root>/ToWP'
       *  UnaryMinus: '<S5>/Unary Minus'
       */
      UAM_FlightMode_B.aacSP.airspeed = 15.0;
      UAM_FlightMode_B.aacSP.altitude = -UAM_FlightMode_U.ToWP.position[2];
      UAM_FlightMode_B.aacSP.L1 = 30.0;
      UAM_FlightMode_B.aacSP.climbrate = 0.0;

      /* Merge: '<S3>/ Merge ' incorporates:
       *  Abs: '<S5>/Abs1'
       *  DataTypeConversion: '<S5>/Data Type Conversion'
       *  Inport: '<Root>/ToWP'
       *  MATLABSystem: '<S5>/UAV Orbit Follower'
       *  RelationalOperator: '<S5>/Relational Operator'
       * */
      UAM_FlightMode_B.Status = (uint8_T)(fabs(t) >
        UAM_FlightMode_U.ToWP.params[2]);
    } else if (UAM_FlightMode_Y.FlightMode == Hover) {
      /* Outport: '<Root>/controlMode' */
      UAM_FlightMode_Y.controlMode_m.inTransition = 0U;

      /* Inport: '<Root>/mode' */
      switch (UAM_FlightMode_U.mode) {
       case 2U:
        UAM_exit_internal_GuidanceLogic();
        UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMode_IN_WP;

        /* Switch: '<S17>/Switch' incorporates:
         *  Inport: '<Root>/FromWP'
         *  Product: '<S21>/Product'
         *  RelationalOperator: '<S17>/Equal'
         */
        if (UAM_FlightMode_U.FromWP.mode == 6) {
          rtb_Product[0] = UAM_FlightMode_U.FromWP.position[0];
          rtb_Product[1] = UAM_FlightMode_U.FromWP.position[1];
          rtb_Product[2] = UAM_FlightMode_U.FromWP.position[2];
        } else {
          rtb_Product[0] = 0.0;
          rtb_Product[1] = 0.0;
          rtb_Product[2] = 0.0;
        }

        /* MATLAB Function: '<S12>/MATLAB Function' incorporates:
         *  BusCreator generated from: '<S12>/MATLAB Function'
         *  Inport: '<Root>/FromWP'
         *  Inport: '<Root>/ToWP'
         *  Product: '<S21>/Product'
         */
        rtb_wps[0] = rtb_Product[0];
        rtb_wps[2] = rtb_Product[1];
        rtb_wps[4] = rtb_Product[2];
        rtb_wps[6] = UAM_FlightMode_U.FromWP.params[3];
        rtb_wps[1] = UAM_FlightMode_U.ToWP.position[0];
        rtb_wps[3] = UAM_FlightMode_U.ToWP.position[1];
        rtb_wps[5] = UAM_FlightMode_U.ToWP.position[2];
        rtb_wps[7] = UAM_FlightMode_U.ToWP.params[3];

        /* BusCreator: '<S12>/Bus Creator' incorporates:
         *  Constant: '<S12>/Constant'
         *  Constant: '<S12>/Lookahead Distance'
         *  Inport: '<Root>/Pose'
         *  MATLABSystem: '<S12>/Waypoint Follower'
         *  Merge: '<S3>/ Merge 2'
         * */
        UAM_FlightMode_SystemCore_step(&UAM_FlightMode_DW.obj,
          UAM_FlightMode_U.Pose, rtb_wps, 5.0,
          UAM_FlightMode_B.InnerLoopCmds.LAP,
          &UAM_FlightMode_B.InnerLoopCmds.HeadingCmd, &turnInternal,
          &b_varargout_4);
        UAM_FlightMode_B.InnerLoopCmds.YawCmd = 0.0;

        /* Sum: '<S18>/Sum1' incorporates:
         *  Inport: '<Root>/ToWP'
         *  Product: '<S21>/Product'
         */
        turnInternal = UAM_FlightMode_U.ToWP.position[0] - rtb_Product[0];
        rtb_Product[0] = turnInternal;

        /* DotProduct: '<S21>/Dot Product1' incorporates:
         *  Product: '<S21>/Product'
         */
        rtb_DotProduct = turnInternal * turnInternal;

        /* Sum: '<S18>/Sum1' incorporates:
         *  Inport: '<Root>/ToWP'
         *  Product: '<S21>/Product'
         */
        turnInternal = UAM_FlightMode_U.ToWP.position[1] - rtb_Product[1];
        rtb_Product[1] = turnInternal;

        /* DotProduct: '<S21>/Dot Product1' incorporates:
         *  Product: '<S21>/Product'
         */
        rtb_DotProduct += turnInternal * turnInternal;

        /* Sum: '<S18>/Sum1' incorporates:
         *  Inport: '<Root>/ToWP'
         *  Product: '<S21>/Product'
         */
        turnInternal = UAM_FlightMode_U.ToWP.position[2] - rtb_Product[2];

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

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S12>/Lookahead Distance'
         *  DataTypeConversion: '<S12>/Data Type Conversion'
         *  DotProduct: '<S18>/Dot Product'
         *  Inport: '<Root>/Pose'
         *  Inport: '<Root>/ToWP'
         *  Product: '<S21>/Product'
         *  RelationalOperator: '<S12>/Relational Operator'
         *  Sum: '<S18>/Sum'
         *  Sum: '<S18>/Sum1'
         */
        UAM_FlightMode_B.Status = (uint8_T)(((UAM_FlightMode_U.ToWP.position[0]
          - UAM_FlightMode_U.Pose[0]) * (rtb_Product[0] / rtb_DotProduct) +
          (UAM_FlightMode_U.ToWP.position[1] - UAM_FlightMode_U.Pose[1]) *
          (rtb_Product[1] / rtb_DotProduct)) + (UAM_FlightMode_U.ToWP.position[2]
          - UAM_FlightMode_U.Pose[2]) * (turnInternal / rtb_DotProduct) <= 5.0);
        break;

       case 3U:
        UAM_exit_internal_GuidanceLogic();
        UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMode_IN_Orbit;

        /* Abs: '<S10>/Abs' incorporates:
         *  Inport: '<Root>/ToWP'
         */
        t = fabs(UAM_FlightMode_U.ToWP.params[0]);

        /* Signum: '<S10>/Sign' incorporates:
         *  Inport: '<Root>/ToWP'
         */
        if (rtIsNaN(UAM_FlightMode_U.ToWP.params[1])) {
          turnInternal = (rtNaN);
        } else if (UAM_FlightMode_U.ToWP.params[1] < 0.0) {
          turnInternal = -1.0;
        } else {
          turnInternal = (UAM_FlightMode_U.ToWP.params[1] > 0.0);
        }

        /* MATLABSystem: '<S10>/UAV Orbit Follower' */
        UAM_FlightMode_DW.obj_l.OrbitRadiusFlag = 0U;
        if (t <= 1.0) {
          t = 1.0;
          UAM_FlightMode_DW.obj_l.OrbitRadiusFlag = 1U;
        }

        UAM_FlightMode_DW.obj_l.LookaheadDistFlag = 0U;

        /* Start for Inport: '<Root>/Pose' incorporates:
         *  Inport: '<Root>/ToWP'
         *  MATLABSystem: '<S10>/UAV Orbit Follower'
         * */
        tmp_0 = _mm_sub_pd(_mm_loadu_pd(&UAM_FlightMode_U.Pose[0]), _mm_loadu_pd
                           (&UAM_FlightMode_U.ToWP.position[0]));

        /* MATLABSystem: '<S10>/UAV Orbit Follower' incorporates:
         *  BusCreator: '<S10>/Bus Creator'
         *  Inport: '<Root>/Pose'
         *  Inport: '<Root>/ToWP'
         *  Merge: '<S3>/ Merge 2'
         */
        _mm_storeu_pd(&rtb_TmpSignalConversionAtUAVO_0[0], tmp_0);
        if (UAM_FlightMode_norm_d(rtb_TmpSignalConversionAtUAVO_0) <
            2.47032822920623E-323) {
          _mm_storeu_pd(&UAM_FlightMode_B.InnerLoopCmds.LAP[0], _mm_add_pd
                        (_mm_mul_pd(_mm_set1_pd(t), _mm_set_pd(sin
            (UAM_FlightMode_U.Pose[3]), cos(UAM_FlightMode_U.Pose[3]))),
                         _mm_loadu_pd(&UAM_FlightMode_U.Pose[0])));

          /* BusCreator: '<S10>/Bus Creator' incorporates:
           *  Inport: '<Root>/Pose'
           *  Inport: '<Root>/ToWP'
           *  MATLABSystem: '<S10>/UAV Orbit Follower'
           *  Merge: '<S3>/ Merge 2'
           */
          UAM_FlightMode_B.InnerLoopCmds.LAP[2] =
            UAM_FlightMode_U.ToWP.position[2];
          UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = UAM_FlightMode_U.Pose[3];
          UAM_FlightMode_B.InnerLoopCmds.YawCmd = UAM_FlightMode_U.Pose[3];
          t = UAM_FlightMode_DW.obj_l.NumCircles;
        } else {
          p = false;
          p_0 = true;
          b_k = 0;
          exitg1 = false;
          while ((!exitg1) && (b_k < 3)) {
            if (!(UAM_FlightMode_DW.obj_l.OrbitCenterInternal[b_k] ==
                  UAM_FlightMode_U.ToWP.position[b_k])) {
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
            if (UAM_FlightMode_DW.obj_l.OrbitRadiusInternal == t) {
              p = true;
            }

            if (!p) {
              guard2 = true;
            }
          }

          if (guard2) {
            UAM_FlightMode_DW.obj_l.NumCircles = 0.0;
            UAM_FlightMode_DW.obj_l.OrbitCenterInternal[0] =
              UAM_FlightMode_U.ToWP.position[0];
            UAM_FlightMode_DW.obj_l.OrbitCenterInternal[1] =
              UAM_FlightMode_U.ToWP.position[1];
            UAM_FlightMode_DW.obj_l.OrbitCenterInternal[2] =
              UAM_FlightMode_U.ToWP.position[2];
            UAM_FlightMode_DW.obj_l.OrbitRadiusInternal = t;
            UAM_FlightMode_DW.obj_l.SelectTurnDirectionFlag = true;
          }

          if (t <= 5.0) {
            UAM_FlightMode_DW.obj_l.LookaheadDistance = 0.9 * t;
          } else {
            UAM_FlightMode_DW.obj_l.LookaheadDistance = 5.0;
          }

          /* Start for MATLABSystem: '<S10>/UAV Orbit Follower' incorporates:
           *  Inport: '<Root>/Pose'
           *  Inport: '<Root>/ToWP'
           */
          absxk = UAM_FlightMode_U.Pose[0] - UAM_FlightMode_U.ToWP.position[0];
          b_tmp[0] = absxk;
          rtb_DotProduct = absxk * absxk;
          absxk = UAM_FlightMode_U.Pose[1] - UAM_FlightMode_U.ToWP.position[1];
          b_tmp[1] = absxk;
          distToCenter = sqrt(absxk * absxk + rtb_DotProduct);
          rtb_DotProduct = t + UAM_FlightMode_DW.obj_l.LookaheadDistance;
          if (rtIsInf(rtb_DotProduct) || rtIsNaN(rtb_DotProduct)) {
            rtb_DotProduct = (rtNaN);
          } else if (rtb_DotProduct < 4.4501477170144028E-308) {
            rtb_DotProduct = 4.94065645841247E-324;
          } else {
            frexp(rtb_DotProduct, &b_exponent);
            rtb_DotProduct = ldexp(1.0, b_exponent - 53);
          }

          guard2 = false;
          if (distToCenter >= (t + UAM_FlightMode_DW.obj_l.LookaheadDistance) -
              5.0 * rtb_DotProduct) {
            guard2 = true;
          } else {
            rtb_DotProduct = t + UAM_FlightMode_DW.obj_l.LookaheadDistance;
            if (rtIsInf(rtb_DotProduct) || rtIsNaN(rtb_DotProduct)) {
              rtb_DotProduct = (rtNaN);
            } else if (rtb_DotProduct < 4.4501477170144028E-308) {
              rtb_DotProduct = 4.94065645841247E-324;
            } else {
              frexp(rtb_DotProduct, &b_exponent_0);
              rtb_DotProduct = ldexp(1.0, b_exponent_0 - 53);
            }

            if (distToCenter <= (t - UAM_FlightMode_DW.obj_l.LookaheadDistance)
                + 5.0 * rtb_DotProduct) {
              guard2 = true;
            } else {
              if (UAM_FlightMode_DW.obj_l.StartFlag) {
                UAM_FlightMode_DW.obj_l.PrevPosition[0] = UAM_FlightMode_U.Pose
                  [0];
                UAM_FlightMode_DW.obj_l.PrevPosition[1] = UAM_FlightMode_U.Pose
                  [1];
                UAM_FlightMode_DW.obj_l.PrevPosition[2] = UAM_FlightMode_U.Pose
                  [2];
                UAM_FlightMode_DW.obj_l.StartFlag = false;
              }

              if ((turnInternal == 0.0) &&
                  (!UAM_FlightMode_DW.obj_l.SelectTurnDirectionFlag)) {
                turnInternal = UAM_FlightMode_DW.obj_l.TurnDirectionInternal;
              }

              rtb_Product[2] = 0.0;
              _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
                (UAM_FlightMode_DW.obj_l.PrevPosition[0], UAM_FlightMode_U.Pose
                 [0]), _mm_set1_pd(UAM_FlightMode_U.ToWP.position[0])));
              xyPose[0] = tmp[0];
              rtb_Product[0] = tmp[1];
              _mm_storeu_pd(&tmp[0], tmp_0);
              v[0] = tmp[0];
              xyPose[1] = tmp[1];
              _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
                (UAM_FlightMode_U.Pose[1], UAM_FlightMode_DW.obj_l.PrevPosition
                 [1]), _mm_set1_pd(UAM_FlightMode_U.ToWP.position[1])));
              rtb_Product[1] = tmp[0];
              v[1] = tmp[1];
              absxk = UAM_FlightMode_norm_d(xyPose);
              rtb_DotProduct_tmp = UAM_FlightMode_DW.obj_l.LookaheadDistance *
                UAM_FlightMode_DW.obj_l.LookaheadDistance;
              rtb_DotProduct = ((rtb_DotProduct_tmp - t * t) + absxk * absxk) /
                (2.0 * absxk);
              tmp_1 = _mm_set1_pd(absxk);
              tmp_0 = _mm_sub_pd(_mm_loadu_pd(&UAM_FlightMode_U.ToWP.position[0]),
                                 _mm_loadu_pd(&UAM_FlightMode_U.Pose[0]));
              _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(tmp_0,
                _mm_set1_pd(rtb_DotProduct)), tmp_1), _mm_loadu_pd
                (&UAM_FlightMode_U.Pose[0])));
              t = tmp[1];
              distToCenter = sqrt(rtb_DotProduct_tmp - rtb_DotProduct *
                                  rtb_DotProduct);
              b_tmp[0] = tmp[0] - (UAM_FlightMode_U.ToWP.position[1] -
                                   UAM_FlightMode_U.Pose[1]) * distToCenter /
                absxk;
              _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd
                (_mm_set_pd(UAM_FlightMode_U.ToWP.position[0],
                            UAM_FlightMode_U.ToWP.position[1]), _mm_set_pd
                 (UAM_FlightMode_U.Pose[0], UAM_FlightMode_U.Pose[1])),
                _mm_set1_pd(distToCenter)), tmp_1), _mm_set_pd(tmp[1], tmp[0])));
              b_tmp[1] = tmp[0];
              rtb_DotProduct = tmp[1];
              absxk = t - (UAM_FlightMode_U.ToWP.position[0] -
                           UAM_FlightMode_U.Pose[0]) * distToCenter / absxk;
              v[2] = 0.0;
              if (turnInternal < 0.0) {
                rtb_Product[0] = v[0];
                rtb_Product[1] = v[1];
                rtb_Product[2] = 0.0;
                tmp_1 = _mm_sub_pd(_mm_loadu_pd
                                   (&UAM_FlightMode_DW.obj_l.PrevPosition[0]),
                                   _mm_loadu_pd(&UAM_FlightMode_U.ToWP.position
                  [0]));
                _mm_storeu_pd(&v[0], tmp_1);
                v[2] = 0.0;
              }

              distToCenter = UAM_FlightMode_norm_de(rtb_Product);
              t = UAM_FlightMode_norm_de(v);
              tmp_1 = _mm_set_pd(t, distToCenter);
              _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(v[0], rtb_Product[0]),
                tmp_1));
              rtb_Product[0] = tmp[0];
              v[0] = tmp[1];
              _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(v[1], rtb_Product[1]),
                tmp_1));
              v[1] = tmp[1];
              turnVector[2] = rtb_Product[0] * tmp[1] - v[0] * tmp[0];
              UAM_FlightMode_DW.obj_l.PrevPosition[0] = UAM_FlightMode_U.Pose[0];
              UAM_FlightMode_DW.obj_l.PrevPosition[1] = UAM_FlightMode_U.Pose[1];
              UAM_FlightMode_DW.obj_l.PrevPosition[2] = UAM_FlightMode_U.Pose[2];
              UAM_FlightMode_DW.obj_l.NumCircles += rt_atan2d_snf(turnVector[2],
                (rtb_Product[0] * v[0] + tmp[0] * tmp[1]) + 0.0 / distToCenter *
                (0.0 / t)) / 2.0 / 3.1415926535897931;
              t = UAM_FlightMode_DW.obj_l.NumCircles;
              _mm_storeu_pd(&v[0], tmp_0);
              if (rtIsNaN(turnInternal)) {
                distToCenter = (rtNaN);
              } else if (turnInternal < 0.0) {
                distToCenter = -1.0;
              } else {
                distToCenter = (turnInternal > 0.0);
              }

              switch ((int32_T)distToCenter) {
               case 1:
                if ((b_tmp[0] - UAM_FlightMode_U.Pose[0]) * v[1] -
                    (rtb_DotProduct - UAM_FlightMode_U.Pose[1]) * v[0] > 0.0) {
                  turnInternal = b_tmp[0];
                  absxk = rtb_DotProduct;
                } else {
                  turnInternal = b_tmp[1];
                }
                break;

               case -1:
                if ((b_tmp[0] - UAM_FlightMode_U.Pose[0]) * v[1] -
                    (rtb_DotProduct - UAM_FlightMode_U.Pose[1]) * v[0] < 0.0) {
                  turnInternal = b_tmp[0];
                  absxk = rtb_DotProduct;
                } else {
                  turnInternal = b_tmp[1];
                }
                break;

               default:
                if (fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(rtb_DotProduct -
                       UAM_FlightMode_U.Pose[1], b_tmp[0] -
                       UAM_FlightMode_U.Pose[0]), UAM_FlightMode_U.Pose[3])) <
                    fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(absxk -
                       UAM_FlightMode_U.Pose[1], b_tmp[1] -
                       UAM_FlightMode_U.Pose[0]), UAM_FlightMode_U.Pose[3]))) {
                  turnInternal = b_tmp[0];
                  absxk = rtb_DotProduct;
                } else {
                  turnInternal = b_tmp[1];
                }

                if ((turnInternal - UAM_FlightMode_U.Pose[0]) * v[1] - (absxk -
                     UAM_FlightMode_U.Pose[1]) * v[0] > 0.0) {
                  UAM_FlightMode_DW.obj_l.TurnDirectionInternal = 1.0;
                } else {
                  UAM_FlightMode_DW.obj_l.TurnDirectionInternal = -1.0;
                }

                UAM_FlightMode_DW.obj_l.SelectTurnDirectionFlag = false;
                break;
              }

              /* BusCreator: '<S10>/Bus Creator' incorporates:
               *  Inport: '<Root>/Pose'
               *  Inport: '<Root>/ToWP'
               *  Merge: '<S3>/ Merge 2'
               */
              UAM_FlightMode_B.InnerLoopCmds.YawCmd = rt_atan2d_snf
                (UAM_FlightMode_U.ToWP.position[1] - UAM_FlightMode_U.Pose[1],
                 UAM_FlightMode_U.ToWP.position[0] - UAM_FlightMode_U.Pose[0]);
            }
          }

          if (guard2) {
            _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_mul_pd(_mm_div_pd(_mm_set_pd
              (absxk, b_tmp[0]), _mm_set1_pd(UAM_FlightMode_norm_d(b_tmp))),
              _mm_set1_pd(t)), _mm_loadu_pd(&UAM_FlightMode_U.ToWP.position[0])));
            turnInternal = tmp[0];
            absxk = tmp[1];

            /* BusCreator: '<S10>/Bus Creator' incorporates:
             *  Inport: '<Root>/Pose'
             *  Inport: '<Root>/ToWP'
             *  Merge: '<S3>/ Merge 2'
             */
            UAM_FlightMode_B.InnerLoopCmds.YawCmd = rt_atan2d_snf(tmp[1] -
              UAM_FlightMode_U.Pose[1], tmp[0] - UAM_FlightMode_U.Pose[0]);
            t = UAM_FlightMode_DW.obj_l.NumCircles;
          }

          /* BusCreator: '<S10>/Bus Creator' incorporates:
           *  Inport: '<Root>/Pose'
           *  Inport: '<Root>/ToWP'
           *  MATLABSystem: '<S10>/UAV Orbit Follower'
           *  Merge: '<S3>/ Merge 2'
           */
          UAM_FlightMode_B.InnerLoopCmds.LAP[0] = turnInternal;
          UAM_FlightMode_B.InnerLoopCmds.LAP[1] = absxk;
          UAM_FlightMode_B.InnerLoopCmds.LAP[2] =
            UAM_FlightMode_U.ToWP.position[2];
          UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = rt_atan2d_snf(absxk -
            UAM_FlightMode_U.Pose[1], turnInternal - UAM_FlightMode_U.Pose[0]);
        }

        /* Merge: '<S3>/ Merge ' incorporates:
         *  DataTypeConversion: '<S10>/Data Type Conversion'
         *  Inport: '<Root>/ToWP'
         *  MATLABSystem: '<S10>/UAV Orbit Follower'
         *  RelationalOperator: '<S10>/Relational Operator'
         * */
        UAM_FlightMode_B.Status = (uint8_T)(t > UAM_FlightMode_U.ToWP.params[2]);
        break;

       case 6U:
        /* Outport: '<Root>/FlightMode' */
        UAM_FlightMode_Y.FlightMode = Transition;

        /* Outport: '<Root>/controlMode' */
        UAM_FlightMode_Y.controlMode_m.inTransition = 1U;
        UAM_FlightMode_Y.controlMode_m.TransitionCondition = 0U;
        UAM_exit_internal_GuidanceLogic();
        UAM_FlightMode_DW.is_GuidanceLogic = UAM_Flight_IN_ForwardTransition;

        /* BusCreator: '<S7>/Bus Creator1' incorporates:
         *  Constant: '<S7>/Constant1'
         *  Constant: '<S7>/Constant2'
         *  Constant: '<S7>/Constant3'
         *  Merge: '<S3>/ Merge 2'
         */
        UAM_FlightMode_B.InnerLoopCmds.LAP[0] = 0.0;
        UAM_FlightMode_B.InnerLoopCmds.LAP[1] = 0.0;
        UAM_FlightMode_B.InnerLoopCmds.LAP[2] = 0.0;
        UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = 0.0;
        UAM_FlightMode_B.InnerLoopCmds.YawCmd = 0.0;

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S7>/Constant'
         *  DataTypeConversion: '<S7>/Data Type Conversion'
         */
        UAM_FlightMode_B.Status = 0U;
        break;

       default:
        UAM_exit_internal_GuidanceLogic();
        UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMode_IN_Land;
        UAM_FlightMode_DW.is_Land = UAM_FlightMode_IN_ToLand;

        /* MATLAB Function: '<S9>/MATLAB Function' incorporates:
         *  Inport: '<Root>/FromWP'
         *  Inport: '<Root>/ToWP'
         */
        rtb_wps[0] = UAM_FlightMode_U.FromWP.position[0];
        rtb_wps[2] = UAM_FlightMode_U.FromWP.position[1];
        rtb_wps[4] = UAM_FlightMode_U.FromWP.position[2];
        rtb_wps[6] = UAM_FlightMode_U.FromWP.params[3];
        rtb_wps[1] = UAM_FlightMode_U.ToWP.position[0];
        rtb_wps[3] = UAM_FlightMode_U.ToWP.position[1];
        rtb_wps[5] = UAM_FlightMode_U.ToWP.position[2];
        rtb_wps[7] = UAM_FlightMode_U.ToWP.params[3];

        /* Inport: '<Root>/Pose' incorporates:
         *  Constant: '<S9>/Lookahead Distance'
         */
        UAM_FlightMode_WaypointFollower(UAM_FlightMode_U.Pose, rtb_wps, 3.0,
          &UAM_FlightMode_B.WaypointFollower_c2,
          &UAM_FlightMode_DW.WaypointFollower_c2);

        /* BusCreator: '<S9>/Bus Creator1' incorporates:
         *  MATLABSystem: '<S9>/Waypoint Follower'
         *  Merge: '<S3>/ Merge 2'
         */
        UAM_FlightMode_B.InnerLoopCmds.LAP[0] =
          UAM_FlightMode_B.WaypointFollower_c2.LAP[0];
        UAM_FlightMode_B.InnerLoopCmds.LAP[1] =
          UAM_FlightMode_B.WaypointFollower_c2.LAP[1];
        UAM_FlightMode_B.InnerLoopCmds.LAP[2] =
          UAM_FlightMode_B.WaypointFollower_c2.LAP[2];
        UAM_FlightMode_B.InnerLoopCmds.HeadingCmd =
          UAM_FlightMode_B.WaypointFollower_c2.HeadingCmd;
        UAM_FlightMode_B.InnerLoopCmds.YawCmd =
          UAM_FlightMode_B.WaypointFollower_c2.YawCmd;

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S9>/Constant'
         *  SignalConversion generated from: '<S9>/Status'
         */
        UAM_FlightMode_B.Status = 0U;
        break;
      }
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    switch (UAM_FlightMode_DW.is_GuidanceLogic) {
     case UAM_FlightMod_IN_BackTransition:
      /* BusCreator: '<S4>/Bus Creator1' incorporates:
       *  Constant: '<S4>/Constant1'
       *  Constant: '<S4>/Constant2'
       *  Merge: '<S3>/ Merge 2'
       */
      UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = 0.0;
      UAM_FlightMode_B.InnerLoopCmds.YawCmd = 0.0;

      /* MATLAB Function: '<S4>/MATLAB Function' */
      rtb_DotProduct = 3.3121686421112381E-170;

      /* BusCreator: '<S4>/Bus Creator1' incorporates:
       *  Inport: '<Root>/States'
       *  Merge: '<S3>/ Merge 2'
       */
      UAM_FlightMode_B.InnerLoopCmds.LAP[0] = UAM_FlightMode_U.States.Xe[0];

      /* MATLAB Function: '<S4>/MATLAB Function' incorporates:
       *  Inport: '<Root>/States'
       */
      absxk = fabs(UAM_FlightMode_U.States.Ve[0]);
      if (absxk > 3.3121686421112381E-170) {
        turnInternal = 1.0;
        rtb_DotProduct = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        turnInternal = t * t;
      }

      /* BusCreator: '<S4>/Bus Creator1' incorporates:
       *  Inport: '<Root>/States'
       *  Merge: '<S3>/ Merge 2'
       */
      UAM_FlightMode_B.InnerLoopCmds.LAP[1] = UAM_FlightMode_U.States.Xe[1];
      UAM_FlightMode_B.InnerLoopCmds.LAP[2] = UAM_FlightMode_U.States.Xe[2];

      /* Merge: '<S3>/ Merge ' incorporates:
       *  DataTypeConversion: '<S4>/Data Type Conversion'
       *  Inport: '<Root>/States'
       *  MATLAB Function: '<S4>/MATLAB Function'
       */
      UAM_FlightMode_B.Status = (uint8_T)((fabs(UAM_FlightMode_U.States.c1) <
        0.1) && (rtb_DotProduct * sqrt(turnInternal) < 4.0));
      break;

     case UAM_FlightM_IN_FIXED_WING_ENTRY:
      UAM_FlightMode_FIXED_WING_ENTRY();
      break;

     case UAM_FlightM_IN_FIXED_WING_ORBIT:
      /* MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
       *  Inport: '<Root>/ToWP'
       */
      t = UAM_FlightMode_U.ToWP.params[0];
      UAM_FlightMode_DW.obj_b.OrbitRadiusFlag = 0U;
      if (UAM_FlightMode_U.ToWP.params[0] <= 50.0) {
        t = 50.0;
        UAM_FlightMode_DW.obj_b.OrbitRadiusFlag = 1U;
      }

      UAM_FlightMode_DW.obj_b.LookaheadDistFlag = 0U;

      /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
       *  Inport: '<Root>/ToWP'
       */
      tmp_0 = _mm_loadu_pd(&UAM_FlightMode_U.ToWP.position[0]);

      /* SignalConversion generated from: '<S5>/UAV Orbit Follower' incorporates:
       *  Inport: '<Root>/States'
       */
      tmp_2 = _mm_loadu_pd(&UAM_FlightMode_U.States.Xe[0]);
      tmp_1 = _mm_sub_pd(tmp_2, tmp_0);

      /* MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
       *  Inport: '<Root>/States'
       *  Inport: '<Root>/ToWP'
       *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
       */
      _mm_storeu_pd(&rtb_TmpSignalConversionAtUAVO_0[0], tmp_1);
      if (UAM_FlightMode_norm_d(rtb_TmpSignalConversionAtUAVO_0) <
          2.47032822920623E-323) {
        /* BusCreator: '<S5>/Bus Creator' incorporates:
         *  Inport: '<Root>/States'
         *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
         */
        UAM_FlightMode_B.aacSP.course = UAM_FlightMode_U.States.course;
        t = UAM_FlightMode_DW.obj_b.NumCircles;
      } else {
        p = false;
        p_0 = true;
        b_k = 0;
        exitg1 = false;
        while ((!exitg1) && (b_k < 3)) {
          if (!(UAM_FlightMode_DW.obj_b.OrbitCenterInternal[b_k] ==
                UAM_FlightMode_U.ToWP.position[b_k])) {
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
          if (UAM_FlightMode_DW.obj_b.OrbitRadiusInternal == t) {
            p = true;
          }

          if (!p) {
            guard2 = true;
          }
        }

        if (guard2) {
          UAM_FlightMode_DW.obj_b.NumCircles = 0.0;
          UAM_FlightMode_DW.obj_b.OrbitCenterInternal[0] =
            UAM_FlightMode_U.ToWP.position[0];
          UAM_FlightMode_DW.obj_b.OrbitCenterInternal[1] =
            UAM_FlightMode_U.ToWP.position[1];
          UAM_FlightMode_DW.obj_b.OrbitCenterInternal[2] =
            UAM_FlightMode_U.ToWP.position[2];
          UAM_FlightMode_DW.obj_b.OrbitRadiusInternal = t;
          UAM_FlightMode_DW.obj_b.SelectTurnDirectionFlag = true;
        }

        if (t <= 30.0) {
          UAM_FlightMode_DW.obj_b.LookaheadDistance = 0.9 * t;
        } else {
          UAM_FlightMode_DW.obj_b.LookaheadDistance = 30.0;
        }

        /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
         *  Inport: '<Root>/States'
         *  Inport: '<Root>/ToWP'
         *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
         */
        absxk = UAM_FlightMode_U.States.Xe[0] - UAM_FlightMode_U.ToWP.position[0];
        b_tmp[0] = absxk;
        rtb_DotProduct = absxk * absxk;
        absxk = UAM_FlightMode_U.States.Xe[1] - UAM_FlightMode_U.ToWP.position[1];
        b_tmp[1] = absxk;
        distToCenter = sqrt(absxk * absxk + rtb_DotProduct);
        rtb_DotProduct = fabs(t + UAM_FlightMode_DW.obj_b.LookaheadDistance);
        if (rtIsInf(rtb_DotProduct) || rtIsNaN(rtb_DotProduct)) {
          rtb_DotProduct = (rtNaN);
        } else if (rtb_DotProduct < 4.4501477170144028E-308) {
          rtb_DotProduct = 4.94065645841247E-324;
        } else {
          frexp(rtb_DotProduct, &b_exponent);
          rtb_DotProduct = ldexp(1.0, b_exponent - 53);
        }

        /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' */
        turnInternal = t + UAM_FlightMode_DW.obj_b.LookaheadDistance;
        guard2 = false;
        if (distToCenter >= turnInternal - 5.0 * rtb_DotProduct) {
          guard2 = true;
        } else {
          rtb_DotProduct = fabs(turnInternal);
          if (rtIsInf(rtb_DotProduct) || rtIsNaN(rtb_DotProduct)) {
            rtb_DotProduct = (rtNaN);
          } else if (rtb_DotProduct < 4.4501477170144028E-308) {
            rtb_DotProduct = 4.94065645841247E-324;
          } else {
            frexp(rtb_DotProduct, &b_exponent_0);
            rtb_DotProduct = ldexp(1.0, b_exponent_0 - 53);
          }

          if (distToCenter <= (t - UAM_FlightMode_DW.obj_b.LookaheadDistance) +
              5.0 * rtb_DotProduct) {
            guard2 = true;
          } else {
            if (UAM_FlightMode_DW.obj_b.StartFlag) {
              UAM_FlightMode_DW.obj_b.PrevPosition[0] =
                UAM_FlightMode_U.States.Xe[0];
              UAM_FlightMode_DW.obj_b.PrevPosition[1] =
                UAM_FlightMode_U.States.Xe[1];
              UAM_FlightMode_DW.obj_b.PrevPosition[2] =
                UAM_FlightMode_U.States.Xe[2];
              UAM_FlightMode_DW.obj_b.StartFlag = false;
            }

            if ((UAM_FlightMode_U.ToWP.params[1] == 0.0) &&
                (!UAM_FlightMode_DW.obj_b.SelectTurnDirectionFlag)) {
              turnInternal = UAM_FlightMode_DW.obj_b.TurnDirectionInternal;
            } else {
              turnInternal = UAM_FlightMode_U.ToWP.params[1];
            }

            rtb_Product[2] = 0.0;
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
              (UAM_FlightMode_DW.obj_b.PrevPosition[0],
               UAM_FlightMode_U.States.Xe[0]), _mm_set1_pd
              (UAM_FlightMode_U.ToWP.position[0])));
            xyPose[0] = tmp[0];
            rtb_Product[0] = tmp[1];
            _mm_storeu_pd(&tmp[0], tmp_1);
            v[0] = tmp[0];
            xyPose[1] = tmp[1];
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
              (UAM_FlightMode_U.States.Xe[1],
               UAM_FlightMode_DW.obj_b.PrevPosition[1]), _mm_set1_pd
              (UAM_FlightMode_U.ToWP.position[1])));
            rtb_Product[1] = tmp[0];
            v[1] = tmp[1];
            absxk = UAM_FlightMode_norm_d(xyPose);

            /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
             *  Inport: '<Root>/States'
             *  Inport: '<Root>/ToWP'
             *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
             */
            rtb_DotProduct_tmp = UAM_FlightMode_DW.obj_b.LookaheadDistance *
              UAM_FlightMode_DW.obj_b.LookaheadDistance;
            rtb_DotProduct = ((rtb_DotProduct_tmp - t * t) + absxk * absxk) /
              (2.0 * absxk);
            tmp_1 = _mm_set1_pd(absxk);
            tmp_3 = _mm_sub_pd(tmp_0, tmp_2);
            _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(tmp_3,
              _mm_set1_pd(rtb_DotProduct)), tmp_1), tmp_2));
            t = tmp[1];
            distToCenter = sqrt(rtb_DotProduct_tmp - rtb_DotProduct *
                                rtb_DotProduct);
            b_tmp[0] = tmp[0] - (UAM_FlightMode_U.ToWP.position[1] -
                                 UAM_FlightMode_U.States.Xe[1]) * distToCenter /
              absxk;
            _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd
              (_mm_set_pd(UAM_FlightMode_U.ToWP.position[0],
                          UAM_FlightMode_U.ToWP.position[1]), _mm_set_pd
               (UAM_FlightMode_U.States.Xe[0], UAM_FlightMode_U.States.Xe[1])),
              _mm_set1_pd(distToCenter)), tmp_1), _mm_set_pd(tmp[1], tmp[0])));
            b_tmp[1] = tmp[0];
            rtb_DotProduct = tmp[1];
            absxk = t - (UAM_FlightMode_U.ToWP.position[0] -
                         UAM_FlightMode_U.States.Xe[0]) * distToCenter / absxk;

            /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
             *  Inport: '<Root>/States'
             *  Inport: '<Root>/ToWP'
             *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
             */
            v[2] = 0.0;
            if (turnInternal < 0.0) {
              rtb_Product[0] = v[0];
              rtb_Product[1] = v[1];
              rtb_Product[2] = 0.0;
              tmp_1 = _mm_sub_pd(_mm_loadu_pd
                                 (&UAM_FlightMode_DW.obj_b.PrevPosition[0]),
                                 tmp_0);
              _mm_storeu_pd(&v[0], tmp_1);
              v[2] = 0.0;
            }

            distToCenter = UAM_FlightMode_norm_de(rtb_Product);
            t = UAM_FlightMode_norm_de(v);
            tmp_1 = _mm_set_pd(t, distToCenter);
            _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(v[0], rtb_Product[0]),
              tmp_1));
            rtb_Product[0] = tmp[0];
            v[0] = tmp[1];
            _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(v[1], rtb_Product[1]),
              tmp_1));
            v[1] = tmp[1];
            turnVector[2] = rtb_Product[0] * tmp[1] - v[0] * tmp[0];
            UAM_FlightMode_DW.obj_b.PrevPosition[0] =
              UAM_FlightMode_U.States.Xe[0];
            UAM_FlightMode_DW.obj_b.PrevPosition[1] =
              UAM_FlightMode_U.States.Xe[1];
            UAM_FlightMode_DW.obj_b.PrevPosition[2] =
              UAM_FlightMode_U.States.Xe[2];
            UAM_FlightMode_DW.obj_b.NumCircles += rt_atan2d_snf(turnVector[2],
              (rtb_Product[0] * v[0] + tmp[0] * tmp[1]) + 0.0 / distToCenter *
              (0.0 / t)) / 2.0 / 3.1415926535897931;
            t = UAM_FlightMode_DW.obj_b.NumCircles;
            _mm_storeu_pd(&v[0], tmp_3);

            /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
             *  Inport: '<Root>/States'
             *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
             */
            if (rtIsNaN(turnInternal)) {
              distToCenter = (rtNaN);
            } else if (turnInternal < 0.0) {
              distToCenter = -1.0;
            } else {
              distToCenter = (turnInternal > 0.0);
            }

            switch ((int32_T)distToCenter) {
             case 1:
              if ((b_tmp[0] - UAM_FlightMode_U.States.Xe[0]) * v[1] -
                  (rtb_DotProduct - UAM_FlightMode_U.States.Xe[1]) * v[0] > 0.0)
              {
                turnInternal = b_tmp[0];
                absxk = rtb_DotProduct;
              } else {
                turnInternal = b_tmp[1];
              }
              break;

             case -1:
              if ((b_tmp[0] - UAM_FlightMode_U.States.Xe[0]) * v[1] -
                  (rtb_DotProduct - UAM_FlightMode_U.States.Xe[1]) * v[0] < 0.0)
              {
                turnInternal = b_tmp[0];
                absxk = rtb_DotProduct;
              } else {
                turnInternal = b_tmp[1];
              }
              break;

             default:
              if (fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(rtb_DotProduct -
                     UAM_FlightMode_U.States.Xe[1], b_tmp[0] -
                     UAM_FlightMode_U.States.Xe[0]),
                    UAM_FlightMode_U.States.course)) < fabs
                  (UAM_FlightMode_angdiff(rt_atan2d_snf(absxk -
                     UAM_FlightMode_U.States.Xe[1], b_tmp[1] -
                     UAM_FlightMode_U.States.Xe[0]),
                    UAM_FlightMode_U.States.course))) {
                turnInternal = b_tmp[0];
                absxk = rtb_DotProduct;
              } else {
                turnInternal = b_tmp[1];
              }

              if ((turnInternal - UAM_FlightMode_U.States.Xe[0]) * v[1] - (absxk
                   - UAM_FlightMode_U.States.Xe[1]) * v[0] > 0.0) {
                UAM_FlightMode_DW.obj_b.TurnDirectionInternal = 1.0;
              } else {
                UAM_FlightMode_DW.obj_b.TurnDirectionInternal = -1.0;
              }

              UAM_FlightMode_DW.obj_b.SelectTurnDirectionFlag = false;
              break;
            }
          }
        }

        if (guard2) {
          _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_mul_pd(_mm_div_pd(_mm_set_pd
            (absxk, b_tmp[0]), _mm_set1_pd(UAM_FlightMode_norm_d(b_tmp))),
            _mm_set1_pd(t)), tmp_0));
          turnInternal = tmp[0];
          absxk = tmp[1];
          t = UAM_FlightMode_DW.obj_b.NumCircles;
        }

        /* BusCreator: '<S5>/Bus Creator' incorporates:
         *  Inport: '<Root>/States'
         *  Inport: '<Root>/ToWP'
         *  MATLABSystem: '<S5>/UAV Orbit Follower'
         *  SignalConversion generated from: '<S5>/UAV Orbit Follower'
         */
        UAM_FlightMode_B.aacSP.course = rt_atan2d_snf(absxk -
          UAM_FlightMode_U.States.Xe[1], turnInternal -
          UAM_FlightMode_U.States.Xe[0]);
      }

      /* BusCreator: '<S5>/Bus Creator' incorporates:
       *  Constant: '<S5>/Constant'
       *  Constant: '<S5>/Constant1'
       *  Constant: '<S5>/Lookahead Distance'
       *  Inport: '<Root>/ToWP'
       *  UnaryMinus: '<S5>/Unary Minus'
       */
      UAM_FlightMode_B.aacSP.airspeed = 15.0;
      UAM_FlightMode_B.aacSP.altitude = -UAM_FlightMode_U.ToWP.position[2];
      UAM_FlightMode_B.aacSP.L1 = 30.0;
      UAM_FlightMode_B.aacSP.climbrate = 0.0;

      /* Merge: '<S3>/ Merge ' incorporates:
       *  Abs: '<S5>/Abs1'
       *  DataTypeConversion: '<S5>/Data Type Conversion'
       *  Inport: '<Root>/ToWP'
       *  MATLABSystem: '<S5>/UAV Orbit Follower'
       *  RelationalOperator: '<S5>/Relational Operator'
       * */
      UAM_FlightMode_B.Status = (uint8_T)(fabs(t) >
        UAM_FlightMode_U.ToWP.params[2]);
      break;

     case UAM_Flig_IN_FIXED_WING_WAYPOINT:
      /* MATLAB Function: '<S6>/MATLAB Function' incorporates:
       *  Inport: '<Root>/FromWP'
       *  Inport: '<Root>/ToWP'
       */
      rtb_wps_k[0] = UAM_FlightMode_U.FromWP.position[0];
      rtb_wps_k[1] = UAM_FlightMode_U.ToWP.position[0];
      rtb_wps_k[2] = UAM_FlightMode_U.FromWP.position[1];
      rtb_wps_k[3] = UAM_FlightMode_U.ToWP.position[1];
      rtb_wps_k[4] = UAM_FlightMode_U.FromWP.position[2];
      rtb_wps_k[5] = UAM_FlightMode_U.ToWP.position[2];

      /* MATLABSystem: '<S6>/Waypoint Follower' incorporates:
       *  Inport: '<Root>/FromWP'
       *  Inport: '<Root>/ToWP'
       *  MATLAB Function: '<S6>/MATLAB Function'
       */
      UAM_FlightMode_DW.obj_f.LookaheadDistFlag = 0U;
      UAM_FlightMode_DW.obj_f.InitialPose[0] = 0.0;
      UAM_FlightMode_DW.obj_f.InitialPose[1] = 0.0;
      UAM_FlightMode_DW.obj_f.InitialPose[2] = 0.0;
      UAM_FlightMode_DW.obj_f.InitialPose[3] = 0.0;
      UAM_FlightMode_DW.obj_f.NumWaypoints = 2.0;
      p = false;
      p_0 = true;
      b_k = 0;
      exitg1 = false;
      while ((!exitg1) && (b_k <= 5)) {
        i = ((b_k / 2) << 1) + b_k % 2;
        if (!(UAM_FlightMode_DW.obj_f.WaypointsInternal[i] == rtb_wps_k[i])) {
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
          UAM_FlightMode_DW.obj_f.WaypointsInternal[i] = rtb_wps_k[i];
        }

        UAM_FlightMode_DW.obj_f.WaypointIndex = 1.0;
      }

      distinctWptsIdx[1] = true;
      x[0] = (UAM_FlightMode_U.FromWP.position[0] !=
              UAM_FlightMode_U.ToWP.position[0]);
      x[1] = (UAM_FlightMode_U.FromWP.position[1] !=
              UAM_FlightMode_U.ToWP.position[1]);
      x[2] = (UAM_FlightMode_U.FromWP.position[2] !=
              UAM_FlightMode_U.ToWP.position[2]);
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
       *  Inport: '<Root>/States'
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

      UAM_FlightMode_DW.obj_f.LookaheadDistance = 30.0;
      if (tmp_size_idx_1 == 0) {
        /* BusCreator: '<S6>/Bus Creator' incorporates:
         *  Inport: '<Root>/States'
         *  SignalConversion generated from: '<S6>/Waypoint Follower'
         */
        UAM_FlightMode_B.aacSP.course = UAM_FlightMode_U.States.course;

        /* Merge: '<S3>/ Merge ' */
        UAM_FlightMode_B.Status = 1U;
      } else {
        guard2 = false;
        if (tmp_size_idx_1 == 1) {
          if (UAM_FlightMode_DW.obj_f.StartFlag) {
            UAM_FlightMode_DW.obj_f.InitialPose[0] = UAM_FlightMode_U.States.Xe
              [0];
            UAM_FlightMode_DW.obj_f.InitialPose[1] = UAM_FlightMode_U.States.Xe
              [1];
            UAM_FlightMode_DW.obj_f.InitialPose[2] = UAM_FlightMode_U.States.Xe
              [2];
            UAM_FlightMode_DW.obj_f.InitialPose[3] =
              UAM_FlightMode_U.States.course;
          }

          tmp_1 = _mm_sub_pd(_mm_loadu_pd(&b_waypointsIn_data[0]), _mm_loadu_pd(
            &UAM_FlightMode_U.States.Xe[0]));
          _mm_storeu_pd(&rtb_Product[0], tmp_1);
          rtb_Product[2] = b_waypointsIn_data[2] - UAM_FlightMode_U.States.Xe[2];
          if (UAM_FlightMode_norm_de(rtb_Product) < 1.4901161193847656E-8) {
            /* BusCreator: '<S6>/Bus Creator' incorporates:
             *  Inport: '<Root>/States'
             *  SignalConversion generated from: '<S6>/Waypoint Follower'
             */
            UAM_FlightMode_B.aacSP.course = UAM_FlightMode_U.States.course;

            /* Merge: '<S3>/ Merge ' */
            UAM_FlightMode_B.Status = 1U;
            UAM_FlightMode_DW.obj_f.StartFlag = false;
          } else {
            UAM_FlightMode_DW.obj_f.StartFlag = false;
            UAM_FlightMode_DW.obj_f.NumWaypoints = 2.0;
            scalarLB = tmp_size_idx_1 + 1;
            for (i = 0; i < 3; i++) {
              vectorUB = (tmp_size_idx_1 + 1) * i;
              rtb_wps_k[vectorUB] = UAM_FlightMode_DW.obj_f.InitialPose[i];
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
          if (UAM_FlightMode_DW.obj_f.WaypointIndex == 2.0) {
            p = true;
          }

          if (p) {
            UAM_FlightMode_DW.obj_f.LastWaypointFlag = true;
            UAM_FlightMode_DW.obj_f.WaypointIndex--;
          }

          rtb_Product[0] = rtb_wps_k[(int32_T)
            UAM_FlightMode_DW.obj_f.WaypointIndex - 1];
          v[0] = rtb_wps_k[(int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0)
            - 1];
          rtb_Product[1] = rtb_wps_k[((int32_T)
            UAM_FlightMode_DW.obj_f.WaypointIndex + scalarLB) - 1];
          v[1] = rtb_wps_k[((int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex +
            1.0) + scalarLB) - 1];
          rtb_Product[2] = rtb_wps_k[((scalarLB << 1) + (int32_T)
            UAM_FlightMode_DW.obj_f.WaypointIndex) - 1];
          rtb_DotProduct = rtb_wps_k[((int32_T)
            (UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0) + (scalarLB << 1)) - 1];
          v[2] = rtb_DotProduct;
          turnInternal = rtb_wps_k[(int32_T)
            (UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0) - 1];
          tmp_0 = _mm_loadu_pd(&UAM_FlightMode_U.States.Xe[0]);
          _mm_storeu_pd(&turnVector_0[0], _mm_sub_pd(tmp_0, _mm_set_pd
            (rtb_wps_k[((int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0) +
                        scalarLB) - 1], turnInternal)));
          turnVector_0[2] = UAM_FlightMode_U.States.Xe[2] - rtb_DotProduct;
          guard3 = false;
          if (UAM_FlightMode_norm_de(turnVector_0) <= 10.0) {
            guard3 = true;
          } else {
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
              (UAM_FlightMode_U.States.Xe[0], turnInternal), _mm_set_pd
              (turnInternal, rtb_wps_k[(int32_T)
               UAM_FlightMode_DW.obj_f.WaypointIndex - 1])));
            unitVectorUtoV_tmp[0] = tmp[0];
            turnVector[0] = tmp[1];
            turnInternal = rtb_wps_k[((int32_T)
              (UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0) + scalarLB) - 1];
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
              (UAM_FlightMode_U.States.Xe[1], turnInternal), _mm_set_pd
              (turnInternal, rtb_wps_k[((int32_T)
              UAM_FlightMode_DW.obj_f.WaypointIndex + scalarLB) - 1])));
            unitVectorUtoV_tmp[1] = tmp[0];
            turnVector[1] = tmp[1];
            turnInternal = rtb_wps_k[((int32_T)
              (UAM_FlightMode_DW.obj_f.WaypointIndex + 1.0) + (scalarLB << 1)) -
              1];
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
              (UAM_FlightMode_U.States.Xe[2], turnInternal), _mm_set_pd
              (turnInternal, rtb_wps_k[((int32_T)
              UAM_FlightMode_DW.obj_f.WaypointIndex + (scalarLB << 1)) - 1])));
            unitVectorUtoV_tmp[2] = tmp[0];
            turnVector[2] = tmp[1];
            distToCenter = UAM_FlightMode_norm_de(unitVectorUtoV_tmp);
            t = UAM_FlightMode_norm_de(turnVector);
            turnInternal = (unitVectorUtoV_tmp[0] / distToCenter * (turnVector[0]
              / t) + unitVectorUtoV_tmp[1] / distToCenter * (turnVector[1] / t))
              + tmp[0] / distToCenter * (tmp[1] / t);
            if (rtIsNaN(turnInternal) || (turnInternal < 0.0)) {
            } else {
              guard3 = true;
            }
          }

          if (guard3) {
            UAM_FlightMode_DW.obj_f.WaypointIndex++;
            p = false;
            if (UAM_FlightMode_DW.obj_f.WaypointIndex == 2.0) {
              p = true;
            }

            if (p) {
              UAM_FlightMode_DW.obj_f.LastWaypointFlag = true;
              UAM_FlightMode_DW.obj_f.WaypointIndex--;
            }

            rtb_Product[0] = rtb_wps_k[(int32_T)
              UAM_FlightMode_DW.obj_f.WaypointIndex - 1];
            v[0] = rtb_wps_k[(int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex +
              1.0) - 1];
            rtb_Product[1] = rtb_wps_k[((int32_T)
              UAM_FlightMode_DW.obj_f.WaypointIndex + scalarLB) - 1];
            v[1] = rtb_wps_k[((int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex +
              1.0) + scalarLB) - 1];
            rtb_Product[2] = rtb_wps_k[((scalarLB << 1) + (int32_T)
              UAM_FlightMode_DW.obj_f.WaypointIndex) - 1];
            v[2] = rtb_wps_k[((int32_T)(UAM_FlightMode_DW.obj_f.WaypointIndex +
              1.0) + (scalarLB << 1)) - 1];
          }

          _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(v[0],
            UAM_FlightMode_U.States.Xe[0]), _mm_set1_pd(rtb_Product[0])));
          turnVector[0] = tmp[0];
          unitVectorUtoV_tmp[0] = tmp[1];
          _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(v[1],
            UAM_FlightMode_U.States.Xe[1]), _mm_set1_pd(rtb_Product[1])));
          turnVector[1] = tmp[0];
          unitVectorUtoV_tmp[1] = tmp[1];
          _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(v[2],
            UAM_FlightMode_U.States.Xe[2]), _mm_set1_pd(rtb_Product[2])));
          turnVector[2] = tmp[0];
          unitVectorUtoV_tmp[2] = tmp[1];
          turnInternal = unitVectorUtoV_tmp[0] * unitVectorUtoV_tmp[0] +
            unitVectorUtoV_tmp[1] * unitVectorUtoV_tmp[1];
          rtb_DotProduct = tmp[1] * tmp[1] + turnInternal;
          absxk = ((turnVector[0] * unitVectorUtoV_tmp[0] + turnVector[1] *
                    unitVectorUtoV_tmp[1]) + tmp[0] * tmp[1]) / rtb_DotProduct;
          if (absxk < 0.0) {
            absxk = UAM_FlightMode_norm_de(turnVector);
          } else if (absxk > 1.0) {
            tmp_1 = _mm_sub_pd(tmp_0, _mm_loadu_pd(&v[0]));
            _mm_storeu_pd(&turnVector_0[0], tmp_1);
            turnVector_0[2] = UAM_FlightMode_U.States.Xe[2] - v[2];
            absxk = UAM_FlightMode_norm_de(turnVector_0);
          } else {
            tmp_1 = _mm_sub_pd(tmp_0, _mm_add_pd(_mm_mul_pd(_mm_set1_pd(absxk),
              _mm_loadu_pd(&unitVectorUtoV_tmp[0])), _mm_loadu_pd(&rtb_Product[0])));
            _mm_storeu_pd(&turnVector_0[0], tmp_1);
            turnVector_0[2] = UAM_FlightMode_U.States.Xe[2] - (absxk * tmp[1] +
              rtb_Product[2]);
            absxk = UAM_FlightMode_norm_de(turnVector_0);
          }

          if (UAM_FlightMode_DW.obj_f.LastWaypointFlag) {
            absxk = (((UAM_FlightMode_U.States.Xe[0] - rtb_Product[0]) *
                      unitVectorUtoV_tmp[0] + (UAM_FlightMode_U.States.Xe[1] -
                       rtb_Product[1]) * unitVectorUtoV_tmp[1]) +
                     (UAM_FlightMode_U.States.Xe[2] - rtb_Product[2]) * tmp[1]) /
              rtb_DotProduct;
            tmp_1 = _mm_sub_pd(tmp_0, _mm_add_pd(_mm_mul_pd(_mm_set1_pd(absxk),
              _mm_loadu_pd(&unitVectorUtoV_tmp[0])), _mm_loadu_pd(&rtb_Product[0])));
            _mm_storeu_pd(&turnVector_0[0], tmp_1);
            turnVector_0[2] = UAM_FlightMode_U.States.Xe[2] - (absxk * tmp[1] +
              rtb_Product[2]);
            absxk = UAM_FlightMode_norm_de(turnVector_0);
          }

          t = fabs(absxk);
          if (rtIsInf(t) || rtIsNaN(t)) {
            rtb_DotProduct = (rtNaN);
            t = (rtNaN);
          } else if (t < 4.4501477170144028E-308) {
            rtb_DotProduct = 4.94065645841247E-324;
            t = 4.94065645841247E-324;
          } else {
            frexp(t, &b_exponent);
            rtb_DotProduct = ldexp(1.0, b_exponent - 53);
            frexp(t, &b_exponent_0);
            t = ldexp(1.0, b_exponent_0 - 53);
          }

          rtb_DotProduct = sqrt(rtb_DotProduct);
          t *= 5.0;
          if ((rtb_DotProduct >= t) || rtIsNaN(t)) {
            t = rtb_DotProduct;
          }

          if (absxk + t >= 30.0) {
            UAM_FlightMode_DW.obj_f.LookaheadDistance =
              UAM_FlightMode_DW.obj_f.LookaheadFactor * absxk;
          }

          turnVector[0] = unitVectorUtoV_tmp[0];
          turnVector[1] = unitVectorUtoV_tmp[1];
          tmp_1 = _mm_sub_pd(_mm_loadu_pd(&rtb_Product[0]), tmp_0);
          _mm_storeu_pd(&unitVectorUtoV_tmp[0], tmp_1);
          turnInternal += unitVectorUtoV_tmp[2] * unitVectorUtoV_tmp[2];
          unitVectorUtoV_tmp[2] = rtb_Product[2] - UAM_FlightMode_U.States.Xe[2];
          absxk = ((turnVector[0] * unitVectorUtoV_tmp[0] + turnVector[1] *
                    unitVectorUtoV_tmp[1]) + tmp[1] * unitVectorUtoV_tmp[2]) *
            2.0;
          t = sqrt(absxk * absxk - (((unitVectorUtoV_tmp[0] *
                      unitVectorUtoV_tmp[0] + unitVectorUtoV_tmp[1] *
                      unitVectorUtoV_tmp[1]) + unitVectorUtoV_tmp[2] *
                     unitVectorUtoV_tmp[2]) -
                    UAM_FlightMode_DW.obj_f.LookaheadDistance *
                    UAM_FlightMode_DW.obj_f.LookaheadDistance) * (4.0 *
                    turnInternal));
          rtb_DotProduct = (-absxk + t) / 2.0 / turnInternal;
          turnInternal = (-absxk - t) / 2.0 / turnInternal;
          if ((rtb_DotProduct >= turnInternal) || rtIsNaN(turnInternal)) {
            turnInternal = rtb_DotProduct;
          }

          tmp_1 = _mm_set1_pd(turnInternal);
          tmp_0 = _mm_add_pd(_mm_mul_pd(_mm_sub_pd(_mm_set1_pd(1.0), tmp_1),
            _mm_loadu_pd(&rtb_Product[0])), _mm_mul_pd(tmp_1, _mm_loadu_pd(&v[0])));
          _mm_storeu_pd(&v[0], tmp_0);

          /* BusCreator: '<S6>/Bus Creator' incorporates:
           *  Inport: '<Root>/States'
           *  SignalConversion generated from: '<S6>/Waypoint Follower'
           */
          UAM_FlightMode_B.aacSP.course = rt_atan2d_snf(v[1] -
            UAM_FlightMode_U.States.Xe[1], v[0] - UAM_FlightMode_U.States.Xe[0]);

          /* Merge: '<S3>/ Merge ' */
          UAM_FlightMode_B.Status = 0U;
          p = false;
          if (UAM_FlightMode_DW.obj_f.LastWaypointFlag) {
            p = true;
          }

          if (p) {
            /* Merge: '<S3>/ Merge ' */
            UAM_FlightMode_B.Status = 1U;
          }

          UAM_FlightMode_DW.obj_f.LastWaypointFlag = false;
        }
      }

      /* BusCreator: '<S6>/Bus Creator' incorporates:
       *  Constant: '<S6>/Constant'
       *  Constant: '<S6>/Constant1'
       *  Constant: '<S6>/Lookahead Distance'
       *  Inport: '<Root>/ToWP'
       *  MATLAB Function: '<S6>/MATLAB Function'
       */
      UAM_FlightMode_B.aacSP.airspeed = 15.0;
      UAM_FlightMode_B.aacSP.altitude = -UAM_FlightMode_U.ToWP.position[2];
      UAM_FlightMode_B.aacSP.L1 = 30.0;
      UAM_FlightMode_B.aacSP.climbrate = 0.0;
      break;

     case UAM_FlightMode_IN_FWCOMPLETE:
      /* Merge: '<S3>/ Merge ' */
      UAM_FlightMode_B.Status = 1U;

      /* Outport: '<Root>/controlMode' */
      UAM_FlightMode_Y.controlMode_m.TransitionCondition = 1U;
      break;

     case UAM_Flight_IN_ForwardTransition:
      if (UAM_Flig_transitionConditionMet()) {
        UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMode_IN_FWCOMPLETE;

        /* Merge: '<S3>/ Merge ' */
        UAM_FlightMode_B.Status = 1U;

        /* Outport: '<Root>/controlMode' */
        UAM_FlightMode_Y.controlMode_m.TransitionCondition = 1U;
      } else {
        /* BusCreator: '<S7>/Bus Creator1' incorporates:
         *  Constant: '<S7>/Constant1'
         *  Constant: '<S7>/Constant2'
         *  Constant: '<S7>/Constant3'
         *  Merge: '<S3>/ Merge 2'
         */
        UAM_FlightMode_B.InnerLoopCmds.LAP[0] = 0.0;
        UAM_FlightMode_B.InnerLoopCmds.LAP[1] = 0.0;
        UAM_FlightMode_B.InnerLoopCmds.LAP[2] = 0.0;
        UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = 0.0;
        UAM_FlightMode_B.InnerLoopCmds.YawCmd = 0.0;

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S7>/Constant'
         *  DataTypeConversion: '<S7>/Data Type Conversion'
         */
        UAM_FlightMode_B.Status = 0U;
      }
      break;

     case UAM_FlightMode_IN_HOVER_ENTRY:
      /* Inport: '<Root>/mode' */
      switch (UAM_FlightMode_U.mode) {
       case 2U:
        UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMode_IN_WP;

        /* Switch: '<S17>/Switch' incorporates:
         *  Inport: '<Root>/FromWP'
         *  Product: '<S21>/Product'
         *  RelationalOperator: '<S17>/Equal'
         */
        if (UAM_FlightMode_U.FromWP.mode == 6) {
          rtb_Product[0] = UAM_FlightMode_U.FromWP.position[0];
          rtb_Product[1] = UAM_FlightMode_U.FromWP.position[1];
          rtb_Product[2] = UAM_FlightMode_U.FromWP.position[2];
        } else {
          rtb_Product[0] = 0.0;
          rtb_Product[1] = 0.0;
          rtb_Product[2] = 0.0;
        }

        /* MATLAB Function: '<S12>/MATLAB Function' incorporates:
         *  BusCreator generated from: '<S12>/MATLAB Function'
         *  Inport: '<Root>/FromWP'
         *  Inport: '<Root>/ToWP'
         *  Product: '<S21>/Product'
         */
        rtb_wps[0] = rtb_Product[0];
        rtb_wps[2] = rtb_Product[1];
        rtb_wps[4] = rtb_Product[2];
        rtb_wps[6] = UAM_FlightMode_U.FromWP.params[3];
        rtb_wps[1] = UAM_FlightMode_U.ToWP.position[0];
        rtb_wps[3] = UAM_FlightMode_U.ToWP.position[1];
        rtb_wps[5] = UAM_FlightMode_U.ToWP.position[2];
        rtb_wps[7] = UAM_FlightMode_U.ToWP.params[3];

        /* BusCreator: '<S12>/Bus Creator' incorporates:
         *  Constant: '<S12>/Constant'
         *  Constant: '<S12>/Lookahead Distance'
         *  Inport: '<Root>/Pose'
         *  MATLABSystem: '<S12>/Waypoint Follower'
         *  Merge: '<S3>/ Merge 2'
         * */
        UAM_FlightMode_SystemCore_step(&UAM_FlightMode_DW.obj,
          UAM_FlightMode_U.Pose, rtb_wps, 5.0,
          UAM_FlightMode_B.InnerLoopCmds.LAP,
          &UAM_FlightMode_B.InnerLoopCmds.HeadingCmd, &turnInternal,
          &b_varargout_4);
        UAM_FlightMode_B.InnerLoopCmds.YawCmd = 0.0;

        /* Sum: '<S18>/Sum1' incorporates:
         *  Inport: '<Root>/ToWP'
         *  Product: '<S21>/Product'
         */
        turnInternal = UAM_FlightMode_U.ToWP.position[0] - rtb_Product[0];
        rtb_Product[0] = turnInternal;

        /* DotProduct: '<S21>/Dot Product1' incorporates:
         *  Product: '<S21>/Product'
         */
        rtb_DotProduct = turnInternal * turnInternal;

        /* Sum: '<S18>/Sum1' incorporates:
         *  Inport: '<Root>/ToWP'
         *  Product: '<S21>/Product'
         */
        turnInternal = UAM_FlightMode_U.ToWP.position[1] - rtb_Product[1];
        rtb_Product[1] = turnInternal;

        /* DotProduct: '<S21>/Dot Product1' incorporates:
         *  Product: '<S21>/Product'
         */
        rtb_DotProduct += turnInternal * turnInternal;

        /* Sum: '<S18>/Sum1' incorporates:
         *  Inport: '<Root>/ToWP'
         *  Product: '<S21>/Product'
         */
        turnInternal = UAM_FlightMode_U.ToWP.position[2] - rtb_Product[2];

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

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S12>/Lookahead Distance'
         *  DataTypeConversion: '<S12>/Data Type Conversion'
         *  DotProduct: '<S18>/Dot Product'
         *  Inport: '<Root>/Pose'
         *  Inport: '<Root>/ToWP'
         *  Product: '<S21>/Product'
         *  RelationalOperator: '<S12>/Relational Operator'
         *  Sum: '<S18>/Sum'
         *  Sum: '<S18>/Sum1'
         */
        UAM_FlightMode_B.Status = (uint8_T)(((UAM_FlightMode_U.ToWP.position[0]
          - UAM_FlightMode_U.Pose[0]) * (rtb_Product[0] / rtb_DotProduct) +
          (UAM_FlightMode_U.ToWP.position[1] - UAM_FlightMode_U.Pose[1]) *
          (rtb_Product[1] / rtb_DotProduct)) + (UAM_FlightMode_U.ToWP.position[2]
          - UAM_FlightMode_U.Pose[2]) * (turnInternal / rtb_DotProduct) <= 5.0);
        break;

       case 3U:
        UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMode_IN_Orbit;

        /* Abs: '<S10>/Abs' incorporates:
         *  Inport: '<Root>/ToWP'
         */
        t = fabs(UAM_FlightMode_U.ToWP.params[0]);

        /* Signum: '<S10>/Sign' incorporates:
         *  Inport: '<Root>/ToWP'
         */
        if (rtIsNaN(UAM_FlightMode_U.ToWP.params[1])) {
          turnInternal = (rtNaN);
        } else if (UAM_FlightMode_U.ToWP.params[1] < 0.0) {
          turnInternal = -1.0;
        } else {
          turnInternal = (UAM_FlightMode_U.ToWP.params[1] > 0.0);
        }

        /* MATLABSystem: '<S10>/UAV Orbit Follower' */
        UAM_FlightMode_DW.obj_l.OrbitRadiusFlag = 0U;
        if (t <= 1.0) {
          t = 1.0;
          UAM_FlightMode_DW.obj_l.OrbitRadiusFlag = 1U;
        }

        UAM_FlightMode_DW.obj_l.LookaheadDistFlag = 0U;

        /* Start for Inport: '<Root>/Pose' incorporates:
         *  Inport: '<Root>/ToWP'
         *  MATLABSystem: '<S10>/UAV Orbit Follower'
         * */
        tmp_0 = _mm_sub_pd(_mm_loadu_pd(&UAM_FlightMode_U.Pose[0]), _mm_loadu_pd
                           (&UAM_FlightMode_U.ToWP.position[0]));

        /* MATLABSystem: '<S10>/UAV Orbit Follower' incorporates:
         *  BusCreator: '<S10>/Bus Creator'
         *  Inport: '<Root>/Pose'
         *  Inport: '<Root>/ToWP'
         *  Merge: '<S3>/ Merge 2'
         */
        _mm_storeu_pd(&rtb_TmpSignalConversionAtUAVO_0[0], tmp_0);
        if (UAM_FlightMode_norm_d(rtb_TmpSignalConversionAtUAVO_0) <
            2.47032822920623E-323) {
          _mm_storeu_pd(&UAM_FlightMode_B.InnerLoopCmds.LAP[0], _mm_add_pd
                        (_mm_mul_pd(_mm_set1_pd(t), _mm_set_pd(sin
            (UAM_FlightMode_U.Pose[3]), cos(UAM_FlightMode_U.Pose[3]))),
                         _mm_loadu_pd(&UAM_FlightMode_U.Pose[0])));

          /* BusCreator: '<S10>/Bus Creator' incorporates:
           *  Inport: '<Root>/Pose'
           *  Inport: '<Root>/ToWP'
           *  MATLABSystem: '<S10>/UAV Orbit Follower'
           *  Merge: '<S3>/ Merge 2'
           */
          UAM_FlightMode_B.InnerLoopCmds.LAP[2] =
            UAM_FlightMode_U.ToWP.position[2];
          UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = UAM_FlightMode_U.Pose[3];
          UAM_FlightMode_B.InnerLoopCmds.YawCmd = UAM_FlightMode_U.Pose[3];
          t = UAM_FlightMode_DW.obj_l.NumCircles;
        } else {
          p = false;
          p_0 = true;
          b_k = 0;
          exitg1 = false;
          while ((!exitg1) && (b_k < 3)) {
            if (!(UAM_FlightMode_DW.obj_l.OrbitCenterInternal[b_k] ==
                  UAM_FlightMode_U.ToWP.position[b_k])) {
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
            if (UAM_FlightMode_DW.obj_l.OrbitRadiusInternal == t) {
              p = true;
            }

            if (!p) {
              guard2 = true;
            }
          }

          if (guard2) {
            UAM_FlightMode_DW.obj_l.NumCircles = 0.0;
            UAM_FlightMode_DW.obj_l.OrbitCenterInternal[0] =
              UAM_FlightMode_U.ToWP.position[0];
            UAM_FlightMode_DW.obj_l.OrbitCenterInternal[1] =
              UAM_FlightMode_U.ToWP.position[1];
            UAM_FlightMode_DW.obj_l.OrbitCenterInternal[2] =
              UAM_FlightMode_U.ToWP.position[2];
            UAM_FlightMode_DW.obj_l.OrbitRadiusInternal = t;
            UAM_FlightMode_DW.obj_l.SelectTurnDirectionFlag = true;
          }

          if (t <= 5.0) {
            UAM_FlightMode_DW.obj_l.LookaheadDistance = 0.9 * t;
          } else {
            UAM_FlightMode_DW.obj_l.LookaheadDistance = 5.0;
          }

          /* Start for MATLABSystem: '<S10>/UAV Orbit Follower' incorporates:
           *  Inport: '<Root>/Pose'
           *  Inport: '<Root>/ToWP'
           */
          absxk = UAM_FlightMode_U.Pose[0] - UAM_FlightMode_U.ToWP.position[0];
          b_tmp[0] = absxk;
          rtb_DotProduct = absxk * absxk;
          absxk = UAM_FlightMode_U.Pose[1] - UAM_FlightMode_U.ToWP.position[1];
          b_tmp[1] = absxk;
          distToCenter = sqrt(absxk * absxk + rtb_DotProduct);

          /* Start for MATLABSystem: '<S10>/UAV Orbit Follower' */
          rtb_DotProduct_tmp = t + UAM_FlightMode_DW.obj_l.LookaheadDistance;
          if (rtIsInf(rtb_DotProduct_tmp) || rtIsNaN(rtb_DotProduct_tmp)) {
            rtb_DotProduct = (rtNaN);
          } else if (rtb_DotProduct_tmp < 4.4501477170144028E-308) {
            rtb_DotProduct = 4.94065645841247E-324;
          } else {
            frexp(rtb_DotProduct_tmp, &b_exponent);
            rtb_DotProduct = ldexp(1.0, b_exponent - 53);
          }

          guard2 = false;
          if (distToCenter >= rtb_DotProduct_tmp - 5.0 * rtb_DotProduct) {
            guard2 = true;
          } else {
            if (rtIsInf(rtb_DotProduct_tmp) || rtIsNaN(rtb_DotProduct_tmp)) {
              rtb_DotProduct = (rtNaN);
            } else if (rtb_DotProduct_tmp < 4.4501477170144028E-308) {
              rtb_DotProduct = 4.94065645841247E-324;
            } else {
              frexp(rtb_DotProduct_tmp, &b_exponent_0);
              rtb_DotProduct = ldexp(1.0, b_exponent_0 - 53);
            }

            if (distToCenter <= (t - UAM_FlightMode_DW.obj_l.LookaheadDistance)
                + 5.0 * rtb_DotProduct) {
              guard2 = true;
            } else {
              if (UAM_FlightMode_DW.obj_l.StartFlag) {
                UAM_FlightMode_DW.obj_l.PrevPosition[0] = UAM_FlightMode_U.Pose
                  [0];
                UAM_FlightMode_DW.obj_l.PrevPosition[1] = UAM_FlightMode_U.Pose
                  [1];
                UAM_FlightMode_DW.obj_l.StartFlag = false;
              }

              if ((turnInternal == 0.0) &&
                  (!UAM_FlightMode_DW.obj_l.SelectTurnDirectionFlag)) {
                turnInternal = UAM_FlightMode_DW.obj_l.TurnDirectionInternal;
              }

              _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
                (UAM_FlightMode_DW.obj_l.PrevPosition[0], UAM_FlightMode_U.Pose
                 [0]), _mm_set1_pd(UAM_FlightMode_U.ToWP.position[0])));
              xyPose[0] = tmp[0];
              turnVector[0] = tmp[1];
              _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
                (UAM_FlightMode_DW.obj_l.PrevPosition[1], UAM_FlightMode_U.Pose
                 [1]), _mm_set1_pd(UAM_FlightMode_U.ToWP.position[1])));
              xyPose[1] = tmp[0];
              turnVector[1] = tmp[1];
              absxk = UAM_FlightMode_norm_d(xyPose);
              rtb_DotProduct = ((UAM_FlightMode_DW.obj_l.LookaheadDistance *
                                 UAM_FlightMode_DW.obj_l.LookaheadDistance - t *
                                 t) + absxk * absxk) / (2.0 * absxk);
              tmp_1 = _mm_set1_pd(absxk);
              tmp_2 = _mm_loadu_pd(&UAM_FlightMode_U.Pose[0]);
              tmp_3 = _mm_sub_pd(_mm_loadu_pd(&UAM_FlightMode_U.ToWP.position[0]),
                                 tmp_2);
              _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(tmp_3,
                _mm_set1_pd(rtb_DotProduct)), tmp_1), tmp_2));
              t = tmp[1];
              distToCenter = sqrt(UAM_FlightMode_DW.obj_l.LookaheadDistance *
                                  UAM_FlightMode_DW.obj_l.LookaheadDistance -
                                  rtb_DotProduct * rtb_DotProduct);
              b_tmp[0] = tmp[0] - (UAM_FlightMode_U.ToWP.position[1] -
                                   UAM_FlightMode_U.Pose[1]) * distToCenter /
                absxk;
              _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd
                (_mm_set_pd(UAM_FlightMode_U.ToWP.position[0],
                            UAM_FlightMode_U.ToWP.position[1]), _mm_set_pd
                 (UAM_FlightMode_U.Pose[0], UAM_FlightMode_U.Pose[1])),
                _mm_set1_pd(distToCenter)), tmp_1), _mm_set_pd(tmp[1], tmp[0])));
              b_tmp[1] = tmp[0];
              rtb_DotProduct = tmp[1];
              absxk = t - (UAM_FlightMode_U.ToWP.position[0] -
                           UAM_FlightMode_U.Pose[0]) * distToCenter / absxk;
              rtb_Product[0] = turnVector[0];
              rtb_Product[1] = turnVector[1];
              rtb_Product[2] = 0.0;
              _mm_storeu_pd(&v[0], tmp_0);
              v[2] = 0.0;
              if (turnInternal < 0.0) {
                rtb_Product[0] = v[0];
                v[0] = turnVector[0];
                rtb_Product[1] = v[1];
                v[1] = turnVector[1];
                rtb_Product[2] = 0.0;
                v[2] = 0.0;
              }

              distToCenter = UAM_FlightMode_norm_de(rtb_Product);
              t = UAM_FlightMode_norm_de(v);
              tmp_1 = _mm_set_pd(t, distToCenter);
              _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(v[0], rtb_Product[0]),
                tmp_1));
              rtb_Product[0] = tmp[0];
              v[0] = tmp[1];
              _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(v[1], rtb_Product[1]),
                tmp_1));
              v[1] = tmp[1];
              turnVector[2] = rtb_Product[0] * tmp[1] - v[0] * tmp[0];
              UAM_FlightMode_DW.obj_l.PrevPosition[0] = UAM_FlightMode_U.Pose[0];
              UAM_FlightMode_DW.obj_l.PrevPosition[1] = UAM_FlightMode_U.Pose[1];
              UAM_FlightMode_DW.obj_l.PrevPosition[2] = UAM_FlightMode_U.Pose[2];
              UAM_FlightMode_DW.obj_l.NumCircles += rt_atan2d_snf(turnVector[2],
                (rtb_Product[0] * v[0] + tmp[0] * tmp[1]) + 0.0 / distToCenter *
                (0.0 / t)) / 2.0 / 3.1415926535897931;
              t = UAM_FlightMode_DW.obj_l.NumCircles;
              _mm_storeu_pd(&v[0], tmp_3);
              if (rtIsNaN(turnInternal)) {
                distToCenter = (rtNaN);
              } else if (turnInternal < 0.0) {
                distToCenter = -1.0;
              } else {
                distToCenter = (turnInternal > 0.0);
              }

              switch ((int32_T)distToCenter) {
               case 1:
                if ((b_tmp[0] - UAM_FlightMode_U.Pose[0]) * v[1] -
                    (rtb_DotProduct - UAM_FlightMode_U.Pose[1]) * v[0] > 0.0) {
                  turnInternal = b_tmp[0];
                  absxk = rtb_DotProduct;
                } else {
                  turnInternal = b_tmp[1];
                }
                break;

               case -1:
                if ((b_tmp[0] - UAM_FlightMode_U.Pose[0]) * v[1] -
                    (rtb_DotProduct - UAM_FlightMode_U.Pose[1]) * v[0] < 0.0) {
                  turnInternal = b_tmp[0];
                  absxk = rtb_DotProduct;
                } else {
                  turnInternal = b_tmp[1];
                }
                break;

               default:
                if (fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(rtb_DotProduct -
                       UAM_FlightMode_U.Pose[1], b_tmp[0] -
                       UAM_FlightMode_U.Pose[0]), UAM_FlightMode_U.Pose[3])) <
                    fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(absxk -
                       UAM_FlightMode_U.Pose[1], b_tmp[1] -
                       UAM_FlightMode_U.Pose[0]), UAM_FlightMode_U.Pose[3]))) {
                  turnInternal = b_tmp[0];
                  absxk = rtb_DotProduct;
                } else {
                  turnInternal = b_tmp[1];
                }

                if ((turnInternal - UAM_FlightMode_U.Pose[0]) * v[1] - (absxk -
                     UAM_FlightMode_U.Pose[1]) * v[0] > 0.0) {
                  UAM_FlightMode_DW.obj_l.TurnDirectionInternal = 1.0;
                } else {
                  UAM_FlightMode_DW.obj_l.TurnDirectionInternal = -1.0;
                }

                UAM_FlightMode_DW.obj_l.SelectTurnDirectionFlag = false;
                break;
              }

              /* BusCreator: '<S10>/Bus Creator' incorporates:
               *  Inport: '<Root>/Pose'
               *  Inport: '<Root>/ToWP'
               *  Merge: '<S3>/ Merge 2'
               */
              UAM_FlightMode_B.InnerLoopCmds.YawCmd = rt_atan2d_snf
                (UAM_FlightMode_U.ToWP.position[1] - UAM_FlightMode_U.Pose[1],
                 UAM_FlightMode_U.ToWP.position[0] - UAM_FlightMode_U.Pose[0]);
            }
          }

          if (guard2) {
            _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_mul_pd(_mm_div_pd(_mm_set_pd
              (absxk, b_tmp[0]), _mm_set1_pd(UAM_FlightMode_norm_d(b_tmp))),
              _mm_set1_pd(t)), _mm_loadu_pd(&UAM_FlightMode_U.ToWP.position[0])));
            turnInternal = tmp[0];
            absxk = tmp[1];

            /* BusCreator: '<S10>/Bus Creator' incorporates:
             *  Inport: '<Root>/Pose'
             *  Inport: '<Root>/ToWP'
             *  Merge: '<S3>/ Merge 2'
             */
            UAM_FlightMode_B.InnerLoopCmds.YawCmd = rt_atan2d_snf(tmp[1] -
              UAM_FlightMode_U.Pose[1], tmp[0] - UAM_FlightMode_U.Pose[0]);
            t = UAM_FlightMode_DW.obj_l.NumCircles;
          }

          /* BusCreator: '<S10>/Bus Creator' incorporates:
           *  Inport: '<Root>/Pose'
           *  Inport: '<Root>/ToWP'
           *  MATLABSystem: '<S10>/UAV Orbit Follower'
           *  Merge: '<S3>/ Merge 2'
           */
          UAM_FlightMode_B.InnerLoopCmds.LAP[0] = turnInternal;
          UAM_FlightMode_B.InnerLoopCmds.LAP[1] = absxk;
          UAM_FlightMode_B.InnerLoopCmds.LAP[2] =
            UAM_FlightMode_U.ToWP.position[2];
          UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = rt_atan2d_snf(absxk -
            UAM_FlightMode_U.Pose[1], turnInternal - UAM_FlightMode_U.Pose[0]);
        }

        /* Merge: '<S3>/ Merge ' incorporates:
         *  DataTypeConversion: '<S10>/Data Type Conversion'
         *  Inport: '<Root>/ToWP'
         *  MATLABSystem: '<S10>/UAV Orbit Follower'
         *  RelationalOperator: '<S10>/Relational Operator'
         * */
        UAM_FlightMode_B.Status = (uint8_T)(t > UAM_FlightMode_U.ToWP.params[2]);
        break;

       case 6U:
        /* Outport: '<Root>/FlightMode' */
        UAM_FlightMode_Y.FlightMode = Transition;

        /* Outport: '<Root>/controlMode' */
        UAM_FlightMode_Y.controlMode_m.inTransition = 1U;
        UAM_FlightMode_Y.controlMode_m.TransitionCondition = 0U;
        UAM_FlightMode_DW.is_GuidanceLogic = UAM_Flight_IN_ForwardTransition;

        /* BusCreator: '<S7>/Bus Creator1' incorporates:
         *  Constant: '<S7>/Constant1'
         *  Constant: '<S7>/Constant2'
         *  Constant: '<S7>/Constant3'
         *  Merge: '<S3>/ Merge 2'
         */
        UAM_FlightMode_B.InnerLoopCmds.LAP[0] = 0.0;
        UAM_FlightMode_B.InnerLoopCmds.LAP[1] = 0.0;
        UAM_FlightMode_B.InnerLoopCmds.LAP[2] = 0.0;
        UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = 0.0;
        UAM_FlightMode_B.InnerLoopCmds.YawCmd = 0.0;

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S7>/Constant'
         *  DataTypeConversion: '<S7>/Data Type Conversion'
         */
        UAM_FlightMode_B.Status = 0U;
        break;

       default:
        UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMode_IN_Land;
        UAM_FlightMode_DW.is_Land = UAM_FlightMode_IN_ToLand;

        /* MATLAB Function: '<S9>/MATLAB Function' incorporates:
         *  Inport: '<Root>/FromWP'
         *  Inport: '<Root>/ToWP'
         */
        rtb_wps[0] = UAM_FlightMode_U.FromWP.position[0];
        rtb_wps[2] = UAM_FlightMode_U.FromWP.position[1];
        rtb_wps[4] = UAM_FlightMode_U.FromWP.position[2];
        rtb_wps[6] = UAM_FlightMode_U.FromWP.params[3];
        rtb_wps[1] = UAM_FlightMode_U.ToWP.position[0];
        rtb_wps[3] = UAM_FlightMode_U.ToWP.position[1];
        rtb_wps[5] = UAM_FlightMode_U.ToWP.position[2];
        rtb_wps[7] = UAM_FlightMode_U.ToWP.params[3];

        /* Inport: '<Root>/Pose' incorporates:
         *  Constant: '<S9>/Lookahead Distance'
         */
        UAM_FlightMode_WaypointFollower(UAM_FlightMode_U.Pose, rtb_wps, 3.0,
          &UAM_FlightMode_B.WaypointFollower_c2,
          &UAM_FlightMode_DW.WaypointFollower_c2);

        /* BusCreator: '<S9>/Bus Creator1' incorporates:
         *  MATLABSystem: '<S9>/Waypoint Follower'
         *  Merge: '<S3>/ Merge 2'
         */
        UAM_FlightMode_B.InnerLoopCmds.LAP[0] =
          UAM_FlightMode_B.WaypointFollower_c2.LAP[0];
        UAM_FlightMode_B.InnerLoopCmds.LAP[1] =
          UAM_FlightMode_B.WaypointFollower_c2.LAP[1];
        UAM_FlightMode_B.InnerLoopCmds.LAP[2] =
          UAM_FlightMode_B.WaypointFollower_c2.LAP[2];
        UAM_FlightMode_B.InnerLoopCmds.HeadingCmd =
          UAM_FlightMode_B.WaypointFollower_c2.HeadingCmd;
        UAM_FlightMode_B.InnerLoopCmds.YawCmd =
          UAM_FlightMode_B.WaypointFollower_c2.YawCmd;

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S9>/Constant'
         *  SignalConversion generated from: '<S9>/Status'
         */
        UAM_FlightMode_B.Status = 0U;
        break;
      }
      break;

     case UAM_FlightMode_IN_Land:
      if (UAM_FlightMode_DW.is_Land == UAM_FlightMode_IN_Descend) {
        /* MATLAB Function: '<S8>/MATLAB Function' incorporates:
         *  Inport: '<Root>/FromWP'
         *  Inport: '<Root>/Ground'
         *  Inport: '<Root>/ToWP'
         *  SignalConversion generated from: '<S15>/ SFunction '
         */
        rtb_wps[0] = UAM_FlightMode_U.FromWP.position[0];
        rtb_wps[2] = UAM_FlightMode_U.FromWP.position[1];
        rtb_wps[4] = UAM_FlightMode_U.FromWP.position[2];
        rtb_wps[6] = UAM_FlightMode_U.FromWP.params[3];
        rtb_wps[1] = UAM_FlightMode_U.ToWP.position[0];
        rtb_wps[3] = UAM_FlightMode_U.ToWP.position[1];
        rtb_wps[5] = UAM_FlightMode_U.Ground;
        rtb_wps[7] = UAM_FlightMode_U.ToWP.params[3];

        /* Inport: '<Root>/Pose' incorporates:
         *  Constant: '<S8>/Constant'
         */
        UAM_FlightMode_WaypointFollower(UAM_FlightMode_U.Pose, rtb_wps, 3.0,
          &UAM_FlightMode_B.WaypointFollower_c,
          &UAM_FlightMode_DW.WaypointFollower_c);

        /* BusCreator: '<S8>/Bus Creator' incorporates:
         *  MATLABSystem: '<S8>/Waypoint Follower'
         *  Merge: '<S3>/ Merge 2'
         */
        UAM_FlightMode_B.InnerLoopCmds.LAP[0] =
          UAM_FlightMode_B.WaypointFollower_c.LAP[0];
        UAM_FlightMode_B.InnerLoopCmds.LAP[1] =
          UAM_FlightMode_B.WaypointFollower_c.LAP[1];
        UAM_FlightMode_B.InnerLoopCmds.LAP[2] =
          UAM_FlightMode_B.WaypointFollower_c.LAP[2];

        /* Saturate: '<S8>/Hdg Cmd Sat' incorporates:
         *  Inport: '<Root>/ToWP'
         *  Saturate: '<S8>/Yaw Cmd Sat'
         */
        if (UAM_FlightMode_U.ToWP.params[3] > 3.1415926535897931) {
          /* BusCreator: '<S8>/Bus Creator' incorporates:
           *  Merge: '<S3>/ Merge 2'
           */
          UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = 3.1415926535897931;
          UAM_FlightMode_B.InnerLoopCmds.YawCmd = 3.1415926535897931;
        } else if (UAM_FlightMode_U.ToWP.params[3] < -3.1415926535897931) {
          /* BusCreator: '<S8>/Bus Creator' incorporates:
           *  Merge: '<S3>/ Merge 2'
           */
          UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = -3.1415926535897931;
          UAM_FlightMode_B.InnerLoopCmds.YawCmd = -3.1415926535897931;
        } else {
          /* BusCreator: '<S8>/Bus Creator' incorporates:
           *  Merge: '<S3>/ Merge 2'
           */
          UAM_FlightMode_B.InnerLoopCmds.HeadingCmd =
            UAM_FlightMode_U.ToWP.params[3];
          UAM_FlightMode_B.InnerLoopCmds.YawCmd = UAM_FlightMode_U.ToWP.params[3];
        }

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S8>/Constant3'
         *  SignalConversion generated from: '<S8>/Status'
         */
        UAM_FlightMode_B.Status = 0U;
      } else {
        /* case IN_ToLand: */
        rtb_DotProduct = 3.3121686421112381E-170;

        /* Inport: '<Root>/Pose' incorporates:
         *  Inport: '<Root>/ToWP'
         */
        absxk = fabs(UAM_FlightMode_U.Pose[0] - UAM_FlightMode_U.ToWP.position[0]);
        if (absxk > 3.3121686421112381E-170) {
          turnInternal = 1.0;
          rtb_DotProduct = absxk;
        } else {
          t = absxk / 3.3121686421112381E-170;
          turnInternal = t * t;
        }

        /* Inport: '<Root>/Pose' incorporates:
         *  Inport: '<Root>/ToWP'
         */
        absxk = fabs(UAM_FlightMode_U.Pose[1] - UAM_FlightMode_U.ToWP.position[1]);
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
          UAM_FlightMode_DW.is_Land = UAM_FlightMode_IN_Descend;

          /* MATLAB Function: '<S8>/MATLAB Function' incorporates:
           *  Inport: '<Root>/FromWP'
           *  Inport: '<Root>/Ground'
           *  Inport: '<Root>/ToWP'
           *  SignalConversion generated from: '<S15>/ SFunction '
           */
          rtb_wps[0] = UAM_FlightMode_U.FromWP.position[0];
          rtb_wps[2] = UAM_FlightMode_U.FromWP.position[1];
          rtb_wps[4] = UAM_FlightMode_U.FromWP.position[2];
          rtb_wps[6] = UAM_FlightMode_U.FromWP.params[3];
          rtb_wps[1] = UAM_FlightMode_U.ToWP.position[0];
          rtb_wps[3] = UAM_FlightMode_U.ToWP.position[1];
          rtb_wps[5] = UAM_FlightMode_U.Ground;
          rtb_wps[7] = UAM_FlightMode_U.ToWP.params[3];

          /* Inport: '<Root>/Pose' incorporates:
           *  Constant: '<S8>/Constant'
           */
          UAM_FlightMode_WaypointFollower(UAM_FlightMode_U.Pose, rtb_wps, 3.0,
            &UAM_FlightMode_B.WaypointFollower_c,
            &UAM_FlightMode_DW.WaypointFollower_c);

          /* BusCreator: '<S8>/Bus Creator' incorporates:
           *  MATLABSystem: '<S8>/Waypoint Follower'
           *  Merge: '<S3>/ Merge 2'
           */
          UAM_FlightMode_B.InnerLoopCmds.LAP[0] =
            UAM_FlightMode_B.WaypointFollower_c.LAP[0];
          UAM_FlightMode_B.InnerLoopCmds.LAP[1] =
            UAM_FlightMode_B.WaypointFollower_c.LAP[1];
          UAM_FlightMode_B.InnerLoopCmds.LAP[2] =
            UAM_FlightMode_B.WaypointFollower_c.LAP[2];

          /* Saturate: '<S8>/Hdg Cmd Sat' incorporates:
           *  Inport: '<Root>/ToWP'
           *  Saturate: '<S8>/Yaw Cmd Sat'
           */
          if (UAM_FlightMode_U.ToWP.params[3] > 3.1415926535897931) {
            /* BusCreator: '<S8>/Bus Creator' incorporates:
             *  Merge: '<S3>/ Merge 2'
             */
            UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = 3.1415926535897931;
            UAM_FlightMode_B.InnerLoopCmds.YawCmd = 3.1415926535897931;
          } else if (UAM_FlightMode_U.ToWP.params[3] < -3.1415926535897931) {
            /* BusCreator: '<S8>/Bus Creator' incorporates:
             *  Merge: '<S3>/ Merge 2'
             */
            UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = -3.1415926535897931;
            UAM_FlightMode_B.InnerLoopCmds.YawCmd = -3.1415926535897931;
          } else {
            /* BusCreator: '<S8>/Bus Creator' incorporates:
             *  Merge: '<S3>/ Merge 2'
             */
            UAM_FlightMode_B.InnerLoopCmds.HeadingCmd =
              UAM_FlightMode_U.ToWP.params[3];
            UAM_FlightMode_B.InnerLoopCmds.YawCmd =
              UAM_FlightMode_U.ToWP.params[3];
          }

          /* Merge: '<S3>/ Merge ' incorporates:
           *  Constant: '<S8>/Constant3'
           *  SignalConversion generated from: '<S8>/Status'
           */
          UAM_FlightMode_B.Status = 0U;
        } else {
          /* MATLAB Function: '<S9>/MATLAB Function' incorporates:
           *  Inport: '<Root>/FromWP'
           *  Inport: '<Root>/ToWP'
           */
          rtb_wps[0] = UAM_FlightMode_U.FromWP.position[0];
          rtb_wps[2] = UAM_FlightMode_U.FromWP.position[1];
          rtb_wps[4] = UAM_FlightMode_U.FromWP.position[2];
          rtb_wps[6] = UAM_FlightMode_U.FromWP.params[3];
          rtb_wps[1] = UAM_FlightMode_U.ToWP.position[0];
          rtb_wps[3] = UAM_FlightMode_U.ToWP.position[1];
          rtb_wps[5] = UAM_FlightMode_U.ToWP.position[2];
          rtb_wps[7] = UAM_FlightMode_U.ToWP.params[3];

          /* Inport: '<Root>/Pose' incorporates:
           *  Constant: '<S9>/Lookahead Distance'
           */
          UAM_FlightMode_WaypointFollower(UAM_FlightMode_U.Pose, rtb_wps, 3.0,
            &UAM_FlightMode_B.WaypointFollower_c2,
            &UAM_FlightMode_DW.WaypointFollower_c2);

          /* BusCreator: '<S9>/Bus Creator1' incorporates:
           *  MATLABSystem: '<S9>/Waypoint Follower'
           *  Merge: '<S3>/ Merge 2'
           */
          UAM_FlightMode_B.InnerLoopCmds.LAP[0] =
            UAM_FlightMode_B.WaypointFollower_c2.LAP[0];
          UAM_FlightMode_B.InnerLoopCmds.LAP[1] =
            UAM_FlightMode_B.WaypointFollower_c2.LAP[1];
          UAM_FlightMode_B.InnerLoopCmds.LAP[2] =
            UAM_FlightMode_B.WaypointFollower_c2.LAP[2];
          UAM_FlightMode_B.InnerLoopCmds.HeadingCmd =
            UAM_FlightMode_B.WaypointFollower_c2.HeadingCmd;
          UAM_FlightMode_B.InnerLoopCmds.YawCmd =
            UAM_FlightMode_B.WaypointFollower_c2.YawCmd;

          /* Merge: '<S3>/ Merge ' incorporates:
           *  Constant: '<S9>/Constant'
           *  SignalConversion generated from: '<S9>/Status'
           */
          UAM_FlightMode_B.Status = 0U;
        }
      }
      break;

     case UAM_FlightMode_IN_Orbit:
      /* Abs: '<S10>/Abs' incorporates:
       *  Inport: '<Root>/ToWP'
       */
      t = fabs(UAM_FlightMode_U.ToWP.params[0]);

      /* Signum: '<S10>/Sign' incorporates:
       *  Inport: '<Root>/ToWP'
       */
      if (rtIsNaN(UAM_FlightMode_U.ToWP.params[1])) {
        turnInternal = (rtNaN);
      } else if (UAM_FlightMode_U.ToWP.params[1] < 0.0) {
        turnInternal = -1.0;
      } else {
        turnInternal = (UAM_FlightMode_U.ToWP.params[1] > 0.0);
      }

      /* MATLABSystem: '<S10>/UAV Orbit Follower' */
      UAM_FlightMode_DW.obj_l.OrbitRadiusFlag = 0U;
      if (t <= 1.0) {
        t = 1.0;
        UAM_FlightMode_DW.obj_l.OrbitRadiusFlag = 1U;
      }

      UAM_FlightMode_DW.obj_l.LookaheadDistFlag = 0U;

      /* Start for Inport: '<Root>/Pose' incorporates:
       *  MATLABSystem: '<S10>/UAV Orbit Follower'
       * */
      tmp_0 = _mm_loadu_pd(&UAM_FlightMode_U.Pose[0]);

      /* Start for MATLABSystem: '<S10>/UAV Orbit Follower' incorporates:
       *  Inport: '<Root>/ToWP'
       */
      tmp_2 = _mm_loadu_pd(&UAM_FlightMode_U.ToWP.position[0]);
      tmp_1 = _mm_sub_pd(tmp_0, tmp_2);

      /* MATLABSystem: '<S10>/UAV Orbit Follower' incorporates:
       *  BusCreator: '<S10>/Bus Creator'
       *  Inport: '<Root>/Pose'
       *  Inport: '<Root>/ToWP'
       *  Merge: '<S3>/ Merge 2'
       */
      _mm_storeu_pd(&rtb_TmpSignalConversionAtUAVO_0[0], tmp_1);
      if (UAM_FlightMode_norm_d(rtb_TmpSignalConversionAtUAVO_0) <
          2.47032822920623E-323) {
        _mm_storeu_pd(&UAM_FlightMode_B.InnerLoopCmds.LAP[0], _mm_add_pd
                      (_mm_mul_pd(_mm_set1_pd(t), _mm_set_pd(sin
          (UAM_FlightMode_U.Pose[3]), cos(UAM_FlightMode_U.Pose[3]))), tmp_0));

        /* BusCreator: '<S10>/Bus Creator' incorporates:
         *  Inport: '<Root>/Pose'
         *  Inport: '<Root>/ToWP'
         *  MATLABSystem: '<S10>/UAV Orbit Follower'
         *  Merge: '<S3>/ Merge 2'
         */
        UAM_FlightMode_B.InnerLoopCmds.LAP[2] = UAM_FlightMode_U.ToWP.position[2];
        UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = UAM_FlightMode_U.Pose[3];
        UAM_FlightMode_B.InnerLoopCmds.YawCmd = UAM_FlightMode_U.Pose[3];
        t = UAM_FlightMode_DW.obj_l.NumCircles;
      } else {
        p = false;
        p_0 = true;
        b_k = 0;
        exitg1 = false;
        while ((!exitg1) && (b_k < 3)) {
          if (!(UAM_FlightMode_DW.obj_l.OrbitCenterInternal[b_k] ==
                UAM_FlightMode_U.ToWP.position[b_k])) {
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
          if (UAM_FlightMode_DW.obj_l.OrbitRadiusInternal == t) {
            p = true;
          }

          if (!p) {
            guard2 = true;
          }
        }

        if (guard2) {
          UAM_FlightMode_DW.obj_l.NumCircles = 0.0;
          UAM_FlightMode_DW.obj_l.OrbitCenterInternal[0] =
            UAM_FlightMode_U.ToWP.position[0];
          UAM_FlightMode_DW.obj_l.OrbitCenterInternal[1] =
            UAM_FlightMode_U.ToWP.position[1];
          UAM_FlightMode_DW.obj_l.OrbitCenterInternal[2] =
            UAM_FlightMode_U.ToWP.position[2];
          UAM_FlightMode_DW.obj_l.OrbitRadiusInternal = t;
          UAM_FlightMode_DW.obj_l.SelectTurnDirectionFlag = true;
        }

        if (t <= 5.0) {
          UAM_FlightMode_DW.obj_l.LookaheadDistance = 0.9 * t;
        } else {
          UAM_FlightMode_DW.obj_l.LookaheadDistance = 5.0;
        }

        /* Start for MATLABSystem: '<S10>/UAV Orbit Follower' incorporates:
         *  Inport: '<Root>/Pose'
         *  Inport: '<Root>/ToWP'
         */
        absxk = UAM_FlightMode_U.Pose[0] - UAM_FlightMode_U.ToWP.position[0];
        b_tmp[0] = absxk;
        rtb_DotProduct = absxk * absxk;
        absxk = UAM_FlightMode_U.Pose[1] - UAM_FlightMode_U.ToWP.position[1];
        b_tmp[1] = absxk;
        distToCenter = sqrt(absxk * absxk + rtb_DotProduct);
        rtb_DotProduct = t + UAM_FlightMode_DW.obj_l.LookaheadDistance;
        if (rtIsInf(rtb_DotProduct) || rtIsNaN(rtb_DotProduct)) {
          rtb_DotProduct = (rtNaN);
        } else if (rtb_DotProduct < 4.4501477170144028E-308) {
          rtb_DotProduct = 4.94065645841247E-324;
        } else {
          frexp(rtb_DotProduct, &b_exponent);
          rtb_DotProduct = ldexp(1.0, b_exponent - 53);
        }

        /* Start for MATLABSystem: '<S10>/UAV Orbit Follower' */
        rtb_DotProduct_tmp = t + UAM_FlightMode_DW.obj_l.LookaheadDistance;
        guard2 = false;
        if (distToCenter >= rtb_DotProduct_tmp - 5.0 * rtb_DotProduct) {
          guard2 = true;
        } else {
          if (rtIsInf(rtb_DotProduct_tmp) || rtIsNaN(rtb_DotProduct_tmp)) {
            rtb_DotProduct = (rtNaN);
          } else if (rtb_DotProduct_tmp < 4.4501477170144028E-308) {
            rtb_DotProduct = 4.94065645841247E-324;
          } else {
            frexp(rtb_DotProduct_tmp, &b_exponent_0);
            rtb_DotProduct = ldexp(1.0, b_exponent_0 - 53);
          }

          if (distToCenter <= (t - UAM_FlightMode_DW.obj_l.LookaheadDistance) +
              5.0 * rtb_DotProduct) {
            guard2 = true;
          } else {
            if (UAM_FlightMode_DW.obj_l.StartFlag) {
              UAM_FlightMode_DW.obj_l.PrevPosition[0] = UAM_FlightMode_U.Pose[0];
              UAM_FlightMode_DW.obj_l.PrevPosition[1] = UAM_FlightMode_U.Pose[1];
              UAM_FlightMode_DW.obj_l.PrevPosition[2] = UAM_FlightMode_U.Pose[2];
              UAM_FlightMode_DW.obj_l.StartFlag = false;
            }

            if ((turnInternal == 0.0) &&
                (!UAM_FlightMode_DW.obj_l.SelectTurnDirectionFlag)) {
              turnInternal = UAM_FlightMode_DW.obj_l.TurnDirectionInternal;
            }

            rtb_Product[2] = 0.0;
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd
              (UAM_FlightMode_DW.obj_l.PrevPosition[0], UAM_FlightMode_U.Pose[0]),
              _mm_set1_pd(UAM_FlightMode_U.ToWP.position[0])));
            xyPose[0] = tmp[0];
            rtb_Product[0] = tmp[1];
            _mm_storeu_pd(&tmp[0], tmp_1);
            v[0] = tmp[0];
            xyPose[1] = tmp[1];
            _mm_storeu_pd(&tmp[0], _mm_sub_pd(_mm_set_pd(UAM_FlightMode_U.Pose[1],
              UAM_FlightMode_DW.obj_l.PrevPosition[1]), _mm_set1_pd
              (UAM_FlightMode_U.ToWP.position[1])));
            rtb_Product[1] = tmp[0];
            v[1] = tmp[1];
            absxk = UAM_FlightMode_norm_d(xyPose);
            rtb_DotProduct = ((UAM_FlightMode_DW.obj_l.LookaheadDistance *
                               UAM_FlightMode_DW.obj_l.LookaheadDistance - t * t)
                              + absxk * absxk) / (2.0 * absxk);
            tmp_1 = _mm_set1_pd(absxk);
            tmp_3 = _mm_sub_pd(_mm_loadu_pd(&UAM_FlightMode_U.ToWP.position[0]),
                               _mm_loadu_pd(&UAM_FlightMode_U.Pose[0]));
            _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(tmp_3,
              _mm_set1_pd(rtb_DotProduct)), tmp_1), tmp_0));
            t = tmp[1];
            distToCenter = sqrt(UAM_FlightMode_DW.obj_l.LookaheadDistance *
                                UAM_FlightMode_DW.obj_l.LookaheadDistance -
                                rtb_DotProduct * rtb_DotProduct);

            /* Start for MATLABSystem: '<S10>/UAV Orbit Follower' incorporates:
             *  Inport: '<Root>/Pose'
             *  Inport: '<Root>/ToWP'
             */
            rtb_DotProduct_tmp = UAM_FlightMode_U.ToWP.position[1] -
              UAM_FlightMode_U.Pose[1];
            b_tmp[0] = tmp[0] - rtb_DotProduct_tmp * distToCenter / absxk;
            _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd
              (_mm_set_pd(UAM_FlightMode_U.ToWP.position[0],
                          UAM_FlightMode_U.ToWP.position[1]), _mm_set_pd
               (UAM_FlightMode_U.Pose[0], UAM_FlightMode_U.Pose[1])),
              _mm_set1_pd(distToCenter)), tmp_1), _mm_set_pd(tmp[1], tmp[0])));
            b_tmp[1] = tmp[0];
            rtb_DotProduct = tmp[1];
            absxk = t - (UAM_FlightMode_U.ToWP.position[0] -
                         UAM_FlightMode_U.Pose[0]) * distToCenter / absxk;

            /* Start for MATLABSystem: '<S10>/UAV Orbit Follower' incorporates:
             *  Inport: '<Root>/Pose'
             *  Inport: '<Root>/ToWP'
             */
            v[2] = 0.0;
            if (turnInternal < 0.0) {
              rtb_Product[0] = v[0];
              rtb_Product[1] = v[1];
              rtb_Product[2] = 0.0;
              tmp_1 = _mm_sub_pd(_mm_loadu_pd
                                 (&UAM_FlightMode_DW.obj_l.PrevPosition[0]),
                                 tmp_2);
              _mm_storeu_pd(&v[0], tmp_1);
              v[2] = 0.0;
            }

            distToCenter = UAM_FlightMode_norm_de(rtb_Product);
            t = UAM_FlightMode_norm_de(v);
            tmp_1 = _mm_set_pd(t, distToCenter);
            _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(v[0], rtb_Product[0]),
              tmp_1));
            rtb_Product[0] = tmp[0];
            v[0] = tmp[1];
            _mm_storeu_pd(&tmp[0], _mm_div_pd(_mm_set_pd(v[1], rtb_Product[1]),
              tmp_1));
            v[1] = tmp[1];
            turnVector[2] = rtb_Product[0] * tmp[1] - v[0] * tmp[0];
            UAM_FlightMode_DW.obj_l.PrevPosition[0] = UAM_FlightMode_U.Pose[0];
            UAM_FlightMode_DW.obj_l.PrevPosition[1] = UAM_FlightMode_U.Pose[1];
            UAM_FlightMode_DW.obj_l.PrevPosition[2] = UAM_FlightMode_U.Pose[2];
            UAM_FlightMode_DW.obj_l.NumCircles += rt_atan2d_snf(turnVector[2],
              (rtb_Product[0] * v[0] + tmp[0] * tmp[1]) + 0.0 / distToCenter *
              (0.0 / t)) / 2.0 / 3.1415926535897931;
            t = UAM_FlightMode_DW.obj_l.NumCircles;
            _mm_storeu_pd(&v[0], tmp_3);

            /* Start for MATLABSystem: '<S10>/UAV Orbit Follower' incorporates:
             *  Inport: '<Root>/Pose'
             */
            if (rtIsNaN(turnInternal)) {
              distToCenter = (rtNaN);
            } else if (turnInternal < 0.0) {
              distToCenter = -1.0;
            } else {
              distToCenter = (turnInternal > 0.0);
            }

            switch ((int32_T)distToCenter) {
             case 1:
              if ((b_tmp[0] - UAM_FlightMode_U.Pose[0]) * v[1] - (rtb_DotProduct
                   - UAM_FlightMode_U.Pose[1]) * v[0] > 0.0) {
                turnInternal = b_tmp[0];
                absxk = rtb_DotProduct;
              } else {
                turnInternal = b_tmp[1];
              }
              break;

             case -1:
              if ((b_tmp[0] - UAM_FlightMode_U.Pose[0]) * v[1] - (rtb_DotProduct
                   - UAM_FlightMode_U.Pose[1]) * v[0] < 0.0) {
                turnInternal = b_tmp[0];
                absxk = rtb_DotProduct;
              } else {
                turnInternal = b_tmp[1];
              }
              break;

             default:
              if (fabs(UAM_FlightMode_angdiff(rt_atan2d_snf(rtb_DotProduct -
                     UAM_FlightMode_U.Pose[1], b_tmp[0] - UAM_FlightMode_U.Pose
                     [0]), UAM_FlightMode_U.Pose[3])) < fabs
                  (UAM_FlightMode_angdiff(rt_atan2d_snf(absxk -
                     UAM_FlightMode_U.Pose[1], b_tmp[1] - UAM_FlightMode_U.Pose
                     [0]), UAM_FlightMode_U.Pose[3]))) {
                turnInternal = b_tmp[0];
                absxk = rtb_DotProduct;
              } else {
                turnInternal = b_tmp[1];
              }

              if ((turnInternal - UAM_FlightMode_U.Pose[0]) * v[1] - (absxk -
                   UAM_FlightMode_U.Pose[1]) * v[0] > 0.0) {
                UAM_FlightMode_DW.obj_l.TurnDirectionInternal = 1.0;
              } else {
                UAM_FlightMode_DW.obj_l.TurnDirectionInternal = -1.0;
              }

              UAM_FlightMode_DW.obj_l.SelectTurnDirectionFlag = false;
              break;
            }

            /* BusCreator: '<S10>/Bus Creator' incorporates:
             *  Inport: '<Root>/Pose'
             *  Inport: '<Root>/ToWP'
             *  MATLABSystem: '<S10>/UAV Orbit Follower'
             *  Merge: '<S3>/ Merge 2'
             */
            UAM_FlightMode_B.InnerLoopCmds.YawCmd = rt_atan2d_snf
              (rtb_DotProduct_tmp, UAM_FlightMode_U.ToWP.position[0] -
               UAM_FlightMode_U.Pose[0]);
          }
        }

        if (guard2) {
          _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_mul_pd(_mm_div_pd(_mm_set_pd
            (absxk, b_tmp[0]), _mm_set1_pd(UAM_FlightMode_norm_d(b_tmp))),
            _mm_set1_pd(t)), tmp_2));
          turnInternal = tmp[0];
          absxk = tmp[1];

          /* BusCreator: '<S10>/Bus Creator' incorporates:
           *  Inport: '<Root>/Pose'
           *  MATLABSystem: '<S10>/UAV Orbit Follower'
           *  Merge: '<S3>/ Merge 2'
           */
          UAM_FlightMode_B.InnerLoopCmds.YawCmd = rt_atan2d_snf(tmp[1] -
            UAM_FlightMode_U.Pose[1], tmp[0] - UAM_FlightMode_U.Pose[0]);
          t = UAM_FlightMode_DW.obj_l.NumCircles;
        }

        /* BusCreator: '<S10>/Bus Creator' incorporates:
         *  Inport: '<Root>/Pose'
         *  Inport: '<Root>/ToWP'
         *  MATLABSystem: '<S10>/UAV Orbit Follower'
         *  Merge: '<S3>/ Merge 2'
         */
        UAM_FlightMode_B.InnerLoopCmds.LAP[0] = turnInternal;
        UAM_FlightMode_B.InnerLoopCmds.LAP[1] = absxk;
        UAM_FlightMode_B.InnerLoopCmds.LAP[2] = UAM_FlightMode_U.ToWP.position[2];
        UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = rt_atan2d_snf(absxk -
          UAM_FlightMode_U.Pose[1], turnInternal - UAM_FlightMode_U.Pose[0]);
      }

      /* Merge: '<S3>/ Merge ' incorporates:
       *  DataTypeConversion: '<S10>/Data Type Conversion'
       *  Inport: '<Root>/ToWP'
       *  MATLABSystem: '<S10>/UAV Orbit Follower'
       *  RelationalOperator: '<S10>/Relational Operator'
       * */
      UAM_FlightMode_B.Status = (uint8_T)(t > UAM_FlightMode_U.ToWP.params[2]);
      break;

     case UAM_FlightMode_IN_PreTransition:
      if (UAM_FlightMode_DW.temporalCounter_i1 >= 63) {
        /* Outport: '<Root>/FlightMode' */
        UAM_FlightMode_Y.FlightMode = BackTransition;

        /* Outport: '<Root>/controlMode' */
        UAM_FlightMode_Y.controlMode_m.inTransition = 1U;
        UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMod_IN_BackTransition;

        /* BusCreator: '<S4>/Bus Creator1' incorporates:
         *  Constant: '<S4>/Constant1'
         *  Constant: '<S4>/Constant2'
         *  Merge: '<S3>/ Merge 2'
         */
        UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = 0.0;
        UAM_FlightMode_B.InnerLoopCmds.YawCmd = 0.0;

        /* MATLAB Function: '<S4>/MATLAB Function' */
        rtb_DotProduct = 3.3121686421112381E-170;

        /* BusCreator: '<S4>/Bus Creator1' incorporates:
         *  Inport: '<Root>/States'
         *  Merge: '<S3>/ Merge 2'
         */
        UAM_FlightMode_B.InnerLoopCmds.LAP[0] = UAM_FlightMode_U.States.Xe[0];

        /* MATLAB Function: '<S4>/MATLAB Function' incorporates:
         *  Inport: '<Root>/States'
         */
        absxk = fabs(UAM_FlightMode_U.States.Ve[0]);
        if (absxk > 3.3121686421112381E-170) {
          turnInternal = 1.0;
          rtb_DotProduct = absxk;
        } else {
          t = absxk / 3.3121686421112381E-170;
          turnInternal = t * t;
        }

        /* BusCreator: '<S4>/Bus Creator1' incorporates:
         *  Inport: '<Root>/States'
         *  Merge: '<S3>/ Merge 2'
         */
        UAM_FlightMode_B.InnerLoopCmds.LAP[1] = UAM_FlightMode_U.States.Xe[1];
        UAM_FlightMode_B.InnerLoopCmds.LAP[2] = UAM_FlightMode_U.States.Xe[2];

        /* Merge: '<S3>/ Merge ' incorporates:
         *  DataTypeConversion: '<S4>/Data Type Conversion'
         *  Inport: '<Root>/States'
         *  MATLAB Function: '<S4>/MATLAB Function'
         */
        UAM_FlightMode_B.Status = (uint8_T)((fabs(UAM_FlightMode_U.States.c1) <
          0.1) && (rtb_DotProduct * sqrt(turnInternal) < 4.0));
      }
      break;

     case UAM_FlightMode_IN_Start:
      UAM_FlightMode_Start(rtb_wps);
      break;

     case UAM_FlightMode_IN_Takeoff:
      /* Inport: '<Root>/Pose' incorporates:
       *  Inport: '<Root>/ToWP'
       */
      if (UAM_FlightMode_U.Pose[2] <= UAM_FlightMode_U.ToWP.position[2]) {
        UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMode_IN_WP;

        /* Switch: '<S17>/Switch' incorporates:
         *  Inport: '<Root>/FromWP'
         *  Product: '<S21>/Product'
         *  RelationalOperator: '<S17>/Equal'
         */
        if (UAM_FlightMode_U.FromWP.mode == 6) {
          rtb_Product[0] = UAM_FlightMode_U.FromWP.position[0];
          rtb_Product[1] = UAM_FlightMode_U.FromWP.position[1];
          rtb_Product[2] = UAM_FlightMode_U.FromWP.position[2];
        } else {
          rtb_Product[0] = 0.0;
          rtb_Product[1] = 0.0;
          rtb_Product[2] = 0.0;
        }

        /* MATLAB Function: '<S12>/MATLAB Function' incorporates:
         *  BusCreator generated from: '<S12>/MATLAB Function'
         *  Inport: '<Root>/FromWP'
         *  Product: '<S21>/Product'
         */
        rtb_wps[0] = rtb_Product[0];
        rtb_wps[2] = rtb_Product[1];
        rtb_wps[4] = rtb_Product[2];
        rtb_wps[6] = UAM_FlightMode_U.FromWP.params[3];
        rtb_wps[1] = UAM_FlightMode_U.ToWP.position[0];
        rtb_wps[3] = UAM_FlightMode_U.ToWP.position[1];
        rtb_wps[5] = UAM_FlightMode_U.ToWP.position[2];
        rtb_wps[7] = UAM_FlightMode_U.ToWP.params[3];

        /* BusCreator: '<S12>/Bus Creator' incorporates:
         *  Constant: '<S12>/Constant'
         *  Constant: '<S12>/Lookahead Distance'
         *  Inport: '<Root>/Pose'
         *  MATLABSystem: '<S12>/Waypoint Follower'
         *  Merge: '<S3>/ Merge 2'
         * */
        UAM_FlightMode_SystemCore_step(&UAM_FlightMode_DW.obj,
          UAM_FlightMode_U.Pose, rtb_wps, 5.0,
          UAM_FlightMode_B.InnerLoopCmds.LAP,
          &UAM_FlightMode_B.InnerLoopCmds.HeadingCmd, &turnInternal,
          &b_varargout_4);
        UAM_FlightMode_B.InnerLoopCmds.YawCmd = 0.0;

        /* Sum: '<S18>/Sum1' incorporates:
         *  Product: '<S21>/Product'
         */
        turnInternal = UAM_FlightMode_U.ToWP.position[0] - rtb_Product[0];
        rtb_Product[0] = turnInternal;

        /* DotProduct: '<S21>/Dot Product1' incorporates:
         *  Product: '<S21>/Product'
         */
        rtb_DotProduct = turnInternal * turnInternal;

        /* Sum: '<S18>/Sum1' incorporates:
         *  Product: '<S21>/Product'
         */
        turnInternal = UAM_FlightMode_U.ToWP.position[1] - rtb_Product[1];
        rtb_Product[1] = turnInternal;

        /* DotProduct: '<S21>/Dot Product1' incorporates:
         *  Product: '<S21>/Product'
         */
        rtb_DotProduct += turnInternal * turnInternal;

        /* Sum: '<S18>/Sum1' incorporates:
         *  Product: '<S21>/Product'
         */
        turnInternal = UAM_FlightMode_U.ToWP.position[2] - rtb_Product[2];

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

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S12>/Lookahead Distance'
         *  DataTypeConversion: '<S12>/Data Type Conversion'
         *  DotProduct: '<S18>/Dot Product'
         *  Product: '<S21>/Product'
         *  RelationalOperator: '<S12>/Relational Operator'
         *  Sum: '<S18>/Sum'
         *  Sum: '<S18>/Sum1'
         */
        UAM_FlightMode_B.Status = (uint8_T)(((UAM_FlightMode_U.ToWP.position[0]
          - UAM_FlightMode_U.Pose[0]) * (rtb_Product[0] / rtb_DotProduct) +
          (UAM_FlightMode_U.ToWP.position[1] - UAM_FlightMode_U.Pose[1]) *
          (rtb_Product[1] / rtb_DotProduct)) + (UAM_FlightMode_U.ToWP.position[2]
          - UAM_FlightMode_U.Pose[2]) * (turnInternal / rtb_DotProduct) <= 5.0);
      } else {
        /* BusCreator: '<S11>/Bus Creator' incorporates:
         *  Merge: '<S3>/ Merge 2'
         *  SignalConversion generated from: '<S11>/Bus Creator'
         */
        UAM_FlightMode_B.InnerLoopCmds.LAP[0] = UAM_FlightMode_U.Pose[0];
        UAM_FlightMode_B.InnerLoopCmds.LAP[1] = UAM_FlightMode_U.Pose[1];
        UAM_FlightMode_B.InnerLoopCmds.LAP[2] = UAM_FlightMode_U.ToWP.position[2];

        /* Saturate: '<S11>/Hdg. Cmd Sat' */
        if (UAM_FlightMode_U.ToWP.params[3] > 3.1415926535897931) {
          /* BusCreator: '<S11>/Bus Creator' incorporates:
           *  Merge: '<S3>/ Merge 2'
           */
          UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = 3.1415926535897931;
        } else if (UAM_FlightMode_U.ToWP.params[3] < -3.1415926535897931) {
          /* BusCreator: '<S11>/Bus Creator' incorporates:
           *  Merge: '<S3>/ Merge 2'
           */
          UAM_FlightMode_B.InnerLoopCmds.HeadingCmd = -3.1415926535897931;
        } else {
          /* BusCreator: '<S11>/Bus Creator' incorporates:
           *  Merge: '<S3>/ Merge 2'
           */
          UAM_FlightMode_B.InnerLoopCmds.HeadingCmd =
            UAM_FlightMode_U.ToWP.params[3];
        }

        /* BusCreator: '<S11>/Bus Creator' incorporates:
         *  Merge: '<S3>/ Merge 2'
         */
        UAM_FlightMode_B.InnerLoopCmds.YawCmd = UAM_FlightMode_ConstB.YawCmd;

        /* Merge: '<S3>/ Merge ' incorporates:
         *  Constant: '<S11>/Constant'
         *  SignalConversion generated from: '<S11>/Status'
         */
        UAM_FlightMode_B.Status = 0U;
      }
      break;

     default:
      /* Switch: '<S17>/Switch' incorporates:
       *  Inport: '<Root>/FromWP'
       *  Product: '<S21>/Product'
       *  RelationalOperator: '<S17>/Equal'
       */
      /* case IN_WP: */
      if (UAM_FlightMode_U.FromWP.mode == 6) {
        rtb_Product[0] = UAM_FlightMode_U.FromWP.position[0];
        rtb_Product[1] = UAM_FlightMode_U.FromWP.position[1];
        rtb_Product[2] = UAM_FlightMode_U.FromWP.position[2];
      } else {
        rtb_Product[0] = 0.0;
        rtb_Product[1] = 0.0;
        rtb_Product[2] = 0.0;
      }

      /* MATLAB Function: '<S12>/MATLAB Function' incorporates:
       *  BusCreator generated from: '<S12>/MATLAB Function'
       *  Inport: '<Root>/FromWP'
       *  Inport: '<Root>/ToWP'
       *  Product: '<S21>/Product'
       */
      rtb_wps[0] = rtb_Product[0];
      rtb_wps[2] = rtb_Product[1];
      rtb_wps[4] = rtb_Product[2];
      rtb_wps[6] = UAM_FlightMode_U.FromWP.params[3];
      rtb_wps[1] = UAM_FlightMode_U.ToWP.position[0];
      rtb_wps[3] = UAM_FlightMode_U.ToWP.position[1];
      rtb_wps[5] = UAM_FlightMode_U.ToWP.position[2];
      rtb_wps[7] = UAM_FlightMode_U.ToWP.params[3];

      /* BusCreator: '<S12>/Bus Creator' incorporates:
       *  Constant: '<S12>/Constant'
       *  Constant: '<S12>/Lookahead Distance'
       *  Inport: '<Root>/Pose'
       *  MATLABSystem: '<S12>/Waypoint Follower'
       *  Merge: '<S3>/ Merge 2'
       * */
      UAM_FlightMode_SystemCore_step(&UAM_FlightMode_DW.obj,
        UAM_FlightMode_U.Pose, rtb_wps, 5.0, UAM_FlightMode_B.InnerLoopCmds.LAP,
        &UAM_FlightMode_B.InnerLoopCmds.HeadingCmd, &turnInternal,
        &b_varargout_4);
      UAM_FlightMode_B.InnerLoopCmds.YawCmd = 0.0;

      /* Sum: '<S18>/Sum1' incorporates:
       *  Inport: '<Root>/ToWP'
       *  Product: '<S21>/Product'
       */
      turnInternal = UAM_FlightMode_U.ToWP.position[0] - rtb_Product[0];
      rtb_Product[0] = turnInternal;

      /* DotProduct: '<S21>/Dot Product1' incorporates:
       *  Product: '<S21>/Product'
       */
      rtb_DotProduct = turnInternal * turnInternal;

      /* Sum: '<S18>/Sum1' incorporates:
       *  Inport: '<Root>/ToWP'
       *  Product: '<S21>/Product'
       */
      turnInternal = UAM_FlightMode_U.ToWP.position[1] - rtb_Product[1];
      rtb_Product[1] = turnInternal;

      /* DotProduct: '<S21>/Dot Product1' incorporates:
       *  Product: '<S21>/Product'
       */
      rtb_DotProduct += turnInternal * turnInternal;

      /* Sum: '<S18>/Sum1' incorporates:
       *  Inport: '<Root>/ToWP'
       *  Product: '<S21>/Product'
       */
      turnInternal = UAM_FlightMode_U.ToWP.position[2] - rtb_Product[2];

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

      /* Merge: '<S3>/ Merge ' incorporates:
       *  Constant: '<S12>/Lookahead Distance'
       *  DataTypeConversion: '<S12>/Data Type Conversion'
       *  DotProduct: '<S18>/Dot Product'
       *  Inport: '<Root>/Pose'
       *  Inport: '<Root>/ToWP'
       *  Product: '<S21>/Product'
       *  RelationalOperator: '<S12>/Relational Operator'
       *  Sum: '<S18>/Sum'
       *  Sum: '<S18>/Sum1'
       */
      UAM_FlightMode_B.Status = (uint8_T)(((UAM_FlightMode_U.ToWP.position[0] -
        UAM_FlightMode_U.Pose[0]) * (rtb_Product[0] / rtb_DotProduct) +
        (UAM_FlightMode_U.ToWP.position[1] - UAM_FlightMode_U.Pose[1]) *
        (rtb_Product[1] / rtb_DotProduct)) + (UAM_FlightMode_U.ToWP.position[2]
        - UAM_FlightMode_U.Pose[2]) * (turnInternal / rtb_DotProduct) <= 5.0);
      break;
    }
  }
}

/* Model step function */
void UAM_FlightMode_step(void)
{
  uint8_T mode_prev;

  /* Chart: '<Root>/Guidance Mode Selector' incorporates:
   *  Inport: '<Root>/mode'
   *  Merge: '<S3>/ Merge 2'
   *  Outport: '<Root>/FixedWingSP'
   *  Outport: '<Root>/controlMode'
   */
  if (UAM_FlightMode_DW.temporalCounter_i1 < 63) {
    UAM_FlightMode_DW.temporalCounter_i1++;
  }

  mode_prev = UAM_FlightMode_DW.mode_start;
  UAM_FlightMode_DW.mode_start = UAM_FlightMode_U.mode;
  if (UAM_FlightMode_DW.is_active_c14_UAM_FlightMode == 0) {
    UAM_FlightMode_DW.is_active_c14_UAM_FlightMode = 1U;
    UAM_FlightMode_DW.is_GuidanceLogic = UAM_FlightMode_IN_Start;

    /* Outport: '<Root>/FlightMode' incorporates:
     *  Constant: '<Root>/Constant2'
     */
    UAM_FlightMode_Y.FlightMode = Hover;
    UAM_FlightMode_Y.controlMode_m.airspeedAltitude = 0U;
    UAM_FlightMode_Y.controlMode_m.attitude = 0U;
    UAM_FlightMode_Y.controlMode_m.lateralGuidance = 0U;
    UAM_FlightMode_Y.controlMode_m.TransitionCondition = 0U;
    UAM_FlightMode_Y.FixedWingSP.yaw = 0.0;
    UAM_FlightMode_Y.FixedWingSP.pitch = 0.0;
    UAM_FlightMode_Y.FixedWingSP.roll = 0.0;
    UAM_FlightMode_Y.FixedWingSP.airspeed = 14.0;
    UAM_FlightMode_B.aacSP.L1 = 25.0;
    UAM_FlightMode_B.aacSP.airspeed = 14.0;
    UAM_FlightMode_B.aacSP.altitude = 0.0;
    UAM_FlightMode_B.aacSP.course = 0.0;
    UAM_FlightMode_B.InnerLoopCmds.YawCmd = 0.0;
  } else {
    UAM_FlightMode_GuidanceLogic(&mode_prev);
  }

  /* End of Chart: '<Root>/Guidance Mode Selector' */

  /* Stop: '<Root>/Stop Simulation' incorporates:
   *  Constant: '<S1>/Constant'
   *  RelationalOperator: '<S1>/Compare'
   */
  if (UAM_FlightMode_B.Status == 2) {
    rtmSetStopRequested(UAM_FlightMode_M, 1);
  }

  /* End of Stop: '<Root>/Stop Simulation' */

  /* BusCreator generated from: '<Root>/HoverSP' incorporates:
   *  Outport: '<Root>/HoverSP'
   */
  UAM_FlightMode_Y.HoverSP.X = UAM_FlightMode_B.InnerLoopCmds.LAP[0];
  UAM_FlightMode_Y.HoverSP.Y = UAM_FlightMode_B.InnerLoopCmds.LAP[1];
  UAM_FlightMode_Y.HoverSP.Z = UAM_FlightMode_B.InnerLoopCmds.LAP[2];
  UAM_FlightMode_Y.HoverSP.Yaw = UAM_FlightMode_B.InnerLoopCmds.YawCmd;

  /* Outport: '<Root>/aacSP' */
  UAM_FlightMode_Y.aacSP = UAM_FlightMode_B.aacSP;

  /* Outport: '<Root>/Status' incorporates:
   *  Constant: '<S2>/Constant'
   *  RelationalOperator: '<S2>/Compare'
   */
  UAM_FlightMode_Y.Status = (uint8_T)(UAM_FlightMode_B.Status == 1);
}

/* Model initialize function */
void UAM_FlightMode_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    int32_T i;
    UAM_FlightMode_PrevZCX.TriggeredSubsystem_Trig_ZCE = POS_ZCSIG;

    /* SystemInitialize for IfAction SubSystem: '<S3>/GuidanceLogic.FIXED_WING_WAYPOINT.WAYPOINT' */
    /* Start for MATLABSystem: '<S6>/Waypoint Follower' incorporates:
     *  Chart: '<Root>/Guidance Mode Selector'
     */
    UAM_FlightMode_DW.obj_f.LastWaypointFlag = false;
    UAM_FlightMode_DW.obj_f.StartFlag = true;
    UAM_FlightMode_DW.obj_f.LookaheadFactor = 1.01;
    UAM_FlightMode_DW.obj_f.isInitialized = 1;
    UAM_FlightMode_DW.obj_f.NumWaypoints = 0.0;

    /* InitializeConditions for MATLABSystem: '<S6>/Waypoint Follower' incorporates:
     *  Chart: '<Root>/Guidance Mode Selector'
     */
    UAM_FlightMode_DW.obj_f.WaypointIndex = 1.0;
    for (i = 0; i < 6; i++) {
      /* InitializeConditions for MATLABSystem: '<S6>/Waypoint Follower' incorporates:
       *  Chart: '<Root>/Guidance Mode Selector'
       */
      UAM_FlightMode_DW.obj_f.WaypointsInternal[i] = 0.0;
    }

    /* End of SystemInitialize for SubSystem: '<S3>/GuidanceLogic.FIXED_WING_WAYPOINT.WAYPOINT' */

    /* SystemInitialize for IfAction SubSystem: '<S3>/GuidanceLogic.FIXED_WING_ORBIT.ORBIT' */
    /* Start for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
     *  Chart: '<Root>/Guidance Mode Selector'
     */
    UAM_FlightMode_DW.obj_b.isInitialized = 1;

    /* InitializeConditions for MATLABSystem: '<S5>/UAV Orbit Follower' incorporates:
     *  Chart: '<Root>/Guidance Mode Selector'
     */
    UAM_FlightMode_DW.obj_b.OrbitRadiusInternal = 0.0;
    UAM_FlightMode_DW.obj_b.PrevResetSignal = 0.0;
    UAM_FlightMode_DW.obj_b.NumCircles = 0.0;
    UAM_FlightMode_DW.obj_b.OrbitCenterInternal[0] = 0.0;
    UAM_FlightMode_DW.obj_b.PrevPosition[0] = 0.0;
    UAM_FlightMode_DW.obj_b.OrbitCenterInternal[1] = 0.0;
    UAM_FlightMode_DW.obj_b.PrevPosition[1] = 0.0;
    UAM_FlightMode_DW.obj_b.OrbitCenterInternal[2] = 0.0;
    UAM_FlightMode_DW.obj_b.PrevPosition[2] = 0.0;
    UAM_FlightMode_DW.obj_b.StartFlag = true;
    UAM_FlightMode_DW.obj_b.SelectTurnDirectionFlag = true;
    UAM_FlightMode_DW.obj_b.TurnDirectionInternal = 1.0;
    UAM_FlightMode_DW.obj_b.OrbitRadiusFlag = 0U;
    UAM_FlightMode_DW.obj_b.LookaheadDistFlag = 0U;

    /* End of SystemInitialize for SubSystem: '<S3>/GuidanceLogic.FIXED_WING_ORBIT.ORBIT' */

    /* SystemInitialize for IfAction SubSystem: '<S3>/GuidanceLogic.WP' */
    /* Start for MATLABSystem: '<S12>/Waypoint Follower' incorporates:
     *  Chart: '<Root>/Guidance Mode Selector'
     */
    UAM_FlightMode_DW.obj.LastWaypointFlag = false;
    UAM_FlightMode_DW.obj.StartFlag = true;
    UAM_FlightMode_DW.obj.LookaheadFactor = 1.01;
    UAM_FlightMode_DW.obj.SearchFlag = true;
    UAM_FlightMode_DW.obj.isInitialized = 1;
    UAM_FlightMode_DW.obj.NumWaypoints = 0.0;

    /* InitializeConditions for MATLABSystem: '<S12>/Waypoint Follower' incorporates:
     *  Chart: '<Root>/Guidance Mode Selector'
     */
    UAM_FlightMode_DW.obj.WaypointIndex = 1.0;
    memset(&UAM_FlightMode_DW.obj.WaypointsInternal[0], 0, sizeof(real_T) << 3U);

    /* End of SystemInitialize for SubSystem: '<S3>/GuidanceLogic.WP' */

    /* SystemInitialize for IfAction SubSystem: '<S3>/GuidanceLogic.Orbit' */
    /* Start for MATLABSystem: '<S10>/UAV Orbit Follower' incorporates:
     *  Chart: '<Root>/Guidance Mode Selector'
     */
    UAM_FlightMode_DW.obj_l.isInitialized = 1;

    /* InitializeConditions for MATLABSystem: '<S10>/UAV Orbit Follower' incorporates:
     *  Chart: '<Root>/Guidance Mode Selector'
     */
    UAM_FlightMode_DW.obj_l.OrbitRadiusInternal = 0.0;
    UAM_FlightMode_DW.obj_l.PrevResetSignal = 0.0;
    UAM_FlightMode_DW.obj_l.NumCircles = 0.0;
    UAM_FlightMode_DW.obj_l.OrbitCenterInternal[0] = 0.0;
    UAM_FlightMode_DW.obj_l.PrevPosition[0] = 0.0;
    UAM_FlightMode_DW.obj_l.OrbitCenterInternal[1] = 0.0;
    UAM_FlightMode_DW.obj_l.PrevPosition[1] = 0.0;
    UAM_FlightMode_DW.obj_l.OrbitCenterInternal[2] = 0.0;
    UAM_FlightMode_DW.obj_l.PrevPosition[2] = 0.0;
    UAM_FlightMode_DW.obj_l.StartFlag = true;
    UAM_FlightMode_DW.obj_l.SelectTurnDirectionFlag = true;
    UAM_FlightMode_DW.obj_l.TurnDirectionInternal = 1.0;
    UAM_FlightMode_DW.obj_l.OrbitRadiusFlag = 0U;
    UAM_FlightMode_DW.obj_l.LookaheadDistFlag = 0U;

    /* End of SystemInitialize for SubSystem: '<S3>/GuidanceLogic.Orbit' */

    /* SystemInitialize for IfAction SubSystem: '<S3>/GuidanceLogic.Land.Descend' */
    /* SystemInitialize for Chart: '<Root>/Guidance Mode Selector' */
    UAM_Fligh_WaypointFollower_Init(&UAM_FlightMode_DW.WaypointFollower_c);

    /* End of SystemInitialize for SubSystem: '<S3>/GuidanceLogic.Land.Descend' */

    /* SystemInitialize for IfAction SubSystem: '<S3>/GuidanceLogic.Land.ToLand' */
    UAM_Fligh_WaypointFollower_Init(&UAM_FlightMode_DW.WaypointFollower_c2);

    /* End of SystemInitialize for SubSystem: '<S3>/GuidanceLogic.Land.ToLand' */
  }
}

/* Model terminate function */
void UAM_FlightMode_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
