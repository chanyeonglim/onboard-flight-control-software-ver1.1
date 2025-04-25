/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_GuidanceAndNavigation.c
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

#include "UAM_GuidanceAndNavigation.h"
#include "rtwtypes.h"
#include "UAM_GuidanceAndNavigation_types.h"
#include <math.h>
#include "rt_modd_snf.h"
#include "rt_atan2d_snf.h"
#include <emmintrin.h>
#include "UAM_GuidanceAndNavigation_private.h"
#include <string.h>
#include "rt_nonfinite.h"

/* Forward declaration for local functions */
static void UAM_Guida_PathManager_setupImpl(uav_sluav_internal_system_Pat_T *obj);
static void UAM_Gu_AHRSFilterBase_resetImpl(fusion_simulink_ahrsfilter_UA_T *obj);
static void UAM_GuidanceAndNav_NED_ecompass(const real_T a[3], const real_T m[3],
  real_T R[9]);
static void UAM_Guida_quaternion_quaternion(const real_T varargin_1[3], real_T
  *obj_a, real_T *obj_b, real_T *obj_c, real_T *obj_d);
static void UAM_Guida_quaternionBase_rotmat(real_T q_a, real_T q_b, real_T q_c,
  real_T q_d, real_T r[9]);
static void UAM_GuidanceAndNavigati_mrdiv_b(real_T A[72], const real_T B[36]);
static void UAM_GuidanceAnd_SystemCore_step(fusion_simulink_ahrsfilter_UA_T *obj,
  const real_T varargin_1[3], const real_T varargin_2[3], const real_T
  varargin_3[3], real_T varargout_1[4], real_T varargout_2[3]);

/*
 * Output and update for action system:
 *    '<S18>/If Action Subsystem'
 *    '<S27>/If Action Subsystem'
 */
void UAM_GuidanceA_IfActionSubsystem(real_T rtu_u, real_T rtu_absu, real_T
  *rty_signu)
{
  /* Product: '<S20>/Divide' */
  *rty_signu = rtu_u / rtu_absu;
}

/*
 * Output and update for action system:
 *    '<S18>/If Action Subsystem1'
 *    '<S27>/If Action Subsystem1'
 */
void UAM_Guidance_IfActionSubsystem1(real_T *rty_zero)
{
  /* SignalConversion generated from: '<S21>/zero' incorporates:
   *  Constant: '<S21>/Constant'
   */
  *rty_zero = 0.0;
}

static void UAM_Guida_PathManager_setupImpl(uav_sluav_internal_system_Pat_T *obj)
{
  int32_T c_j;

  /* Start for MATLABSystem: '<S6>/PathManagerSystemObject' */
  obj->MissionStart = true;
  obj->MissionParams[0].mode = 0U;
  obj->MissionParams[1].mode = 1U;
  for (c_j = 0; c_j < 2; c_j++) {
    obj->MissionParams[c_j].position[0] = 0.0;
    obj->MissionParams[c_j].position[1] = 0.0;
    obj->MissionParams[c_j].position[2] = 0.0;
    obj->MissionParams[c_j].params[0] = 0.0;
    obj->MissionParams[c_j].params[1] = 0.0;
    obj->MissionParams[c_j].params[2] = 0.0;
    obj->MissionParams[c_j].params[3] = 0.0;
  }

  /* Start for MATLABSystem: '<S6>/PathManagerSystemObject' */
  obj->PrevMissionPoint.mode = 0U;
  obj->PrevMissionPoint.position[0] = 0.0;
  obj->PrevMissionPoint.position[1] = 0.0;
  obj->PrevMissionPoint.position[2] = 0.0;
  obj->PrevMissionPoint.params[0] = 0.0;
  obj->HoldPose[0] = 0.0;
  obj->PrevMissionPoint.params[1] = 0.0;
  obj->HoldPose[1] = 0.0;
  obj->PrevMissionPoint.params[2] = 0.0;
  obj->HoldPose[2] = 0.0;
  obj->PrevMissionPoint.params[3] = 0.0;
  obj->HoldPose[3] = 0.0;
}

static void UAM_Gu_AHRSFilterBase_resetImpl(fusion_simulink_ahrsfilter_UA_T *obj)
{
  __m128d tmp_1;
  real_T tmp[2];
  real_T accelMeasNoiseVar;
  real_T magMeasNoiseVar;
  int32_T i;
  int32_T tmp_0;
  int32_T tmp_2;
  int8_T b[9];
  static const int8_T tmp_3[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const real_T tmp_4[144] = { 6.0923483957341713E-6, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0923483957341713E-6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0923483957341713E-6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.6154354946677142E-5, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.6154354946677142E-5,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    7.6154354946677142E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0096236100000000012, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0096236100000000012, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0096236100000000012, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.6 };

  obj->pOrientPost.a = 1.0;
  obj->pOrientPost.b = 0.0;
  obj->pOrientPost.c = 0.0;
  obj->pOrientPost.d = 0.0;
  obj->pGyroOffset[0] = 0.0;
  obj->pMagVec[0] = 0.0;
  obj->pGyroOffset[1] = 0.0;
  obj->pMagVec[1] = 0.0;
  obj->pGyroOffset[2] = 0.0;
  obj->pMagVec[2] = 0.0;
  obj->pMagVec[0] = obj->ExpectedMagneticFieldStrength;

  /* Start for MATLABSystem: '<S2>/AHRS' */
  magMeasNoiseVar = obj->pKalmanPeriod * obj->pKalmanPeriod *
    (obj->GyroscopeDriftNoise + obj->GyroscopeNoise);
  accelMeasNoiseVar = (obj->AccelerometerNoise + obj->LinearAccelerationNoise) +
    magMeasNoiseVar;
  magMeasNoiseVar += obj->MagnetometerNoise + obj->MagneticDisturbanceNoise;
  for (i = 0; i < 9; i++) {
    b[i] = 0;
  }

  b[0] = 1;
  b[4] = 1;
  b[8] = 1;

  /* Start for MATLABSystem: '<S2>/AHRS' */
  memset(&obj->pQv[0], 0, 36U * sizeof(real_T));
  for (i = 0; i < 3; i++) {
    /* Start for MATLABSystem: '<S2>/AHRS' */
    tmp_1 = _mm_set_pd(magMeasNoiseVar, accelMeasNoiseVar);
    _mm_storeu_pd(&tmp[0], _mm_mul_pd(_mm_set_pd(tmp_3[3 * i], b[3 * i]), tmp_1));
    obj->pQv[6 * i] = tmp[0];
    tmp_2 = (i + 3) * 6;
    obj->pQv[tmp_2 + 3] = tmp[1];
    tmp_0 = 3 * i + 1;
    _mm_storeu_pd(&tmp[0], _mm_mul_pd(_mm_set_pd(tmp_3[tmp_0], b[tmp_0]), tmp_1));
    obj->pQv[6 * i + 1] = tmp[0];
    obj->pQv[tmp_2 + 4] = tmp[1];
    tmp_0 = 3 * i + 2;
    _mm_storeu_pd(&tmp[0], _mm_mul_pd(_mm_set_pd(tmp_3[tmp_0], b[tmp_0]), tmp_1));
    obj->pQv[6 * i + 2] = tmp[0];
    obj->pQv[tmp_2 + 5] = tmp[1];
  }

  memcpy(&obj->pQw[0], &tmp_4[0], 144U * sizeof(real_T));
  obj->pLinAccelPost[0] = 0.0;
  obj->pLinAccelPost[1] = 0.0;
  obj->pLinAccelPost[2] = 0.0;
  obj->pFirstTime = true;
  obj->pInclinationLimit = 1.5707963267948966;
}

static void UAM_GuidanceAndNav_NED_ecompass(const real_T a[3], const real_T m[3],
  real_T R[9])
{
  __m128d tmp;
  real_T x[9];
  real_T Reast[3];
  real_T R_0;
  int32_T i2;
  int32_T i2_0;
  int32_T ix;
  int32_T xpageoffset;
  boolean_T b[9];
  boolean_T y[3];
  boolean_T exitg1;
  boolean_T nanPageIdx;

  /* Start for MATLABSystem: '<S2>/AHRS' */
  _mm_storeu_pd(&Reast[0], _mm_sub_pd(_mm_mul_pd(_mm_set_pd(m[0], a[1]),
    _mm_set_pd(a[2], m[2])), _mm_mul_pd(_mm_set_pd(a[0], m[1]), _mm_set_pd(m[2],
    a[2]))));
  Reast[2] = a[0] * m[1] - m[0] * a[1];
  R[6] = a[0];
  R[3] = Reast[0];

  /* Start for MATLABSystem: '<S2>/AHRS' */
  R[7] = a[1];
  R[4] = Reast[1];

  /* Start for MATLABSystem: '<S2>/AHRS' */
  R[8] = a[2];
  R[5] = Reast[2];

  /* Start for MATLABSystem: '<S2>/AHRS' */
  _mm_storeu_pd(&R[0], _mm_sub_pd(_mm_mul_pd(_mm_set_pd(a[0], Reast[1]),
    _mm_set_pd(Reast[2], a[2])), _mm_mul_pd(_mm_set_pd(Reast[0], a[1]),
    _mm_set_pd(a[2], Reast[2]))));
  R[2] = Reast[0] * a[1] - a[0] * Reast[1];
  for (i2 = 0; i2 <= 6; i2 += 2) {
    tmp = _mm_loadu_pd(&R[i2]);
    _mm_storeu_pd(&x[i2], _mm_mul_pd(tmp, tmp));
  }

  for (i2 = 8; i2 < 9; i2++) {
    R_0 = R[i2];
    x[i2] = R_0 * R_0;
  }

  for (i2 = 0; i2 < 3; i2++) {
    xpageoffset = i2 * 3;

    /* Start for MATLABSystem: '<S2>/AHRS' */
    Reast[i2] = (x[xpageoffset + 1] + x[xpageoffset]) + x[xpageoffset + 2];
  }

  /* Start for MATLABSystem: '<S2>/AHRS' */
  Reast[0] = sqrt(Reast[0]);
  Reast[1] = sqrt(Reast[1]);
  Reast[2] = sqrt(Reast[2]);
  memcpy(&x[0], &R[0], 9U * sizeof(real_T));
  for (i2 = 0; i2 < 3; i2++) {
    /* Start for MATLABSystem: '<S2>/AHRS' */
    tmp = _mm_div_pd(_mm_loadu_pd(&x[3 * i2]), _mm_set1_pd(Reast[i2]));
    _mm_storeu_pd(&R[3 * i2], tmp);
    xpageoffset = 3 * i2 + 2;
    R[xpageoffset] = x[xpageoffset] / Reast[i2];
  }

  for (i2 = 0; i2 < 9; i2++) {
    /* Start for MATLABSystem: '<S2>/AHRS' */
    b[i2] = rtIsNaN(R[i2]);
  }

  y[0] = false;
  y[1] = false;
  y[2] = false;

  /* Start for MATLABSystem: '<S2>/AHRS' */
  i2 = 1;
  for (xpageoffset = 0; xpageoffset < 3; xpageoffset++) {
    /* Start for MATLABSystem: '<S2>/AHRS' */
    i2_0 = i2;
    ix = i2;

    /* Start for MATLABSystem: '<S2>/AHRS' */
    i2 += 3;
    exitg1 = false;
    while ((!exitg1) && (ix <= i2_0 + 2)) {
      if (b[ix - 1]) {
        y[xpageoffset] = true;
        exitg1 = true;
      } else {
        ix++;
      }
    }
  }

  nanPageIdx = false;
  i2 = 0;
  exitg1 = false;
  while ((!exitg1) && (i2 < 3)) {
    if (y[i2]) {
      nanPageIdx = true;
      exitg1 = true;
    } else {
      i2++;
    }
  }

  if (nanPageIdx) {
    memset(&R[0], 0, 9U * sizeof(real_T));
    R[0] = 1.0;
    R[4] = 1.0;
    R[8] = 1.0;
  }
}

static void UAM_Guida_quaternion_quaternion(const real_T varargin_1[3], real_T
  *obj_a, real_T *obj_b, real_T *obj_c, real_T *obj_d)
{
  real_T tmp[2];
  real_T st;
  real_T theta;
  *obj_a = 1.0;
  *obj_b = 0.0;
  *obj_c = 0.0;
  *obj_d = 0.0;

  /* Start for MATLABSystem: '<S2>/AHRS' */
  theta = sqrt((varargin_1[0] * varargin_1[0] + varargin_1[1] * varargin_1[1]) +
               varargin_1[2] * varargin_1[2]);
  st = sin(theta / 2.0);
  if (theta != 0.0) {
    /* Start for MATLABSystem: '<S2>/AHRS' */
    *obj_a = cos(theta / 2.0);
    _mm_storeu_pd(&tmp[0], _mm_mul_pd(_mm_div_pd(_mm_loadu_pd(&varargin_1[0]),
      _mm_set1_pd(theta)), _mm_set1_pd(st)));
    *obj_b = tmp[0];
    *obj_c = tmp[1];

    /* Start for MATLABSystem: '<S2>/AHRS' */
    *obj_d = varargin_1[2] / theta * st;
  }
}

static void UAM_Guida_quaternionBase_rotmat(real_T q_a, real_T q_b, real_T q_c,
  real_T q_d, real_T r[9])
{
  real_T aasq;
  real_T ac2;
  real_T ad2;
  real_T bc2;
  real_T bd2;
  real_T cd2;
  real_T n;

  /* Start for MATLABSystem: '<S2>/AHRS' */
  n = sqrt(((q_a * q_a + q_b * q_b) + q_c * q_c) + q_d * q_d);
  q_a /= n;
  q_b /= n;
  q_c /= n;
  q_d /= n;

  /* Start for MATLABSystem: '<S2>/AHRS' */
  n = q_a * q_b * 2.0;
  ac2 = q_a * q_c * 2.0;
  ad2 = q_a * q_d * 2.0;
  bc2 = q_b * q_c * 2.0;
  bd2 = q_b * q_d * 2.0;
  cd2 = q_c * q_d * 2.0;
  aasq = q_a * q_a * 2.0 - 1.0;
  r[0] = q_b * q_b * 2.0 + aasq;
  r[3] = bc2 + ad2;
  r[6] = bd2 - ac2;
  r[1] = bc2 - ad2;
  r[4] = q_c * q_c * 2.0 + aasq;
  r[7] = cd2 + n;
  r[2] = bd2 + ac2;
  r[5] = cd2 - n;
  r[8] = q_d * q_d * 2.0 + aasq;
}

static void UAM_GuidanceAndNavigati_mrdiv_b(real_T A[72], const real_T B[36])
{
  __m128d tmp;
  real_T c_A[36];
  real_T s;
  real_T smax;
  int32_T a;
  int32_T b_j;
  int32_T c;
  int32_T d;
  int32_T jA;
  int32_T jAcol;
  int32_T jBcol;
  int32_T jj;
  int32_T k;
  int8_T b_ipiv[6];

  /* Start for MATLABSystem: '<S2>/AHRS' */
  memcpy(&c_A[0], &B[0], 36U * sizeof(real_T));
  for (c = 0; c < 6; c++) {
    b_ipiv[c] = (int8_T)(c + 1);
  }

  for (b_j = 0; b_j < 5; b_j++) {
    /* Start for MATLABSystem: '<S2>/AHRS' */
    c = b_j * 7 + 2;
    jj = b_j * 7;
    jBcol = 6 - b_j;
    a = 1;

    /* Start for MATLABSystem: '<S2>/AHRS' */
    smax = fabs(c_A[jj]);
    for (k = 2; k <= jBcol; k++) {
      s = fabs(c_A[(c + k) - 3]);
      if (s > smax) {
        a = k;
        smax = s;
      }
    }

    if (c_A[(c + a) - 3] != 0.0) {
      if (a - 1 != 0) {
        a += b_j;
        b_ipiv[b_j] = (int8_T)a;
        for (k = 0; k < 6; k++) {
          jBcol = k * 6 + b_j;
          smax = c_A[jBcol];
          jAcol = (k * 6 + a) - 1;
          c_A[jBcol] = c_A[jAcol];
          c_A[jAcol] = smax;
        }
      }

      k = c - b_j;
      for (a = c; a <= k + 4; a++) {
        c_A[a - 1] /= c_A[jj];
      }
    }

    jA = jj;
    k = 5 - b_j;
    for (a = 0; a < k; a++) {
      /* Start for MATLABSystem: '<S2>/AHRS' */
      smax = c_A[(a * 6 + jj) + 6];
      if (smax != 0.0) {
        /* Start for MATLABSystem: '<S2>/AHRS' */
        jBcol = jA + 8;
        d = (jA - b_j) + 12;
        for (jAcol = jBcol; jAcol <= d; jAcol++) {
          /* Start for MATLABSystem: '<S2>/AHRS' */
          c_A[jAcol - 1] += c_A[((c + jAcol) - jA) - 9] * -smax;
        }
      }

      /* Start for MATLABSystem: '<S2>/AHRS' */
      jA += 6;
    }
  }

  for (b_j = 0; b_j < 6; b_j++) {
    jBcol = 12 * b_j - 1;

    /* Start for MATLABSystem: '<S2>/AHRS' */
    jAcol = 6 * b_j - 1;
    for (a = 0; a < b_j; a++) {
      jA = 12 * a - 1;

      /* Start for MATLABSystem: '<S2>/AHRS' */
      smax = c_A[(a + jAcol) + 1];
      if (smax != 0.0) {
        for (jj = 0; jj < 12; jj++) {
          c = (jj + jBcol) + 1;
          A[c] -= A[(jj + jA) + 1] * smax;
        }
      }
    }

    /* Start for MATLABSystem: '<S2>/AHRS' */
    smax = 1.0 / c_A[(b_j + jAcol) + 1];
    for (a = 0; a <= 10; a += 2) {
      c = (a + jBcol) + 1;
      tmp = _mm_loadu_pd(&A[c]);
      _mm_storeu_pd(&A[c], _mm_mul_pd(tmp, _mm_set1_pd(smax)));
    }
  }

  for (a = 5; a >= 0; a--) {
    jBcol = 12 * a - 1;
    jAcol = 6 * a - 1;
    for (k = a + 2; k < 7; k++) {
      jA = (k - 1) * 12 - 1;

      /* Start for MATLABSystem: '<S2>/AHRS' */
      smax = c_A[k + jAcol];
      if (smax != 0.0) {
        for (jj = 0; jj < 12; jj++) {
          c = (jj + jBcol) + 1;
          A[c] -= A[(jj + jA) + 1] * smax;
        }
      }
    }
  }

  for (a = 4; a >= 0; a--) {
    /* Start for MATLABSystem: '<S2>/AHRS' */
    b_j = b_ipiv[a];
    if (a + 1 != b_j) {
      for (jj = 0; jj < 12; jj++) {
        jBcol = 12 * a + jj;
        smax = A[jBcol];
        c = (b_j - 1) * 12 + jj;
        A[jBcol] = A[c];
        A[c] = smax;
      }
    }
  }
}

static void UAM_GuidanceAnd_SystemCore_step(fusion_simulink_ahrsfilter_UA_T *obj,
  const real_T varargin_1[3], const real_T varargin_2[3], const real_T
  varargin_3[3], real_T varargout_1[4], real_T varargout_2[3])
{
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_3;
  real_T Ppost[144];
  real_T H[72];
  real_T K_tmp[72];
  real_T K_tmp_0[72];
  real_T K_tmp_1[36];
  real_T xe_post[12];
  real_T Rprior[9];
  real_T Rprior_0[9];
  real_T h2[9];
  real_T res[6];
  real_T psquared[4];
  real_T deltaAng[3];
  real_T gravityAccelGyroDiff[3];
  real_T l_tmp[3];
  real_T pLinAccelPostIn[3];
  real_T tmp_2[2];
  real_T absxk;
  real_T accelMeasNoiseVar;
  real_T deltaq_a;
  real_T deltaq_b;
  real_T deltaq_c;
  real_T deltaq_d;
  real_T invpa;
  real_T invpd;
  real_T magMeasNoiseVar;
  real_T pGyroOffsetIn_idx_2;
  real_T pa;
  real_T pd;
  real_T scale;
  real_T t;
  real_T tmp;
  int32_T H_tmp;
  int32_T Rprior_tmp;
  int32_T b_idx;
  int32_T b_idx_0;
  int32_T k;
  boolean_T isJamming;
  static const int8_T tmp_4[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const int8_T tmp_5[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  boolean_T exitg1;

  /* Start for MATLABSystem: '<S2>/AHRS' */
  if (obj->TunablePropsChanged) {
    obj->TunablePropsChanged = false;
    obj->pSensorPeriod = 0.2;
    obj->pKalmanPeriod = obj->pSensorPeriod;
    magMeasNoiseVar = obj->pKalmanPeriod * obj->pKalmanPeriod *
      (obj->GyroscopeDriftNoise + obj->GyroscopeNoise);
    accelMeasNoiseVar = (obj->AccelerometerNoise + obj->LinearAccelerationNoise)
      + magMeasNoiseVar;
    magMeasNoiseVar += obj->MagnetometerNoise + obj->MagneticDisturbanceNoise;
    memset(&Rprior[0], 0, 9U * sizeof(real_T));
    Rprior[0] = 1.0;
    Rprior[4] = 1.0;
    Rprior[8] = 1.0;
    memset(&obj->pQv[0], 0, 36U * sizeof(real_T));
    for (k = 0; k < 3; k++) {
      tmp_3 = _mm_set_pd(magMeasNoiseVar, accelMeasNoiseVar);
      _mm_storeu_pd(&tmp_2[0], _mm_mul_pd(_mm_set_pd(tmp_4[3 * k], Rprior[3 * k]),
        tmp_3));
      obj->pQv[6 * k] = tmp_2[0];
      b_idx = (k + 3) * 6;
      obj->pQv[b_idx + 3] = tmp_2[1];
      b_idx_0 = 3 * k + 1;
      _mm_storeu_pd(&tmp_2[0], _mm_mul_pd(_mm_set_pd(tmp_4[b_idx_0],
        Rprior[b_idx_0]), tmp_3));
      obj->pQv[6 * k + 1] = tmp_2[0];
      obj->pQv[b_idx + 4] = tmp_2[1];
      b_idx_0 = 3 * k + 2;
      _mm_storeu_pd(&tmp_2[0], _mm_mul_pd(_mm_set_pd(tmp_4[b_idx_0],
        Rprior[b_idx_0]), tmp_3));
      obj->pQv[6 * k + 2] = tmp_2[0];
      obj->pQv[b_idx + 5] = tmp_2[1];
    }
  }

  accelMeasNoiseVar = obj->pGyroOffset[0];
  magMeasNoiseVar = obj->pGyroOffset[1];
  pGyroOffsetIn_idx_2 = obj->pGyroOffset[2];
  if (obj->pFirstTime) {
    UAM_GuidanceAndNav_NED_ecompass(varargin_1, varargin_3, Rprior);
    pd = (Rprior[0] + Rprior[4]) + Rprior[8];
    psquared[0] = (pd * 2.0 + 1.0) - pd;
    _mm_storeu_pd(&psquared[1], _mm_sub_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd(2.0),
      _mm_set_pd(Rprior[4], Rprior[0])), _mm_set1_pd(1.0)), _mm_set1_pd(pd)));
    psquared[3] = (2.0 * Rprior[8] + 1.0) - pd;
    if (!rtIsNaN(psquared[0])) {
      b_idx = 1;
    } else {
      b_idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k < 5)) {
        if (!rtIsNaN(psquared[k - 1])) {
          b_idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (b_idx == 0) {
      pd = psquared[0];
      b_idx = 1;
    } else {
      pd = psquared[b_idx - 1];
      b_idx_0 = b_idx;
      for (k = b_idx + 1; k < 5; k++) {
        pa = psquared[k - 1];
        if (pd < pa) {
          pd = pa;
          b_idx_0 = k;
        }
      }

      b_idx = b_idx_0;
    }

    switch (b_idx) {
     case 1:
      pa = sqrt(pd);
      pd = 0.5 * pa;
      invpa = 0.5 / pa;
      tmp_3 = _mm_mul_pd(_mm_sub_pd(_mm_set_pd(Rprior[2], Rprior[7]),
        _mm_loadu_pd(&Rprior[5])), _mm_set1_pd(invpa));
      _mm_storeu_pd(&tmp_2[0], tmp_3);
      pa = tmp_2[0];
      invpd = tmp_2[1];
      invpa *= Rprior[3] - Rprior[1];
      break;

     case 2:
      pd = sqrt(pd);
      pa = 0.5 * pd;
      invpa = 0.5 / pd;
      _mm_storeu_pd(&tmp_2[0], _mm_mul_pd(_mm_add_pd(_mm_set_pd(Rprior[1],
        Rprior[7]), _mm_mul_pd(_mm_set_pd(Rprior[3], Rprior[5]), _mm_set_pd(1.0,
        -1.0))), _mm_set1_pd(invpa)));
      pd = tmp_2[0];
      invpd = tmp_2[1];
      invpa *= Rprior[2] + Rprior[6];
      break;

     case 3:
      pd = sqrt(pd);
      invpd = 0.5 * pd;
      invpa = 0.5 / pd;
      _mm_storeu_pd(&tmp_2[0], _mm_mul_pd(_mm_add_pd(_mm_set_pd(Rprior[1],
        Rprior[2]), _mm_mul_pd(_mm_set_pd(Rprior[3], Rprior[6]), _mm_set_pd(1.0,
        -1.0))), _mm_set1_pd(invpa)));
      pd = tmp_2[0];
      pa = tmp_2[1];
      invpa *= Rprior[5] + Rprior[7];
      break;

     default:
      pd = sqrt(pd);
      invpa = 0.5 * pd;
      invpd = 0.5 / pd;
      _mm_storeu_pd(&tmp_2[0], _mm_mul_pd(_mm_add_pd(_mm_set_pd(Rprior[2],
        Rprior[3]), _mm_mul_pd(_mm_set_pd(Rprior[6], Rprior[1]), _mm_set_pd(1.0,
        -1.0))), _mm_set1_pd(invpd)));
      pd = tmp_2[0];
      pa = tmp_2[1];
      invpd *= Rprior[5] + Rprior[7];
      break;
    }

    if (pd < 0.0) {
      pd = -pd;
      pa = -pa;
      invpd = -invpd;
      invpa = -invpa;
    }

    obj->pOrientPost.a = pd;
    obj->pOrientPost.b = pa;
    obj->pOrientPost.c = invpd;
    obj->pOrientPost.d = invpa;
  }

  tmp_3 = _mm_mul_pd(_mm_sub_pd(_mm_loadu_pd(&varargin_2[0]), _mm_loadu_pd
    (&obj->pGyroOffset[0])), _mm_set1_pd(obj->pSensorPeriod));
  _mm_storeu_pd(&deltaAng[0], tmp_3);

  /* Start for MATLABSystem: '<S2>/AHRS' */
  deltaAng[2] = (varargin_2[2] - obj->pGyroOffset[2]) * obj->pSensorPeriod;
  UAM_Guida_quaternion_quaternion(deltaAng, &deltaq_a, &deltaq_b, &deltaq_c,
    &deltaq_d);

  /* Start for MATLABSystem: '<S2>/AHRS' */
  pd = ((obj->pOrientPost.a * deltaq_a - obj->pOrientPost.b * deltaq_b) -
        obj->pOrientPost.c * deltaq_c) - obj->pOrientPost.d * deltaq_d;
  pa = ((obj->pOrientPost.a * deltaq_b + obj->pOrientPost.b * deltaq_a) +
        obj->pOrientPost.c * deltaq_d) - obj->pOrientPost.d * deltaq_c;
  invpd = ((obj->pOrientPost.a * deltaq_c - obj->pOrientPost.b * deltaq_d) +
           obj->pOrientPost.c * deltaq_a) + obj->pOrientPost.d * deltaq_b;
  invpa = ((obj->pOrientPost.a * deltaq_d + obj->pOrientPost.b * deltaq_c) -
           obj->pOrientPost.c * deltaq_b) + obj->pOrientPost.d * deltaq_a;
  if (pd < 0.0) {
    /* Start for MATLABSystem: '<S2>/AHRS' */
    pd = -pd;
    pa = -pa;
    invpd = -invpd;
    invpa = -invpa;
  }

  UAM_Guida_quaternionBase_rotmat(pd, pa, invpd, invpa, Rprior);

  /* Start for MATLABSystem: '<S2>/AHRS' */
  for (k = 0; k <= 0; k += 2) {
    tmp_3 = _mm_loadu_pd(&Rprior[k + 6]);
    tmp_0 = _mm_mul_pd(tmp_3, _mm_set1_pd(9.81));
    _mm_storeu_pd(&deltaAng[k], tmp_0);
    tmp_1 = _mm_loadu_pd(&obj->pLinAccelPost[k]);
    tmp_1 = _mm_mul_pd(_mm_set1_pd(obj->LinearAccelerationDecayFactor), tmp_1);
    _mm_storeu_pd(&pLinAccelPostIn[k], tmp_1);
    _mm_storeu_pd(&gravityAccelGyroDiff[k], _mm_sub_pd(_mm_add_pd(_mm_loadu_pd
      (&varargin_1[k]), tmp_1), tmp_0));
    tmp_0 = _mm_loadu_pd(&Rprior[k + 3]);
    tmp_1 = _mm_loadu_pd(&Rprior[k]);
    _mm_storeu_pd(&l_tmp[k], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_0, _mm_set1_pd
      (obj->pMagVec[1])), _mm_mul_pd(tmp_1, _mm_set1_pd(obj->pMagVec[0]))),
      _mm_mul_pd(tmp_3, _mm_set1_pd(obj->pMagVec[2]))));
  }

  for (k = 2; k < 3; k++) {
    scale = Rprior[k + 6];
    absxk = scale * 9.81;
    deltaAng[k] = absxk;
    t = obj->LinearAccelerationDecayFactor * obj->pLinAccelPost[k];
    pLinAccelPostIn[k] = t;
    gravityAccelGyroDiff[k] = (varargin_1[k] + t) - absxk;
    l_tmp[k] = (Rprior[k + 3] * obj->pMagVec[1] + Rprior[k] * obj->pMagVec[0]) +
      scale * obj->pMagVec[2];
  }

  memset(&Rprior[0], 0, 9U * sizeof(real_T));

  /* Start for MATLABSystem: '<S2>/AHRS' */
  Rprior[3] = deltaAng[2];
  Rprior[6] = -deltaAng[1];
  Rprior[7] = deltaAng[0];
  for (k = 0; k < 3; k++) {
    Rprior_0[3 * k] = Rprior[3 * k];
    b_idx = 3 * k + 1;
    tmp_3 = _mm_sub_pd(_mm_loadu_pd(&Rprior[b_idx]), _mm_set_pd(Rprior[k + 6],
      Rprior[k + 3]));
    _mm_storeu_pd(&Rprior_0[b_idx], tmp_3);
  }

  for (k = 0; k < 9; k++) {
    Rprior[k] = Rprior_0[k];
    h2[k] = 0.0;
  }

  h2[3] = l_tmp[2];
  h2[6] = -l_tmp[1];
  h2[7] = l_tmp[0];
  for (k = 0; k < 3; k++) {
    Rprior_0[3 * k] = h2[3 * k];
    b_idx = 3 * k + 1;
    tmp_3 = _mm_sub_pd(_mm_loadu_pd(&h2[b_idx]), _mm_set_pd(h2[k + 6], h2[k + 3]));
    _mm_storeu_pd(&Rprior_0[b_idx], tmp_3);
  }

  memcpy(&h2[0], &Rprior_0[0], 9U * sizeof(real_T));
  for (k = 0; k < 3; k++) {
    scale = Rprior[3 * k];
    H[6 * k] = scale;
    b_idx = (k + 3) * 6;

    /* Start for MATLABSystem: '<S2>/AHRS' */
    H[b_idx] = -scale * obj->pKalmanPeriod;
    b_idx_0 = (k + 6) * 6;
    H[b_idx_0] = tmp_4[3 * k];
    H_tmp = (k + 9) * 6;
    H[H_tmp] = 0.0;
    scale = h2[3 * k];
    H[6 * k + 3] = scale;

    /* Start for MATLABSystem: '<S2>/AHRS' */
    H[b_idx + 3] = -scale * obj->pKalmanPeriod;
    H[b_idx_0 + 3] = 0.0;
    H[H_tmp + 3] = tmp_5[3 * k];
    Rprior_tmp = 3 * k + 1;
    scale = Rprior[Rprior_tmp];
    H[6 * k + 1] = scale;

    /* Start for MATLABSystem: '<S2>/AHRS' */
    H[b_idx + 1] = -scale * obj->pKalmanPeriod;
    H[b_idx_0 + 1] = tmp_4[Rprior_tmp];
    H[H_tmp + 1] = 0.0;
    scale = h2[Rprior_tmp];
    H[6 * k + 4] = scale;

    /* Start for MATLABSystem: '<S2>/AHRS' */
    H[b_idx + 4] = -scale * obj->pKalmanPeriod;
    H[b_idx_0 + 4] = 0.0;
    H[H_tmp + 4] = tmp_5[Rprior_tmp];
    Rprior_tmp = 3 * k + 2;
    scale = Rprior[Rprior_tmp];
    H[6 * k + 2] = scale;

    /* Start for MATLABSystem: '<S2>/AHRS' */
    H[b_idx + 2] = -scale * obj->pKalmanPeriod;
    H[b_idx_0 + 2] = tmp_4[Rprior_tmp];
    H[H_tmp + 2] = 0.0;
    scale = h2[Rprior_tmp];
    H[6 * k + 5] = scale;

    /* Start for MATLABSystem: '<S2>/AHRS' */
    H[b_idx + 5] = -scale * obj->pKalmanPeriod;
    H[b_idx_0 + 5] = 0.0;
    H[H_tmp + 5] = tmp_5[Rprior_tmp];
  }

  /* Start for MATLABSystem: '<S2>/AHRS' */
  for (k = 0; k < 6; k++) {
    for (b_idx = 0; b_idx < 12; b_idx++) {
      H_tmp = 6 * b_idx + k;
      K_tmp[b_idx + 12 * k] = H[H_tmp];
      scale = 0.0;
      for (b_idx_0 = 0; b_idx_0 < 12; b_idx_0++) {
        scale += H[6 * b_idx_0 + k] * obj->pQw[12 * b_idx + b_idx_0];
      }

      K_tmp_0[H_tmp] = scale;
    }
  }

  for (k = 0; k < 6; k++) {
    for (b_idx = 0; b_idx < 12; b_idx++) {
      scale = 0.0;
      for (b_idx_0 = 0; b_idx_0 < 12; b_idx_0++) {
        scale += obj->pQw[12 * b_idx_0 + b_idx] * K_tmp[12 * k + b_idx_0];
      }

      H[b_idx + 12 * k] = scale;
    }

    for (b_idx = 0; b_idx < 6; b_idx++) {
      tmp = 0.0;
      for (b_idx_0 = 0; b_idx_0 < 12; b_idx_0++) {
        tmp += K_tmp_0[6 * b_idx_0 + b_idx] * K_tmp[12 * k + b_idx_0];
      }

      K_tmp_1[k + 6 * b_idx] = obj->pQv[6 * k + b_idx] + tmp;
    }
  }

  UAM_GuidanceAndNavigati_mrdiv_b(H, K_tmp_1);
  res[0] = gravityAccelGyroDiff[0];

  /* Start for MATLABSystem: '<S2>/AHRS' */
  res[3] = varargin_3[0] - l_tmp[0];
  res[1] = gravityAccelGyroDiff[1];

  /* Start for MATLABSystem: '<S2>/AHRS' */
  res[4] = varargin_3[1] - l_tmp[1];
  res[2] = gravityAccelGyroDiff[2];

  /* Start for MATLABSystem: '<S2>/AHRS' */
  res[5] = varargin_3[2] - l_tmp[2];
  for (k = 0; k < 3; k++) {
    deltaq_a = 0.0;
    for (b_idx = 0; b_idx < 6; b_idx++) {
      deltaq_a += H[(12 * b_idx + k) + 9] * res[b_idx];
    }

    l_tmp[k] = deltaq_a;
  }

  scale = 3.3121686421112381E-170;

  /* Start for MATLABSystem: '<S2>/AHRS' */
  absxk = fabs(l_tmp[0]);
  if (absxk > 3.3121686421112381E-170) {
    deltaq_a = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    deltaq_a = t * t;
  }

  /* Start for MATLABSystem: '<S2>/AHRS' */
  absxk = fabs(l_tmp[1]);
  if (absxk > scale) {
    t = scale / absxk;
    deltaq_a = deltaq_a * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    deltaq_a += t * t;
  }

  /* Start for MATLABSystem: '<S2>/AHRS' */
  absxk = fabs(l_tmp[2]);
  if (absxk > scale) {
    t = scale / absxk;
    deltaq_a = deltaq_a * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    deltaq_a += t * t;
  }

  deltaq_a = scale * sqrt(deltaq_a);

  /* Start for MATLABSystem: '<S2>/AHRS' */
  isJamming = (deltaq_a * deltaq_a > obj->ExpectedMagneticFieldStrength *
               obj->ExpectedMagneticFieldStrength * 4.0);
  if (isJamming) {
    /* Start for MATLABSystem: '<S2>/AHRS' */
    scale = gravityAccelGyroDiff[1];
    absxk = gravityAccelGyroDiff[0];
    t = gravityAccelGyroDiff[2];
    for (k = 0; k <= 6; k += 2) {
      tmp_3 = _mm_loadu_pd(&H[k + 12]);
      tmp_0 = _mm_loadu_pd(&H[k]);
      tmp_1 = _mm_loadu_pd(&H[k + 24]);
      _mm_storeu_pd(&Rprior[k], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_3,
        _mm_set1_pd(scale)), _mm_mul_pd(tmp_0, _mm_set1_pd(absxk))), _mm_mul_pd
        (tmp_1, _mm_set1_pd(t))));
    }

    for (k = 8; k < 9; k++) {
      Rprior[k] = (H[k + 12] * scale + H[k] * absxk) + H[k + 24] * t;
    }

    deltaAng[0] = Rprior[0];
    gravityAccelGyroDiff[0] = Rprior[3];
    scale = Rprior[6];
    deltaAng[1] = Rprior[1];
    gravityAccelGyroDiff[1] = Rprior[4];
    absxk = Rprior[7];
    deltaAng[2] = Rprior[2];
    gravityAccelGyroDiff[2] = Rprior[5];
    t = Rprior[8];
  } else {
    for (k = 0; k < 12; k++) {
      /* Start for MATLABSystem: '<S2>/AHRS' */
      scale = 0.0;
      for (b_idx = 0; b_idx < 6; b_idx++) {
        /* Start for MATLABSystem: '<S2>/AHRS' */
        scale += H[12 * b_idx + k] * res[b_idx];
      }

      /* Start for MATLABSystem: '<S2>/AHRS' */
      xe_post[k] = scale;
    }

    /* Start for MATLABSystem: '<S2>/AHRS' */
    deltaAng[0] = xe_post[0];
    gravityAccelGyroDiff[0] = xe_post[3];
    scale = xe_post[6];
    deltaAng[1] = xe_post[1];
    gravityAccelGyroDiff[1] = xe_post[4];
    absxk = xe_post[7];
    deltaAng[2] = xe_post[2];
    gravityAccelGyroDiff[2] = xe_post[5];
    t = xe_post[8];
  }

  /* Start for MATLABSystem: '<S2>/AHRS' */
  UAM_Guida_quaternion_quaternion(deltaAng, &deltaq_a, &deltaq_b, &deltaq_c,
    &deltaq_d);
  tmp = ((pd * deltaq_a - pa * -deltaq_b) - invpd * -deltaq_c) - invpa *
    -deltaq_d;
  obj->pOrientPost.a = tmp;
  obj->pOrientPost.b = ((pd * -deltaq_b + pa * deltaq_a) + invpd * -deltaq_d) -
    invpa * -deltaq_c;
  obj->pOrientPost.c = ((pd * -deltaq_c - pa * -deltaq_d) + invpd * deltaq_a) +
    invpa * -deltaq_b;
  obj->pOrientPost.d = ((pd * -deltaq_d + pa * -deltaq_c) - invpd * -deltaq_b) +
    invpa * deltaq_a;
  if (tmp < 0.0) {
    obj->pOrientPost.a = -obj->pOrientPost.a;
    obj->pOrientPost.b = -obj->pOrientPost.b;
    obj->pOrientPost.c = -obj->pOrientPost.c;
    obj->pOrientPost.d = -obj->pOrientPost.d;
  }

  /* Start for MATLABSystem: '<S2>/AHRS' */
  pd = sqrt(((obj->pOrientPost.a * obj->pOrientPost.a + obj->pOrientPost.b *
              obj->pOrientPost.b) + obj->pOrientPost.c * obj->pOrientPost.c) +
            obj->pOrientPost.d * obj->pOrientPost.d);
  obj->pOrientPost.a /= pd;
  obj->pOrientPost.b /= pd;
  obj->pOrientPost.c /= pd;
  obj->pOrientPost.d /= pd;
  UAM_Guida_quaternionBase_rotmat(obj->pOrientPost.a, obj->pOrientPost.b,
    obj->pOrientPost.c, obj->pOrientPost.d, Rprior);
  if (!isJamming) {
    /* Start for MATLABSystem: '<S2>/AHRS' */
    deltaq_a = l_tmp[0];
    pd = l_tmp[1];
    pa = l_tmp[2];
    for (b_idx = 0; b_idx <= 0; b_idx += 2) {
      k = (b_idx + 1) * 3;

      /* Start for MATLABSystem: '<S2>/AHRS' */
      _mm_storeu_pd(&deltaAng[b_idx], _mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_set_pd(Rprior[k], Rprior[b_idx * 3]), _mm_set1_pd(deltaq_a)),
        _mm_mul_pd(_mm_set_pd(Rprior[k + 1], Rprior[b_idx * 3 + 1]), _mm_set1_pd
                   (pd))), _mm_mul_pd(_mm_set_pd(Rprior[k + 2], Rprior[b_idx * 3
        + 2]), _mm_set1_pd(pa))));
    }

    for (b_idx = 2; b_idx < 3; b_idx++) {
      /* Start for MATLABSystem: '<S2>/AHRS' */
      deltaAng[b_idx] = (Rprior[b_idx * 3 + 1] * pd + Rprior[b_idx * 3] *
                         deltaq_a) + Rprior[b_idx * 3 + 2] * pa;
    }

    /* Start for MATLABSystem: '<S2>/AHRS' */
    tmp_3 = _mm_sub_pd(_mm_loadu_pd(&obj->pMagVec[0]), _mm_loadu_pd(&deltaAng[0]));
    _mm_storeu_pd(&obj->pMagVec[0], tmp_3);

    /* Start for MATLABSystem: '<S2>/AHRS' */
    obj->pMagVec[2] -= deltaAng[2];
    pd = rt_atan2d_snf(obj->pMagVec[2], obj->pMagVec[0]);
    if (pd < -obj->pInclinationLimit) {
      pd = -1.5707963267948966;
    }

    if (pd > obj->pInclinationLimit) {
      pd = 1.5707963267948966;
    }

    obj->pMagVec[0] = 0.0;
    obj->pMagVec[1] = 0.0;
    obj->pMagVec[2] = 0.0;

    /* Start for MATLABSystem: '<S2>/AHRS' */
    obj->pMagVec[0] = cos(pd);
    obj->pMagVec[2] = sin(pd);
    tmp_3 = _mm_mul_pd(_mm_set1_pd(obj->ExpectedMagneticFieldStrength),
                       _mm_loadu_pd(&obj->pMagVec[0]));
    _mm_storeu_pd(&obj->pMagVec[0], tmp_3);

    /* Start for MATLABSystem: '<S2>/AHRS' */
    obj->pMagVec[2] *= obj->ExpectedMagneticFieldStrength;
  }

  for (k = 0; k < 12; k++) {
    for (b_idx = 0; b_idx < 12; b_idx++) {
      tmp = 0.0;
      for (b_idx_0 = 0; b_idx_0 < 6; b_idx_0++) {
        /* Start for MATLABSystem: '<S2>/AHRS' */
        tmp += H[12 * b_idx_0 + k] * K_tmp_0[6 * b_idx + b_idx_0];
      }

      /* Start for MATLABSystem: '<S2>/AHRS' */
      b_idx_0 = 12 * b_idx + k;
      Ppost[b_idx_0] = obj->pQw[b_idx_0] - tmp;
    }
  }

  memset(&obj->pQw[0], 0, 144U * sizeof(real_T));

  /* Start for MATLABSystem: '<S2>/AHRS' */
  pd = obj->pKalmanPeriod * obj->pKalmanPeriod;
  pa = obj->GyroscopeDriftNoise + obj->GyroscopeNoise;
  obj->pQw[0] = (Ppost[39] + pa) * pd + Ppost[0];
  obj->pQw[39] = Ppost[39] + obj->GyroscopeDriftNoise;
  obj->pQw[13] = (Ppost[52] + pa) * pd + Ppost[13];
  obj->pQw[52] = Ppost[52] + obj->GyroscopeDriftNoise;
  obj->pQw[26] = (Ppost[65] + pa) * pd + Ppost[26];
  obj->pQw[65] = Ppost[65] + obj->GyroscopeDriftNoise;
  _mm_storeu_pd(&deltaAng[0], _mm_mul_pd(_mm_set1_pd(-obj->pKalmanPeriod),
    _mm_set_pd(obj->pQw[52], obj->pQw[39])));
  deltaAng[2] = -obj->pKalmanPeriod * obj->pQw[65];
  obj->pFirstTime = false;
  varargout_1[0] = obj->pOrientPost.a;
  varargout_1[1] = obj->pOrientPost.b;
  varargout_1[2] = obj->pOrientPost.c;
  varargout_1[3] = obj->pOrientPost.d;
  obj->pQw[3] = deltaAng[0];
  obj->pQw[36] = deltaAng[0];

  /* Start for MATLABSystem: '<S2>/AHRS' */
  tmp_3 = _mm_set_pd(obj->MagneticDisturbanceDecayFactor *
                     obj->MagneticDisturbanceDecayFactor,
                     obj->LinearAccelerationDecayFactor *
                     obj->LinearAccelerationDecayFactor);
  tmp_0 = _mm_set_pd(obj->MagneticDisturbanceNoise, obj->LinearAccelerationNoise);
  _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_mul_pd(tmp_3, _mm_set_pd(Ppost[117],
    Ppost[78])), tmp_0));
  obj->pQw[78] = tmp_2[0];
  obj->pQw[117] = tmp_2[1];
  _mm_storeu_pd(&tmp_2[0], _mm_sub_pd(_mm_set_pd(pLinAccelPostIn[0],
    obj->pGyroOffset[0]), _mm_set_pd(scale, gravityAccelGyroDiff[0])));
  obj->pGyroOffset[0] = tmp_2[0];
  obj->pLinAccelPost[0] = tmp_2[1];
  varargout_2[0] = varargin_2[0] - accelMeasNoiseVar;
  obj->pQw[16] = deltaAng[1];
  obj->pQw[49] = deltaAng[1];

  /* Start for MATLABSystem: '<S2>/AHRS' */
  _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_mul_pd(tmp_3, _mm_set_pd(Ppost[130],
    Ppost[91])), tmp_0));
  obj->pQw[91] = tmp_2[0];
  obj->pQw[130] = tmp_2[1];
  _mm_storeu_pd(&tmp_2[0], _mm_sub_pd(_mm_set_pd(pLinAccelPostIn[1],
    obj->pGyroOffset[1]), _mm_set_pd(absxk, gravityAccelGyroDiff[1])));
  obj->pGyroOffset[1] = tmp_2[0];
  obj->pLinAccelPost[1] = tmp_2[1];
  varargout_2[1] = varargin_2[1] - magMeasNoiseVar;
  obj->pQw[29] = deltaAng[2];
  obj->pQw[62] = deltaAng[2];

  /* Start for MATLABSystem: '<S2>/AHRS' */
  _mm_storeu_pd(&tmp_2[0], _mm_add_pd(_mm_mul_pd(tmp_3, _mm_set_pd(Ppost[143],
    Ppost[104])), tmp_0));
  obj->pQw[104] = tmp_2[0];
  obj->pQw[143] = tmp_2[1];
  _mm_storeu_pd(&tmp_2[0], _mm_sub_pd(_mm_set_pd(pLinAccelPostIn[2],
    obj->pGyroOffset[2]), _mm_set_pd(t, gravityAccelGyroDiff[2])));
  obj->pGyroOffset[2] = tmp_2[0];
  obj->pLinAccelPost[2] = tmp_2[1];
  varargout_2[2] = varargin_2[2] - pGyroOffsetIn_idx_2;
}

/* System initialize for referenced model: 'UAM_GuidanceAndNavigation' */
void UAM_GuidanceAndNavigation_Init(uint8_T *rty_mode,
  DW_UAM_GuidanceAndNavigatio_f_T *localDW)
{
  /* Start for InitialCondition: '<S3>/IC' */
  *rty_mode = 1U;
  localDW->IC_FirstOutputTime = true;

  /* Start for MATLABSystem: '<S6>/PathManagerSystemObject' */
  localDW->obj_e.PoseRTLFlag = true;
  localDW->objisempty_o = true;
  localDW->obj_e.isInitialized = 1;
  UAM_Guida_PathManager_setupImpl(&localDW->obj_e);

  /* InitializeConditions for MATLABSystem: '<S6>/PathManagerSystemObject' */
  localDW->obj_e.MissionPointCounter = 1.0;
  localDW->obj_e.MissionStart = true;
  localDW->obj_e.PrevModeStatus = false;
  localDW->obj_e.PoseHoldFlag = true;
  localDW->obj_e.EnableRepeat = false;
  localDW->obj_e.FinalMissionPoint = false;

  /* Start for MATLABSystem: '<S2>/AHRS' */
  localDW->objisempty_b = true;
  localDW->obj.AccelerometerNoise = 0.0001924722;
  localDW->obj.GyroscopeNoise = 9.1385E-5;
  localDW->obj.MagnetometerNoise = 0.1;
  localDW->obj.GyroscopeDriftNoise = 3.0462E-13;
  localDW->obj.LinearAccelerationNoise = 0.0096236100000000012;
  localDW->obj.MagneticDisturbanceNoise = 0.5;
  localDW->obj.LinearAccelerationDecayFactor = 0.5;
  localDW->obj.MagneticDisturbanceDecayFactor = 0.5;
  localDW->obj.ExpectedMagneticFieldStrength = 50.0;
  localDW->obj.isInitialized = 1;
  localDW->obj.pInputPrototype[0] = 0.0;
  localDW->obj.pInputPrototype[1] = 0.0;
  localDW->obj.pInputPrototype[2] = 0.0;
  localDW->obj.pSensorPeriod = 0.2;
  localDW->obj.pKalmanPeriod = 0.2;
  localDW->obj.TunablePropsChanged = false;

  /* InitializeConditions for MATLABSystem: '<S2>/AHRS' */
  UAM_Gu_AHRSFilterBase_resetImpl(&localDW->obj);

  /* Start for MATLABSystem: '<S2>/Coordinate Transformation Conversion1' */
  localDW->objisempty_f = true;
  localDW->obj_p.isInitialized = 1;

  /* Start for MATLABSystem: '<S2>/Coordinate Transformation Conversion2' */
  localDW->objisempty = true;
  localDW->obj_a.isInitialized = 1;
}

/* Output and update for referenced model: 'UAM_GuidanceAndNavigation' */
void UAM_GuidanceAndNavigation(const real_T *rtu_sensorData_GPSCourse, const
  real_T rtu_sensorData_LLA[3], const real_T rtu_sensorData_Abb[3], const real_T
  rtu_sensorData_Gyro[3], const real_T rtu_sensorData_Mag[3], const real_T
  rtu_sensorData_GPSVelocity[3], const real_T *rtu_sensorData_RotorParameters_,
  const UAVPathManagerBus rtu_Globalmission[9], const boolean_T *rtu_ModeDone,
  uint8_T *rty_mode, uint8_T *rty_ToWP_mode, real_T rty_ToWP_position[3], real_T
  rty_ToWP_params[4], uint8_T *rty_FromWP_mode, real_T rty_FromWP_position[3],
  real_T rty_FromWP_params[4], real_T rty_pose[4], real_T rty_States_Euler[3],
  real_T rty_States_pqr[3], real_T rty_States_Ve[3], real_T rty_States_Xe[3],
  real_T *rty_States_course, real_T *rty_States_c1,
  DW_UAM_GuidanceAndNavigatio_f_T *localDW)
{
  __m128d tmp_1;
  __m128d tmp_3;
  real_T tmp[4];
  real_T tmp_2[2];
  real_T Merge;
  real_T Merge_a;
  real_T rtb_Atan2;
  real_T rtb_Gain1;
  real_T rtb_Sum1_idx_0;
  int32_T tmp_0;
  boolean_T p;

  /* If: '<S27>/If' */
  if (UAM_GuidanceAndNavigatio_ConstB.Abs_o > 0.0) {
    /* Outputs for IfAction SubSystem: '<S27>/If Action Subsystem' incorporates:
     *  ActionPort: '<S29>/Action Port'
     */
    UAM_GuidanceA_IfActionSubsystem(UAM_GuidanceAndNavigatio_ConstB.Switch,
      UAM_GuidanceAndNavigatio_ConstB.Abs_o, &Merge);

    /* End of Outputs for SubSystem: '<S27>/If Action Subsystem' */
  } else {
    /* Outputs for IfAction SubSystem: '<S27>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S30>/Action Port'
     */
    UAM_Guidance_IfActionSubsystem1(&Merge);

    /* End of Outputs for SubSystem: '<S27>/If Action Subsystem1' */
  }

  /* End of If: '<S27>/If' */

  /* Switch: '<S24>/Switch' incorporates:
   *  Product: '<S24>/Divide1'
   */
  if (UAM_GuidanceAndNavigatio_ConstB.Compare_p) {
    Merge *= UAM_GuidanceAndNavigatio_ConstB.Bias1_l;
  } else {
    Merge = UAM_GuidanceAndNavigatio_ConstB.Switch;
  }

  /* End of Switch: '<S24>/Switch' */

  /* Sum: '<S10>/Sum1' */
  rtb_Sum1_idx_0 = rtu_sensorData_LLA[0] - Merge;

  /* Switch: '<S19>/Switch' incorporates:
   *  Abs: '<S19>/Abs'
   *  Bias: '<S19>/Bias'
   *  Bias: '<S19>/Bias1'
   *  Constant: '<S19>/Constant2'
   *  Constant: '<S22>/Constant'
   *  Math: '<S19>/Math Function1'
   *  RelationalOperator: '<S22>/Compare'
   */
  if (fabs(rtb_Sum1_idx_0) > 180.0) {
    rtb_Sum1_idx_0 = rt_modd_snf(rtb_Sum1_idx_0 + 180.0, 360.0) - 180.0;
  }

  /* End of Switch: '<S19>/Switch' */

  /* Abs: '<S18>/Abs' incorporates:
   *  Abs: '<S15>/Abs1'
   */
  rtb_Gain1 = fabs(rtb_Sum1_idx_0);

  /* If: '<S18>/If' incorporates:
   *  Abs: '<S18>/Abs'
   */
  if (rtb_Gain1 > 0.0) {
    /* Outputs for IfAction SubSystem: '<S18>/If Action Subsystem' incorporates:
     *  ActionPort: '<S20>/Action Port'
     */
    UAM_GuidanceA_IfActionSubsystem(rtb_Sum1_idx_0, rtb_Gain1, &Merge_a);

    /* End of Outputs for SubSystem: '<S18>/If Action Subsystem' */
  } else {
    /* Outputs for IfAction SubSystem: '<S18>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S21>/Action Port'
     */
    UAM_Guidance_IfActionSubsystem1(&Merge_a);

    /* End of Outputs for SubSystem: '<S18>/If Action Subsystem1' */
  }

  /* End of If: '<S18>/If' */

  /* Switch: '<S15>/Switch' incorporates:
   *  Bias: '<S15>/Bias'
   *  Bias: '<S15>/Bias1'
   *  Constant: '<S11>/Constant'
   *  Constant: '<S11>/Constant1'
   *  Constant: '<S17>/Constant'
   *  Gain: '<S15>/Gain'
   *  Product: '<S15>/Divide1'
   *  RelationalOperator: '<S17>/Compare'
   *  Switch: '<S11>/Switch1'
   */
  if (rtb_Gain1 > 90.0) {
    rtb_Gain1 = (-(rtb_Gain1 - 90.0) + 90.0) * Merge_a;
    tmp_0 = 180;
  } else {
    rtb_Gain1 = rtb_Sum1_idx_0;
    tmp_0 = 0;
  }

  /* End of Switch: '<S15>/Switch' */

  /* Sum: '<S11>/Sum' incorporates:
   *  Sum: '<S10>/Sum1'
   *  Switch: '<S11>/Switch1'
   */
  Merge_a = (rtu_sensorData_LLA[1] - UAM_GuidanceAndNavigatio_ConstB.Switch_o) +
    (real_T)tmp_0;

  /* UnitConversion: '<S14>/Unit Conversion' */
  /* Unit Conversion - from: deg to: rad
     Expression: output = (0.0174533*input) + (0) */
  rtb_Sum1_idx_0 = 0.017453292519943295 * rtb_Gain1;

  /* UnitConversion: '<S35>/Unit Conversion' */
  /* Unit Conversion - from: deg to: rad
     Expression: output = (0.0174533*input) + (0) */
  Merge *= 0.017453292519943295;

  /* Trigonometry: '<S36>/Trigonometric Function1' */
  rtb_Atan2 = sin(Merge);

  /* Sum: '<S36>/Sum1' incorporates:
   *  Constant: '<S36>/Constant'
   *  Product: '<S36>/Product1'
   */
  rtb_Atan2 = 1.0 - UAM_GuidanceAndNavigatio_ConstB.sqrt_g *
    UAM_GuidanceAndNavigatio_ConstB.sqrt_g * rtb_Atan2 * rtb_Atan2;

  /* Product: '<S34>/Product1' incorporates:
   *  Constant: '<S34>/Constant1'
   *  Sqrt: '<S34>/sqrt'
   */
  rtb_Gain1 = 6.378137E+6 / sqrt(rtb_Atan2);

  /* Product: '<S13>/dNorth' incorporates:
   *  Product: '<S34>/Product3'
   */
  rtb_Atan2 = rtb_Gain1 * UAM_GuidanceAndNavigatio_ConstB.Sum1_l / rtb_Atan2 *
    rtb_Sum1_idx_0;

  /* Switch: '<S16>/Switch' incorporates:
   *  Abs: '<S16>/Abs'
   *  Bias: '<S16>/Bias'
   *  Bias: '<S16>/Bias1'
   *  Constant: '<S16>/Constant2'
   *  Constant: '<S23>/Constant'
   *  Math: '<S16>/Math Function1'
   *  RelationalOperator: '<S23>/Compare'
   */
  if (fabs(Merge_a) > 180.0) {
    Merge_a = rt_modd_snf(Merge_a + 180.0, 360.0) - 180.0;
  }

  /* Product: '<S13>/dEast' incorporates:
   *  Product: '<S34>/Product4'
   *  Switch: '<S16>/Switch'
   *  Trigonometry: '<S34>/Trigonometric Function'
   *  UnitConversion: '<S14>/Unit Conversion'
   */
  rtb_Gain1 = rtb_Gain1 * cos(Merge) * (0.017453292519943295 * Merge_a);

  /* RateTransition generated from: '<S2>/Rate Transition1' incorporates:
   *  Product: '<S13>/x*cos'
   *  Product: '<S13>/x*sin'
   *  Product: '<S13>/y*cos'
   *  Product: '<S13>/y*sin'
   *  Sum: '<S10>/Sum'
   *  Sum: '<S13>/Sum2'
   *  Sum: '<S13>/Sum3'
   *  UnaryMinus: '<S10>/Ze2height'
   */
  rty_States_Xe[0] = rtb_Atan2 * UAM_GuidanceAndNavigatio_ConstB.SinCos_o2 +
    rtb_Gain1 * UAM_GuidanceAndNavigatio_ConstB.SinCos_o1;
  rty_States_Xe[1] = rtb_Gain1 * UAM_GuidanceAndNavigatio_ConstB.SinCos_o2 -
    rtb_Atan2 * UAM_GuidanceAndNavigatio_ConstB.SinCos_o1;
  rty_States_Xe[2] = -rtu_sensorData_LLA[2];

  /* RateTransition generated from: '<S2>/Rate Transition1' incorporates:
   *  Math: '<S2>/Transpose4'
   */
  rty_States_Ve[0] = rtu_sensorData_GPSVelocity[0];

  /* SignalConversion generated from: '<S6>/PathManagerSystemObject' incorporates:
   *  SignalConversion generated from: '<Root>/pose'
   */
  rty_pose[0] = rty_States_Xe[0];

  /* RateTransition generated from: '<S2>/Rate Transition1' incorporates:
   *  Math: '<S2>/Transpose4'
   */
  rty_States_Ve[1] = rtu_sensorData_GPSVelocity[1];

  /* SignalConversion generated from: '<S6>/PathManagerSystemObject' incorporates:
   *  SignalConversion generated from: '<Root>/pose'
   */
  rty_pose[1] = rty_States_Xe[1];

  /* RateTransition generated from: '<S2>/Rate Transition1' incorporates:
   *  Math: '<S2>/Transpose4'
   */
  rty_States_Ve[2] = rtu_sensorData_GPSVelocity[2];

  /* SignalConversion generated from: '<S6>/PathManagerSystemObject' incorporates:
   *  SignalConversion generated from: '<Root>/pose'
   *  Trigonometry: '<S1>/Atan2'
   */
  rty_pose[2] = rty_States_Xe[2];
  rty_pose[3] = rt_atan2d_snf(rty_States_Ve[0], rty_States_Ve[1]);

  /* MATLABSystem: '<S6>/PathManagerSystemObject' incorporates:
   *  Constant: '<S3>/Constant1'
   *  SignalConversion generated from: '<S6>/PathManagerSystemObject'
   */
  if (localDW->obj_e.MissionStart) {
    localDW->obj_e.PrevMissionPoint.mode = 0U;
    localDW->obj_e.PrevMissionPoint.position[0] = rty_pose[0];
    localDW->obj_e.PrevMissionPoint.position[1] = rty_pose[1];
    localDW->obj_e.PrevMissionPoint.position[2] = rty_pose[2];
    localDW->obj_e.PrevMissionPoint.params[0] = -1.0;
    localDW->obj_e.PrevMissionPoint.params[1] = -1.0;
    localDW->obj_e.PrevMissionPoint.params[2] = -1.0;
    localDW->obj_e.PrevMissionPoint.params[3] = -1.0;
    localDW->obj_e.MissionStart = false;
  }

  if ((*rtu_ModeDone != localDW->obj_e.PrevModeStatus) && (*rtu_ModeDone) &&
      (rtu_Globalmission[(int32_T)localDW->obj_e.MissionPointCounter - 1].mode
       != 7)) {
    localDW->obj_e.PrevMissionPoint = rtu_Globalmission[(int32_T)
      localDW->obj_e.MissionPointCounter - 1];
    if (localDW->obj_e.MissionPointCounter == 9.0) {
      localDW->obj_e.FinalMissionPoint = true;
      switch (rtu_Globalmission[8].mode) {
       case 1U:
       case 2U:
       case 3U:
       case 6U:
        localDW->obj_e.MissionParams[0].mode = 7U;
        localDW->obj_e.MissionParams[0].position[0] = rty_pose[0];
        localDW->obj_e.MissionParams[0].position[1] = rty_pose[1];
        localDW->obj_e.MissionParams[0].position[2] = rty_pose[2];
        localDW->obj_e.MissionParams[0].params[0] = -1.0;
        localDW->obj_e.MissionParams[0].params[1] = -1.0;
        localDW->obj_e.MissionParams[0].params[2] = -1.0;
        localDW->obj_e.MissionParams[0].params[3] = -1.0;
        localDW->obj_e.MissionParams[1] = localDW->obj_e.PrevMissionPoint;
        break;

       case 4U:
        localDW->obj_e.MissionParams[0] = rtu_Globalmission[8];
        localDW->obj_e.MissionParams[1] = localDW->obj_e.PrevMissionPoint;
        break;

       case 5U:
        localDW->obj_e.MissionParams[0].mode = 5U;
        localDW->obj_e.MissionParams[0].position[0] = 0.0;
        localDW->obj_e.MissionParams[0].position[1] = 0.0;
        localDW->obj_e.MissionParams[0].position[2] = 0.0;
        localDW->obj_e.MissionParams[0].params[0] = -1.0;
        localDW->obj_e.MissionParams[0].params[1] = -1.0;
        localDW->obj_e.MissionParams[0].params[2] = -1.0;
        localDW->obj_e.MissionParams[0].params[3] = -1.0;
        localDW->obj_e.MissionParams[1] = localDW->obj_e.PrevMissionPoint;
        break;
      }

      localDW->obj_e.EnableRepeat = true;
      localDW->obj_e.MissionPointCounter = 8.0;
    }

    localDW->obj_e.MissionPointCounter++;
  }

  localDW->obj_e.PoseHoldFlag = true;
  localDW->obj_e.PoseRTLFlag = true;
  if (!localDW->obj_e.FinalMissionPoint) {
    switch (rtu_Globalmission[(int32_T)localDW->obj_e.MissionPointCounter - 1].
            mode) {
     case 6U:
      localDW->obj_e.MissionParams[0] = rtu_Globalmission[(int32_T)
        localDW->obj_e.MissionPointCounter - 1];
      localDW->obj_e.MissionParams[1] = localDW->obj_e.PrevMissionPoint;
      break;

     case 1U:
      localDW->obj_e.MissionParams[0] = rtu_Globalmission[(int32_T)
        localDW->obj_e.MissionPointCounter - 1];
      localDW->obj_e.MissionParams[1] = localDW->obj_e.PrevMissionPoint;
      break;

     case 2U:
      p = false;
      if (localDW->obj_e.PrevMissionPoint.mode == 2) {
        p = true;
      }

      if (p) {
        localDW->obj_e.MissionParams[1] = localDW->obj_e.PrevMissionPoint;
      } else {
        localDW->obj_e.MissionParams[1].mode =
          localDW->obj_e.PrevMissionPoint.mode;
        localDW->obj_e.MissionParams[1].position[0] =
          localDW->obj_e.PrevMissionPoint.position[0];
        localDW->obj_e.MissionParams[1].position[1] =
          localDW->obj_e.PrevMissionPoint.position[1];
        localDW->obj_e.MissionParams[1].position[2] =
          localDW->obj_e.PrevMissionPoint.position[2];
        localDW->obj_e.MissionParams[1].params[0] = rt_atan2d_snf
          (rtu_Globalmission[(int32_T)localDW->obj_e.MissionPointCounter - 1].
           position[1] - localDW->obj_e.PrevMissionPoint.position[1],
           rtu_Globalmission[(int32_T)localDW->obj_e.MissionPointCounter - 1].
           position[0] - localDW->obj_e.PrevMissionPoint.position[0]);
        localDW->obj_e.MissionParams[1].params[1] = 25.0;
        localDW->obj_e.MissionParams[1].params[2] = -1.0;
        localDW->obj_e.MissionParams[1].params[3] = -1.0;
      }

      localDW->obj_e.MissionParams[0] = rtu_Globalmission[(int32_T)
        localDW->obj_e.MissionPointCounter - 1];
      break;

     case 3U:
      localDW->obj_e.MissionParams[0] = rtu_Globalmission[(int32_T)
        localDW->obj_e.MissionPointCounter - 1];
      localDW->obj_e.MissionParams[1] = localDW->obj_e.PrevMissionPoint;
      break;

     case 4U:
      localDW->obj_e.MissionParams[0] = rtu_Globalmission[(int32_T)
        localDW->obj_e.MissionPointCounter - 1];
      localDW->obj_e.MissionParams[1] = localDW->obj_e.PrevMissionPoint;
      break;

     case 5U:
      localDW->obj_e.MissionParams[0].mode = 5U;
      localDW->obj_e.MissionParams[0].position[0] = 0.0;
      localDW->obj_e.MissionParams[0].position[1] = 0.0;
      localDW->obj_e.MissionParams[0].position[2] = 0.0;
      localDW->obj_e.MissionParams[0].params[0] = -1.0;
      localDW->obj_e.MissionParams[0].params[1] = -1.0;
      localDW->obj_e.MissionParams[0].params[2] = -1.0;
      localDW->obj_e.MissionParams[0].params[3] = -1.0;
      localDW->obj_e.MissionParams[1] = localDW->obj_e.PrevMissionPoint;
      break;

     default:
      localDW->obj_e.MissionParams[0] = rtu_Globalmission[(int32_T)
        localDW->obj_e.MissionPointCounter - 1];
      localDW->obj_e.MissionParams[1] = localDW->obj_e.PrevMissionPoint;
      break;
    }
  }

  localDW->obj_e.PrevModeStatus = *rtu_ModeDone;

  /* SignalConversion generated from: '<Root>/FromWP' incorporates:
   *  MATLABSystem: '<S6>/PathManagerSystemObject'
   * */
  *rty_FromWP_mode = localDW->obj_e.MissionParams[1].mode;

  /* SignalConversion generated from: '<Root>/FromWP' incorporates:
   *  MATLABSystem: '<S6>/PathManagerSystemObject'
   * */
  rty_FromWP_position[0] = localDW->obj_e.MissionParams[1].position[0];
  rty_FromWP_position[1] = localDW->obj_e.MissionParams[1].position[1];
  rty_FromWP_position[2] = localDW->obj_e.MissionParams[1].position[2];

  /* SignalConversion generated from: '<Root>/FromWP' incorporates:
   *  MATLABSystem: '<S6>/PathManagerSystemObject'
   * */
  rty_FromWP_params[0] = localDW->obj_e.MissionParams[1].params[0];
  rty_FromWP_params[1] = localDW->obj_e.MissionParams[1].params[1];
  rty_FromWP_params[2] = localDW->obj_e.MissionParams[1].params[2];
  rty_FromWP_params[3] = localDW->obj_e.MissionParams[1].params[3];

  /* SignalConversion generated from: '<Root>/ToWP' incorporates:
   *  MATLABSystem: '<S6>/PathManagerSystemObject'
   * */
  *rty_ToWP_mode = localDW->obj_e.MissionParams[0].mode;

  /* SignalConversion generated from: '<Root>/ToWP' incorporates:
   *  MATLABSystem: '<S6>/PathManagerSystemObject'
   * */
  rty_ToWP_position[0] = localDW->obj_e.MissionParams[0].position[0];
  rty_ToWP_position[1] = localDW->obj_e.MissionParams[0].position[1];
  rty_ToWP_position[2] = localDW->obj_e.MissionParams[0].position[2];

  /* SignalConversion generated from: '<Root>/ToWP' incorporates:
   *  MATLABSystem: '<S6>/PathManagerSystemObject'
   * */
  rty_ToWP_params[0] = localDW->obj_e.MissionParams[0].params[0];
  rty_ToWP_params[1] = localDW->obj_e.MissionParams[0].params[1];
  rty_ToWP_params[2] = localDW->obj_e.MissionParams[0].params[2];
  rty_ToWP_params[3] = localDW->obj_e.MissionParams[0].params[3];

  /* InitialCondition: '<S3>/IC' incorporates:
   *  MATLABSystem: '<S6>/PathManagerSystemObject'
   * */
  if (localDW->IC_FirstOutputTime) {
    localDW->IC_FirstOutputTime = false;
    *rty_mode = 1U;
  } else {
    *rty_mode = localDW->obj_e.MissionParams[0].mode;
  }

  /* End of InitialCondition: '<S3>/IC' */

  /* MATLABSystem: '<S2>/AHRS' */
  if (localDW->obj.AccelerometerNoise != 0.0001924722) {
    if (localDW->obj.isInitialized == 1) {
      localDW->obj.TunablePropsChanged = true;
    }

    localDW->obj.AccelerometerNoise = 0.0001924722;
  }

  if (localDW->obj.GyroscopeNoise != 9.1385E-5) {
    if (localDW->obj.isInitialized == 1) {
      localDW->obj.TunablePropsChanged = true;
    }

    localDW->obj.GyroscopeNoise = 9.1385E-5;
  }

  if (localDW->obj.MagnetometerNoise != 0.1) {
    if (localDW->obj.isInitialized == 1) {
      localDW->obj.TunablePropsChanged = true;
    }

    localDW->obj.MagnetometerNoise = 0.1;
  }

  if (localDW->obj.GyroscopeDriftNoise != 3.0462E-13) {
    if (localDW->obj.isInitialized == 1) {
      localDW->obj.TunablePropsChanged = true;
    }

    localDW->obj.GyroscopeDriftNoise = 3.0462E-13;
  }

  if (localDW->obj.LinearAccelerationNoise != 0.0096236100000000012) {
    if (localDW->obj.isInitialized == 1) {
      localDW->obj.TunablePropsChanged = true;
    }

    localDW->obj.LinearAccelerationNoise = 0.0096236100000000012;
  }

  if (localDW->obj.MagneticDisturbanceNoise != 0.5) {
    if (localDW->obj.isInitialized == 1) {
      localDW->obj.TunablePropsChanged = true;
    }

    localDW->obj.MagneticDisturbanceNoise = 0.5;
  }

  if (localDW->obj.LinearAccelerationDecayFactor != 0.5) {
    if (localDW->obj.isInitialized == 1) {
      localDW->obj.TunablePropsChanged = true;
    }

    localDW->obj.LinearAccelerationDecayFactor = 0.5;
  }

  if (localDW->obj.MagneticDisturbanceDecayFactor != 0.5) {
    if (localDW->obj.isInitialized == 1) {
      localDW->obj.TunablePropsChanged = true;
    }

    localDW->obj.MagneticDisturbanceDecayFactor = 0.5;
  }

  if (localDW->obj.ExpectedMagneticFieldStrength != 50.0) {
    if (localDW->obj.isInitialized == 1) {
      localDW->obj.TunablePropsChanged = true;
    }

    localDW->obj.ExpectedMagneticFieldStrength = 50.0;
  }

  UAM_GuidanceAnd_SystemCore_step(&localDW->obj, rtu_sensorData_Abb,
    rtu_sensorData_Gyro, rtu_sensorData_Mag, tmp, rty_States_pqr);

  /* MATLABSystem: '<S2>/Coordinate Transformation Conversion1' incorporates:
   *  MATLABSystem: '<S2>/AHRS'
   */
  tmp_3 = _mm_set1_pd(1.0 / sqrt(((tmp[0] * tmp[0] + tmp[1] * tmp[1]) + tmp[2] *
    tmp[2]) + tmp[3] * tmp[3]));

  /* MATLABSystem: '<S2>/AHRS' incorporates:
   *  MATLABSystem: '<S2>/Coordinate Transformation Conversion1'
   * */
  tmp_1 = _mm_mul_pd(_mm_loadu_pd(&tmp[0]), tmp_3);
  _mm_storeu_pd(&tmp_2[0], tmp_1);

  /* MATLABSystem: '<S2>/Coordinate Transformation Conversion1' */
  rtb_Sum1_idx_0 = tmp_2[0];
  rtb_Gain1 = tmp_2[1];

  /* MATLABSystem: '<S2>/AHRS' incorporates:
   *  MATLABSystem: '<S2>/Coordinate Transformation Conversion1'
   * */
  tmp_3 = _mm_mul_pd(_mm_loadu_pd(&tmp[2]), tmp_3);
  _mm_storeu_pd(&tmp_2[0], tmp_3);

  /* MATLABSystem: '<S2>/Coordinate Transformation Conversion1' */
  Merge = rtb_Gain1 * tmp_2[1] * 2.0 - rtb_Sum1_idx_0 * tmp_2[0] * 2.0;
  if (Merge > 1.0) {
    Merge = 1.0;
  }

  rtb_Atan2 = Merge;
  if (Merge < -1.0) {
    rtb_Atan2 = -1.0;
  }

  if ((rtb_Atan2 < 0.0) && (fabs(rtb_Atan2 + 1.0) < 2.2204460492503131E-15)) {
    Merge = -2.0 * rt_atan2d_snf(rtb_Gain1, rtb_Sum1_idx_0);

    /* RateTransition generated from: '<S2>/Rate Transition1' */
    rty_States_Euler[0] = 0.0;
    rtb_Sum1_idx_0 = 1.5707963267948966;
  } else if ((rtb_Atan2 > 0.0) && (fabs(rtb_Atan2 - 1.0) <
              2.2204460492503131E-15)) {
    Merge = 2.0 * rt_atan2d_snf(rtb_Gain1, rtb_Sum1_idx_0);

    /* RateTransition generated from: '<S2>/Rate Transition1' */
    rty_States_Euler[0] = 0.0;
    rtb_Sum1_idx_0 = -1.5707963267948966;
  } else {
    Merge_a = rtb_Sum1_idx_0 * rtb_Sum1_idx_0 * 2.0 - 1.0;
    Merge = rt_atan2d_snf(rtb_Sum1_idx_0 * tmp_2[1] * 2.0 + rtb_Gain1 * tmp_2[0]
                          * 2.0, rtb_Gain1 * rtb_Gain1 * 2.0 + Merge_a);

    /* RateTransition generated from: '<S2>/Rate Transition1' */
    rty_States_Euler[0] = rt_atan2d_snf(rtb_Sum1_idx_0 * rtb_Gain1 * 2.0 +
      tmp_2[0] * tmp_2[1] * 2.0, tmp_2[1] * tmp_2[1] * 2.0 + Merge_a);
    rtb_Sum1_idx_0 = -asin(rtb_Atan2);
  }

  /* RateTransition generated from: '<S2>/Rate Transition1' incorporates:
   *  Gain: '<S8>/Gain1'
   */
  *rty_States_course = 0.017453292519943295 * *rtu_sensorData_GPSCourse;

  /* RateTransition generated from: '<S2>/Rate Transition1' incorporates:
   *  MATLABSystem: '<S2>/Coordinate Transformation Conversion1'
   * */
  rty_States_Euler[1] = rtb_Sum1_idx_0;
  rty_States_Euler[2] = Merge;

  /* RateTransition generated from: '<S2>/Rate Transition1' */
  *rty_States_c1 = *rtu_sensorData_RotorParameters_;
}

/* Model initialize function */
void UAM_GuidanceAndNavig_initialize(const char_T **rt_errorStatus,
  RT_MODEL_UAM_GuidanceAndNavig_T *const UAM_GuidanceAndNavigation_M)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize error status */
  rtmSetErrorStatusPointer(UAM_GuidanceAndNavigation_M, rt_errorStatus);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
