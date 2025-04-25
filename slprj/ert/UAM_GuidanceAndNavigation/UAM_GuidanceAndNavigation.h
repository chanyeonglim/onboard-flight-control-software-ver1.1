/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_GuidanceAndNavigation.h
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

#ifndef UAM_GuidanceAndNavigation_h_
#define UAM_GuidanceAndNavigation_h_
#ifndef UAM_GuidanceAndNavigation_COMMON_INCLUDES_
#define UAM_GuidanceAndNavigation_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                          /* UAM_GuidanceAndNavigation_COMMON_INCLUDES_ */

#include "UAM_GuidanceAndNavigation_types.h"
#include "rt_nonfinite.h"
#include "rtGetNaN.h"

/* Block states (default storage) for model 'UAM_GuidanceAndNavigation' */
typedef struct {
  fusion_simulink_ahrsfilter_UA_T obj; /* '<S2>/AHRS' */
  uav_sluav_internal_system_Pat_T obj_e;/* '<S6>/PathManagerSystemObject' */
  robotics_slcore_internal_bloc_T obj_a;
                              /* '<S2>/Coordinate Transformation Conversion2' */
  robotics_slcore_internal_bloc_T obj_p;
                              /* '<S2>/Coordinate Transformation Conversion1' */
  boolean_T IC_FirstOutputTime;        /* '<S3>/IC' */
  boolean_T objisempty;       /* '<S2>/Coordinate Transformation Conversion2' */
  boolean_T objisempty_f;     /* '<S2>/Coordinate Transformation Conversion1' */
  boolean_T objisempty_b;              /* '<S2>/AHRS' */
  boolean_T objisempty_o;              /* '<S6>/PathManagerSystemObject' */
} DW_UAM_GuidanceAndNavigatio_f_T;

/* Invariant block signals for model 'UAM_GuidanceAndNavigation' */
typedef struct {
  const real_T Ground;                 /* '<S1>/Ground' */
  const real_T Bias;                   /* '<S28>/Bias' */
  const real_T MathFunction1;          /* '<S28>/Math Function1' */
  const real_T Bias1;                  /* '<S28>/Bias1' */
  const real_T Abs;                    /* '<S28>/Abs' */
  const real_T Switch;                 /* '<S28>/Switch' */
  const real_T Abs1;                   /* '<S24>/Abs1' */
  const real_T Bias_o;                 /* '<S24>/Bias' */
  const real_T Gain;                   /* '<S24>/Gain' */
  const real_T Bias1_l;                /* '<S24>/Bias1' */
  const real_T Abs_o;                  /* '<S27>/Abs' */
  const real_T Switch1;                /* '<S12>/Switch1' */
  const real_T Sum;                    /* '<S12>/Sum' */
  const real_T Abs_g;                  /* '<S25>/Abs' */
  const real_T Bias_ok;                /* '<S25>/Bias' */
  const real_T MathFunction1_h;        /* '<S25>/Math Function1' */
  const real_T Bias1_o;                /* '<S25>/Bias1' */
  const real_T Switch_o;               /* '<S25>/Switch' */
  const real_T UnitConversion;         /* '<S33>/Unit Conversion' */
  const real_T Sum_j;                  /* '<S37>/Sum' */
  const real_T Product1;               /* '<S38>/Product1' */
  const real_T Sum1;                   /* '<S38>/Sum1' */
  const real_T sqrt_g;                 /* '<S38>/sqrt' */
  const real_T Product2;               /* '<S34>/Product2' */
  const real_T Sum1_l;                 /* '<S34>/Sum1' */
  const real_T SinCos_o1;              /* '<S13>/SinCos' */
  const real_T SinCos_o2;              /* '<S13>/SinCos' */
  const boolean_T Compare;             /* '<S31>/Compare' */
  const boolean_T Compare_p;           /* '<S26>/Compare' */
  const boolean_T Compare_l;           /* '<S32>/Compare' */
} ConstB_UAM_GuidanceAndNavig_h_T;

/* Real-time Model Data Structure */
struct tag_RTM_UAM_GuidanceAndNaviga_T {
  const char_T **errorStatus;
};

typedef struct {
  DW_UAM_GuidanceAndNavigatio_f_T rtdw;
  RT_MODEL_UAM_GuidanceAndNavig_T rtm;
} MdlrefDW_UAM_GuidanceAndNavig_T;

/* Model reference registration function */
extern void UAM_GuidanceAndNavig_initialize(const char_T **rt_errorStatus,
  RT_MODEL_UAM_GuidanceAndNavig_T *const UAM_GuidanceAndNavigation_M);
extern void UAM_GuidanceA_IfActionSubsystem(real_T rtu_u, real_T rtu_absu,
  real_T *rty_signu);
extern void UAM_Guidance_IfActionSubsystem1(real_T *rty_zero);
extern void UAM_GuidanceAndNavigation_Init(uint8_T *rty_mode,
  DW_UAM_GuidanceAndNavigatio_f_T *localDW);
extern void UAM_GuidanceAndNavigation(const real_T *rtu_sensorData_GPSCourse,
  const real_T rtu_sensorData_LLA[3], const real_T rtu_sensorData_Abb[3], const
  real_T rtu_sensorData_Gyro[3], const real_T rtu_sensorData_Mag[3], const
  real_T rtu_sensorData_GPSVelocity[3], const real_T
  *rtu_sensorData_RotorParameters_, const UAVPathManagerBus rtu_Globalmission[9],
  const boolean_T *rtu_ModeDone, uint8_T *rty_mode, uint8_T *rty_ToWP_mode,
  real_T rty_ToWP_position[3], real_T rty_ToWP_params[4], uint8_T
  *rty_FromWP_mode, real_T rty_FromWP_position[3], real_T rty_FromWP_params[4],
  real_T rty_pose[4], real_T rty_States_Euler[3], real_T rty_States_pqr[3],
  real_T rty_States_Ve[3], real_T rty_States_Xe[3], real_T *rty_States_course,
  real_T *rty_States_c1, DW_UAM_GuidanceAndNavigatio_f_T *localDW);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S7>/Constant' : Unused code path elimination
 * Block '<S7>/Divide' : Unused code path elimination
 * Block '<S7>/Product' : Unused code path elimination
 * Block '<S7>/Sqrt' : Unused code path elimination
 * Block '<S10>/Data Type Duplicate' : Unused code path elimination
 * Block '<S2>/Matrix Multiply1' : Unused code path elimination
 * Block '<S2>/Transpose' : Unused code path elimination
 * Block '<S2>/Transpose5' : Unused code path elimination
 * Block '<S2>/Transpose6' : Unused code path elimination
 * Block '<S2>/Gain' : Eliminated nontunable gain of 1
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
 * '<Root>' : 'UAM_GuidanceAndNavigation'
 * '<S1>'   : 'UAM_GuidanceAndNavigation/Guidance'
 * '<S2>'   : 'UAM_GuidanceAndNavigation/Navigation'
 * '<S3>'   : 'UAM_GuidanceAndNavigation/Guidance/Subsystem'
 * '<S4>'   : 'UAM_GuidanceAndNavigation/Guidance/UAV POS'
 * '<S5>'   : 'UAM_GuidanceAndNavigation/Guidance/UAV States'
 * '<S6>'   : 'UAM_GuidanceAndNavigation/Guidance/Subsystem/Path Manager'
 * '<S7>'   : 'UAM_GuidanceAndNavigation/Navigation/Airspeed Filter'
 * '<S8>'   : 'UAM_GuidanceAndNavigation/Navigation/Degrees to Radians'
 * '<S9>'   : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local'
 * '<S10>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1'
 * '<S11>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap'
 * '<S12>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap LL0'
 * '<S13>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/Subsystem'
 * '<S14>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/pos_rad'
 * '<S15>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap/Latitude Wrap 90'
 * '<S16>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap/Wrap Longitude'
 * '<S17>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap/Latitude Wrap 90/Compare To Constant'
 * '<S18>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap/Latitude Wrap 90/Sign1 Or Zero'
 * '<S19>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap/Latitude Wrap 90/Wrap Angle 180'
 * '<S20>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap/Latitude Wrap 90/Sign1 Or Zero/If Action Subsystem'
 * '<S21>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap/Latitude Wrap 90/Sign1 Or Zero/If Action Subsystem1'
 * '<S22>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap/Latitude Wrap 90/Wrap Angle 180/Compare To Constant'
 * '<S23>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap/Wrap Longitude/Compare To Constant'
 * '<S24>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap LL0/Latitude Wrap 90'
 * '<S25>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap LL0/Wrap Longitude'
 * '<S26>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap LL0/Latitude Wrap 90/Compare To Constant'
 * '<S27>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap LL0/Latitude Wrap 90/Sign1 Or Zero'
 * '<S28>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap LL0/Latitude Wrap 90/Wrap Angle 180'
 * '<S29>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap LL0/Latitude Wrap 90/Sign1 Or Zero/If Action Subsystem'
 * '<S30>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap LL0/Latitude Wrap 90/Sign1 Or Zero/If Action Subsystem1'
 * '<S31>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap LL0/Latitude Wrap 90/Wrap Angle 180/Compare To Constant'
 * '<S32>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/LatLong wrap LL0/Wrap Longitude/Compare To Constant'
 * '<S33>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/Subsystem/Angle Conversion2'
 * '<S34>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/Subsystem/Find Rn and Rm'
 * '<S35>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/Subsystem/Find Rn and Rm/Angle Conversion2'
 * '<S36>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/Subsystem/Find Rn and Rm/denom'
 * '<S37>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/Subsystem/Find Rn and Rm/e'
 * '<S38>'  : 'UAM_GuidanceAndNavigation/Navigation/LLA2Local/LLA to Flat Earth1/Subsystem/Find Rn and Rm/e^4'
 */
#endif                                 /* UAM_GuidanceAndNavigation_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
