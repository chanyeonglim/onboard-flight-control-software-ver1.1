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
 * C/C++ source code generated on : Fri Apr 25 10:33:43 2025
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
#include "rtGetNaN.h"
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  fusion_simulink_ahrsfilter_UA_T obj; /* '<S2>/AHRS' */
  uav_sluav_internal_system_Pat_T obj_e;/* '<S6>/PathManagerSystemObject' */
  boolean_T IC_FirstOutputTime;        /* '<S3>/IC' */
} DW_UAM_GuidanceAndNavigation_T;

/* Invariant block signals (default storage) */
typedef struct {
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
} ConstB_UAM_GuidanceAndNavigat_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  sensorData sensorData_e;             /* '<Root>/sensorData' */
  UAVPathManagerBus Globalmission[9];  /* '<Root>/Global mission' */
  boolean_T ModeDone;                  /* '<Root>/ModeDone' */
} ExtU_UAM_GuidanceAndNavigatio_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  uint8_T mode;                        /* '<Root>/mode' */
  UAVPathManagerBus ToWP;              /* '<Root>/ToWP' */
  UAVPathManagerBus FromWP;            /* '<Root>/FromWP' */
  real_T pose[4];                      /* '<Root>/pose' */
  GuidanceStates States;               /* '<Root>/States' */
  real_T Ground_a;                     /* '<Root>/Ground' */
} ExtY_UAM_GuidanceAndNavigatio_T;

/* Real-time Model Data Structure */
struct tag_RTM_UAM_GuidanceAndNaviga_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_UAM_GuidanceAndNavigation_T UAM_GuidanceAndNavigation_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_UAM_GuidanceAndNavigatio_T UAM_GuidanceAndNavigation_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_UAM_GuidanceAndNavigatio_T UAM_GuidanceAndNavigation_Y;
extern const ConstB_UAM_GuidanceAndNavigat_T UAM_GuidanceAndNavigatio_ConstB;/* constant block i/o */

/* Model entry point functions */
extern void UAM_GuidanceAndNavigation_initialize(void);
extern void UAM_GuidanceAndNavigation_step(void);
extern void UAM_GuidanceAndNavigation_terminate(void);

/* Real-time Model object */
extern RT_MODEL_UAM_GuidanceAndNavig_T *const UAM_GuidanceAndNavigation_M;

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
 * Block '<S3>/Rate Transition' : Eliminated since input and output rates are identical
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
