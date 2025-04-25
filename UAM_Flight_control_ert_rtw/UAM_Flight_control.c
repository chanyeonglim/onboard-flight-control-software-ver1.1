/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: UAM_Flight_control.c
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

#include "UAM_Flight_control.h"
#include "rtwtypes.h"
#include "UAM_Status.h"
#include "UAM_GuidanceAndNavigation.h"
#include "UAM_FlightMode.h"
#include "UAM_GCS.h"

const real_T UAM_Flight_control_RGND = 0.0;/* real_T ground */

/* Block signals (default storage) */
B_UAM_Flight_control_T UAM_Flight_control_B;

/* Block states (default storage) */
DW_UAM_Flight_control_T UAM_Flight_control_DW;

/* Real-time model */
static RT_MODEL_UAM_Flight_control_T UAM_Flight_control_M_;
RT_MODEL_UAM_Flight_control_T *const UAM_Flight_control_M =
  &UAM_Flight_control_M_;

/* Model step function for TID0 */
void UAM_Flight_control_step0(void)    /* Sample time: [0.005s, 0.0s] */
{
  /* Update the flag to indicate when data transfers from
   *  Sample time: [0.005s, 0.0s] to Sample time: [0.08s, 0.0s]  */
  UAM_Flight_control_M->Timing.RateInteraction.b_TID0_1 =
    (UAM_Flight_control_M->Timing.RateInteraction.TID0_1 == 0);
  (UAM_Flight_control_M->Timing.RateInteraction.TID0_1)++;
  if ((UAM_Flight_control_M->Timing.RateInteraction.TID0_1) > 15) {
    UAM_Flight_control_M->Timing.RateInteraction.TID0_1 = 0;
  }

  /* RateTransition: '<Root>/Rate Transition' incorporates:
   *  RateTransition generated from: '<Root>/Model'
   */
  UAM_Flight_control_B.b = (UAM_Flight_control_M->Timing.RateInteraction.TID0_1 ==
    1);
  if (UAM_Flight_control_B.b) {
    /* RateTransition: '<Root>/Rate Transition' */
    UAM_Flight_control_B.RateTransition =
      UAM_Flight_control_DW.RateTransition_Buffer0;
  }

  /* End of RateTransition: '<Root>/Rate Transition' */

  /* DataTypeConversion: '<Root>/Data Type Conversion' */
  UAM_Flight_control_B.DataTypeConversion = (UAM_Flight_control_B.RateTransition
    != 0);

  /* ModelReference generated from: '<Root>/UAM_Status' */
  UAM_Status(((const real_T*) &UAM_Flight_control_RGND), ((const real_T*)
              &UAM_Flight_control_RGND), ((const real_T*)
              &UAM_Flight_control_RGND), ((const real_T*)
              &UAM_Flight_control_RGND), ((const real_T*)
              &UAM_Flight_control_RGND), ((const real_T*)
              &UAM_Flight_control_RGND), &UAM_Flight_control_B.GPSCourse,
             &UAM_Flight_control_B.gndspeed, &UAM_Flight_control_B.DiffPress,
             &UAM_Flight_control_B.rho, &UAM_Flight_control_B.LLA[0],
             &UAM_Flight_control_B.Abb[0], &UAM_Flight_control_B.Gyro[0],
             &UAM_Flight_control_B.Mag[0], &UAM_Flight_control_B.GPSVelocity[0],
             &UAM_Flight_control_B.Abe[0], &UAM_Flight_control_B.w1,
             &UAM_Flight_control_B.w2, &UAM_Flight_control_B.w3,
             &UAM_Flight_control_B.w4, &UAM_Flight_control_B.c1_m,
             &UAM_Flight_control_B.c2);

  /* ModelReference: '<Root>/UAM_GCS' */
  UAM_GCSTID0(&UAM_Flight_control_B.UAM_GCS_l[0],
              &(UAM_Flight_control_DW.UAM_GCS_InstanceData.rtdw));

  /* ModelReference generated from: '<Root>/UAM_GuidanceAndNavigation' */
  UAM_GuidanceAndNavigation(&UAM_Flight_control_B.GPSCourse,
    &UAM_Flight_control_B.LLA[0], &UAM_Flight_control_B.Abb[0],
    &UAM_Flight_control_B.Gyro[0], &UAM_Flight_control_B.Mag[0],
    &UAM_Flight_control_B.GPSVelocity[0], &UAM_Flight_control_B.c1_m,
    &UAM_Flight_control_B.UAM_GCS_l[0], &UAM_Flight_control_B.DataTypeConversion,
    &UAM_Flight_control_B.UAM_GuidanceAndNavigation_o1,
    &UAM_Flight_control_B.mode_h, &UAM_Flight_control_B.position[0],
    &UAM_Flight_control_B.params[0], &UAM_Flight_control_B.mode_b,
    &UAM_Flight_control_B.position_i[0], &UAM_Flight_control_B.params_a[0],
    &UAM_Flight_control_B.UAM_GuidanceAndNavigation_o[0],
    &UAM_Flight_control_B.Euler[0], &UAM_Flight_control_B.pqr[0],
    &UAM_Flight_control_B.Ve[0], &UAM_Flight_control_B.Xe[0],
    &UAM_Flight_control_B.course, &UAM_Flight_control_B.c1,
    &(UAM_Flight_control_DW.UAM_GuidanceAndNavigation_Insta.rtdw));

  /* RateTransition generated from: '<Root>/Model' incorporates:
   *  ModelReference generated from: '<Root>/UAM_GuidanceAndNavigation'
   */
  if (UAM_Flight_control_B.b) {
    UAM_Flight_control_DW.params_Buffer[0] = UAM_Flight_control_B.params[0];
    UAM_Flight_control_DW.params_Buffer[1] = UAM_Flight_control_B.params[1];
    UAM_Flight_control_DW.params_Buffer[2] = UAM_Flight_control_B.params[2];
    UAM_Flight_control_DW.params_Buffer[3] = UAM_Flight_control_B.params[3];
    UAM_Flight_control_DW.params_Buffer_a[0] = UAM_Flight_control_B.params_a[0];
    UAM_Flight_control_DW.params_Buffer_a[1] = UAM_Flight_control_B.params_a[1];
    UAM_Flight_control_DW.params_Buffer_a[2] = UAM_Flight_control_B.params_a[2];
    UAM_Flight_control_DW.params_Buffer_a[3] = UAM_Flight_control_B.params_a[3];
    UAM_Flight_control_DW.TmpRTBAtModelInport4_Buffer[0] =
      UAM_Flight_control_B.UAM_GuidanceAndNavigation_o[0];
    UAM_Flight_control_DW.TmpRTBAtModelInport4_Buffer[1] =
      UAM_Flight_control_B.UAM_GuidanceAndNavigation_o[1];
    UAM_Flight_control_DW.TmpRTBAtModelInport4_Buffer[2] =
      UAM_Flight_control_B.UAM_GuidanceAndNavigation_o[2];
    UAM_Flight_control_DW.TmpRTBAtModelInport4_Buffer[3] =
      UAM_Flight_control_B.UAM_GuidanceAndNavigation_o[3];
    UAM_Flight_control_DW.position_Buffer[0] = UAM_Flight_control_B.position[0];
    UAM_Flight_control_DW.position_Buffer[1] = UAM_Flight_control_B.position[1];
    UAM_Flight_control_DW.position_Buffer[2] = UAM_Flight_control_B.position[2];
    UAM_Flight_control_DW.position_Buffer_b[0] =
      UAM_Flight_control_B.position_i[0];
    UAM_Flight_control_DW.position_Buffer_b[1] =
      UAM_Flight_control_B.position_i[1];
    UAM_Flight_control_DW.position_Buffer_b[2] =
      UAM_Flight_control_B.position_i[2];
    UAM_Flight_control_DW.Euler_Buffer[0] = UAM_Flight_control_B.Euler[0];
    UAM_Flight_control_DW.Euler_Buffer[1] = UAM_Flight_control_B.Euler[1];
    UAM_Flight_control_DW.Euler_Buffer[2] = UAM_Flight_control_B.Euler[2];
    UAM_Flight_control_DW.Ve_Buffer[0] = UAM_Flight_control_B.Ve[0];
    UAM_Flight_control_DW.Ve_Buffer[1] = UAM_Flight_control_B.Ve[1];
    UAM_Flight_control_DW.Ve_Buffer[2] = UAM_Flight_control_B.Ve[2];
    UAM_Flight_control_DW.Xe_Buffer[0] = UAM_Flight_control_B.Xe[0];
    UAM_Flight_control_DW.Xe_Buffer[1] = UAM_Flight_control_B.Xe[1];
    UAM_Flight_control_DW.Xe_Buffer[2] = UAM_Flight_control_B.Xe[2];
    UAM_Flight_control_DW.pqr_Buffer[0] = UAM_Flight_control_B.pqr[0];
    UAM_Flight_control_DW.pqr_Buffer[1] = UAM_Flight_control_B.pqr[1];
    UAM_Flight_control_DW.pqr_Buffer[2] = UAM_Flight_control_B.pqr[2];
    UAM_Flight_control_DW.mode_Buffer = UAM_Flight_control_B.mode_h;
    UAM_Flight_control_DW.mode_Buffer_k = UAM_Flight_control_B.mode_b;
    UAM_Flight_control_DW.c1_Buffer = UAM_Flight_control_B.c1;
    UAM_Flight_control_DW.course_Buffer = UAM_Flight_control_B.course;
    UAM_Flight_control_DW.TmpRTBAtModelInport1_Buffer =
      UAM_Flight_control_B.UAM_GuidanceAndNavigation_o1;
  }
}

/* Model step function for TID1 */
void UAM_Flight_control_step1(void)    /* Sample time: [0.08s, 0.0s] */
{
  /* local block i/o variables */
  real_T rtb_airspeed;
  real_T rtb_altitude;
  real_T rtb_course_k;
  real_T rtb_L1;
  real_T rtb_climbrate;
  real_T rtb_roll;
  real_T rtb_pitch;
  real_T rtb_yaw;
  real_T rtb_airspeed_b;
  uint8_T rtb_lateralGuidance;
  uint8_T rtb_airspeedAltitude;
  uint8_T rtb_attitude;
  uint8_T rtb_manual;
  uint8_T rtb_armed;
  uint8_T rtb_inTransition;
  uint8_T rtb_TransitionCondition;
  uint8_T rtb_Model_o22;

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Mod_d.mode =
    UAM_Flight_control_DW.mode_Buffer;

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Mod_d.position[0] =
    UAM_Flight_control_DW.position_Buffer[0];
  UAM_Flight_control_B.BusConversion_InsertedFor_Mod_d.position[1] =
    UAM_Flight_control_DW.position_Buffer[1];
  UAM_Flight_control_B.BusConversion_InsertedFor_Mod_d.position[2] =
    UAM_Flight_control_DW.position_Buffer[2];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.TmpRTBAtModelInport1 =
    UAM_Flight_control_DW.TmpRTBAtModelInport1_Buffer;

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Mod_d.params[0] =
    UAM_Flight_control_DW.params_Buffer[0];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.TmpRTBAtModelInport4[0] =
    UAM_Flight_control_DW.TmpRTBAtModelInport4_Buffer[0];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Mod_d.params[1] =
    UAM_Flight_control_DW.params_Buffer[1];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.TmpRTBAtModelInport4[1] =
    UAM_Flight_control_DW.TmpRTBAtModelInport4_Buffer[1];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Mod_d.params[2] =
    UAM_Flight_control_DW.params_Buffer[2];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.TmpRTBAtModelInport4[2] =
    UAM_Flight_control_DW.TmpRTBAtModelInport4_Buffer[2];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Mod_d.params[3] =
    UAM_Flight_control_DW.params_Buffer[3];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.TmpRTBAtModelInport4[3] =
    UAM_Flight_control_DW.TmpRTBAtModelInport4_Buffer[3];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Mod_m.mode =
    UAM_Flight_control_DW.mode_Buffer_k;

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Mod_m.position[0] =
    UAM_Flight_control_DW.position_Buffer_b[0];
  UAM_Flight_control_B.BusConversion_InsertedFor_Mod_m.position[1] =
    UAM_Flight_control_DW.position_Buffer_b[1];
  UAM_Flight_control_B.BusConversion_InsertedFor_Mod_m.position[2] =
    UAM_Flight_control_DW.position_Buffer_b[2];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Mod_m.params[0] =
    UAM_Flight_control_DW.params_Buffer_a[0];
  UAM_Flight_control_B.BusConversion_InsertedFor_Mod_m.params[1] =
    UAM_Flight_control_DW.params_Buffer_a[1];
  UAM_Flight_control_B.BusConversion_InsertedFor_Mod_m.params[2] =
    UAM_Flight_control_DW.params_Buffer_a[2];
  UAM_Flight_control_B.BusConversion_InsertedFor_Mod_m.params[3] =
    UAM_Flight_control_DW.params_Buffer_a[3];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Model.Euler[0] =
    UAM_Flight_control_DW.Euler_Buffer[0];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Model.pqr[0] =
    UAM_Flight_control_DW.pqr_Buffer[0];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Model.Ve[0] =
    UAM_Flight_control_DW.Ve_Buffer[0];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Model.Xe[0] =
    UAM_Flight_control_DW.Xe_Buffer[0];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Model.Euler[1] =
    UAM_Flight_control_DW.Euler_Buffer[1];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Model.pqr[1] =
    UAM_Flight_control_DW.pqr_Buffer[1];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Model.Ve[1] =
    UAM_Flight_control_DW.Ve_Buffer[1];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Model.Xe[1] =
    UAM_Flight_control_DW.Xe_Buffer[1];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Model.Euler[2] =
    UAM_Flight_control_DW.Euler_Buffer[2];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Model.pqr[2] =
    UAM_Flight_control_DW.pqr_Buffer[2];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Model.Ve[2] =
    UAM_Flight_control_DW.Ve_Buffer[2];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Model.Xe[2] =
    UAM_Flight_control_DW.Xe_Buffer[2];

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Model.course =
    UAM_Flight_control_DW.course_Buffer;

  /* RateTransition generated from: '<Root>/Model' */
  UAM_Flight_control_B.BusConversion_InsertedFor_Model.c1 =
    UAM_Flight_control_DW.c1_Buffer;

  /* ModelReference generated from: '<Root>/Model' */
  UAM_FlightMode(&(UAM_Flight_control_DW.Model_InstanceData.rtm),
                 &UAM_Flight_control_B.TmpRTBAtModelInport1,
                 &UAM_Flight_control_B.BusConversion_InsertedFor_Mod_d,
                 &UAM_Flight_control_B.BusConversion_InsertedFor_Mod_m,
                 &UAM_Flight_control_B.TmpRTBAtModelInport4[0],
                 &UAM_Flight_control_B.BusConversion_InsertedFor_Model,
                 &UAM_Flight_control_B.UAM_GuidanceAndNavigation_o15,
                 &UAM_Flight_control_B.X, &UAM_Flight_control_B.Y,
                 &UAM_Flight_control_B.Z, &UAM_Flight_control_B.Yaw,
                 &rtb_airspeed, &rtb_altitude, &rtb_course_k, &rtb_L1,
                 &rtb_climbrate, &rtb_roll, &rtb_pitch, &rtb_yaw,
                 &rtb_airspeed_b, &rtb_lateralGuidance, &rtb_airspeedAltitude,
                 &rtb_attitude, &rtb_manual, &rtb_armed, &rtb_inTransition,
                 &rtb_TransitionCondition, &UAM_Flight_control_B.Model_o21,
                 &rtb_Model_o22, &(UAM_Flight_control_DW.Model_InstanceData.rtb),
                 &(UAM_Flight_control_DW.Model_InstanceData.rtdw));

  /* RateTransition: '<Root>/Rate Transition' */
  UAM_Flight_control_DW.RateTransition_Buffer0 = rtb_Model_o22;

  /* ModelReference: '<Root>/UAM_GCS' */
  UAM_GCSTID1(&(UAM_Flight_control_DW.UAM_GCS_InstanceData.rtdw));
}

/* Model initialize function */
void UAM_Flight_control_initialize(void)
{
  /* Registration code */

  /* Set task counter limit used by the static main program */
  (UAM_Flight_control_M)->Timing.TaskCounters.cLimit[0] = 1;
  (UAM_Flight_control_M)->Timing.TaskCounters.cLimit[1] = 16;

  /* Model Initialize function for ModelReference Block: '<Root>/UAM_GCS' */
  UAM_GCS_initialize(rtmGetErrorStatusPointer(UAM_Flight_control_M),
                     &(UAM_Flight_control_DW.UAM_GCS_InstanceData.rtm));

  /* Model Initialize function for ModelReference Block: '<Root>/Model' */
  UAM_FlightMode_initialize(rtmGetErrorStatusPointer(UAM_Flight_control_M),
    rtmGetStopRequestedPtr(UAM_Flight_control_M),
    &(UAM_Flight_control_DW.Model_InstanceData.rtm),
    &(UAM_Flight_control_DW.Model_InstanceData.rtzce));

  /* Model Initialize function for ModelReference Block: '<Root>/UAM_GuidanceAndNavigation' */
  UAM_GuidanceAndNavig_initialize(rtmGetErrorStatusPointer(UAM_Flight_control_M),
    &(UAM_Flight_control_DW.UAM_GuidanceAndNavigation_Insta.rtm));

  /* Model Initialize function for ModelReference Block: '<Root>/UAM_Status' */
  UAM_Status_initialize(rtmGetErrorStatusPointer(UAM_Flight_control_M),
                        &(UAM_Flight_control_DW.UAM_Status_InstanceData.rtm));

  /* SystemInitialize for ModelReference generated from: '<Root>/Model' */
  UAM_FlightMode_Init(&(UAM_Flight_control_DW.Model_InstanceData.rtdw));

  /* SystemInitialize for ModelReference generated from: '<Root>/UAM_GuidanceAndNavigation' */
  UAM_GuidanceAndNavigation_Init
    (&UAM_Flight_control_B.UAM_GuidanceAndNavigation_o1,
     &(UAM_Flight_control_DW.UAM_GuidanceAndNavigation_Insta.rtdw));
}

/* Model terminate function */
void UAM_Flight_control_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
