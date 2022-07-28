//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: AttControl.cpp
//
// Code generated for Simulink model 'AttControl'.
//
// Model version                  : 1.15
// Simulink Coder version         : 9.6 (R2021b) 14-May-2021
// C/C++ source code generated on : Thu Jul 21 14:56:47 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "AttControl.h"
#include "AttControl_private.h"

//
// Output and update for action system:
//    '<S1>/if_action'
//    '<S2>/if_action'
//
void att_controllerModelClass::AttControl_if_action(real_T rtu_q_0, const real_T
  rtu_q_bar[3], real_T rty_theta_bar[3])
{
  real_T rtb_Acos;
  real_T rtb_Switch_idx_0;
  real_T rtb_Switch_idx_1;
  real_T rtb_Switch_idx_2;
  real_T rtb_Switch_idx_3;

  // Sum: '<S5>/Sum of Elements' incorporates:
  //   Math: '<S5>/Math Function'

  rtb_Switch_idx_0 = (rtu_q_bar[0] * rtu_q_bar[0] + rtu_q_bar[1] * rtu_q_bar[1])
    + rtu_q_bar[2] * rtu_q_bar[2];

  // Math: '<S5>/Math Function1' incorporates:
  //   Sum: '<S5>/Sum of Elements'
  //
  //  About '<S5>/Math Function1':
  //   Operator: sqrt

  if (rtb_Switch_idx_0 < 0.0) {
    rtb_Acos = -std::sqrt(std::abs(rtb_Switch_idx_0));
  } else {
    rtb_Acos = std::sqrt(rtb_Switch_idx_0);
  }

  // End of Math: '<S5>/Math Function1'

  // Switch: '<S5>/Switch' incorporates:
  //   Constant: '<S5>/Constant'
  //   Product: '<S5>/Product'

  if (rtb_Acos > 0.0) {
    rtb_Switch_idx_0 = rtu_q_bar[0];
    rtb_Switch_idx_1 = rtu_q_bar[1];
    rtb_Switch_idx_2 = rtu_q_bar[2];
    rtb_Switch_idx_3 = rtb_Acos;
  } else {
    rtb_Switch_idx_0 = rtu_q_bar[0] * 0.0;
    rtb_Switch_idx_1 = rtu_q_bar[1] * 0.0;
    rtb_Switch_idx_2 = rtu_q_bar[2] * 0.0;
    rtb_Switch_idx_3 = 1.0;
  }

  // End of Switch: '<S5>/Switch'

  // Trigonometry: '<S4>/Acos'
  if (rtu_q_0 > 1.0) {
    rtb_Acos = 1.0;
  } else if (rtu_q_0 < -1.0) {
    rtb_Acos = -1.0;
  } else {
    rtb_Acos = rtu_q_0;
  }

  rtb_Acos = std::acos(rtb_Acos);

  // End of Trigonometry: '<S4>/Acos'

  // Gain: '<S4>/Multiply' incorporates:
  //   Product: '<S4>/Product'
  //   Product: '<S5>/Divide'

  rty_theta_bar[0] = rtb_Switch_idx_0 / rtb_Switch_idx_3 * rtb_Acos * 2.0;
  rty_theta_bar[1] = rtb_Switch_idx_1 / rtb_Switch_idx_3 * rtb_Acos * 2.0;
  rty_theta_bar[2] = rtb_Switch_idx_2 / rtb_Switch_idx_3 * rtb_Acos * 2.0;
}

//
// Output and update for action system:
//    '<S1>/else_action'
//    '<S2>/else_action'
//
void att_controllerModelClass::AttControl_else_action(real_T rty_theta_bar[3])
{
  // SignalConversion generated from: '<S3>/theta_bar' incorporates:
  //   Constant: '<S3>/Constant'

  rty_theta_bar[0] = 0.0;
  rty_theta_bar[1] = 0.0;
  rty_theta_bar[2] = 0.0;
}

// Model step function
void att_controllerModelClass::step()
{
  real_T rtb_Sum[3];
  real_T rtb_theta_bar_d[3];
  real_T rtb_Sum_idx_0;
  real_T rtb_Sum_idx_1;
  real_T rtb_Sum_idx_2;
  int32_T i;

  // If: '<S1>/If' incorporates:
  //   DotProduct: '<S1>/Dot Product'
  //   Inport: '<Root>/q'
  //   Sqrt: '<S1>/Sqrt'

  if (std::sqrt((AttControl_U.q[1] * AttControl_U.q[1] + AttControl_U.q[2] *
                 AttControl_U.q[2]) + AttControl_U.q[3] * AttControl_U.q[3]) >
      0.0001) {
    // Outputs for IfAction SubSystem: '<S1>/if_action' incorporates:
    //   ActionPort: '<S4>/Action Port'

    AttControl_if_action(AttControl_U.q[0], &AttControl_U.q[1], rtb_Sum);

    // End of Outputs for SubSystem: '<S1>/if_action'
  } else {
    // Outputs for IfAction SubSystem: '<S1>/else_action' incorporates:
    //   ActionPort: '<S3>/Action Port'

    AttControl_else_action(rtb_Sum);

    // End of Outputs for SubSystem: '<S1>/else_action'
  }

  // End of If: '<S1>/If'

  // If: '<S2>/If' incorporates:
  //   DotProduct: '<S2>/Dot Product'
  //   Inport: '<Root>/q_d'
  //   Sqrt: '<S2>/Sqrt'

  if (std::sqrt((AttControl_U.q_d[1] * AttControl_U.q_d[1] + AttControl_U.q_d[2]
                 * AttControl_U.q_d[2]) + AttControl_U.q_d[3] *
                AttControl_U.q_d[3]) > 0.0001) {
    // Outputs for IfAction SubSystem: '<S2>/if_action' incorporates:
    //   ActionPort: '<S7>/Action Port'

    AttControl_if_action(AttControl_U.q_d[0], &AttControl_U.q_d[1],
                         rtb_theta_bar_d);

    // End of Outputs for SubSystem: '<S2>/if_action'
  } else {
    // Outputs for IfAction SubSystem: '<S2>/else_action' incorporates:
    //   ActionPort: '<S6>/Action Port'

    AttControl_else_action(rtb_theta_bar_d);

    // End of Outputs for SubSystem: '<S2>/else_action'
  }

  // End of If: '<S2>/If'

  // Sum: '<Root>/Sum'
  rtb_Sum_idx_0 = rtb_Sum[0] - rtb_theta_bar_d[0];
  rtb_Sum_idx_1 = rtb_Sum[1] - rtb_theta_bar_d[1];
  rtb_Sum_idx_2 = rtb_Sum[2] - rtb_theta_bar_d[2];
  for (i = 0; i <= 0; i += 2) {
    __m128d tmp;

    // Outport: '<Root>/rate_sp' incorporates:
    //   Gain: '<Root>/K_att'

    _mm_storeu_pd(&AttControl_Y.rate_sp[i], _mm_set1_pd(0.0));
    tmp = _mm_loadu_pd(&AttControl_Y.rate_sp[i]);
    _mm_storeu_pd(&AttControl_Y.rate_sp[i], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
      (&AttControl_ConstP.K_att_Gain[i]), _mm_set1_pd(rtb_Sum_idx_0)), tmp));
    tmp = _mm_loadu_pd(&AttControl_Y.rate_sp[i]);
    _mm_storeu_pd(&AttControl_Y.rate_sp[i], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
      (&AttControl_ConstP.K_att_Gain[i + 3]), _mm_set1_pd(rtb_Sum_idx_1)), tmp));
    tmp = _mm_loadu_pd(&AttControl_Y.rate_sp[i]);
    _mm_storeu_pd(&AttControl_Y.rate_sp[i], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
      (&AttControl_ConstP.K_att_Gain[i + 6]), _mm_set1_pd(rtb_Sum_idx_2)), tmp));
  }

  // Outport: '<Root>/rate_sp' incorporates:
  //   Gain: '<Root>/K_att'

  for (i = 2; i < 3; i++) {
    AttControl_Y.rate_sp[i] = 0.0;
    AttControl_Y.rate_sp[i] += AttControl_ConstP.K_att_Gain[i] * rtb_Sum_idx_0;
    AttControl_Y.rate_sp[i] += AttControl_ConstP.K_att_Gain[i + 3] *
      rtb_Sum_idx_1;
    AttControl_Y.rate_sp[i] += AttControl_ConstP.K_att_Gain[i + 6] *
      rtb_Sum_idx_2;
  }
}

// Model initialize function
void att_controllerModelClass::initialize()
{
  // (no initialization code required)
}

// Model terminate function
void att_controllerModelClass::terminate()
{
  // (no terminate code required)
}

// Constructor
att_controllerModelClass::att_controllerModelClass() :
  AttControl_U(),
  AttControl_Y(),
  AttControl_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
att_controllerModelClass::~att_controllerModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
att_controllerModelClass::RT_MODEL_AttControl_T * att_controllerModelClass::
  getRTM()
{
  return (&AttControl_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
