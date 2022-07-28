//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: att_controller.cpp
//
// Code generated for Simulink model 'att_controller'.
//
// Model version                  : 1.11
// Simulink Coder version         : 9.6 (R2021b) 14-May-2021
// C/C++ source code generated on : Thu Apr 14 15:53:10 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "att_controller.h"
#include "att_controller_private.h"

//
// Output and update for action system:
//    '<S1>/if_action'
//    '<S2>/if_action'
//
void att_controllerModelClass::att_controller_if_action(real_T rtu_q_0, const
  real_T rtu_q_bar[3], real_T rty_theta_bar[3])
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
void att_controllerModelClass::att_controller_else_action(real_T rty_theta_bar[3])
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
  __m128d tmp;
  real_T rtb_Merge_0[6];
  real_T rtb_u_att_0[6];
  real_T rtb_u_att_1[6];
  real_T rtb_Merge[3];
  real_T rtb_u_att[3];
  int32_T i;

  // If: '<S1>/If' incorporates:
  //   DotProduct: '<S1>/Dot Product'
  //   Inport: '<Root>/q'
  //   Sqrt: '<S1>/Sqrt'

  if (std::sqrt((att_controller_U.q[1] * att_controller_U.q[1] +
                 att_controller_U.q[2] * att_controller_U.q[2]) +
                att_controller_U.q[3] * att_controller_U.q[3]) > 0.0001) {
    // Outputs for IfAction SubSystem: '<S1>/if_action' incorporates:
    //   ActionPort: '<S4>/Action Port'

    att_controller_if_action(att_controller_U.q[0], &att_controller_U.q[1],
      rtb_u_att);

    // End of Outputs for SubSystem: '<S1>/if_action'
  } else {
    // Outputs for IfAction SubSystem: '<S1>/else_action' incorporates:
    //   ActionPort: '<S3>/Action Port'

    att_controller_else_action(rtb_u_att);

    // End of Outputs for SubSystem: '<S1>/else_action'
  }

  // End of If: '<S1>/If'

  // If: '<S2>/If' incorporates:
  //   DotProduct: '<S2>/Dot Product'
  //   Inport: '<Root>/q_d'
  //   Sqrt: '<S2>/Sqrt'

  if (std::sqrt((att_controller_U.q_d[1] * att_controller_U.q_d[1] +
                 att_controller_U.q_d[2] * att_controller_U.q_d[2]) +
                att_controller_U.q_d[3] * att_controller_U.q_d[3]) > 0.0001) {
    // Outputs for IfAction SubSystem: '<S2>/if_action' incorporates:
    //   ActionPort: '<S7>/Action Port'

    att_controller_if_action(att_controller_U.q_d[0], &att_controller_U.q_d[1],
      rtb_Merge);

    // End of Outputs for SubSystem: '<S2>/if_action'
  } else {
    // Outputs for IfAction SubSystem: '<S2>/else_action' incorporates:
    //   ActionPort: '<S6>/Action Port'

    att_controller_else_action(rtb_Merge);

    // End of Outputs for SubSystem: '<S2>/else_action'
  }

  // End of If: '<S2>/If'

  // Sum: '<Root>/Sum' incorporates:
  //   Constant: '<Root>/w_d'
  //   Inport: '<Root>/w'

  rtb_u_att_0[0] = rtb_u_att[0];
  rtb_u_att_0[3] = att_controller_U.w[0];
  rtb_Merge_0[0] = rtb_Merge[0];
  rtb_Merge_0[3] = 0.0;
  rtb_u_att_0[1] = rtb_u_att[1];
  rtb_u_att_0[4] = att_controller_U.w[1];
  rtb_Merge_0[1] = rtb_Merge[1];
  rtb_Merge_0[4] = 0.0;
  rtb_u_att_0[2] = rtb_u_att[2];
  rtb_u_att_0[5] = att_controller_U.w[2];
  rtb_Merge_0[2] = rtb_Merge[2];
  rtb_Merge_0[5] = 0.0;
  for (i = 0; i <= 4; i += 2) {
    __m128d tmp_0;

    // Sum: '<Root>/Sum'
    tmp = _mm_loadu_pd(&rtb_u_att_0[i]);
    tmp_0 = _mm_loadu_pd(&rtb_Merge_0[i]);
    _mm_storeu_pd(&rtb_u_att_1[i], _mm_sub_pd(tmp, tmp_0));
  }

  // Gain: '<Root>/K_att'
  for (i = 0; i < 3; i++) {
    rtb_u_att[i] = 0.0;
    for (int32_T i_0{0}; i_0 < 6; i_0++) {
      rtb_u_att[i] += att_controller_ConstP.K_att_Gain[3 * i_0 + i] *
        rtb_u_att_1[i_0];
    }
  }

  // End of Gain: '<Root>/K_att'
  for (i = 0; i <= 0; i += 2) {
    // Outport: '<Root>/tau' incorporates:
    //   Gain: '<Root>/J'

    _mm_storeu_pd(&att_controller_Y.tau[i], _mm_set1_pd(0.0));
    tmp = _mm_loadu_pd(&att_controller_Y.tau[i]);
    _mm_storeu_pd(&att_controller_Y.tau[i], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
      (&att_controller_ConstP.J_Gain[i]), _mm_set1_pd(rtb_u_att[0])), tmp));
    tmp = _mm_loadu_pd(&att_controller_Y.tau[i]);
    _mm_storeu_pd(&att_controller_Y.tau[i], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
      (&att_controller_ConstP.J_Gain[i + 3]), _mm_set1_pd(rtb_u_att[1])), tmp));
    tmp = _mm_loadu_pd(&att_controller_Y.tau[i]);
    _mm_storeu_pd(&att_controller_Y.tau[i], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
      (&att_controller_ConstP.J_Gain[i + 6]), _mm_set1_pd(rtb_u_att[2])), tmp));
  }

  // Outport: '<Root>/tau' incorporates:
  //   Gain: '<Root>/J'

  for (i = 2; i < 3; i++) {
    att_controller_Y.tau[i] = 0.0;
    att_controller_Y.tau[i] += att_controller_ConstP.J_Gain[i] * rtb_u_att[0];
    att_controller_Y.tau[i] += att_controller_ConstP.J_Gain[i + 3] * rtb_u_att[1];
    att_controller_Y.tau[i] += att_controller_ConstP.J_Gain[i + 6] * rtb_u_att[2];
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
  att_controller_U(),
  att_controller_Y(),
  att_controller_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
att_controllerModelClass::~att_controllerModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
att_controllerModelClass::RT_MODEL_att_controller_T * att_controllerModelClass::
  getRTM()
{
  return (&att_controller_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
