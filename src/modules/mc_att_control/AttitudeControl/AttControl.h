//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: AttControl.h
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
#ifndef RTW_HEADER_AttControl_h_
#define RTW_HEADER_AttControl_h_
#include <cmath>
#include <emmintrin.h>
#include "rtwtypes.h"
#include "AttControl_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// Class declaration for model AttControl
class att_controllerModelClass
{
  // public data and function members
 public:
  // Constant parameters (default storage)
  struct ConstP_AttControl_T {
    // Expression: -[100, 0, 0; 0, 100, 0; 0, 0, 100]
    //  Referenced by: '<Root>/K_att'

    real_T K_att_Gain[9];
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_AttControl_T {
    real_T q[4];                       // '<Root>/q'
    real_T q_d[4];                     // '<Root>/q_d'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_AttControl_T {
    real_T rate_sp[3];                 // '<Root>/rate_sp'
  };

  // Real-time Model Data Structure
  struct RT_MODEL_AttControl_T {
    const char_T * volatile errorStatus;
  };

  // Copy Constructor
  att_controllerModelClass(att_controllerModelClass const&) =delete;

  // Assignment Operator
  att_controllerModelClass& operator= (att_controllerModelClass const&) & =
    delete;

  // Real-Time Model get method
  att_controllerModelClass::RT_MODEL_AttControl_T * getRTM();

  // Root inports set method
  void setExternalInputs(const ExtU_AttControl_T *pExtU_AttControl_T)
  {
    AttControl_U = *pExtU_AttControl_T;
  }

  // Root outports get method
  const ExtY_AttControl_T &getExternalOutputs() const
  {
    return AttControl_Y;
  }

  // model initialize function
  static void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  att_controllerModelClass();

  // Destructor
  ~att_controllerModelClass();

  // private data and function members
 private:
  // External inputs
  ExtU_AttControl_T AttControl_U;

  // External outputs
  ExtY_AttControl_T AttControl_Y;

  // private member function(s) for subsystem '<S1>/if_action'
  static void AttControl_if_action(real_T rtu_q_0, const real_T rtu_q_bar[3],
    real_T rty_theta_bar[3]);

  // private member function(s) for subsystem '<S1>/else_action'
  static void AttControl_else_action(real_T rty_theta_bar[3]);

  // Real-Time Model
  RT_MODEL_AttControl_T AttControl_M;
};

// Constant parameters (default storage)
extern const att_controllerModelClass::ConstP_AttControl_T AttControl_ConstP;

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'AttControl'
//  '<S1>'   : 'AttControl/cal_actual_state'
//  '<S2>'   : 'AttControl/cal_desired_state'
//  '<S3>'   : 'AttControl/cal_actual_state/else_action'
//  '<S4>'   : 'AttControl/cal_actual_state/if_action'
//  '<S5>'   : 'AttControl/cal_actual_state/if_action/Normalize Vector'
//  '<S6>'   : 'AttControl/cal_desired_state/else_action'
//  '<S7>'   : 'AttControl/cal_desired_state/if_action'
//  '<S8>'   : 'AttControl/cal_desired_state/if_action/Normalize Vector'

#endif                                 // RTW_HEADER_AttControl_h_

//
// File trailer for generated code.
//
// [EOF]
//
