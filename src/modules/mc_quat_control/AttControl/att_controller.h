//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: att_controller.h
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
#ifndef RTW_HEADER_att_controller_h_
#define RTW_HEADER_att_controller_h_
#include <cmath>
#include <emmintrin.h>
#include "rtwtypes.h"
#include "att_controller_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// Class declaration for model att_controller
class att_controllerModelClass
{
  // public data and function members
 public:
  // Constant parameters (default storage)
  struct ConstP_att_controller_T {
    // Expression: -[100, 0, 0, 14.142, 0, 0; 0, 100, 0, 0, 14.142, 0;0, 0, 3.16e-6, 0, 0, 2.51e-3]
    //  Referenced by: '<Root>/K_att'

    real_T K_att_Gain[18];

    // Expression: [0.029125, 0, 0; 0, 0.029125, 0; 0, 0, 0.055225]
    //  Referenced by: '<Root>/J'

    real_T J_Gain[9];
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_att_controller_T {
    real_T q[4];                       // '<Root>/q'
    real_T w[3];                       // '<Root>/w'
    real_T q_d[4];                     // '<Root>/q_d'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_att_controller_T {
    real_T tau[3];                     // '<Root>/tau'
  };

  // Real-time Model Data Structure
  struct RT_MODEL_att_controller_T {
    const char_T * volatile errorStatus;
  };

  // Copy Constructor
  att_controllerModelClass(att_controllerModelClass const&) =delete;

  // Assignment Operator
  att_controllerModelClass& operator= (att_controllerModelClass const&) & =
    delete;

  // Real-Time Model get method
  att_controllerModelClass::RT_MODEL_att_controller_T * getRTM();

  // Root inports set method
  void setExternalInputs(const ExtU_att_controller_T *pExtU_att_controller_T)
  {
    att_controller_U = *pExtU_att_controller_T;
  }

  // Root outports get method
  const ExtY_att_controller_T &getExternalOutputs() const
  {
    return att_controller_Y;
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
  ExtU_att_controller_T att_controller_U;

  // External outputs
  ExtY_att_controller_T att_controller_Y;

  // private member function(s) for subsystem '<S1>/if_action'
  static void att_controller_if_action(real_T rtu_q_0, const real_T rtu_q_bar[3],
    real_T rty_theta_bar[3]);

  // private member function(s) for subsystem '<S1>/else_action'
  static void att_controller_else_action(real_T rty_theta_bar[3]);

  // Real-Time Model
  RT_MODEL_att_controller_T att_controller_M;
};

// Constant parameters (default storage)
extern const att_controllerModelClass::ConstP_att_controller_T
  att_controller_ConstP;

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
//  '<Root>' : 'att_controller'
//  '<S1>'   : 'att_controller/cal_actual_state'
//  '<S2>'   : 'att_controller/cal_desired_state'
//  '<S3>'   : 'att_controller/cal_actual_state/else_action'
//  '<S4>'   : 'att_controller/cal_actual_state/if_action'
//  '<S5>'   : 'att_controller/cal_actual_state/if_action/Normalize Vector'
//  '<S6>'   : 'att_controller/cal_desired_state/else_action'
//  '<S7>'   : 'att_controller/cal_desired_state/if_action'
//  '<S8>'   : 'att_controller/cal_desired_state/if_action/Normalize Vector'

#endif                                 // RTW_HEADER_att_controller_h_

//
// File trailer for generated code.
//
// [EOF]
//
