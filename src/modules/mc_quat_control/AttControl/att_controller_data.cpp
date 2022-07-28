//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: att_controller_data.cpp
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

// Constant parameters (default storage)
const att_controllerModelClass::ConstP_att_controller_T att_controller_ConstP{
  // Expression: -[100, 0, 0, 14.142, 0, 0; 0, 100, 0, 0, 14.142, 0;0, 0, 3.16e-6, 0, 0, 2.51e-3]
  //  Referenced by: '<Root>/K_att'

  { -100.0, -0.0, -0.0, -0.0, -100.0, -0.0, -0.0, -0.0, -3.16E-6, -14.142, -0.0,
    -0.0, -0.0, -14.142, -0.0, -0.0, -0.0, -0.00251 },

  // Expression: [0.029125, 0, 0; 0, 0.029125, 0; 0, 0, 0.055225]
  //  Referenced by: '<Root>/J'

  { 0.029125, 0.0, 0.0, 0.0, 0.029125, 0.0, 0.0, 0.0, 0.055225 }
};

//
// File trailer for generated code.
//
// [EOF]
//
