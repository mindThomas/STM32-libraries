//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: QEKF_initialize.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 29-Oct-2018 20:49:17
//
#ifndef QEKF_INITIALIZE_H
#define QEKF_INITIALIZE_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "QEKF_types.h"

// Function Declarations
extern void QEKF_initialize(const float P_diagonal_init[10], float X[10], float P[10*10]);

#endif

//
// File trailer for QEKF_initialize.h
//
// [EOF]
//
