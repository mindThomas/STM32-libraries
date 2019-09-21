//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sqrt.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 12-Feb-2019 21:10:49
//

// Include Files
#include "mw_cmsis.h"
#include "rt_nonfinite.h"
#include "svd.h"
#include "sqrt.h"

// Function Definitions

//
// Arguments    : float *x
// Return Type  : void
//
void b_sqrt(float *x)
{
  float f2;
  mw_arm_sqrt_f32(*x, &f2);
  *x = f2;
}

//
// File trailer for sqrt.cpp
//
// [EOF]
//
