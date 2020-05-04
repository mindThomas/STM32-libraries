//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xscal.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 23:01:08
//

// Include Files
#include "rt_nonfinite.h"
#include "svd_5x5.h"
#include "xscal_5x5.h"

// Function Definitions

//
// Arguments    : float a
//                float x[25]
//                int ix0
// Return Type  : void
//
void xscal_5x5(float a, float x[25], int ix0)
{
  int k;
  for (k = ix0; k <= ix0 + 4; k++) {
    x[k - 1] *= a;
  }
}

//
// File trailer for xscal.cpp
//
// [EOF]
//
