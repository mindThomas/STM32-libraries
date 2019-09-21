//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xnrm2.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 12-Feb-2019 21:10:49
//

// Include Files
#include <math.h>
#include "mw_cmsis.h"
#include "rt_nonfinite.h"
#include "svd.h"
#include "xnrm2.h"

// Function Definitions

//
// Arguments    : int n
//                const float x[3]
//                int ix0
// Return Type  : float
//
float b_xnrm2(int n, const float x[3], int ix0)
{
  float y;
  float scale;
  int kend;
  int k;
  float f1;
  float absxk;
  float t;
  y = 0.0F;
  if (!(n < 1)) {
    if (n == 1) {
      y = (float)fabs((double)x[ix0 - 1]);
    } else {
      scale = 1.29246971E-26F;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = (float)fabs((double)x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = 1.0F + y * t * t;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      mw_arm_sqrt_f32(y, &f1);
      y = scale * f1;
    }
  }

  return y;
}

//
// Arguments    : int n
//                const float x[9]
//                int ix0
// Return Type  : float
//
float xnrm2(int n, const float x[9], int ix0)
{
  float y;
  float scale;
  int kend;
  int k;
  float f0;
  float absxk;
  float t;
  y = 0.0F;
  if (!(n < 1)) {
    if (n == 1) {
      y = (float)fabs((double)x[ix0 - 1]);
    } else {
      scale = 1.29246971E-26F;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = (float)fabs((double)x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = 1.0F + y * t * t;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      mw_arm_sqrt_f32(y, &f0);
      y = scale * f0;
    }
  }

  return y;
}

//
// File trailer for xnrm2.cpp
//
// [EOF]
//
