//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xaxpy.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 12-Feb-2019 21:10:49
//

// Include Files
#include "rt_nonfinite.h"
#include "svd.h"
#include "xaxpy.h"

// Function Definitions

//
// Arguments    : int n
//                float a
//                const float x[9]
//                int ix0
//                float y[3]
//                int iy0
// Return Type  : void
//
void b_xaxpy(int n, float a, const float x[9], int ix0, float y[3], int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0F)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : int n
//                float a
//                const float x[3]
//                int ix0
//                float y[9]
//                int iy0
// Return Type  : void
//
void c_xaxpy(int n, float a, const float x[3], int ix0, float y[9], int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0F)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : int n
//                float a
//                int ix0
//                float y[9]
//                int iy0
// Return Type  : void
//
void xaxpy(int n, float a, int ix0, float y[9], int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0F)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

//
// File trailer for xaxpy.cpp
//
// [EOF]
//
