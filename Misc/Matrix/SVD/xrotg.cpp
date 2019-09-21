//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xrotg.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 12-Feb-2019 21:10:49
//

// Include Files
#include <math.h>
#include "mw_cmsis.h"
#include "rt_nonfinite.h"
#include "svd.h"
#include "xrotg.h"

// Function Definitions

//
// Arguments    : float *a
//                float *b
//                float *c
//                float *s
// Return Type  : void
//
void xrotg(float *a, float *b, float *c, float *s)
{
  float roe;
  float absa;
  float absb;
  float scale;
  float ads;
  float bds;
  roe = *b;
  absa = (float)fabs((double)*a);
  absb = (float)fabs((double)*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0F) {
    *s = 0.0F;
    *c = 1.0F;
    ads = 0.0F;
    *b = 0.0F;
  } else {
    ads = absa / scale;
    bds = absb / scale;
    mw_arm_sqrt_f32(ads * ads + bds * bds, &ads);
    ads *= scale;
    if (roe < 0.0F) {
      ads = -ads;
    }

    *c = *a / ads;
    *s = *b / ads;
    if (absa > absb) {
      *b = *s;
    } else if (*c != 0.0F) {
      *b = 1.0F / *c;
    } else {
      *b = 1.0F;
    }
  }

  *a = ads;
}

//
// File trailer for xrotg.cpp
//
// [EOF]
//
