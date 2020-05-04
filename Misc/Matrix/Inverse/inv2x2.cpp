//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: inv.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 03-May-2020 21:13:57
//

// Include Files
#include <math.h>
//#include "rt_nonfinite.h"
#include "inv2x2.h"

// Function Definitions

//
// Arguments    : const float x[4]
//                float y[4]
// Return Type  : void
//
void inv2x2_robust(const float x[4], float y[4])
{
  float r;
  float t;
  if ((float)fabs((double)x[2]) > (float)fabs((double)x[0])) {
    r = x[0] / x[2];
    t = 1.0F / (r * x[3] - x[1]);
    y[0] = x[3] / x[2] * t;
    y[2] = -t;
    y[1] = -x[1] / x[2] * t;
    y[3] = r * t;
  } else {
    r = x[2] / x[0];
    t = 1.0F / (x[3] - r * x[1]);
    y[0] = x[3] / x[0] * t;
    y[2] = -r * t;
    y[1] = -x[1] / x[0] * t;
    y[3] = t;
  }
}

void inv2x2_fast(const float x[4], float y[4])
{
	const float& a = x[0];
	const float& b = x[1];
	const float& c = x[2];
	const float& d = x[3];

	const float det = (a*d - b*c);
	const float det_inv = 1 / det;

	y[0] = det_inv * d;
	y[1] = -det_inv * b;
	y[2] = -det_inv * c;
	y[3] = det_inv * a;
}

//
// File trailer for inv.cpp
//
// [EOF]
//
