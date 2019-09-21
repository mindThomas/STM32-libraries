/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: inv.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 18-Oct-2018 16:10:58
 */

/* Include Files */
#include <math.h>
//#include "rt_nonfinite.h"
#include "inv3x3.h"

/* Function Definitions */

/*
 * Arguments    : const float x[9]
 *                float y[9]
 * Return Type  : void
 */
void inv3x3(const float x[9], float y[9])
{
  int p1;
  float b_x[9];
  int p2;
  int p3;
  float absx11;
  float absx21;
  float absx31;
  int itmp;
  for (p1 = 0; p1 < 9; p1++) {
    b_x[p1] = x[p1];
  }

  p1 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = (float)fabs(x[0]);
  absx21 = (float)fabs(x[3]);
  absx31 = (float)fabs(x[6]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    b_x[0] = x[3];
    b_x[3] = x[0];
    b_x[1] = x[4];
    b_x[4] = x[1];
    b_x[2] = x[5];
    b_x[5] = x[2];
  } else {
    if (absx31 > absx11) {
      p1 = 6;
      p3 = 0;
      b_x[0] = x[6];
      b_x[6] = x[0];
      b_x[1] = x[7];
      b_x[7] = x[1];
      b_x[2] = x[8];
      b_x[8] = x[2];
    }
  }

  absx11 = b_x[3] / b_x[0];
  b_x[3] /= b_x[0];
  absx21 = b_x[6] / b_x[0];
  b_x[6] /= b_x[0];
  b_x[4] -= absx11 * b_x[1];
  b_x[7] -= absx21 * b_x[1];
  b_x[5] -= absx11 * b_x[2];
  b_x[8] -= absx21 * b_x[2];
  if ((float)fabs(b_x[7]) > (float)fabs(b_x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    b_x[3] = absx21;
    b_x[6] = absx11;
    absx11 = b_x[4];
    b_x[4] = b_x[7];
    b_x[7] = absx11;
    absx11 = b_x[5];
    b_x[5] = b_x[8];
    b_x[8] = absx11;
  }

  absx11 = b_x[7] / b_x[4];
  b_x[7] /= b_x[4];
  b_x[8] -= absx11 * b_x[5];
  absx11 = (b_x[7] * b_x[3] - b_x[6]) / b_x[8];
  absx21 = -(b_x[3] + b_x[5] * absx11) / b_x[4];
  y[p1 % 3 * 3 + p1 / 3] = ((1.0F - b_x[1] * absx21) - b_x[2] * absx11) / b_x[0];
  y[(p1 + 1) % 3 * 3 + (p1 + 1) / 3] = absx21;
  y[(p1 + 2) % 3 * 3 + (p1 + 2) / 3] = absx11;
  absx11 = -b_x[7] / b_x[8];
  absx21 = (1.0F - b_x[5] * absx11) / b_x[4];
  y[p2 % 3 * 3 + p2 / 3] = -(b_x[1] * absx21 + b_x[2] * absx11) / b_x[0];
  y[(p2 + 1) % 3 * 3 + (p2 + 1) / 3] = absx21;
  y[(p2 + 2) % 3 * 3 + (p2 + 2) / 3] = absx11;
  absx11 = 1.0F / b_x[8];
  absx21 = -b_x[5] * absx11 / b_x[4];
  y[p3 % 3 * 3 + p3 / 3] = -(b_x[1] * absx21 + b_x[2] * absx11) / b_x[0];
  y[(p3 + 1) % 3 * 3 + (p3 + 1) / 3] = absx21;
  y[(p3 + 2) % 3 * 3 + (p3 + 2) / 3] = absx11;
}

/*
 * File trailer for inv.c
 *
 * [EOF]
 */
