//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd1.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 22:13:44
//

// Include Files
#include <math.h>
#include "rt_nonfinite.h"
#include "svd_2x2.h"
#include "svd_2x2_impl.h"
#include "xrot_2x2.h"
#include "xrotg_2x2.h"
#include "sqrt.h"
#include "xswap_2x2.h"
#include "xscal_2x2.h"
#include "xaxpy_2x2.h"
#include "xdotc_2x2.h"
#include "xnrm2_2x2.h"

// Function Definitions

//
// Arguments    : const float A[4]
//                float U[4]
//                float s[2]
//                float V[4]
// Return Type  : void
//
void b_svd_2x2(const float A[4], float U[4], float s[2], float V[4])
{
  int kase;
  float nrm;
  float b_A[4];
  float b_s[2];
  float r;
  int m;
  float e[2];
  int q;
  int iter;
  float snorm;
  float rt;
  boolean_T exitg1;
  int qs;
  float f;
  float scale;
  float sqds;
  for (kase = 0; kase < 4; kase++) {
    b_A[kase] = A[kase];
    V[kase] = 0.0F;
  }

  nrm = xnrm2_2x2(A);
  if (nrm > 0.0F) {
    if (A[0] < 0.0F) {
      r = -nrm;
    } else {
      r = nrm;
    }

    if ((float)fabs((double)r) >= 9.86076132E-32F) {
      nrm = 1.0F / r;
      for (kase = 0; kase < 2; kase++) {
        b_A[kase] *= nrm;
      }
    } else {
      for (kase = 0; kase < 2; kase++) {
        b_A[kase] /= r;
      }
    }

    b_A[0]++;
    b_s[0] = -r;
    xaxpy_2x2(-(xdotc_2x2(b_A, b_A) / b_A[0]), b_A);
  } else {
    b_s[0] = 0.0F;
  }

  m = 0;
  b_s[1] = b_A[3];
  e[0] = b_A[2];
  e[1] = 0.0F;
  for (kase = 0; kase < 2; kase++) {
    U[kase] = b_A[kase];
    U[2 + kase] = 0.0F;
  }

  U[3] = 1.0F;
  if (b_s[0] != 0.0F) {
    xaxpy_2x2(-(xdotc_2x2(U, U) / U[0]), U);
    for (kase = 0; kase < 2; kase++) {
      U[kase] = -U[kase];
    }

    U[0]++;
  } else {
    for (kase = 0; kase < 2; kase++) {
      U[kase] = 0.0F;
    }

    U[0] = 1.0F;
  }

  for (q = 1; q >= 0; q--) {
    for (kase = 0; kase < 2; kase++) {
      V[kase + (q << 1)] = 0.0F;
    }

    V[q + (q << 1)] = 1.0F;
  }

  nrm = b_A[2];
  for (q = 0; q < 2; q++) {
    if (b_s[q] != 0.0F) {
      rt = (float)fabs((double)b_s[q]);
      r = b_s[q] / rt;
      b_s[q] = rt;
      if (q + 1 < 2) {
        nrm /= r;
      }

      xscal_2x2(r, U, 1 + (q << 1));
    }

    if ((q + 1 < 2) && (nrm != 0.0F)) {
      rt = (float)fabs((double)nrm);
      r = rt / nrm;
      nrm = rt;
      b_s[1] *= r;
      xscal_2x2(r, V, 3);
    }

    e[0] = nrm;
  }

  iter = 0;
  snorm = 0.0F;
  for (kase = 0; kase < 2; kase++) {
    nrm = (float)fabs((double)b_s[kase]);
    r = (float)fabs((double)e[kase]);
    if ((nrm > r) || rtIsNaNF(r)) {
      r = nrm;
    }

    if (!((snorm > r) || rtIsNaNF(r))) {
      snorm = r;
    }
  }

  while ((m + 2 > 0) && (!(iter >= 75))) {
    q = m + 1;
    exitg1 = false;
    while (!(exitg1 || (q == 0))) {
      nrm = (float)fabs((double)e[0]);
      if ((nrm <= 1.1920929E-7F * ((float)fabs((double)b_s[0]) + (float)fabs
            ((double)b_s[1]))) || (nrm <= 9.86076132E-32F) || ((iter > 20) &&
           (nrm <= 1.1920929E-7F * snorm))) {
        e[0] = 0.0F;
        exitg1 = true;
      } else {
        q = 0;
      }
    }

    if (q == m + 1) {
      kase = 4;
    } else {
      qs = m + 2;
      kase = m + 2;
      exitg1 = false;
      while ((!exitg1) && (kase >= q)) {
        qs = kase;
        if (kase == q) {
          exitg1 = true;
        } else {
          nrm = 0.0F;
          if (kase < m + 2) {
            nrm = (float)fabs((double)e[kase - 1]);
          }

          if (kase > q + 1) {
            nrm += (float)fabs((double)e[0]);
          }

          r = (float)fabs((double)b_s[kase - 1]);
          if ((r <= 1.1920929E-7F * nrm) || (r <= 9.86076132E-32F)) {
            b_s[kase - 1] = 0.0F;
            exitg1 = true;
          } else {
            kase--;
          }
        }
      }

      if (qs == q) {
        kase = 3;
      } else if (qs == m + 2) {
        kase = 1;
      } else {
        kase = 2;
        q = qs;
      }
    }

    switch (kase) {
     case 1:
      f = e[m];
      e[m] = 0.0F;
      kase = m + 1;
      while (kase >= q + 1) {
        xrotg_2x2(&b_s[0], &f, &nrm, &r);
        xrot_2x2(V, 1, 1 + ((m + 1) << 1), nrm, r);
        kase = 0;
      }
      break;

     case 2:
      f = e[q - 1];
      e[q - 1] = 0.0F;
      for (kase = q; kase < m + 2; kase++) {
        xrotg_2x2(&b_s[kase], &f, &nrm, &r);
        f = -r * e[kase];
        e[kase] *= nrm;
        xrot_2x2(U, 1 + (kase << 1), 1 + ((q - 1) << 1), nrm, r);
      }
      break;

     case 3:
      scale = (float)fabs((double)b_s[m + 1]);
      r = (float)fabs((double)b_s[m]);
      if (!((scale > r) || rtIsNaNF(r))) {
        scale = r;
      }

      r = (float)fabs((double)e[m]);
      if (!((scale > r) || rtIsNaNF(r))) {
        scale = r;
      }

      r = (float)fabs((double)b_s[q]);
      if (!((scale > r) || rtIsNaNF(r))) {
        scale = r;
      }

      r = (float)fabs((double)e[q]);
      if (!((scale > r) || rtIsNaNF(r))) {
        scale = r;
      }

      f = b_s[m + 1] / scale;
      nrm = b_s[m] / scale;
      r = e[m] / scale;
      sqds = b_s[q] / scale;
      rt = ((nrm + f) * (nrm - f) + r * r) / 2.0F;
      nrm = f * r;
      nrm *= nrm;
      if ((rt != 0.0F) || (nrm != 0.0F)) {
        r = rt * rt + nrm;
        b_sqrt(&r);
        if (rt < 0.0F) {
          r = -r;
        }

        r = nrm / (rt + r);
      } else {
        r = 0.0F;
      }

      f = (sqds + f) * (sqds - f) + r;
      rt = sqds * (e[q] / scale);
      while (q + 1 <= m + 1) {
        xrotg_2x2(&f, &rt, &nrm, &r);
        f = nrm * b_s[0] + r * e[0];
        e[0] = nrm * e[0] - r * b_s[0];
        rt = r * b_s[1];
        b_s[1] *= nrm;
        xrot_2x2(V, 1, 3, nrm, r);
        b_s[0] = f;
        xrotg_2x2(&b_s[0], &rt, &nrm, &r);
        f = nrm * e[0] + r * b_s[1];
        b_s[1] = -r * e[0] + nrm * b_s[1];
        rt = r * e[1];
        e[1] *= nrm;
        xrot_2x2(U, 1, 3, nrm, r);
        q = 1;
      }

      e[m] = f;
      iter++;
      break;

     default:
      if (b_s[q] < 0.0F) {
        b_s[q] = -b_s[q];
        xscal_2x2(-1.0F, V, 1 + (q << 1));
      }

      kase = q + 1;
      while ((q + 1 < 2) && (b_s[q] < b_s[kase])) {
        rt = b_s[q];
        b_s[q] = b_s[kase];
        b_s[kase] = rt;
        xswap_2x2(V, 1 + (q << 1), 1 + ((q + 1) << 1));
        xswap_2x2(U, 1 + (q << 1), 1 + ((q + 1) << 1));
        q = kase;
        kase++;
      }

      iter = 0;
      m--;
      break;
    }
  }

  for (kase = 0; kase < 2; kase++) {
    s[kase] = b_s[kase];
  }
}

//
// File trailer for svd1.cpp
//
// [EOF]
//
