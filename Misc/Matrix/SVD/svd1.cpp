//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd1.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 12-Feb-2019 21:10:49
//

// Include Files
#include <math.h>
#include "rt_nonfinite.h"
#include "svd.h"
#include "svd1.h"
#include "xrot.h"
#include "xrotg.h"
#include "sqrt.h"
#include "xswap.h"
#include "xscal.h"
#include "xaxpy.h"
#include "xdotc.h"
#include "xnrm2.h"

// Function Definitions

//
// Arguments    : const float A[9]
//                float U[9]
//                float s[3]
//                float V[9]
// Return Type  : void
//
void b_svd(const float A[9], float U[9], float s[3], float V[9])
{
  int i;
  float b_A[9];
  float b_s[3];
  float e[3];
  int q;
  float work[3];
  int m;
  int qq;
  boolean_T apply_transform;
  float nrm;
  int k;
  float r;
  int iter;
  float snorm;
  float rt;
  int exitg1;
  boolean_T exitg2;
  float f;
  float scale;
  float sqds;
  for (i = 0; i < 9; i++) {
    b_A[i] = A[i];
  }

  for (i = 0; i < 3; i++) {
    b_s[i] = 0.0F;
    e[i] = 0.0F;
    work[i] = 0.0F;
  }

  for (i = 0; i < 9; i++) {
    U[i] = 0.0F;
    V[i] = 0.0F;
  }

  for (q = 0; q < 2; q++) {
    qq = q + 3 * q;
    apply_transform = false;
    nrm = xnrm2(3 - q, b_A, qq + 1);
    if (nrm > 0.0F) {
      apply_transform = true;
      if (b_A[qq] < 0.0F) {
        b_s[q] = -nrm;
      } else {
        b_s[q] = nrm;
      }

      if ((float)fabs((double)b_s[q]) >= 9.86076132E-32F) {
        r = 1.0F / b_s[q];
        i = (qq - q) + 3;
        for (k = qq; k < i; k++) {
          b_A[k] *= r;
        }
      } else {
        i = (qq - q) + 3;
        for (k = qq; k < i; k++) {
          b_A[k] /= b_s[q];
        }
      }

      b_A[qq]++;
      b_s[q] = -b_s[q];
    } else {
      b_s[q] = 0.0F;
    }

    for (k = q + 1; k + 1 < 4; k++) {
      i = q + 3 * k;
      if (apply_transform) {
        xaxpy(3 - q, -(xdotc(3 - q, b_A, qq + 1, b_A, i + 1) / b_A[q + 3 * q]),
              qq + 1, b_A, i + 1);
      }

      e[k] = b_A[i];
    }

    for (k = q; k + 1 < 4; k++) {
      U[k + 3 * q] = b_A[k + 3 * q];
    }

    if (q + 1 <= 1) {
      nrm = b_xnrm2(2, e, 2);
      if (nrm == 0.0F) {
        e[0] = 0.0F;
      } else {
        if (e[1] < 0.0F) {
          r = -nrm;
        } else {
          r = nrm;
        }

        if (e[1] < 0.0F) {
          e[0] = -nrm;
        } else {
          e[0] = nrm;
        }

        if ((float)fabs((double)r) >= 9.86076132E-32F) {
          r = 1.0F / r;
          for (k = 1; k < 3; k++) {
            e[k] *= r;
          }
        } else {
          for (k = 1; k < 3; k++) {
            e[k] /= r;
          }
        }

        e[1]++;
        e[0] = -e[0];
        for (k = 2; k < 4; k++) {
          work[k - 1] = 0.0F;
        }

        for (k = 1; k + 1 < 4; k++) {
          b_xaxpy(2, e[k], b_A, 3 * k + 2, work, 2);
        }

        for (k = 1; k + 1 < 4; k++) {
          c_xaxpy(2, -e[k] / e[1], work, 2, b_A, 3 * k + 2);
        }
      }

      for (k = 1; k + 1 < 4; k++) {
        V[k] = e[k];
      }
    }
  }

  m = 1;
  b_s[2] = b_A[8];
  e[1] = b_A[7];
  e[2] = 0.0F;
  for (k = 0; k < 3; k++) {
    U[6 + k] = 0.0F;
  }

  U[8] = 1.0F;
  for (q = 1; q >= 0; q--) {
    qq = q + 3 * q;
    if (b_s[q] != 0.0F) {
      for (k = q + 1; k + 1 < 4; k++) {
        i = (q + 3 * k) + 1;
        xaxpy(3 - q, -(xdotc(3 - q, U, qq + 1, U, i) / U[qq]), qq + 1, U, i);
      }

      for (k = q; k + 1 < 4; k++) {
        U[k + 3 * q] = -U[k + 3 * q];
      }

      U[qq]++;
      if (1 <= q) {
        U[3] = 0.0F;
      }
    } else {
      for (k = 0; k < 3; k++) {
        U[k + 3 * q] = 0.0F;
      }

      U[qq] = 1.0F;
    }
  }

  for (q = 2; q >= 0; q--) {
    if ((q + 1 <= 1) && (e[0] != 0.0F)) {
      for (k = 2; k < 4; k++) {
        i = 3 * (k - 1) + 2;
        xaxpy(2, -(xdotc(2, V, 2, V, i) / V[1]), 2, V, i);
      }
    }

    for (k = 0; k < 3; k++) {
      V[k + 3 * q] = 0.0F;
    }

    V[q + 3 * q] = 1.0F;
  }

  for (q = 0; q < 3; q++) {
    nrm = e[q];
    if (b_s[q] != 0.0F) {
      rt = (float)fabs((double)b_s[q]);
      r = b_s[q] / rt;
      b_s[q] = rt;
      if (q + 1 < 3) {
        nrm = e[q] / r;
      }

      xscal(r, U, 1 + 3 * q);
    }

    if ((q + 1 < 3) && (nrm != 0.0F)) {
      rt = (float)fabs((double)nrm);
      r = rt / nrm;
      nrm = rt;
      b_s[q + 1] *= r;
      xscal(r, V, 1 + 3 * (q + 1));
    }

    e[q] = nrm;
  }

  iter = 0;
  snorm = 0.0F;
  for (k = 0; k < 3; k++) {
    nrm = (float)fabs((double)b_s[k]);
    r = (float)fabs((double)e[k]);
    if ((nrm > r) || rtIsNaNF(r)) {
      r = nrm;
    }

    if (!((snorm > r) || rtIsNaNF(r))) {
      snorm = r;
    }
  }

  while ((m + 2 > 0) && (!(iter >= 75))) {
    k = m;
    do {
      exitg1 = 0;
      q = k + 1;
      if (k + 1 == 0) {
        exitg1 = 1;
      } else {
        nrm = (float)fabs((double)e[k]);
        if ((nrm <= 1.1920929E-7F * ((float)fabs((double)b_s[k]) + (float)fabs
              ((double)b_s[k + 1]))) || (nrm <= 9.86076132E-32F) || ((iter > 20)
             && (nrm <= 1.1920929E-7F * snorm))) {
          e[k] = 0.0F;
          exitg1 = 1;
        } else {
          k--;
        }
      }
    } while (exitg1 == 0);

    if (k + 1 == m + 1) {
      i = 4;
    } else {
      qq = m + 2;
      i = m + 2;
      exitg2 = false;
      while ((!exitg2) && (i >= k + 1)) {
        qq = i;
        if (i == k + 1) {
          exitg2 = true;
        } else {
          nrm = 0.0F;
          if (i < m + 2) {
            nrm = (float)fabs((double)e[i - 1]);
          }

          if (i > k + 2) {
            nrm += (float)fabs((double)e[i - 2]);
          }

          r = (float)fabs((double)b_s[i - 1]);
          if ((r <= 1.1920929E-7F * nrm) || (r <= 9.86076132E-32F)) {
            b_s[i - 1] = 0.0F;
            exitg2 = true;
          } else {
            i--;
          }
        }
      }

      if (qq == k + 1) {
        i = 3;
      } else if (qq == m + 2) {
        i = 1;
      } else {
        i = 2;
        q = qq;
      }
    }

    switch (i) {
     case 1:
      f = e[m];
      e[m] = 0.0F;
      for (k = m; k + 1 >= q + 1; k--) {
        xrotg(&b_s[k], &f, &nrm, &r);
        if (k + 1 > q + 1) {
          f = -r * e[0];
          e[0] *= nrm;
        }

        xrot(V, 1 + 3 * k, 1 + 3 * (m + 1), nrm, r);
      }
      break;

     case 2:
      f = e[q - 1];
      e[q - 1] = 0.0F;
      for (k = q; k < m + 2; k++) {
        xrotg(&b_s[k], &f, &nrm, &r);
        f = -r * e[k];
        e[k] *= nrm;
        xrot(U, 1 + 3 * k, 1 + 3 * (q - 1), nrm, r);
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
      for (k = q + 1; k <= m + 1; k++) {
        xrotg(&f, &rt, &nrm, &r);
        if (k > q + 1) {
          e[0] = f;
        }

        f = nrm * b_s[k - 1] + r * e[k - 1];
        e[k - 1] = nrm * e[k - 1] - r * b_s[k - 1];
        rt = r * b_s[k];
        b_s[k] *= nrm;
        xrot(V, 1 + 3 * (k - 1), 1 + 3 * k, nrm, r);
        b_s[k - 1] = f;
        xrotg(&b_s[k - 1], &rt, &nrm, &r);
        f = nrm * e[k - 1] + r * b_s[k];
        b_s[k] = -r * e[k - 1] + nrm * b_s[k];
        rt = r * e[k];
        e[k] *= nrm;
        xrot(U, 1 + 3 * (k - 1), 1 + 3 * k, nrm, r);
      }

      e[m] = f;
      iter++;
      break;

     default:
      if (b_s[q] < 0.0F) {
        b_s[q] = -b_s[q];
        xscal(-1.0F, V, 1 + 3 * q);
      }

      i = q + 1;
      while ((q + 1 < 3) && (b_s[q] < b_s[i])) {
        rt = b_s[q];
        b_s[q] = b_s[i];
        b_s[i] = rt;
        xswap(V, 1 + 3 * q, 1 + 3 * (q + 1));
        xswap(U, 1 + 3 * q, 1 + 3 * (q + 1));
        q = i;
        i++;
      }

      iter = 0;
      m--;
      break;
    }
  }

  for (k = 0; k < 3; k++) {
    s[k] = b_s[k];
  }
}

//
// File trailer for svd1.cpp
//
// [EOF]
//
