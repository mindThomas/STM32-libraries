//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: inv.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 03-May-2020 21:15:59
//

// Include Files
#include <math.h>
#include <string.h>
//#include <MATLABCoderInit/rt_nonfinite.h>
#include "inv4x4.h"

// Function Definitions

//
// Arguments    : const float x[16]
//                float y[16]
// Return Type  : void
//
void inv4x4(const float x[16], float y[16])
{
    int         i0;
    float       b_y[16];
    int         iy;
    float       b_x[16];
    int         j;
    signed char ipiv[4];
    int         c;
    int         k;
    signed char p[4];
    int         ix;
    float       smax;
    float       s;
    int         jy;
    int         i;
    for (i0 = 0; i0 < 4; i0++) {
        for (iy = 0; iy < 4; iy++) {
            b_x[iy + (i0 << 2)] = x[i0 + (iy << 2)];
        }
    }

    memset(&b_y[0], 0, sizeof(float) << 4);
    for (i0 = 0; i0 < 4; i0++) {
        ipiv[i0] = (signed char)(1 + i0);
    }

    for (j = 0; j < 3; j++) {
        c    = j * 5;
        iy   = 0;
        ix   = c;
        smax = (float)fabs((double)b_x[c]);
        for (k = 2; k <= 4 - j; k++) {
            ix++;
            s = (float)fabs((double)b_x[ix]);
            if (s > smax) {
                iy   = k - 1;
                smax = s;
            }
        }

        if (b_x[c + iy] != 0.0F) {
            if (iy != 0) {
                ipiv[j] = (signed char)((j + iy) + 1);
                ix      = j;
                iy += j;
                for (k = 0; k < 4; k++) {
                    smax    = b_x[ix];
                    b_x[ix] = b_x[iy];
                    b_x[iy] = smax;
                    ix += 4;
                    iy += 4;
                }
            }

            i0 = (c - j) + 4;
            for (i = c + 1; i < i0; i++) {
                b_x[i] /= b_x[c];
            }
        }

        iy = c;
        jy = c + 4;
        for (i = 1; i <= 3 - j; i++) {
            smax = b_x[jy];
            if (b_x[jy] != 0.0F) {
                ix = c + 1;
                i0 = (iy - j) + 8;
                for (k = 5 + iy; k < i0; k++) {
                    b_x[k] += b_x[ix] * -smax;
                    ix++;
                }
            }

            jy += 4;
            iy += 4;
        }
    }

    for (i0 = 0; i0 < 4; i0++) {
        p[i0] = (signed char)(1 + i0);
    }

    for (k = 0; k < 3; k++) {
        if (ipiv[k] > 1 + k) {
            iy             = p[ipiv[k] - 1];
            p[ipiv[k] - 1] = p[k];
            p[k]           = (signed char)iy;
        }
    }

    for (k = 0; k < 4; k++) {
        c                          = p[k] - 1;
        b_y[k + ((p[k] - 1) << 2)] = 1.0F;
        for (j = k; j + 1 < 5; j++) {
            if (b_y[j + (c << 2)] != 0.0F) {
                for (i = j + 1; i + 1 < 5; i++) {
                    b_y[i + (c << 2)] -= b_y[j + (c << 2)] * b_x[i + (j << 2)];
                }
            }
        }
    }

    for (j = 0; j < 4; j++) {
        iy = j << 2;
        for (k = 3; k >= 0; k--) {
            jy = k << 2;
            if (b_y[k + iy] != 0.0F) {
                b_y[k + iy] /= b_x[k + jy];
                for (i = 0; i < k; i++) {
                    b_y[i + iy] -= b_y[k + iy] * b_x[i + jy];
                }
            }
        }
    }

    for (i0 = 0; i0 < 4; i0++) {
        for (iy = 0; iy < 4; iy++) {
            y[iy + (i0 << 2)] = b_y[i0 + (iy << 2)];
        }
    }
}

//
// File trailer for inv.cpp
//
// [EOF]
//
