//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: inv.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 03-May-2020 21:16:49
//

// Include Files
#include <math.h>
#include <string.h>
//#include "rt_nonfinite.h"
#include "inv5x5.h"

// Function Definitions

//
// Arguments    : const float x[25]
//                float y[25]
// Return Type  : void
//
void inv5x5(const float x[25], float y[25])
{
    int         i0;
    float       b_y[25];
    int         iy;
    float       b_x[25];
    int         j;
    signed char ipiv[5];
    int         c;
    int         k;
    signed char p[5];
    int         ix;
    float       smax;
    float       s;
    int         jy;
    int         i;
    for (i0 = 0; i0 < 5; i0++) {
        for (iy = 0; iy < 5; iy++) {
            b_x[iy + 5 * i0] = x[i0 + 5 * iy];
        }
    }

    memset(&b_y[0], 0, 25U * sizeof(float));
    for (i0 = 0; i0 < 5; i0++) {
        ipiv[i0] = (signed char)(1 + i0);
    }

    for (j = 0; j < 4; j++) {
        c    = j * 6;
        iy   = 0;
        ix   = c;
        smax = (float)fabs((double)b_x[c]);
        for (k = 2; k <= 5 - j; k++) {
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
                for (k = 0; k < 5; k++) {
                    smax    = b_x[ix];
                    b_x[ix] = b_x[iy];
                    b_x[iy] = smax;
                    ix += 5;
                    iy += 5;
                }
            }

            i0 = (c - j) + 5;
            for (i = c + 1; i < i0; i++) {
                b_x[i] /= b_x[c];
            }
        }

        iy = c;
        jy = c + 5;
        for (i = 1; i <= 4 - j; i++) {
            smax = b_x[jy];
            if (b_x[jy] != 0.0F) {
                ix = c + 1;
                i0 = (iy - j) + 10;
                for (k = 6 + iy; k < i0; k++) {
                    b_x[k] += b_x[ix] * -smax;
                    ix++;
                }
            }

            jy += 5;
            iy += 5;
        }
    }

    for (i0 = 0; i0 < 5; i0++) {
        p[i0] = (signed char)(1 + i0);
    }

    for (k = 0; k < 4; k++) {
        if (ipiv[k] > 1 + k) {
            iy             = p[ipiv[k] - 1];
            p[ipiv[k] - 1] = p[k];
            p[k]           = (signed char)iy;
        }
    }

    for (k = 0; k < 5; k++) {
        c                       = p[k] - 1;
        b_y[k + 5 * (p[k] - 1)] = 1.0F;
        for (j = k; j + 1 < 6; j++) {
            if (b_y[j + 5 * c] != 0.0F) {
                for (i = j + 1; i + 1 < 6; i++) {
                    b_y[i + 5 * c] -= b_y[j + 5 * c] * b_x[i + 5 * j];
                }
            }
        }
    }

    for (j = 0; j < 5; j++) {
        iy = 5 * j;
        for (k = 4; k >= 0; k--) {
            jy = 5 * k;
            if (b_y[k + iy] != 0.0F) {
                b_y[k + iy] /= b_x[k + jy];
                for (i = 0; i < k; i++) {
                    b_y[i + iy] -= b_y[k + iy] * b_x[i + jy];
                }
            }
        }
    }

    for (i0 = 0; i0 < 5; i0++) {
        for (iy = 0; iy < 5; iy++) {
            y[iy + 5 * i0] = b_y[i0 + 5 * iy];
        }
    }
}

//
// File trailer for inv.cpp
//
// [EOF]
//
