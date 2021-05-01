//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: det.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 22:04:17
//

// Include Files
#include <math.h>
//#include "rt_nonfinite.h"
#include "det3x3.h"

// Function Definitions

//
// Arguments    : const float x[9]
// Return Type  : float
//
float det3x3(const float x[9])
{
    float       y;
    int         i0;
    float       b_x[9];
    int         j;
    signed char ipiv[3];
    int         c;
    boolean_T   isodd;
    int         iy;
    int         jy;
    int         ix;
    float       smax;
    float       s;
    int         b_j;
    int         ijA;
    for (i0 = 0; i0 < 9; i0++) {
        b_x[i0] = x[i0];
    }

    for (i0 = 0; i0 < 3; i0++) {
        ipiv[i0] = (signed char)(1 + i0);
    }

    for (j = 0; j < 2; j++) {
        c    = j << 2;
        iy   = 0;
        ix   = c;
        smax = (float)fabs((double)b_x[c]);
        for (jy = 2; jy <= 3 - j; jy++) {
            ix++;
            s = (float)fabs((double)b_x[ix]);
            if (s > smax) {
                iy   = jy - 1;
                smax = s;
            }
        }

        if (b_x[c + iy] != 0.0F) {
            if (iy != 0) {
                ipiv[j] = (signed char)((j + iy) + 1);
                ix      = j;
                iy += j;
                for (jy = 0; jy < 3; jy++) {
                    smax    = b_x[ix];
                    b_x[ix] = b_x[iy];
                    b_x[iy] = smax;
                    ix += 3;
                    iy += 3;
                }
            }

            i0 = (c - j) + 3;
            for (iy = c + 1; iy < i0; iy++) {
                b_x[iy] /= b_x[c];
            }
        }

        iy = c;
        jy = c + 3;
        for (b_j = 1; b_j <= 2 - j; b_j++) {
            smax = b_x[jy];
            if (b_x[jy] != 0.0F) {
                ix = c + 1;
                i0 = (iy - j) + 6;
                for (ijA = 4 + iy; ijA < i0; ijA++) {
                    b_x[ijA] += b_x[ix] * -smax;
                    ix++;
                }
            }

            jy += 3;
            iy += 3;
        }
    }

    y     = b_x[0];
    isodd = false;
    for (jy = 0; jy < 2; jy++) {
        y *= b_x[(jy + 3 * (jy + 1)) + 1];
        if (ipiv[jy] > 1 + jy) {
            isodd = !isodd;
        }
    }

    if (isodd) {
        y = -y;
    }

    return y;
}

//
// File trailer for det.cpp
//
// [EOF]
//
