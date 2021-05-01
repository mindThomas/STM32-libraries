//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: det.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 22:04:44
//

// Include Files
#include <math.h>
#include <string.h>
//#include "rt_nonfinite.h"
#include "det4x4.h"

// Function Definitions

//
// Arguments    : const float x[16]
// Return Type  : float
//
float det4x4(const float x[16])
{
    float       y;
    float       b_x[16];
    int         i0;
    int         j;
    signed char ipiv[4];
    int         c;
    boolean_T   isodd;
    int         iy;
    int         jy;
    int         ix;
    float       smax;
    float       s;
    int         b_j;
    int         ijA;
    memcpy(&b_x[0], &x[0], sizeof(float) << 4);
    for (i0 = 0; i0 < 4; i0++) {
        ipiv[i0] = (signed char)(1 + i0);
    }

    for (j = 0; j < 3; j++) {
        c    = j * 5;
        iy   = 0;
        ix   = c;
        smax = (float)fabs((double)b_x[c]);
        for (jy = 2; jy <= 4 - j; jy++) {
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
                for (jy = 0; jy < 4; jy++) {
                    smax    = b_x[ix];
                    b_x[ix] = b_x[iy];
                    b_x[iy] = smax;
                    ix += 4;
                    iy += 4;
                }
            }

            i0 = (c - j) + 4;
            for (iy = c + 1; iy < i0; iy++) {
                b_x[iy] /= b_x[c];
            }
        }

        iy = c;
        jy = c + 4;
        for (b_j = 1; b_j <= 3 - j; b_j++) {
            smax = b_x[jy];
            if (b_x[jy] != 0.0F) {
                ix = c + 1;
                i0 = (iy - j) + 8;
                for (ijA = 5 + iy; ijA < i0; ijA++) {
                    b_x[ijA] += b_x[ix] * -smax;
                    ix++;
                }
            }

            jy += 4;
            iy += 4;
        }
    }

    y     = b_x[0];
    isodd = false;
    for (jy = 0; jy < 3; jy++) {
        y *= b_x[(jy + ((jy + 1) << 2)) + 1];
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
