//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: exp.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 08-May-2020 23:23:34
//

// Include Files
#include "exp.h"
#include "fitExponentialDecay2.h"
#include <MATLABCoderInit/rt_nonfinite.h>
#include <math.h>

// Function Definitions

//
// Arguments    : emxArray_real32_T *x
// Return Type  : void
//
void b_exp(emxArray_real32_T* x)
{
    int i2;
    int k;
    int b_x[2];
    int c_x[2];
    int i3;
    int d_x[2];
    int e_x[2];
    i2 = x->size[0];
    for (k = 1; k <= i2; k++) {
        b_x[0] = x->size[0];
        b_x[1] = 1;
        c_x[0] = x->size[0];
        c_x[1] = 1;
        for (i3 = 0; i3 < 2; i3++) {
            e_x[i3] = b_x[1 - i3];
            d_x[i3] = c_x[1 - i3];
        }

        x->data[d_x[0] * (k - 1)] = (float)exp((double)x->data[e_x[0] * (k - 1)]);
    }
}

//
// File trailer for exp.cpp
//
// [EOF]
//
