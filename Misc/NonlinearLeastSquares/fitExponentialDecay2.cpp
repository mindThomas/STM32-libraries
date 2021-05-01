//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fitExponentialDecay2.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 08-May-2020 23:23:34
//

// Include Files
#include "fitExponentialDecay2.h"
#include "exp.h"
#include "fitExponentialDecay2_emxutil.h"
#include "mw_cmsis.h"
#include "rt_nonfinite.h"
#include <math.h>

// Function Declarations
static float __anon_fcn(const cell_wrap_1 error_tunableEnvironment[2], float a, float b, float c);
static float rt_powf_snf(float u0, float u1);

// Function Definitions

//
// Arguments    : const cell_wrap_1 error_tunableEnvironment[2]
//                float a
//                float b
//                float c
// Return Type  : float
//
static float __anon_fcn(const cell_wrap_1 error_tunableEnvironment[2], float a, float b, float c)
{
    float              varargout_1;
    emxArray_real32_T* b_varargout_1;
    int                i1;
    int                firstBlockLength;
    emxArray_real32_T* y;
    unsigned int       varargout_1_idx_0;
    int                k;
    unsigned int       c_varargout_1[2];
    int                vlen;
    unsigned int       b_y[2];
    int                nblocks;
    int                lastBlockLength;
    int                c_y[2];
    int                d_varargout_1[2];
    int                offset;
    float              bsum;
    int                hi;
    emxInit_real32_T(&b_varargout_1, 1);
    i1                     = b_varargout_1->size[0];
    b_varargout_1->size[0] = error_tunableEnvironment[1].f1->size[0];
    emxEnsureCapacity_real32_T(b_varargout_1, i1);
    firstBlockLength = error_tunableEnvironment[1].f1->size[0];
    for (i1 = 0; i1 < firstBlockLength; i1++) {
        b_varargout_1->data[i1] = c * error_tunableEnvironment[1].f1->data[i1];
    }

    b_exp(b_varargout_1);
    i1                     = b_varargout_1->size[0];
    b_varargout_1->size[0] = error_tunableEnvironment[0].f1->size[0];
    emxEnsureCapacity_real32_T(b_varargout_1, i1);
    firstBlockLength = error_tunableEnvironment[0].f1->size[0];
    for (i1 = 0; i1 < firstBlockLength; i1++) {
        b_varargout_1->data[i1] = error_tunableEnvironment[0].f1->data[i1] - (a + b * b_varargout_1->data[i1]);
    }

    emxInit_real32_T(&y, 1);
    varargout_1_idx_0 = (unsigned int)b_varargout_1->size[0];
    i1                = y->size[0];
    y->size[0]        = (int)varargout_1_idx_0;
    emxEnsureCapacity_real32_T(y, i1);
    varargout_1_idx_0 = (unsigned int)b_varargout_1->size[0];
    for (k = 1; k <= (int)varargout_1_idx_0; k++) {
        c_varargout_1[0] = (unsigned int)b_varargout_1->size[0];
        c_varargout_1[1] = 1U;
        b_y[0]           = (unsigned int)y->size[0];
        b_y[1]           = 1U;
        for (i1 = 0; i1 < 2; i1++) {
            d_varargout_1[i1] = (int)c_varargout_1[1 - i1];
            c_y[i1]           = (int)b_y[1 - i1];
        }

        y->data[c_y[0] * (k - 1)] = rt_powf_snf(b_varargout_1->data[d_varargout_1[0] * (k - 1)], 2.0F);
    }

    emxFree_real32_T(&b_varargout_1);
    vlen = y->size[0];
    if (y->size[0] == 0) {
        varargout_1 = 0.0F;
    } else {
        if (y->size[0] <= 1024) {
            firstBlockLength = y->size[0];
            lastBlockLength  = 0;
            nblocks          = 1;
        } else {
            firstBlockLength = 1024;
            nblocks          = y->size[0] / 1024;
            lastBlockLength  = y->size[0] - (nblocks << 10);
            if (lastBlockLength > 0) {
                nblocks++;
            } else {
                lastBlockLength = 1024;
            }
        }

        varargout_1 = y->data[0];
        for (k = 2; k <= firstBlockLength; k++) {
            if (vlen >= 2) {
                b_y[0] = (unsigned int)y->size[0];
                b_y[1] = 1U;
                for (i1 = 0; i1 < 2; i1++) {
                    c_y[i1] = (int)b_y[1 - i1];
                }

                varargout_1 += y->data[c_y[0] * (k - 1)];
            }
        }

        for (firstBlockLength = 2; firstBlockLength <= nblocks; firstBlockLength++) {
            offset = (firstBlockLength - 1) << 10;
            b_y[0] = (unsigned int)y->size[0];
            b_y[1] = 1U;
            for (i1 = 0; i1 < 2; i1++) {
                c_y[i1] = (int)b_y[1 - i1];
            }

            bsum = y->data[c_y[0] * offset];
            if (firstBlockLength == nblocks) {
                hi = lastBlockLength;
            } else {
                hi = 1024;
            }

            for (k = 1; k < hi; k++) {
                if (vlen >= 2) {
                    b_y[0] = (unsigned int)y->size[0];
                    b_y[1] = 1U;
                    for (i1 = 0; i1 < 2; i1++) {
                        c_y[i1] = (int)b_y[1 - i1];
                    }

                    bsum += y->data[c_y[0] * (offset + k)];
                }
            }

            varargout_1 += bsum;
        }
    }

    emxFree_real32_T(&y);
    return varargout_1;
}

//
// Arguments    : float u0
//                float u1
// Return Type  : float
//
static float rt_powf_snf(float u0, float u1)
{
    float y;
    float f1;
    float f2;
    if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
        y = ((real32_T)rtNaN);
    } else {
        f1 = (float)fabs((double)u0);
        f2 = (float)fabs((double)u1);
        if (rtIsInfF(u1)) {
            if (f1 == 1.0F) {
                y = 1.0F;
            } else if (f1 > 1.0F) {
                if (u1 > 0.0F) {
                    y = ((real32_T)rtInf);
                } else {
                    y = 0.0F;
                }
            } else if (u1 > 0.0F) {
                y = 0.0F;
            } else {
                y = ((real32_T)rtInf);
            }
        } else if (f2 == 0.0F) {
            y = 1.0F;
        } else if (f2 == 1.0F) {
            if (u1 > 0.0F) {
                y = u0;
            } else {
                y = 1.0F / u0;
            }
        } else if (u1 == 2.0F) {
            y = u0 * u0;
        } else if ((u1 == 0.5F) && (u0 >= 0.0F)) {
            mw_arm_sqrt_f32(u0, &y);
        } else if ((u0 < 0.0F) && (u1 > (float)floor((double)u1))) {
            y = ((real32_T)rtNaN);
        } else {
            y = (float)pow((double)u0, (double)u1);
        }
    }

    return y;
}

//
// function [a, b, c] = fitExponentialDecay2(x, y)
// To make initialization point valid, subtract starting point from x
//  We can always shift the results
// Arguments    : emxArray_real32_T *x
//                const emxArray_real32_T *y
//                float *a
//                float *b
//                float *c
// Return Type  : void
//
void fitExponentialDecay2(emxArray_real32_T* x, const emxArray_real32_T* y, float* a, float* b, float* c)
{
    float              x0;
    float              temp;
    int                i0;
    int                p1;
    cell_wrap_1        tunableEnvironment[2];
    cell_wrap_1        error_tunableEnvironment[2];
    emxArray_real32_T* tunableEnvironment_f2;
    int                tunableEnvironment_f1;
    emxArray_real32_T* de_dC_tunableEnvironment_f2;
    float              coeffs[3];
    emxArray_real32_T* Jr_;
    emxArray_real32_T* varargin_1;
    emxArray_real32_T* varargin_2;
    emxArray_real32_T* varargin_3;
    emxArray_real32_T* b_a;
    int                exitg1;
    int                p3;
    int                p2;
    int                itmp;
    float              b_x[9];
    float              b_y[9];
    float              absx21;
    float              absx31;
    float              b_varargin_1[3];
    float              c_y[3];
    unsigned int       c_varargin_1[2];
    float              delta[3];
    int                d_varargin_1[2];
    float              f0;
    float              t;

    //  x and y should be column vectors; m x 1
    //  f(x) = a + b * exp(c*x)
    // 'fitExponentialDecay2:7' x0 = x(1);
    x0 = x->data[0];

    // 'fitExponentialDecay2:8' x = x - x0;
    temp = x->data[0];
    i0   = x->size[0];
    emxEnsureCapacity_real32_T(x, i0);
    p1 = x->size[0];
    for (i0 = 0; i0 < p1; i0++) {
        x->data[i0] -= temp;
    }

    emxInitMatrix_cell_wrap_1(tunableEnvironment);

    // 'fitExponentialDecay2:9' n = length(x);
    //  Formulate error function
    // 'fitExponentialDecay2:12' model = @(x, a, b, c) a + b.*exp(c.*x);
    // 'fitExponentialDecay2:13' error = @(a, b, c) y - model(x, a, b, c);
    i0                                = tunableEnvironment[0].f1->size[0];
    tunableEnvironment[0].f1->size[0] = y->size[0];
    emxEnsureCapacity_real32_T(tunableEnvironment[0].f1, i0);
    p1 = y->size[0];
    for (i0 = 0; i0 < p1; i0++) {
        tunableEnvironment[0].f1->data[i0] = y->data[i0];
    }

    i0                                = tunableEnvironment[1].f1->size[0];
    tunableEnvironment[1].f1->size[0] = x->size[0];
    emxEnsureCapacity_real32_T(tunableEnvironment[1].f1, i0);
    p1 = x->size[0];
    for (i0 = 0; i0 < p1; i0++) {
        tunableEnvironment[1].f1->data[i0] = x->data[i0];
    }

    emxInitMatrix_cell_wrap_1(error_tunableEnvironment);
    for (i0 = 0; i0 < 2; i0++) {
        emxCopyStruct_cell_wrap_1(&error_tunableEnvironment[i0], &tunableEnvironment[i0]);
    }

    emxFreeMatrix_cell_wrap_1(tunableEnvironment);
    emxInit_real32_T(&tunableEnvironment_f2, 1);

    // 'fitExponentialDecay2:14' de_dC = @(a, b, c) -[single(ones(n,1)), exp(c*x), b .* x.*exp(c*x)];
    tunableEnvironment_f1          = x->size[0];
    i0                             = tunableEnvironment_f2->size[0];
    tunableEnvironment_f2->size[0] = x->size[0];
    emxEnsureCapacity_real32_T(tunableEnvironment_f2, i0);
    p1 = x->size[0];
    for (i0 = 0; i0 < p1; i0++) {
        tunableEnvironment_f2->data[i0] = x->data[i0];
    }

    emxInit_real32_T(&de_dC_tunableEnvironment_f2, 1);
    i0                                   = de_dC_tunableEnvironment_f2->size[0];
    de_dC_tunableEnvironment_f2->size[0] = tunableEnvironment_f2->size[0];
    emxEnsureCapacity_real32_T(de_dC_tunableEnvironment_f2, i0);
    p1 = tunableEnvironment_f2->size[0];
    for (i0 = 0; i0 < p1; i0++) {
        de_dC_tunableEnvironment_f2->data[i0] = tunableEnvironment_f2->data[i0];
    }

    // 'fitExponentialDecay2:15' cost = @(a, b, c) sum( error(a, b, c).^2 );
    //  Compute Jacobian
    // 'fitExponentialDecay2:18' J = @(a, b, c) 2*sum( error(a, b, c) .* de_dC(a, b, c) );
    // 'fitExponentialDecay2:19' Jr = @(a, b, c) de_dC(a, b, c);
    //  Jacobian of residual
    //     %% Perform Gauss-Newton algorithm
    //  Initialization point
    // 'fitExponentialDecay2:23' a = single(0);
    // 'fitExponentialDecay2:24' b = y(1);
    // 'fitExponentialDecay2:25' c = single(-0.1);
    // 'fitExponentialDecay2:27' coeffs = [a, b, c]';
    coeffs[0] = 0.0F;
    coeffs[1] = y->data[0];
    coeffs[2] = -0.1F;

    // 'fitExponentialDecay2:29' delta = single([inf, inf, inf])';
    // 'fitExponentialDecay2:30' prev_delta = single([0, 0, 0])';
    // 'fitExponentialDecay2:31' new_cost = cost(coeffs(1), coeffs(2), coeffs(3));
    __anon_fcn(error_tunableEnvironment, 0.0F, coeffs[1], -0.1F);

    // 'fitExponentialDecay2:32' while (true)
    emxInit_real32_T1(&Jr_, 2);
    emxInit_real32_T(&varargin_1, 1);
    emxInit_real32_T(&varargin_2, 1);
    emxInit_real32_T(&varargin_3, 1);
    emxInit_real32_T1(&b_a, 2);
    do {
        exitg1 = 0;

        // 'fitExponentialDecay2:33' Jr_ = Jr(coeffs(1), coeffs(2), coeffs(3));
        i0                  = varargin_1->size[0];
        varargin_1->size[0] = tunableEnvironment_f1;
        emxEnsureCapacity_real32_T(varargin_1, i0);
        for (i0 = 0; i0 < tunableEnvironment_f1; i0++) {
            varargin_1->data[i0] = 1.0F;
        }

        i0                  = varargin_2->size[0];
        varargin_2->size[0] = tunableEnvironment_f2->size[0];
        emxEnsureCapacity_real32_T(varargin_2, i0);
        p1 = tunableEnvironment_f2->size[0];
        for (i0 = 0; i0 < p1; i0++) {
            varargin_2->data[i0] = coeffs[2] * tunableEnvironment_f2->data[i0];
        }

        b_exp(varargin_2);
        i0                  = varargin_3->size[0];
        varargin_3->size[0] = de_dC_tunableEnvironment_f2->size[0];
        emxEnsureCapacity_real32_T(varargin_3, i0);
        p1 = de_dC_tunableEnvironment_f2->size[0];
        for (i0 = 0; i0 < p1; i0++) {
            varargin_3->data[i0] = coeffs[2] * de_dC_tunableEnvironment_f2->data[i0];
        }

        b_exp(varargin_3);
        i0                  = varargin_3->size[0];
        varargin_3->size[0] = tunableEnvironment_f2->size[0];
        emxEnsureCapacity_real32_T(varargin_3, i0);
        p1 = tunableEnvironment_f2->size[0];
        for (i0 = 0; i0 < p1; i0++) {
            varargin_3->data[i0] *= coeffs[1] * tunableEnvironment_f2->data[i0];
        }

        p3           = varargin_1->size[0];
        p1           = varargin_2->size[0];
        p2           = varargin_3->size[0];
        i0           = Jr_->size[0] * Jr_->size[1];
        Jr_->size[1] = 3;
        Jr_->size[0] = p3;
        emxEnsureCapacity_real32_T1(Jr_, i0);
        for (i0 = 0; i0 < p3; i0++) {
            Jr_->data[Jr_->size[1] * i0] = -varargin_1->data[i0];
        }

        for (i0 = 0; i0 < p1; i0++) {
            Jr_->data[1 + Jr_->size[1] * i0] = -varargin_2->data[i0];
        }

        for (i0 = 0; i0 < p2; i0++) {
            Jr_->data[2 + Jr_->size[1] * i0] = -varargin_3->data[i0];
        }

        // 'fitExponentialDecay2:34' delta = inv(Jr_'*Jr_) * (Jr_' * error(coeffs(1), coeffs(2), coeffs(3)));
        i0           = b_a->size[0] * b_a->size[1];
        b_a->size[1] = Jr_->size[0];
        b_a->size[0] = 3;
        emxEnsureCapacity_real32_T1(b_a, i0);
        for (i0 = 0; i0 < 3; i0++) {
            p1 = Jr_->size[0];
            for (itmp = 0; itmp < p1; itmp++) {
                b_a->data[itmp + b_a->size[1] * i0] = Jr_->data[i0 + Jr_->size[1] * itmp];
            }
        }

        if ((b_a->size[1] == 1) || (Jr_->size[0] == 1)) {
            p1 = Jr_->size[0];
            for (i0 = 0; i0 < 3; i0++) {
                for (itmp = 0; itmp < 3; itmp++) {
                    b_y[i0 + 3 * itmp] = 0.0F;
                    for (p2 = 0; p2 < p1; p2++) {
                        b_y[i0 + 3 * itmp] += Jr_->data[i0 + Jr_->size[1] * p2] * b_a->data[p2 + b_a->size[1] * itmp];
                    }
                }
            }
        } else {
            for (p2 = 0; p2 < 3; p2++) {
                for (p1 = 0; p1 < 3; p1++) {
                    temp = 0.0F;
                    for (p3 = 0; p3 < b_a->size[1]; p3++) {
                        temp += b_a->data[p3 + b_a->size[1] * p2] * Jr_->data[p1 + Jr_->size[1] * p3];
                    }

                    b_y[p1 + 3 * p2] = temp;
                }
            }
        }

        for (i0 = 0; i0 < 9; i0++) {
            b_x[i0] = b_y[i0];
        }

        p1     = 0;
        p2     = 3;
        p3     = 6;
        temp   = (float)fabs((double)b_y[0]);
        absx21 = (float)fabs((double)b_y[3]);
        absx31 = (float)fabs((double)b_y[6]);
        if ((absx21 > temp) && (absx21 > absx31)) {
            p1     = 3;
            p2     = 0;
            b_x[0] = b_y[3];
            b_x[3] = b_y[0];
            b_x[1] = b_y[4];
            b_x[4] = b_y[1];
            b_x[2] = b_y[5];
            b_x[5] = b_y[2];
        } else {
            if (absx31 > temp) {
                p1     = 6;
                p3     = 0;
                b_x[0] = b_y[6];
                b_x[6] = b_y[0];
                b_x[1] = b_y[7];
                b_x[7] = b_y[1];
                b_x[2] = b_y[8];
                b_x[8] = b_y[2];
            }
        }

        temp = b_x[3] / b_x[0];
        b_x[3] /= b_x[0];
        absx21 = b_x[6] / b_x[0];
        b_x[6] /= b_x[0];
        b_x[4] -= temp * b_x[1];
        b_x[7] -= absx21 * b_x[1];
        b_x[5] -= temp * b_x[2];
        b_x[8] -= absx21 * b_x[2];
        if ((float)fabs((double)b_x[7]) > (float)fabs((double)b_x[4])) {
            itmp   = p2;
            p2     = p3;
            p3     = itmp;
            b_x[3] = absx21;
            b_x[6] = temp;
            temp   = b_x[4];
            b_x[4] = b_x[7];
            b_x[7] = temp;
            temp   = b_x[5];
            b_x[5] = b_x[8];
            b_x[8] = temp;
        }

        temp = b_x[7] / b_x[4];
        b_x[7] /= b_x[4];
        b_x[8] -= temp * b_x[5];
        temp                                 = (b_x[7] * b_x[3] - b_x[6]) / b_x[8];
        absx21                               = -(b_x[3] + b_x[5] * temp) / b_x[4];
        b_y[p1 % 3 * 3 + p1 / 3]             = ((1.0F - b_x[1] * absx21) - b_x[2] * temp) / b_x[0];
        b_y[(p1 + 1) % 3 * 3 + (p1 + 1) / 3] = absx21;
        b_y[(p1 + 2) % 3 * 3 + (p1 + 2) / 3] = temp;
        temp                                 = -b_x[7] / b_x[8];
        absx21                               = (1.0F - b_x[5] * temp) / b_x[4];
        b_y[p2 % 3 * 3 + p2 / 3]             = -(b_x[1] * absx21 + b_x[2] * temp) / b_x[0];
        b_y[(p2 + 1) % 3 * 3 + (p2 + 1) / 3] = absx21;
        b_y[(p2 + 2) % 3 * 3 + (p2 + 2) / 3] = temp;
        temp                                 = 1.0F / b_x[8];
        absx21                               = -b_x[5] * temp / b_x[4];
        b_y[p3 % 3 * 3 + p3 / 3]             = -(b_x[1] * absx21 + b_x[2] * temp) / b_x[0];
        b_y[(p3 + 1) % 3 * 3 + (p3 + 1) / 3] = absx21;
        b_y[(p3 + 2) % 3 * 3 + (p3 + 2) / 3] = temp;
        i0                                   = varargin_1->size[0];
        varargin_1->size[0]                  = x->size[0];
        emxEnsureCapacity_real32_T(varargin_1, i0);
        p1 = x->size[0];
        for (i0 = 0; i0 < p1; i0++) {
            varargin_1->data[i0] = coeffs[2] * x->data[i0];
        }

        b_exp(varargin_1);
        i0                  = varargin_1->size[0];
        varargin_1->size[0] = y->size[0];
        emxEnsureCapacity_real32_T(varargin_1, i0);
        p1 = y->size[0];
        for (i0 = 0; i0 < p1; i0++) {
            varargin_1->data[i0] = y->data[i0] - (coeffs[0] + coeffs[1] * varargin_1->data[i0]);
        }

        i0           = b_a->size[0] * b_a->size[1];
        b_a->size[1] = Jr_->size[0];
        b_a->size[0] = 3;
        emxEnsureCapacity_real32_T1(b_a, i0);
        for (i0 = 0; i0 < 3; i0++) {
            p1 = Jr_->size[0];
            for (itmp = 0; itmp < p1; itmp++) {
                b_a->data[itmp + b_a->size[1] * i0] = Jr_->data[i0 + Jr_->size[1] * itmp];
            }
        }

        if ((b_a->size[1] == 1) || (varargin_1->size[0] == 1)) {
            p3 = varargin_1->size[0];
            for (i0 = 0; i0 < 3; i0++) {
                b_varargin_1[i0] = 0.0F;
                for (itmp = 0; itmp < p3; itmp++) {
                    temp             = b_varargin_1[i0] + varargin_1->data[itmp] * b_a->data[itmp + b_a->size[1] * i0];
                    b_varargin_1[i0] = temp;
                }

                c_y[i0] = b_varargin_1[i0];
            }
        } else {
            for (p2 = 0; p2 < 3; p2++) {
                temp = 0.0F;
                for (p3 = 0; p3 < b_a->size[1]; p3++) {
                    c_varargin_1[0] = (unsigned int)varargin_1->size[0];
                    c_varargin_1[1] = 1U;
                    for (i0 = 0; i0 < 2; i0++) {
                        d_varargin_1[i0] = (int)c_varargin_1[1 - i0];
                    }

                    temp += b_a->data[p3 + b_a->size[1] * p2] * varargin_1->data[d_varargin_1[0] * p3];
                }

                c_y[p2] = temp;
            }
        }

        // 'fitExponentialDecay2:35' coeffs = coeffs - delta;
        for (i0 = 0; i0 < 3; i0++) {
            b_varargin_1[i0] = 0.0F;
            for (itmp = 0; itmp < 3; itmp++) {
                b_varargin_1[i0] += c_y[itmp] * b_y[itmp + 3 * i0];
            }

            delta[i0] = b_varargin_1[i0];
            coeffs[i0] -= b_varargin_1[i0];
        }

        // 'fitExponentialDecay2:36' if (coeffs(3) > 0)
        if (coeffs[2] > 0.0F) {
            // 'fitExponentialDecay2:37' coeffs(3) = single(-0.1);
            coeffs[2] = -0.1F;
        }

        //  check for convergence
        // 'fitExponentialDecay2:41' if (norm(delta) < 1/100*abs(coeffs(3)) )
        temp   = 0.0F;
        absx21 = 1.29246971E-26F;
        for (p3 = 0; p3 < 3; p3++) {
            absx31 = (float)fabs((double)delta[p3]);
            if (absx31 > absx21) {
                t      = absx21 / absx31;
                temp   = 1.0F + temp * t * t;
                absx21 = absx31;
            } else {
                t = absx31 / absx21;
                temp += t * t;
            }
        }

        mw_arm_sqrt_f32(temp, &f0);
        temp = absx21 * f0;
        if (temp < 0.01F * (float)fabs((double)coeffs[2])) {
            exitg1 = 1;
        } else {
            // 'fitExponentialDecay2:45' new_cost = cost(coeffs(1), coeffs(2), coeffs(3));
            __anon_fcn(error_tunableEnvironment, coeffs[0], coeffs[1], coeffs[2]);

            // 'fitExponentialDecay2:46' prev_delta = delta;
        }
    } while (exitg1 == 0);

    emxFree_real32_T(&b_a);
    emxFree_real32_T(&varargin_3);
    emxFree_real32_T(&varargin_2);
    emxFree_real32_T(&varargin_1);
    emxFree_real32_T(&tunableEnvironment_f2);
    emxFree_real32_T(&Jr_);
    emxFree_real32_T(&de_dC_tunableEnvironment_f2);
    emxFreeMatrix_cell_wrap_1(error_tunableEnvironment);

    // 'fitExponentialDecay2:49' a = coeffs(1);
    *a = coeffs[0];

    // 'fitExponentialDecay2:50' c = coeffs(3);
    *c = coeffs[2];

    // 'fitExponentialDecay2:51' b = coeffs(2) * exp(c * (-x0));
    *b = coeffs[1] * (float)exp((double)(coeffs[2] * -x0));

    //  correct parameter with offset shift
}

//
// File trailer for fitExponentialDecay2.cpp
//
// [EOF]
//
