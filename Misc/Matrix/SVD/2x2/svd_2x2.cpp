//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 22:13:44
//

// Include Files
#include "svd_2x2.h"
#include "rt_nonfinite.h"
#include "svd_2x2_impl.h"
#include <arm_math.h>

// Function Definitions

//
// Arguments    : const float A[4]
//                float U[4]
//                float S[4]
//                float V[4]
// Return Type  : void
//
void svd_2x2(const float Ain[4], float U[4], float S[4], float V[4])
{
    boolean_T p;
    int       i;
    float     s[2];
    float     fv0[4];
    float     U1[4];
    float     V1[4];
    p = true;

    arm_matrix_instance_f32 Ain_;
    arm_mat_init_f32(&Ain_, 2, 2, (float32_t*)Ain);
    float                   A[2 * 2];
    arm_matrix_instance_f32 A_;
    arm_mat_init_f32(&A_, 2, 2, (float32_t*)A);
    arm_mat_trans_f32(&Ain_, &A_);

    for (i = 0; i < 4; i++) {
        if (p && ((!rtIsInfF(A[i])) && (!rtIsNaNF(A[i])))) {
            p = true;
        } else {
            p = false;
        }
    }

    if (p) {
        b_svd_2x2(A, U, s, V);
    } else {
        for (i = 0; i < 4; i++) {
            fv0[i] = 0.0F;
        }

        b_svd_2x2(fv0, U1, s, V1);
        for (i = 0; i < 4; i++) {
            U[i] = ((real32_T)rtNaN);
        }

        for (i = 0; i < 2; i++) {
            s[i] = ((real32_T)rtNaN);
        }

        for (i = 0; i < 4; i++) {
            V[i] = ((real32_T)rtNaN);
        }
    }

    for (i = 0; i < 4; i++) {
        S[i] = 0.0F;
    }

    for (i = 0; i < 2; i++) {
        S[i + (i << 1)] = s[i];
    }

    /* Transpose matrices since MATLAB coder export of SVD with row-major order does not seem to work properly */
    arm_matrix_instance_f32 U_;
    arm_mat_init_f32(&U_, 2, 2, (float32_t*)U);
    arm_matrix_instance_f32 S_;
    arm_mat_init_f32(&S_, 2, 2, (float32_t*)S);
    arm_matrix_instance_f32 V_;
    arm_mat_init_f32(&V_, 2, 2, (float32_t*)V);
    arm_mat_trans_f32(&U_, &A_);
    arm_copy_f32(A, U, 4);
    arm_mat_trans_f32(&S_, &A_);
    arm_copy_f32(A, S, 4);
    arm_mat_trans_f32(&V_, &A_);
    arm_copy_f32(A, V, 4);
}

//
// File trailer for svd.cpp
//
// [EOF]
//
