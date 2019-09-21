//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 12-Feb-2019 21:10:49
//

// Include Files
#include "rt_nonfinite.h"
#include "svd.h"
#include "svd1.h"
#include <arm_math.h>

// Function Definitions

//
// Arguments    : const float A[9]
//                float U[9]
//                float S[9]
//                float V[9]
// Return Type  : void
//
void svd(const float Ain[9], float U[9], float S[9], float V[9])
{
  boolean_T p;
  int i;
  float s[3];
  float fv0[9];
  float U1[9];
  float V1[9];
  p = true;

  arm_matrix_instance_f32 Ain_; arm_mat_init_f32(&Ain_, 3, 3, (float32_t *)Ain);
  float A[3*3]; arm_matrix_instance_f32 A_; arm_mat_init_f32(&A_, 3, 3, (float32_t *)A);
  arm_mat_trans_f32(&Ain_, &A_);

  for (i = 0; i < 9; i++) {
    if (p && ((!rtIsInfF(A[i])) && (!rtIsNaNF(A[i])))) {
      p = true;
    } else {
      p = false;
    }
  }

  if (p) {
    b_svd(A, U, s, V);
  } else {
    for (i = 0; i < 9; i++) {
      fv0[i] = 0.0F;
    }

    b_svd(fv0, U1, s, V1);
    for (i = 0; i < 9; i++) {
      U[i] = ((real32_T)rtNaN);
    }

    for (i = 0; i < 3; i++) {
      s[i] = ((real32_T)rtNaN);
    }

    for (i = 0; i < 9; i++) {
      V[i] = ((real32_T)rtNaN);
    }
  }

  for (i = 0; i < 9; i++) {
    S[i] = 0.0F;
  }

  for (i = 0; i < 3; i++) {
    S[i + 3 * i] = s[i];
  }

  /* Transpose matrices since MATLAB coder export of SVD with row-major order does not seem to work properly */
  arm_matrix_instance_f32 U_; arm_mat_init_f32(&U_, 3, 3, (float32_t *)U);
  arm_matrix_instance_f32 S_; arm_mat_init_f32(&S_, 3, 3, (float32_t *)S);
  arm_matrix_instance_f32 V_; arm_mat_init_f32(&V_, 3, 3, (float32_t *)V);
  float tmp[3*3]; arm_matrix_instance_f32 tmp_; arm_mat_init_f32(&tmp_, 3, 3, (float32_t *)tmp);
  arm_mat_trans_f32(&U_, &tmp_);
  arm_copy_f32(tmp, U, 9);
  arm_mat_trans_f32(&S_, &tmp_);
  arm_copy_f32(tmp, S, 9);
  arm_mat_trans_f32(&V_, &tmp_);
  arm_copy_f32(tmp, V, 9);
}

//
// File trailer for svd.cpp
//
// [EOF]
//
