//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 22:15:43
//

// Include Files
#include <string.h>
#include "rt_nonfinite.h"
#include "svd_4x4.h"
#include "svd_4x4_impl.h"
#include <arm_math.h>

// Function Definitions

//
// Arguments    : const float A[16]
//                float U[16]
//                float S[16]
//                float V[16]
// Return Type  : void
//
void svd_4x4(const float Ain[16], float U[16], float S[16], float V[16])
{
  boolean_T p;
  int i;
  float fv0[16];
  float s[4];
  float U1[16];
  float V1[16];
  p = true;

  arm_matrix_instance_f32 Ain_; arm_mat_init_f32(&Ain_, 4, 4, (float32_t *)Ain);
  float A[4*4]; arm_matrix_instance_f32 A_; arm_mat_init_f32(&A_, 4, 4, (float32_t *)A);
  arm_mat_trans_f32(&Ain_, &A_);  
  
  for (i = 0; i < 16; i++) {
    if (p && ((!rtIsInfF(A[i])) && (!rtIsNaNF(A[i])))) {
      p = true;
    } else {
      p = false;
    }
  }

  if (p) {
    b_svd_4x4(A, U, s, V);
  } else {
    memset(&fv0[0], 0, sizeof(float) << 4);
    b_svd_4x4(fv0, U1, s, V1);
    for (i = 0; i < 16; i++) {
      U[i] = ((real32_T)rtNaN);
    }

    for (i = 0; i < 4; i++) {
      s[i] = ((real32_T)rtNaN);
    }

    for (i = 0; i < 16; i++) {
      V[i] = ((real32_T)rtNaN);
    }
  }

  memset(&S[0], 0, sizeof(float) << 4);
  for (i = 0; i < 4; i++) {
    S[i + (i << 2)] = s[i];
  }
  
  /* Transpose matrices since MATLAB coder export of SVD with row-major order does not seem to work properly */
  arm_matrix_instance_f32 U_; arm_mat_init_f32(&U_, 4, 4, (float32_t *)U);
  arm_matrix_instance_f32 S_; arm_mat_init_f32(&S_, 4, 4, (float32_t *)S);
  arm_matrix_instance_f32 V_; arm_mat_init_f32(&V_, 4, 4, (float32_t *)V);
  arm_mat_trans_f32(&U_, &A_);
  arm_copy_f32(A, U, 16);
  arm_mat_trans_f32(&S_, &A_);
  arm_copy_f32(A, S, 16);
  arm_mat_trans_f32(&V_, &A_);
  arm_copy_f32(A, V, 16);  
}

//
// File trailer for svd.cpp
//
// [EOF]
//
