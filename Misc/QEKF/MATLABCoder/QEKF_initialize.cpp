//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: QEKF_initialize.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 29-Oct-2018 20:49:17
//

// Include Files
#include "QEKF_initialize.h"
#include "QEKF_coder.h"
#include <MATLABCoderInit/rt_nonfinite.h>

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void QEKF_initialize(const float P_diagonal_init[10], float X[10], float P[10 * 10])
{
    for (int i = 0; i < 10; i++) {
        X[i] = 0.f;
    }
    for (int i = 0; i < 100; i++) {
        P[i] = 0.f;
    }

    // Set the initial state vector
    // X = [eul2quat(deg2rad([0,10,20]), 'ZYX')'; zeros(4,1); zeros(2,1)];
    X[0] = 1.f; // initialize as unit quaternion

    // Set diagonal elements of the covariance P
    for (int i = 0; i < 10; i++) {
        P[11 * i] = P_diagonal_init[i];
    }

    acc_norm_old_not_empty_init();
    acc_norm_filtered_not_empty_init();
}

//
// File trailer for QEKF_initialize.cpp
//
// [EOF]
//
