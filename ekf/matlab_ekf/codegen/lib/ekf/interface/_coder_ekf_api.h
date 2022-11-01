//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_ekf_api.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Nov-2022 22:36:16
//

#ifndef _CODER_EKF_API_H
#define _CODER_EKF_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void ekf(real_T dt, real_T lat, real_T lon, real_T psi_1dot, real_T V,
         real_T V_1dot, real_T P[4]);

void ekf_api(const mxArray *const prhs[6], const mxArray **plhs);

void ekf_atexit();

void ekf_initialize();

void ekf_terminate();

void ekf_xil_shutdown();

void ekf_xil_terminate();

#endif
//
// File trailer for _coder_ekf_api.h
//
// [EOF]
//
