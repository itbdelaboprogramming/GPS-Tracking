/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_ekf_api.h
 *
 * MATLAB Coder version            : 5.4
 * C/C++ source code generated on  : 08-Dec-2022 10:22:30
 */

#ifndef _CODER_EKF_API_H
#define _CODER_EKF_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void ekf(real_T mode, real_T dt, real_T lat, real_T lon, real_T psi_1dot,
         real_T V, real_T V_1dot, real_T result_ekf[4]);

void ekf_api(const mxArray *const prhs[7], const mxArray **plhs);

void ekf_atexit(void);

void ekf_initialize(void);

void ekf_terminate(void);

void ekf_xil_shutdown(void);

void ekf_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_ekf_api.h
 *
 * [EOF]
 */
