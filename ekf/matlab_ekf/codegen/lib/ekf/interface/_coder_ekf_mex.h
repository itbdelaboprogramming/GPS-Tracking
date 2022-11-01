//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_ekf_mex.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Nov-2022 22:36:16
//

#ifndef _CODER_EKF_MEX_H
#define _CODER_EKF_MEX_H

// Include Files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

// Function Declarations
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS();

void unsafe_ekf_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
                            const mxArray *prhs[6]);

#endif
//
// File trailer for _coder_ekf_mex.h
//
// [EOF]
//
