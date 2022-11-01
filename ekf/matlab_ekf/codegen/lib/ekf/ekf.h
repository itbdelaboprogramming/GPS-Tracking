//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ekf.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Nov-2022 22:36:16
//

#ifndef EKF_H
#define EKF_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void ekf(double dt, double lat, double lon, double psi_1dot, double V,
                double V_1dot, double P[4]);

void x_est_not_empty_init();

#endif
//
// File trailer for ekf.h
//
// [EOF]
//
