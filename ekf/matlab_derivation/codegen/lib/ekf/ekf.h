/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ekf.h
 *
 * MATLAB Coder version            : 5.4
 * C/C++ source code generated on  : 08-Dec-2022 10:22:30
 */

#ifndef EKF_H
#define EKF_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void ekf(double mode, double dt, double lat, double lon, double psi_1dot,
                double V, double V_1dot, double result_ekf[4]);

void ekf_init(void);

void x_est_not_empty_init(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for ekf.h
 *
 * [EOF]
 */
