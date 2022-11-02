/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ekf_initialize.c
 *
 * MATLAB Coder version            : 5.4
 * C/C++ source code generated on  : 02-Nov-2022 22:58:20
 */

/* Include Files */
#include "ekf_initialize.h"
#include "ekf.h"
#include "ekf_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void ekf_initialize(void)
{
  x_est_not_empty_init();
  isInitialized_ekf = true;
}

/*
 * File trailer for ekf_initialize.c
 *
 * [EOF]
 */
