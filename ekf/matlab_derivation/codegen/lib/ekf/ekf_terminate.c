/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ekf_terminate.c
 *
 * MATLAB Coder version            : 5.4
 * C/C++ source code generated on  : 04-Nov-2022 07:25:36
 */

/* Include Files */
#include "ekf_terminate.h"
#include "ekf_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void ekf_terminate(void)
{
  isInitialized_ekf = false;
}

/*
 * File trailer for ekf_terminate.c
 *
 * [EOF]
 */
