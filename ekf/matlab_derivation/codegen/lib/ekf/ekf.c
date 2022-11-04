/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ekf.c
 *
 * MATLAB Coder version            : 5.4
 * C/C++ source code generated on  : 04-Nov-2022 07:25:36
 */

/* Include Files */
#include "ekf.h"
#include "cosd.h"
#include "ekf_data.h"
#include "ekf_initialize.h"
#include "rt_nonfinite.h"
#include "sind.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Variable Definitions */
static boolean_T x_est_not_empty;

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);

static double rt_hypotd_snf(double u0, double u1);

static double rt_powd_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int b_u0;
    int b_u1;
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }
    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }
    y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }
  return y;
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_hypotd_snf(double u0, double u1)
{
  double a;
  double y;
  a = fabs(u0);
  y = fabs(u1);
  if (a < y) {
    a /= y;
    y *= sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = a * sqrt(y * y + 1.0);
  } else if (!rtIsNaN(y)) {
    y = a * 1.4142135623730951;
  }
  return y;
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }
  return y;
}

/*
 * KALMAN FILTER VARIABLE & TUNING PARAMETER
 *  dt = time increment [s]
 *  zx,zy = latitude,longitude GPS measurment [deg]
 *  psi_1dot = turning velocity [rad/s]
 *  V = linear velocity [m/s]
 *  V_1dot = linear acceleration [m/s2]
 *
 * Arguments    : double dt
 *                double lat
 *                double lon
 *                double psi_1dot
 *                double V
 *                double V_1dot
 *                double result_ekf[4]
 * Return Type  : void
 */
void ekf(double dt, double lat, double lon, double psi_1dot, double V,
         double V_1dot, double result_ekf[4])
{
  static double p_est[16];
  static double x_est[4];
  double b_x_prd[16];
  double jac_fx[16];
  double p_prd[16];
  double A[12];
  double K[12];
  double ecefPosWithENUOrigin[12];
  double jac_hx[12];
  double b_jac_hx[9];
  double dv[9];
  double x_prd[4];
  double Cv;
  double N;
  double a21;
  double b_N;
  double b_sinphi;
  double c_sinphi;
  double cosbeta;
  double coslambda;
  double cosphi;
  double ecefPosWithENUOrigin_idx_0;
  double ecefPosWithENUOrigin_idx_1;
  double ecefPos_idx_1;
  double enu_idx_0;
  double rho;
  double sinlambda;
  double sinphi;
  int K_tmp;
  int b_K_tmp;
  int exponent;
  int k;
  int r1;
  int r2;
  int r3;
  int rtemp;
  boolean_T iterate;
  if (!isInitialized_ekf) {
    ekf_initialize();
  }
  /*  EKF GPS-Tracking */
  /*  numerical jacobian perturbation increment */
  /*  GPS distance to mid section of tire [m] */
  /*  GPS meas standard deviation [m] */
  /*  turning velocity input/meas standard deviation [rad/s] */
  /*  linear velocity input/meas standard deviation [m/s2] */
  /*  linear accel input/meas standard deviation [m/s2] */
  /*  GPS datum (latitude [deg] longitude [deg] altitude [m] of Bandung) */
  /*  GPS coordinate conversion from latitude,longitude [deg] to east-x,north-y
   * [m] */
  sinphi = lat;
  b_sind(&sinphi);
  N = 6.378137E+6 / sqrt(1.0 - 0.0066943799901413165 * (sinphi * sinphi));
  ecefPosWithENUOrigin_idx_0 = lat;
  b_cosd(&ecefPosWithENUOrigin_idx_0);
  rho = (N + 800.0) * ecefPosWithENUOrigin_idx_0;
  cosphi = -6.914744;
  b_cosd(&cosphi);
  b_sinphi = -6.914744;
  b_sind(&b_sinphi);
  coslambda = 107.60981;
  b_cosd(&coslambda);
  sinlambda = 107.60981;
  b_sind(&sinlambda);
  c_sinphi = -6.914744;
  b_sind(&c_sinphi);
  b_N = 6.378137E+6 / sqrt(1.0 - 0.0066943799901413165 * (c_sinphi * c_sinphi));
  ecefPosWithENUOrigin_idx_0 = -6.914744;
  b_cosd(&ecefPosWithENUOrigin_idx_0);
  a21 = (b_N + 800.0) * ecefPosWithENUOrigin_idx_0;
  ecefPosWithENUOrigin_idx_0 = lon;
  b_cosd(&ecefPosWithENUOrigin_idx_0);
  ecefPos_idx_1 = lon;
  b_sind(&ecefPos_idx_1);
  cosbeta = 107.60981;
  b_cosd(&cosbeta);
  Cv = 107.60981;
  b_sind(&Cv);
  ecefPosWithENUOrigin_idx_0 = rho * ecefPosWithENUOrigin_idx_0 - a21 * cosbeta;
  ecefPosWithENUOrigin_idx_1 = rho * ecefPos_idx_1 - a21 * Cv;
  enu_idx_0 = -sinlambda * ecefPosWithENUOrigin_idx_0 +
              coslambda * ecefPosWithENUOrigin_idx_1;
  cosphi = -b_sinphi * (coslambda * ecefPosWithENUOrigin_idx_0 +
                        sinlambda * ecefPosWithENUOrigin_idx_1) +
           cosphi * ((N * 0.99330562000985867 + 800.0) * sinphi -
                     (b_N * 0.99330562000985867 + 800.0) * c_sinphi);
  /*  Initial state & covariance (x_est = [Px; Py; psi; V]) */
  if (!x_est_not_empty) {
    x_est[0] = enu_idx_0;
    x_est[1] = cosphi;
    x_est[2] = 0.0;
    x_est[3] = V;
    x_est_not_empty = true;
    memset(&p_est[0], 0, 16U * sizeof(double));
    p_est[0] = 1.0;
    p_est[5] = 1.0;
    p_est[10] = 1.0;
    p_est[15] = 1.0;
  }
  /*  Skid-steering parameter */
  if (psi_1dot * psi_1dot < 0.1) {
    Cv = 1.0;
    ecefPosWithENUOrigin_idx_0 = 0.0;
  } else {
    ecefPosWithENUOrigin_idx_0 = V * V;
    if ((ecefPosWithENUOrigin_idx_0 < 0.3) && (psi_1dot > 0.0)) {
      Cv = 0.356;
      ecefPosWithENUOrigin_idx_0 = 1.5707963267948966;
    } else if ((ecefPosWithENUOrigin_idx_0 < 0.3) && (psi_1dot < 0.0)) {
      Cv = 0.356;
      ecefPosWithENUOrigin_idx_0 = -1.5707963267948966;
    } else {
      ecefPosWithENUOrigin_idx_1 = V / psi_1dot;
      Cv = sqrt(ecefPosWithENUOrigin_idx_1 * ecefPosWithENUOrigin_idx_1 +
                0.126736) /
           ecefPosWithENUOrigin_idx_1;
      ecefPosWithENUOrigin_idx_0 = atan(0.356 / ecefPosWithENUOrigin_idx_1);
    }
  }
  /*  PREDICTION STEP */
  /*  Predicted state */
  a21 = x_est[2] - ecefPosWithENUOrigin_idx_0;
  c_sinphi = sin(a21);
  b_N = cos(a21);
  ecefPosWithENUOrigin_idx_1 = dt * psi_1dot;
  cosbeta = x_est[2] + ecefPosWithENUOrigin_idx_1;
  /* rad2deg(x_est(3,1)) */
  /*   ~wrapAngle [to make sure -pi/2 < psi < pi/2] */
  if (cosbeta > 1.5707963267948966) {
    coslambda = (cosbeta - 3.1415926535897931) + 3.1415926535897931;
  } else if (cosbeta < -1.5707963267948966) {
    coslambda = (cosbeta + 3.1415926535897931) - 3.1415926535897931;
  } else {
    coslambda = cosbeta;
  }
  a21 = dt * Cv;
  N = a21 * V;
  x_prd[0] = x_est[0] + N * c_sinphi;
  x_prd[1] = x_est[1] + N * b_N;
  x_prd[2] = coslambda;
  sinlambda = dt * V_1dot;
  N = x_est[3] + sinlambda;
  x_prd[3] = N;
  /*  Jacobian of system function (predicted state) */
  cosbeta = (x_est[2] + 0.001) + ecefPosWithENUOrigin_idx_1;
  /* rad2deg(x_est(3,1)) */
  /*   ~wrapAngle [to make sure -pi/2 < psi < pi/2] */
  a21 *= x_est[3];
  jac_fx[0] = (x_est[0] + 0.001) + a21 * c_sinphi;
  jac_fx[1] = x_est[1] + a21 * b_N;
  jac_fx[2] = coslambda;
  jac_fx[3] = N;
  jac_fx[4] = x_est[0] +
              dt * Cv * x_est[3] * sin(x_est[2] - ecefPosWithENUOrigin_idx_0);
  jac_fx[5] = (x_est[1] + 0.001) +
              dt * Cv * x_est[3] * cos(x_est[2] - ecefPosWithENUOrigin_idx_0);
  jac_fx[6] = coslambda;
  jac_fx[7] = N;
  ecefPosWithENUOrigin_idx_1 = (x_est[2] + 0.001) - ecefPosWithENUOrigin_idx_0;
  jac_fx[8] = x_est[0] + a21 * sin(ecefPosWithENUOrigin_idx_1);
  jac_fx[9] = x_est[1] + a21 * cos(ecefPosWithENUOrigin_idx_1);
  if (cosbeta > 1.5707963267948966) {
    jac_fx[10] = (cosbeta - 3.1415926535897931) + 3.1415926535897931;
  } else if (cosbeta < -1.5707963267948966) {
    jac_fx[10] = (cosbeta + 3.1415926535897931) - 3.1415926535897931;
  } else {
    jac_fx[10] = cosbeta;
  }
  jac_fx[11] = N;
  jac_fx[12] = (x_est[0] + a21) + 0.001 * c_sinphi;
  jac_fx[13] = (x_est[1] + a21) + 0.001 * b_N;
  jac_fx[14] = coslambda;
  jac_fx[15] = (x_est[3] + 0.001) + sinlambda;
  b_x_prd[0] = x_prd[0];
  b_x_prd[4] = x_prd[0];
  b_x_prd[8] = x_prd[0];
  b_x_prd[12] = x_prd[0];
  b_x_prd[1] = x_prd[1];
  b_x_prd[5] = x_prd[1];
  b_x_prd[9] = x_prd[1];
  b_x_prd[13] = x_prd[1];
  b_x_prd[2] = coslambda;
  b_x_prd[6] = coslambda;
  b_x_prd[10] = coslambda;
  b_x_prd[14] = coslambda;
  b_x_prd[3] = N;
  b_x_prd[7] = N;
  b_x_prd[11] = N;
  b_x_prd[15] = N;
  for (K_tmp = 0; K_tmp < 16; K_tmp++) {
    jac_fx[K_tmp] = (jac_fx[K_tmp] - b_x_prd[K_tmp]) / 0.001;
  }
  /*  Predicted State Noise Covariance */
  ecefPosWithENUOrigin_idx_1 = 0.30000000000000004 * dt;
  /*  Predicted State Total Covariance */
  for (K_tmp = 0; K_tmp < 4; K_tmp++) {
    ecefPosWithENUOrigin_idx_0 = jac_fx[K_tmp];
    ecefPos_idx_1 = jac_fx[K_tmp + 4];
    cosbeta = jac_fx[K_tmp + 8];
    Cv = jac_fx[K_tmp + 12];
    for (k = 0; k < 4; k++) {
      rtemp = k << 2;
      b_x_prd[K_tmp + rtemp] = ((ecefPosWithENUOrigin_idx_0 * p_est[rtemp] +
                                 ecefPos_idx_1 * p_est[rtemp + 1]) +
                                cosbeta * p_est[rtemp + 2]) +
                               Cv * p_est[rtemp + 3];
    }
    ecefPosWithENUOrigin_idx_0 = b_x_prd[K_tmp];
    ecefPos_idx_1 = b_x_prd[K_tmp + 4];
    cosbeta = b_x_prd[K_tmp + 8];
    Cv = b_x_prd[K_tmp + 12];
    for (k = 0; k < 4; k++) {
      p_prd[K_tmp + (k << 2)] = ((ecefPosWithENUOrigin_idx_0 * jac_fx[k] +
                                  ecefPos_idx_1 * jac_fx[k + 4]) +
                                 cosbeta * jac_fx[k + 8]) +
                                Cv * jac_fx[k + 12];
    }
  }
  b_x_prd[0] = 0.0;
  b_x_prd[1] = 0.0;
  b_x_prd[4] = 0.0;
  b_x_prd[5] = 0.0;
  b_x_prd[8] = 0.0;
  b_x_prd[9] = 0.0;
  b_x_prd[12] = 0.0;
  b_x_prd[13] = 0.0;
  b_x_prd[2] = 0.0;
  b_x_prd[6] = 0.0;
  b_x_prd[10] = ecefPosWithENUOrigin_idx_1 * ecefPosWithENUOrigin_idx_1;
  b_x_prd[14] = 0.0;
  b_x_prd[3] = 0.0;
  b_x_prd[7] = 0.0;
  b_x_prd[11] = 0.0;
  b_x_prd[15] = dt * dt;
  for (K_tmp = 0; K_tmp < 16; K_tmp++) {
    p_prd[K_tmp] += b_x_prd[K_tmp];
  }
  /*  UPDATE STEP */
  /*  Measurement Innovation */
  /* angleWrap(atan((x_prd(2,1)-x_est(2,1))/(x_prd(1,1)-x_est(1,1)))); */
  /*  Jacobian of measurement function */
  jac_hx[0] = x_prd[0] + 0.001;
  jac_hx[1] = x_prd[1];
  jac_hx[2] = N;
  jac_hx[3] = x_prd[0];
  jac_hx[4] = x_prd[1] + 0.001;
  jac_hx[5] = N;
  jac_hx[6] = x_prd[0];
  jac_hx[7] = x_prd[1];
  jac_hx[8] = N;
  jac_hx[9] = x_prd[0];
  jac_hx[10] = x_prd[1];
  jac_hx[11] = N + 0.001;
  ecefPosWithENUOrigin[0] = x_prd[0];
  ecefPosWithENUOrigin[3] = x_prd[0];
  ecefPosWithENUOrigin[6] = x_prd[0];
  ecefPosWithENUOrigin[9] = x_prd[0];
  ecefPosWithENUOrigin[1] = x_prd[1];
  ecefPosWithENUOrigin[4] = x_prd[1];
  ecefPosWithENUOrigin[7] = x_prd[1];
  ecefPosWithENUOrigin[10] = x_prd[1];
  ecefPosWithENUOrigin[2] = N;
  ecefPosWithENUOrigin[5] = N;
  ecefPosWithENUOrigin[8] = N;
  ecefPosWithENUOrigin[11] = N;
  for (K_tmp = 0; K_tmp < 12; K_tmp++) {
    jac_hx[K_tmp] = (jac_hx[K_tmp] - ecefPosWithENUOrigin[K_tmp]) / 0.001;
  }
  /*  Measurement Noise Covariance */
  /*  Kalman Gain */
  for (K_tmp = 0; K_tmp < 3; K_tmp++) {
    b_K_tmp = K_tmp << 2;
    K[b_K_tmp] = jac_hx[K_tmp];
    K[b_K_tmp + 1] = jac_hx[K_tmp + 3];
    K[b_K_tmp + 2] = jac_hx[K_tmp + 6];
    K[b_K_tmp + 3] = jac_hx[K_tmp + 9];
  }
  for (K_tmp = 0; K_tmp < 4; K_tmp++) {
    ecefPosWithENUOrigin_idx_0 = p_prd[K_tmp];
    ecefPos_idx_1 = p_prd[K_tmp + 4];
    cosbeta = p_prd[K_tmp + 8];
    Cv = p_prd[K_tmp + 12];
    for (k = 0; k < 3; k++) {
      rtemp = k << 2;
      A[K_tmp + rtemp] = ((ecefPosWithENUOrigin_idx_0 * K[rtemp] +
                           ecefPos_idx_1 * K[rtemp + 1]) +
                          cosbeta * K[rtemp + 2]) +
                         Cv * K[rtemp + 3];
    }
  }
  for (K_tmp = 0; K_tmp < 3; K_tmp++) {
    ecefPosWithENUOrigin_idx_0 = jac_hx[K_tmp];
    ecefPos_idx_1 = jac_hx[K_tmp + 3];
    cosbeta = jac_hx[K_tmp + 6];
    Cv = jac_hx[K_tmp + 9];
    for (k = 0; k < 4; k++) {
      rtemp = k << 2;
      ecefPosWithENUOrigin[K_tmp + 3 * k] =
          ((ecefPosWithENUOrigin_idx_0 * p_prd[rtemp] +
            ecefPos_idx_1 * p_prd[rtemp + 1]) +
           cosbeta * p_prd[rtemp + 2]) +
          Cv * p_prd[rtemp + 3];
    }
    ecefPosWithENUOrigin_idx_0 = ecefPosWithENUOrigin[K_tmp];
    ecefPos_idx_1 = ecefPosWithENUOrigin[K_tmp + 3];
    cosbeta = ecefPosWithENUOrigin[K_tmp + 6];
    Cv = ecefPosWithENUOrigin[K_tmp + 9];
    for (k = 0; k < 3; k++) {
      rtemp = k << 2;
      b_jac_hx[K_tmp + 3 * k] = ((ecefPosWithENUOrigin_idx_0 * K[rtemp] +
                                  ecefPos_idx_1 * K[rtemp + 1]) +
                                 cosbeta * K[rtemp + 2]) +
                                Cv * K[rtemp + 3];
    }
  }
  dv[0] = 0.1111111111111111;
  dv[3] = 0.0;
  dv[6] = 0.0;
  dv[1] = 0.0;
  dv[4] = 0.1111111111111111;
  dv[7] = 0.0;
  dv[2] = 0.0;
  dv[5] = 0.0;
  dv[8] = 0.0001;
  for (K_tmp = 0; K_tmp < 9; K_tmp++) {
    b_jac_hx[K_tmp] += dv[K_tmp];
  }
  r1 = 0;
  r2 = 1;
  r3 = 2;
  ecefPosWithENUOrigin_idx_1 = fabs(b_jac_hx[0]);
  a21 = fabs(b_jac_hx[1]);
  if (a21 > ecefPosWithENUOrigin_idx_1) {
    ecefPosWithENUOrigin_idx_1 = a21;
    r1 = 1;
    r2 = 0;
  }
  if (fabs(b_jac_hx[2]) > ecefPosWithENUOrigin_idx_1) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }
  b_jac_hx[r2] /= b_jac_hx[r1];
  b_jac_hx[r3] /= b_jac_hx[r1];
  b_jac_hx[r2 + 3] -= b_jac_hx[r2] * b_jac_hx[r1 + 3];
  b_jac_hx[r3 + 3] -= b_jac_hx[r3] * b_jac_hx[r1 + 3];
  b_jac_hx[r2 + 6] -= b_jac_hx[r2] * b_jac_hx[r1 + 6];
  b_jac_hx[r3 + 6] -= b_jac_hx[r3] * b_jac_hx[r1 + 6];
  if (fabs(b_jac_hx[r3 + 3]) > fabs(b_jac_hx[r2 + 3])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }
  b_jac_hx[r3 + 3] /= b_jac_hx[r2 + 3];
  b_jac_hx[r3 + 6] -= b_jac_hx[r3 + 3] * b_jac_hx[r2 + 6];
  /*  Estimated State */
  enu_idx_0 -= x_prd[0];
  cosphi -= x_prd[1];
  ecefPosWithENUOrigin_idx_1 = V - N;
  for (k = 0; k < 4; k++) {
    b_K_tmp = k + (r1 << 2);
    K[b_K_tmp] = A[k] / b_jac_hx[r1];
    rtemp = k + (r2 << 2);
    K[rtemp] = A[k + 4] - K[b_K_tmp] * b_jac_hx[r1 + 3];
    K_tmp = k + (r3 << 2);
    K[K_tmp] = A[k + 8] - K[b_K_tmp] * b_jac_hx[r1 + 6];
    K[rtemp] /= b_jac_hx[r2 + 3];
    K[K_tmp] -= K[rtemp] * b_jac_hx[r2 + 6];
    K[K_tmp] /= b_jac_hx[r3 + 6];
    K[rtemp] -= K[K_tmp] * b_jac_hx[r3 + 3];
    K[b_K_tmp] -= K[K_tmp] * b_jac_hx[r3];
    K[b_K_tmp] -= K[rtemp] * b_jac_hx[r2];
    x_est[k] = x_prd[k] + ((K[k] * enu_idx_0 + K[k + 4] * cosphi) +
                           K[k + 8] * ecefPosWithENUOrigin_idx_1);
  }
  /*  Estimated State Covariance */
  memset(&jac_fx[0], 0, 16U * sizeof(double));
  jac_fx[0] = 1.0;
  jac_fx[5] = 1.0;
  jac_fx[10] = 1.0;
  jac_fx[15] = 1.0;
  for (K_tmp = 0; K_tmp < 4; K_tmp++) {
    ecefPosWithENUOrigin_idx_0 = K[K_tmp];
    ecefPos_idx_1 = K[K_tmp + 4];
    cosbeta = K[K_tmp + 8];
    for (k = 0; k < 4; k++) {
      rtemp = K_tmp + (k << 2);
      b_x_prd[rtemp] =
          jac_fx[rtemp] - ((ecefPosWithENUOrigin_idx_0 * jac_hx[3 * k] +
                            ecefPos_idx_1 * jac_hx[3 * k + 1]) +
                           cosbeta * jac_hx[3 * k + 2]);
    }
    ecefPosWithENUOrigin_idx_0 = b_x_prd[K_tmp];
    ecefPos_idx_1 = b_x_prd[K_tmp + 4];
    cosbeta = b_x_prd[K_tmp + 8];
    Cv = b_x_prd[K_tmp + 12];
    for (k = 0; k < 4; k++) {
      rtemp = k << 2;
      p_est[K_tmp + rtemp] = ((ecefPosWithENUOrigin_idx_0 * p_prd[rtemp] +
                               ecefPos_idx_1 * p_prd[rtemp + 1]) +
                              cosbeta * p_prd[rtemp + 2]) +
                             Cv * p_prd[rtemp + 3];
    }
  }
  /*  Updated Measurements (from estimated state) */
  cosphi = -6.914744;
  b_cosd(&cosphi);
  sinphi = -6.914744;
  b_sind(&sinphi);
  coslambda = 107.60981;
  b_cosd(&coslambda);
  sinlambda = 107.60981;
  b_sind(&sinlambda);
  b_sinphi = -6.914744;
  b_sind(&b_sinphi);
  N = 6.378137E+6 / sqrt(1.0 - 0.0066943799901413165 * (b_sinphi * b_sinphi));
  ecefPosWithENUOrigin_idx_0 = -6.914744;
  b_cosd(&ecefPosWithENUOrigin_idx_0);
  rho = (N + 800.0) * ecefPosWithENUOrigin_idx_0;
  ecefPosWithENUOrigin_idx_0 = 107.60981;
  b_cosd(&ecefPosWithENUOrigin_idx_0);
  ecefPos_idx_1 = 107.60981;
  b_sind(&ecefPos_idx_1);
  a21 = cosphi * 800.0 - sinphi * x_est[1];
  enu_idx_0 = rho * ecefPosWithENUOrigin_idx_0 +
              (coslambda * a21 - sinlambda * x_est[0]);
  ecefPos_idx_1 =
      rho * ecefPos_idx_1 + (sinlambda * a21 + coslambda * x_est[0]);
  Cv = (N * 0.99330562000985867 + 800.0) * b_sinphi +
       (sinphi * 800.0 + cosphi * x_est[1]);
  rho = rt_hypotd_snf(enu_idx_0, ecefPos_idx_1);
  c_sinphi = 6.378137E+6 * rho;
  b_N = 6.3567523142451793E+6 * Cv *
        (42841.311513313565 / rt_hypotd_snf(rho, Cv) + 1.0);
  a21 = c_sinphi;
  if (!rtIsNaN(c_sinphi)) {
    a21 = (c_sinphi > 0.0);
  }
  cosbeta = a21 / rt_hypotd_snf(1.0, b_N / c_sinphi);
  a21 = b_N;
  if (!rtIsNaN(b_N)) {
    if (b_N < 0.0) {
      a21 = -1.0;
    } else {
      a21 = (b_N > 0.0);
    }
  }
  ecefPosWithENUOrigin_idx_1 = a21 / rt_hypotd_snf(1.0, c_sinphi / b_N);
  rtemp = 0;
  iterate = true;
  while (iterate && (rtemp < 5)) {
    coslambda = cosbeta;
    sinlambda = ecefPosWithENUOrigin_idx_1;
    c_sinphi = rho - 42697.672707179969 * rt_powd_snf(cosbeta, 3.0);
    b_N =
        Cv + 42841.311513313565 * rt_powd_snf(ecefPosWithENUOrigin_idx_1, 3.0);
    ecefPosWithENUOrigin_idx_1 = 6.378137E+6 * c_sinphi;
    ecefPosWithENUOrigin_idx_0 = 6.3567523142451793E+6 * b_N;
    a21 = ecefPosWithENUOrigin_idx_1;
    if (!rtIsNaN(ecefPosWithENUOrigin_idx_1)) {
      if (ecefPosWithENUOrigin_idx_1 < 0.0) {
        a21 = -1.0;
      } else {
        a21 = (ecefPosWithENUOrigin_idx_1 > 0.0);
      }
    }
    cosbeta = a21 / rt_hypotd_snf(1.0, ecefPosWithENUOrigin_idx_0 /
                                           ecefPosWithENUOrigin_idx_1);
    a21 = ecefPosWithENUOrigin_idx_0;
    if (!rtIsNaN(ecefPosWithENUOrigin_idx_0)) {
      if (ecefPosWithENUOrigin_idx_0 < 0.0) {
        a21 = -1.0;
      } else {
        a21 = (ecefPosWithENUOrigin_idx_0 > 0.0);
      }
    }
    ecefPosWithENUOrigin_idx_1 =
        a21 / rt_hypotd_snf(1.0, ecefPosWithENUOrigin_idx_1 /
                                     ecefPosWithENUOrigin_idx_0);
    frexp(1.5707963267948966, &exponent);
    iterate = (rt_hypotd_snf(cosbeta - coslambda,
                             ecefPosWithENUOrigin_idx_1 - sinlambda) >
               2.2204460492503131E-16);
    rtemp++;
  }
  ecefPosWithENUOrigin_idx_1 =
      57.295779513082323 * rt_atan2d_snf(b_N, c_sinphi);
  sinphi = ecefPosWithENUOrigin_idx_1;
  b_sind(&sinphi);
  ecefPosWithENUOrigin_idx_0 = ecefPosWithENUOrigin_idx_1;
  b_cosd(&ecefPosWithENUOrigin_idx_0);
  result_ekf[0] = ecefPosWithENUOrigin_idx_1;
  result_ekf[1] = 57.295779513082323 * rt_atan2d_snf(ecefPos_idx_1, enu_idx_0);
  result_ekf[2] = x_est[2];
  result_ekf[3] = x_est[3];
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void x_est_not_empty_init(void)
{
  x_est_not_empty = false;
}

/*
 * File trailer for ekf.c
 *
 * [EOF]
 */
