/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ekf.c
 *
 * MATLAB Coder version            : 5.4
 * C/C++ source code generated on  : 02-Nov-2022 22:58:20
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
 * KALMAN FILTER TUNING PARAMETER
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
  double K[8];
  double c_x_prd[8];
  double jac_hx[8];
  double y[8];
  double b_jac_hx[4];
  double x_prd[4];
  double N;
  double a22;
  double b_N;
  double b_sinphi;
  double c_sinphi;
  double coslambda;
  double cosphi;
  double cosprev;
  double d;
  double d1;
  double ecefPosWithENUOrigin_idx_0;
  double ecefPosWithENUOrigin_idx_1;
  double ecefPos_idx_0;
  double rho;
  double sinlambda;
  double sinphi;
  double u;
  int K_tmp;
  int exponent;
  int i;
  int r1;
  int r2;
  boolean_T iterate;
  if (!isInitialized_ekf) {
    ekf_initialize();
  }
  /*  EKF GPS-Tracking */
  /*  numerical jacobian perturbation increment */
  /*  GPS distance to mid section of tire [m] */
  /*  GPS meas standard deviation [m] */
  /*  turning velocity input/meas standard deviation [rad/s] */
  /*  linear accel input/meas standard deviation [m/s2] */
  /*  GPS datum (latitude [deg] longitude [deg] altitude [m] of Bandung) */
  /*  KALMAN FILTER VARIABLE */
  /*  GPS-coordinate conversion from [deg] to [m] */
  sinphi = lat;
  b_sind(&sinphi);
  N = 6.378137E+6 / sqrt(1.0 - 0.0066943799901413165 * (sinphi * sinphi));
  d = lat;
  b_cosd(&d);
  rho = (N + 800.0) * d;
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
  d = -6.914744;
  b_cosd(&d);
  a22 = (b_N + 800.0) * d;
  d = lon;
  b_cosd(&d);
  d1 = lon;
  b_sind(&d1);
  u = 107.60981;
  b_cosd(&u);
  cosprev = 107.60981;
  b_sind(&cosprev);
  ecefPosWithENUOrigin_idx_0 = rho * d - a22 * u;
  ecefPosWithENUOrigin_idx_1 = rho * d1 - a22 * cosprev;
  /*  GPS measurement in east-x [m], north-y [m] */
  ecefPos_idx_0 = -sinlambda * ecefPosWithENUOrigin_idx_0 +
                  coslambda * ecefPosWithENUOrigin_idx_1;
  cosphi = -b_sinphi * (coslambda * ecefPosWithENUOrigin_idx_0 +
                        sinlambda * ecefPosWithENUOrigin_idx_1) +
           cosphi * ((N * 0.99330562000985867 + 800.0) * sinphi -
                     (b_N * 0.99330562000985867 + 800.0) * c_sinphi);
  /*  Initial state % covariance (x_est = [Px; Py; psi; V]) */
  if (!x_est_not_empty) {
    x_est[0] = ecefPos_idx_0;
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
    ecefPosWithENUOrigin_idx_1 = 1.0;
    ecefPosWithENUOrigin_idx_0 = 0.0;
  } else {
    d = V * V;
    if ((d < 0.1) && (psi_1dot > 0.0)) {
      ecefPosWithENUOrigin_idx_1 = 0.3;
      ecefPosWithENUOrigin_idx_0 = 1.5707963267948966;
    } else if ((d < 0.1) && (psi_1dot < 0.0)) {
      ecefPosWithENUOrigin_idx_1 = 0.3;
      ecefPosWithENUOrigin_idx_0 = -1.5707963267948966;
    } else {
      ecefPosWithENUOrigin_idx_0 = V / psi_1dot;
      ecefPosWithENUOrigin_idx_1 =
          sqrt(ecefPosWithENUOrigin_idx_0 * ecefPosWithENUOrigin_idx_0 + 0.09) /
          ecefPosWithENUOrigin_idx_0;
      ecefPosWithENUOrigin_idx_0 = atan(0.3 / ecefPosWithENUOrigin_idx_0);
    }
  }
  /*  PREDICTION STEP */
  /*  Predicted state */
  a22 = dt * psi_1dot;
  b_N = x_est[2] + a22;
  /*   ~wrapAngle [to make sure -pi/2 < psi < pi/2] */
  if (b_N > 1.5707963267948966) {
    u = b_N - 3.1415926535897931;
  } else if (b_N < -1.5707963267948966) {
    u = b_N + 3.1415926535897931;
  } else {
    u = b_N;
  }
  coslambda = x_est[2] - ecefPosWithENUOrigin_idx_0;
  sinlambda = sin(coslambda);
  cosprev = cos(coslambda);
  N = dt * ecefPosWithENUOrigin_idx_1 * x_est[3];
  x_prd[0] = x_est[0] + N * sinlambda;
  x_prd[1] = x_est[1] + N * cosprev;
  x_prd[2] = u;
  coslambda = dt * V_1dot;
  c_sinphi = x_est[3] + coslambda;
  x_prd[3] = c_sinphi;
  /*  Jacobian of system function (predicted state) */
  b_N = (x_est[2] + 0.001) + a22;
  /*   ~wrapAngle [to make sure -pi/2 < psi < pi/2] */
  jac_fx[0] =
      (x_est[0] + 0.001) + dt * ecefPosWithENUOrigin_idx_1 * x_est[3] *
                               sin(x_est[2] - ecefPosWithENUOrigin_idx_0);
  jac_fx[1] = x_est[1] + dt * ecefPosWithENUOrigin_idx_1 * x_est[3] *
                             cos(x_est[2] - ecefPosWithENUOrigin_idx_0);
  jac_fx[2] = u;
  jac_fx[3] = c_sinphi;
  jac_fx[4] = x_est[0] + dt * ecefPosWithENUOrigin_idx_1 * x_est[3] *
                             sin(x_est[2] - ecefPosWithENUOrigin_idx_0);
  jac_fx[5] =
      (x_est[1] + 0.001) + dt * ecefPosWithENUOrigin_idx_1 * x_est[3] *
                               cos(x_est[2] - ecefPosWithENUOrigin_idx_0);
  jac_fx[6] = u;
  jac_fx[7] = c_sinphi;
  ecefPosWithENUOrigin_idx_1 = (x_est[2] + 0.001) - ecefPosWithENUOrigin_idx_0;
  jac_fx[8] = x_est[0] + N * sin(ecefPosWithENUOrigin_idx_1);
  jac_fx[9] = x_est[1] + N * cos(ecefPosWithENUOrigin_idx_1);
  if (b_N > 1.5707963267948966) {
    jac_fx[10] = b_N - 3.1415926535897931;
  } else if (b_N < -1.5707963267948966) {
    jac_fx[10] = b_N + 3.1415926535897931;
  } else {
    jac_fx[10] = b_N;
  }
  jac_fx[11] = c_sinphi;
  jac_fx[12] = (x_est[0] + N) + 0.001 * sinlambda;
  jac_fx[13] = (x_est[1] + N) + 0.001 * cosprev;
  jac_fx[14] = u;
  jac_fx[15] = (x_est[3] + 0.001) + coslambda;
  b_x_prd[0] = x_prd[0];
  b_x_prd[4] = x_prd[0];
  b_x_prd[8] = x_prd[0];
  b_x_prd[12] = x_prd[0];
  b_x_prd[1] = x_prd[1];
  b_x_prd[5] = x_prd[1];
  b_x_prd[9] = x_prd[1];
  b_x_prd[13] = x_prd[1];
  b_x_prd[2] = u;
  b_x_prd[6] = u;
  b_x_prd[10] = u;
  b_x_prd[14] = u;
  b_x_prd[3] = c_sinphi;
  b_x_prd[7] = c_sinphi;
  b_x_prd[11] = c_sinphi;
  b_x_prd[15] = c_sinphi;
  for (i = 0; i < 16; i++) {
    jac_fx[i] = (jac_fx[i] - b_x_prd[i]) / 0.001;
  }
  /*  Predicted State Noise Covariance */
  ecefPosWithENUOrigin_idx_1 = 100.0 * dt;
  ecefPosWithENUOrigin_idx_0 = 10.0 * dt;
  /*  Predicted State Total Covariance */
  for (i = 0; i < 4; i++) {
    d = jac_fx[i];
    d1 = jac_fx[i + 4];
    u = jac_fx[i + 8];
    cosprev = jac_fx[i + 12];
    for (r2 = 0; r2 < 4; r2++) {
      r1 = r2 << 2;
      b_x_prd[i + r1] =
          ((d * p_est[r1] + d1 * p_est[r1 + 1]) + u * p_est[r1 + 2]) +
          cosprev * p_est[r1 + 3];
    }
    d = b_x_prd[i];
    d1 = b_x_prd[i + 4];
    u = b_x_prd[i + 8];
    cosprev = b_x_prd[i + 12];
    for (r2 = 0; r2 < 4; r2++) {
      p_prd[i + (r2 << 2)] =
          ((d * jac_fx[r2] + d1 * jac_fx[r2 + 4]) + u * jac_fx[r2 + 8]) +
          cosprev * jac_fx[r2 + 12];
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
  b_x_prd[15] = ecefPosWithENUOrigin_idx_0 * ecefPosWithENUOrigin_idx_0;
  for (i = 0; i < 16; i++) {
    p_prd[i] += b_x_prd[i];
  }
  /*  MEASUREMENT STEP */
  /*  Measurement Innovation */
  /*  Jacobian of measurement function */
  jac_hx[0] = x_prd[0] + 0.001;
  jac_hx[1] = x_prd[1];
  jac_hx[2] = x_prd[0];
  jac_hx[3] = x_prd[1] + 0.001;
  jac_hx[4] = x_prd[0];
  jac_hx[5] = x_prd[1];
  jac_hx[6] = x_prd[0];
  jac_hx[7] = x_prd[1];
  c_x_prd[0] = x_prd[0];
  c_x_prd[2] = x_prd[0];
  c_x_prd[4] = x_prd[0];
  c_x_prd[6] = x_prd[0];
  c_x_prd[1] = x_prd[1];
  c_x_prd[3] = x_prd[1];
  c_x_prd[5] = x_prd[1];
  c_x_prd[7] = x_prd[1];
  for (i = 0; i < 8; i++) {
    jac_hx[i] = (jac_hx[i] - c_x_prd[i]) / 0.001;
  }
  /*  Measurement Noise Covariance */
  /*  Kalman Gain */
  for (i = 0; i < 2; i++) {
    K_tmp = i << 2;
    K[K_tmp] = jac_hx[i];
    K[K_tmp + 1] = jac_hx[i + 2];
    K[K_tmp + 2] = jac_hx[i + 4];
    K[K_tmp + 3] = jac_hx[i + 6];
  }
  for (i = 0; i < 4; i++) {
    d = p_prd[i];
    d1 = p_prd[i + 4];
    u = p_prd[i + 8];
    cosprev = p_prd[i + 12];
    for (r2 = 0; r2 < 2; r2++) {
      r1 = r2 << 2;
      y[i + r1] =
          ((d * K[r1] + d1 * K[r1 + 1]) + u * K[r1 + 2]) + cosprev * K[r1 + 3];
    }
  }
  for (i = 0; i < 2; i++) {
    d = jac_hx[i];
    d1 = jac_hx[i + 2];
    u = jac_hx[i + 4];
    cosprev = jac_hx[i + 6];
    for (r2 = 0; r2 < 4; r2++) {
      r1 = r2 << 2;
      c_x_prd[i + (r2 << 1)] =
          ((d * p_prd[r1] + d1 * p_prd[r1 + 1]) + u * p_prd[r1 + 2]) +
          cosprev * p_prd[r1 + 3];
    }
    d = c_x_prd[i];
    d1 = c_x_prd[i + 2];
    u = c_x_prd[i + 4];
    cosprev = c_x_prd[i + 6];
    for (r2 = 0; r2 < 2; r2++) {
      r1 = r2 << 2;
      b_jac_hx[i + (r2 << 1)] =
          ((d * K[r1] + d1 * K[r1 + 1]) + u * K[r1 + 2]) + cosprev * K[r1 + 3];
    }
  }
  b_jac_hx[0]++;
  b_jac_hx[3]++;
  if (fabs(b_jac_hx[1]) > fabs(b_jac_hx[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }
  ecefPosWithENUOrigin_idx_0 = b_jac_hx[r2] / b_jac_hx[r1];
  ecefPosWithENUOrigin_idx_1 = b_jac_hx[r1 + 2];
  a22 = b_jac_hx[r2 + 2] -
        ecefPosWithENUOrigin_idx_0 * ecefPosWithENUOrigin_idx_1;
  K_tmp = r1 << 2;
  K[K_tmp] = y[0] / b_jac_hx[r1];
  r2 <<= 2;
  K[r2] = (y[4] - K[K_tmp] * ecefPosWithENUOrigin_idx_1) / a22;
  K[K_tmp] -= K[r2] * ecefPosWithENUOrigin_idx_0;
  K[K_tmp + 1] = y[1] / b_jac_hx[r1];
  K[r2 + 1] = (y[5] - K[K_tmp + 1] * ecefPosWithENUOrigin_idx_1) / a22;
  K[K_tmp + 1] -= K[r2 + 1] * ecefPosWithENUOrigin_idx_0;
  K[K_tmp + 2] = y[2] / b_jac_hx[r1];
  K[r2 + 2] = (y[6] - K[K_tmp + 2] * ecefPosWithENUOrigin_idx_1) / a22;
  K[K_tmp + 2] -= K[r2 + 2] * ecefPosWithENUOrigin_idx_0;
  K[K_tmp + 3] = y[3] / b_jac_hx[r1];
  K[r2 + 3] = (y[7] - K[K_tmp + 3] * ecefPosWithENUOrigin_idx_1) / a22;
  K[K_tmp + 3] -= K[r2 + 3] * ecefPosWithENUOrigin_idx_0;
  /*  UPDATE/ESTIMATION STEP */
  /*  Estimated State */
  ecefPosWithENUOrigin_idx_1 = ecefPos_idx_0 - x_prd[0];
  ecefPosWithENUOrigin_idx_0 = cosphi - x_prd[1];
  for (i = 0; i < 4; i++) {
    x_est[i] = x_prd[i] + (K[i] * ecefPosWithENUOrigin_idx_1 +
                           K[i + 4] * ecefPosWithENUOrigin_idx_0);
  }
  /*  Estimated State Covariance */
  memset(&jac_fx[0], 0, 16U * sizeof(double));
  jac_fx[0] = 1.0;
  jac_fx[5] = 1.0;
  jac_fx[10] = 1.0;
  jac_fx[15] = 1.0;
  for (i = 0; i < 4; i++) {
    d = K[i];
    d1 = K[i + 4];
    for (r2 = 0; r2 < 4; r2++) {
      r1 = r2 << 1;
      K_tmp = i + (r2 << 2);
      b_x_prd[K_tmp] = jac_fx[K_tmp] - (d * jac_hx[r1] + d1 * jac_hx[r1 + 1]);
    }
    d = b_x_prd[i];
    d1 = b_x_prd[i + 4];
    u = b_x_prd[i + 8];
    cosprev = b_x_prd[i + 12];
    for (r2 = 0; r2 < 4; r2++) {
      r1 = r2 << 2;
      p_est[i + r1] =
          ((d * p_prd[r1] + d1 * p_prd[r1 + 1]) + u * p_prd[r1 + 2]) +
          cosprev * p_prd[r1 + 3];
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
  d = -6.914744;
  b_cosd(&d);
  rho = (N + 800.0) * d;
  d = 107.60981;
  b_cosd(&d);
  d1 = 107.60981;
  b_sind(&d1);
  ecefPosWithENUOrigin_idx_0 = cosphi * 800.0 - sinphi * x_est[1];
  ecefPos_idx_0 =
      rho * d + (coslambda * ecefPosWithENUOrigin_idx_0 - sinlambda * x_est[0]);
  c_sinphi = rho * d1 +
             (sinlambda * ecefPosWithENUOrigin_idx_0 + coslambda * x_est[0]);
  b_N = (N * 0.99330562000985867 + 800.0) * b_sinphi +
        (sinphi * 800.0 + cosphi * x_est[1]);
  rho = rt_hypotd_snf(ecefPos_idx_0, c_sinphi);
  u = 6.378137E+6 * rho;
  coslambda = 6.3567523142451793E+6 * b_N *
              (42841.311513313565 / rt_hypotd_snf(rho, b_N) + 1.0);
  a22 = u;
  if (!rtIsNaN(u)) {
    a22 = (u > 0.0);
  }
  sinlambda = a22 / rt_hypotd_snf(1.0, coslambda / u);
  a22 = coslambda;
  if (!rtIsNaN(coslambda)) {
    if (coslambda < 0.0) {
      a22 = -1.0;
    } else {
      a22 = (coslambda > 0.0);
    }
  }
  ecefPosWithENUOrigin_idx_1 = a22 / rt_hypotd_snf(1.0, u / coslambda);
  r2 = 0;
  iterate = true;
  while (iterate && (r2 < 5)) {
    cosprev = sinlambda;
    N = ecefPosWithENUOrigin_idx_1;
    u = rho - 42697.672707179969 * rt_powd_snf(sinlambda, 3.0);
    coslambda =
        b_N + 42841.311513313565 * rt_powd_snf(ecefPosWithENUOrigin_idx_1, 3.0);
    ecefPosWithENUOrigin_idx_1 = 6.378137E+6 * u;
    ecefPosWithENUOrigin_idx_0 = 6.3567523142451793E+6 * coslambda;
    a22 = ecefPosWithENUOrigin_idx_1;
    if (!rtIsNaN(ecefPosWithENUOrigin_idx_1)) {
      if (ecefPosWithENUOrigin_idx_1 < 0.0) {
        a22 = -1.0;
      } else {
        a22 = (ecefPosWithENUOrigin_idx_1 > 0.0);
      }
    }
    sinlambda = a22 / rt_hypotd_snf(1.0, ecefPosWithENUOrigin_idx_0 /
                                             ecefPosWithENUOrigin_idx_1);
    a22 = ecefPosWithENUOrigin_idx_0;
    if (!rtIsNaN(ecefPosWithENUOrigin_idx_0)) {
      if (ecefPosWithENUOrigin_idx_0 < 0.0) {
        a22 = -1.0;
      } else {
        a22 = (ecefPosWithENUOrigin_idx_0 > 0.0);
      }
    }
    ecefPosWithENUOrigin_idx_1 =
        a22 / rt_hypotd_snf(1.0, ecefPosWithENUOrigin_idx_1 /
                                     ecefPosWithENUOrigin_idx_0);
    frexp(1.5707963267948966, &exponent);
    iterate =
        (rt_hypotd_snf(sinlambda - cosprev, ecefPosWithENUOrigin_idx_1 - N) >
         2.2204460492503131E-16);
    r2++;
  }
  ecefPosWithENUOrigin_idx_1 = 57.295779513082323 * rt_atan2d_snf(coslambda, u);
  sinphi = ecefPosWithENUOrigin_idx_1;
  b_sind(&sinphi);
  d = ecefPosWithENUOrigin_idx_1;
  b_cosd(&d);
  result_ekf[0] = ecefPosWithENUOrigin_idx_1;
  result_ekf[1] = 57.295779513082323 * rt_atan2d_snf(c_sinphi, ecefPos_idx_0);
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
