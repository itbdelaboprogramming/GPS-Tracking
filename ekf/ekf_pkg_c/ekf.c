/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ekf.c
 *
 * MATLAB Coder version            : 5.4
 * C/C++ source code generated on  : 21-Nov-2022 13:51:11
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

/* Type Definitions */
#ifndef typedef_captured_var
#define typedef_captured_var
typedef struct {
  double contents;
} captured_var;
#endif /* typedef_captured_var */

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
 * Arguments    : boolean_T mode
 *                double dt
 *                double lat
 *                double lon
 *                double psi_1dot
 *                double V
 *                double V_1dot
 *                double psi0
 *                double result_ekf[4]
 * Return Type  : void
 */
void ekf(boolean_T mode, double dt, double lat, double lon, double psi_1dot,
         double V, double V_1dot, double psi0, double result_ekf[4])
{
  static captured_var Cv;
  static captured_var alpha;
  static double p_est[16];
  static double x_est[4];
  double b_jac_fx[16];
  double b_x_prd[16];
  double jac_fx[16];
  double x_prd[4];
  double N;
  double a22;
  double b_N;
  double b_sinphi;
  double c_sinphi;
  double coslambda;
  double cosphi;
  double d;
  double ecefPosWithENUOrigin_idx_0;
  double ecefPosWithENUOrigin_idx_1;
  double ecefPos_idx_1;
  double enu_idx_0;
  double rho;
  double sinlambda;
  double sinphi;
  double sinprev;
  int K_tmp;
  int exponent;
  int k;
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
  /*  angular velocity input/meas standard deviation [rad/s] */
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
  a22 = (b_N + 800.0) * ecefPosWithENUOrigin_idx_0;
  ecefPosWithENUOrigin_idx_0 = lon;
  b_cosd(&ecefPosWithENUOrigin_idx_0);
  d = lon;
  b_sind(&d);
  sinprev = 107.60981;
  b_cosd(&sinprev);
  ecefPos_idx_1 = 107.60981;
  b_sind(&ecefPos_idx_1);
  ecefPosWithENUOrigin_idx_0 = rho * ecefPosWithENUOrigin_idx_0 - a22 * sinprev;
  ecefPosWithENUOrigin_idx_1 = rho * d - a22 * ecefPos_idx_1;
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
    x_est[2] = 0.017453292519943295 * psi0;
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
    Cv.contents = 1.0;
    alpha.contents = 0.0;
  } else {
    ecefPosWithENUOrigin_idx_0 = V * V;
    if ((ecefPosWithENUOrigin_idx_0 < 0.3) && (psi_1dot > 0.0)) {
      Cv.contents = 0.356;
      alpha.contents = 1.5707963267948966;
    } else if ((ecefPosWithENUOrigin_idx_0 < 0.3) && (psi_1dot < 0.0)) {
      Cv.contents = 0.356;
      alpha.contents = -1.5707963267948966;
    } else {
      ecefPosWithENUOrigin_idx_1 = V / psi_1dot;
      Cv.contents =
          sqrt(ecefPosWithENUOrigin_idx_1 * ecefPosWithENUOrigin_idx_1 +
               0.126736) /
          ecefPosWithENUOrigin_idx_1;
      alpha.contents = atan(0.356 / ecefPosWithENUOrigin_idx_1);
    }
  }
  /*  KALMAN FILTERING */
  /*  Predicted state */
  /*  PREDICTION STEP */
  ecefPosWithENUOrigin_idx_1 = dt * psi_1dot;
  ecefPosWithENUOrigin_idx_0 = x_est[2] + ecefPosWithENUOrigin_idx_1;
  /*   ~wrapAngle [to make sure -pi/2 < psi < pi/2] */
  if (ecefPosWithENUOrigin_idx_0 > 6.2831853071795862) {
    sinlambda = ecefPosWithENUOrigin_idx_0 - 6.2831853071795862;
  } else if (ecefPosWithENUOrigin_idx_0 < 0.0) {
    sinlambda = ecefPosWithENUOrigin_idx_0 + 6.2831853071795862;
  } else {
    sinlambda = ecefPosWithENUOrigin_idx_0;
  }
  a22 = x_est[2] - alpha.contents;
  c_sinphi = sin(a22);
  b_N = cos(a22);
  coslambda = dt * Cv.contents * x_est[3];
  x_prd[0] = x_est[0] + coslambda * c_sinphi;
  x_prd[1] = x_est[1] + coslambda * b_N;
  x_prd[2] = sinlambda;
  a22 = dt * V_1dot;
  N = x_est[3] + a22;
  x_prd[3] = N;
  /*  Jacobian of system function (predicted state) */
  ecefPosWithENUOrigin_idx_0 = (x_est[2] + 0.0001) + ecefPosWithENUOrigin_idx_1;
  /*   ~wrapAngle [to make sure -pi/2 < psi < pi/2] */
  jac_fx[0] = (x_est[0] + 0.0001) +
              dt * Cv.contents * x_est[3] * sin(x_est[2] - alpha.contents);
  jac_fx[1] =
      x_est[1] + dt * Cv.contents * x_est[3] * cos(x_est[2] - alpha.contents);
  jac_fx[2] = sinlambda;
  jac_fx[3] = N;
  jac_fx[4] =
      x_est[0] + dt * Cv.contents * x_est[3] * sin(x_est[2] - alpha.contents);
  jac_fx[5] = (x_est[1] + 0.0001) +
              dt * Cv.contents * x_est[3] * cos(x_est[2] - alpha.contents);
  jac_fx[6] = sinlambda;
  jac_fx[7] = N;
  ecefPosWithENUOrigin_idx_1 = (x_est[2] + 0.0001) - alpha.contents;
  jac_fx[8] = x_est[0] + coslambda * sin(ecefPosWithENUOrigin_idx_1);
  jac_fx[9] = x_est[1] + coslambda * cos(ecefPosWithENUOrigin_idx_1);
  if (ecefPosWithENUOrigin_idx_0 > 6.2831853071795862) {
    jac_fx[10] = ecefPosWithENUOrigin_idx_0 - 6.2831853071795862;
  } else if (ecefPosWithENUOrigin_idx_0 < 0.0) {
    jac_fx[10] = ecefPosWithENUOrigin_idx_0 + 6.2831853071795862;
  } else {
    jac_fx[10] = ecefPosWithENUOrigin_idx_0;
  }
  jac_fx[11] = N;
  jac_fx[12] = (x_est[0] + coslambda) + 0.0001 * c_sinphi;
  jac_fx[13] = (x_est[1] + coslambda) + 0.0001 * b_N;
  jac_fx[14] = sinlambda;
  jac_fx[15] = (x_est[3] + 0.0001) + a22;
  b_x_prd[0] = x_prd[0];
  b_x_prd[4] = x_prd[0];
  b_x_prd[8] = x_prd[0];
  b_x_prd[12] = x_prd[0];
  b_x_prd[1] = x_prd[1];
  b_x_prd[5] = x_prd[1];
  b_x_prd[9] = x_prd[1];
  b_x_prd[13] = x_prd[1];
  b_x_prd[2] = sinlambda;
  b_x_prd[6] = sinlambda;
  b_x_prd[10] = sinlambda;
  b_x_prd[14] = sinlambda;
  b_x_prd[3] = N;
  b_x_prd[7] = N;
  b_x_prd[11] = N;
  b_x_prd[15] = N;
  for (K_tmp = 0; K_tmp < 16; K_tmp++) {
    jac_fx[K_tmp] = (jac_fx[K_tmp] - b_x_prd[K_tmp]) / 0.0001;
  }
  /*  Predicted State Noise Covariance */
  ecefPosWithENUOrigin_idx_1 = 8.0 * dt;
  /*  Predicted State Total Covariance */
  for (K_tmp = 0; K_tmp < 4; K_tmp++) {
    ecefPosWithENUOrigin_idx_0 = jac_fx[K_tmp];
    d = jac_fx[K_tmp + 4];
    sinprev = jac_fx[K_tmp + 8];
    ecefPos_idx_1 = jac_fx[K_tmp + 12];
    for (r1 = 0; r1 < 4; r1++) {
      r2 = r1 << 2;
      b_x_prd[K_tmp + r2] =
          ((ecefPosWithENUOrigin_idx_0 * p_est[r2] + d * p_est[r2 + 1]) +
           sinprev * p_est[r2 + 2]) +
          ecefPos_idx_1 * p_est[r2 + 3];
    }
    ecefPosWithENUOrigin_idx_0 = b_x_prd[K_tmp];
    d = b_x_prd[K_tmp + 4];
    sinprev = b_x_prd[K_tmp + 8];
    ecefPos_idx_1 = b_x_prd[K_tmp + 12];
    for (r1 = 0; r1 < 4; r1++) {
      b_jac_fx[K_tmp + (r1 << 2)] =
          ((ecefPosWithENUOrigin_idx_0 * jac_fx[r1] + d * jac_fx[r1 + 4]) +
           sinprev * jac_fx[r1 + 8]) +
          ecefPos_idx_1 * jac_fx[r1 + 12];
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
  b_x_prd[10] = dt * dt;
  b_x_prd[14] = 0.0;
  b_x_prd[3] = 0.0;
  b_x_prd[7] = 0.0;
  b_x_prd[11] = 0.0;
  b_x_prd[15] = ecefPosWithENUOrigin_idx_1 * ecefPosWithENUOrigin_idx_1;
  for (K_tmp = 0; K_tmp < 16; K_tmp++) {
    p_est[K_tmp] = b_jac_fx[K_tmp] + b_x_prd[K_tmp];
  }
  if (mode) {
    double K[8];
    double jac_hx[8];
    double y[8];
    double z_prd[8];
    double b_jac_hx[4];
    int b_K_tmp;
    /*  with GPS measurement */
    /*  Measurement Innovation */
    /*  UPDATE STEP */
    /*  Jacobian of measurement function */
    jac_hx[0] = x_prd[0] + 0.0001;
    jac_hx[1] = x_prd[1];
    jac_hx[2] = x_prd[0];
    jac_hx[3] = x_prd[1] + 0.0001;
    jac_hx[4] = x_prd[0];
    jac_hx[5] = x_prd[1];
    jac_hx[6] = x_prd[0];
    jac_hx[7] = x_prd[1];
    z_prd[0] = x_prd[0];
    z_prd[2] = x_prd[0];
    z_prd[4] = x_prd[0];
    z_prd[6] = x_prd[0];
    z_prd[1] = x_prd[1];
    z_prd[3] = x_prd[1];
    z_prd[5] = x_prd[1];
    z_prd[7] = x_prd[1];
    for (K_tmp = 0; K_tmp < 8; K_tmp++) {
      jac_hx[K_tmp] = (jac_hx[K_tmp] - z_prd[K_tmp]) / 0.0001;
    }
    /*  Measurement Noise Covariance */
    /*  Kalman Gain */
    for (K_tmp = 0; K_tmp < 2; K_tmp++) {
      b_K_tmp = K_tmp << 2;
      K[b_K_tmp] = jac_hx[K_tmp];
      K[b_K_tmp + 1] = jac_hx[K_tmp + 2];
      K[b_K_tmp + 2] = jac_hx[K_tmp + 4];
      K[b_K_tmp + 3] = jac_hx[K_tmp + 6];
    }
    for (K_tmp = 0; K_tmp < 4; K_tmp++) {
      ecefPosWithENUOrigin_idx_0 = p_est[K_tmp];
      d = p_est[K_tmp + 4];
      sinprev = p_est[K_tmp + 8];
      ecefPos_idx_1 = p_est[K_tmp + 12];
      for (r1 = 0; r1 < 2; r1++) {
        r2 = r1 << 2;
        y[K_tmp + r2] = ((ecefPosWithENUOrigin_idx_0 * K[r2] + d * K[r2 + 1]) +
                         sinprev * K[r2 + 2]) +
                        ecefPos_idx_1 * K[r2 + 3];
      }
    }
    for (K_tmp = 0; K_tmp < 2; K_tmp++) {
      ecefPosWithENUOrigin_idx_0 = jac_hx[K_tmp];
      d = jac_hx[K_tmp + 2];
      sinprev = jac_hx[K_tmp + 4];
      ecefPos_idx_1 = jac_hx[K_tmp + 6];
      for (r1 = 0; r1 < 4; r1++) {
        r2 = r1 << 2;
        z_prd[K_tmp + (r1 << 1)] =
            ((ecefPosWithENUOrigin_idx_0 * p_est[r2] + d * p_est[r2 + 1]) +
             sinprev * p_est[r2 + 2]) +
            ecefPos_idx_1 * p_est[r2 + 3];
      }
      ecefPosWithENUOrigin_idx_0 = z_prd[K_tmp];
      d = z_prd[K_tmp + 2];
      sinprev = z_prd[K_tmp + 4];
      ecefPos_idx_1 = z_prd[K_tmp + 6];
      for (r1 = 0; r1 < 2; r1++) {
        r2 = r1 << 2;
        b_jac_hx[K_tmp + (r1 << 1)] =
            ((ecefPosWithENUOrigin_idx_0 * K[r2] + d * K[r2 + 1]) +
             sinprev * K[r2 + 2]) +
            ecefPos_idx_1 * K[r2 + 3];
      }
    }
    b_jac_hx[0] += 100.0;
    b_jac_hx[3] += 100.0;
    if (fabs(b_jac_hx[1]) > fabs(b_jac_hx[0])) {
      r1 = 1;
      r2 = 0;
    } else {
      r1 = 0;
      r2 = 1;
    }
    ecefPosWithENUOrigin_idx_1 = b_jac_hx[r2] / b_jac_hx[r1];
    ecefPosWithENUOrigin_idx_0 = b_jac_hx[r1 + 2];
    a22 = b_jac_hx[r2 + 2] -
          ecefPosWithENUOrigin_idx_1 * ecefPosWithENUOrigin_idx_0;
    /*  Estimated State */
    enu_idx_0 -= x_prd[0];
    cosphi -= x_prd[1];
    for (k = 0; k < 4; k++) {
      b_K_tmp = k + (r1 << 2);
      K[b_K_tmp] = y[k] / b_jac_hx[r1];
      K_tmp = k + (r2 << 2);
      K[K_tmp] = (y[k + 4] - K[b_K_tmp] * ecefPosWithENUOrigin_idx_0) / a22;
      K[b_K_tmp] -= K[K_tmp] * ecefPosWithENUOrigin_idx_1;
      x_est[k] = x_prd[k] + (K[k] * enu_idx_0 + K[k + 4] * cosphi);
    }
    /*  Estimated State Covariance */
    memset(&jac_fx[0], 0, 16U * sizeof(double));
    jac_fx[0] = 1.0;
    jac_fx[5] = 1.0;
    jac_fx[10] = 1.0;
    jac_fx[15] = 1.0;
    for (K_tmp = 0; K_tmp < 4; K_tmp++) {
      ecefPosWithENUOrigin_idx_0 = K[K_tmp];
      d = K[K_tmp + 4];
      for (r1 = 0; r1 < 4; r1++) {
        r2 = r1 << 1;
        k = K_tmp + (r1 << 2);
        b_x_prd[k] = jac_fx[k] - (ecefPosWithENUOrigin_idx_0 * jac_hx[r2] +
                                  d * jac_hx[r2 + 1]);
      }
      ecefPosWithENUOrigin_idx_0 = b_x_prd[K_tmp];
      d = b_x_prd[K_tmp + 4];
      sinprev = b_x_prd[K_tmp + 8];
      ecefPos_idx_1 = b_x_prd[K_tmp + 12];
      for (r1 = 0; r1 < 4; r1++) {
        r2 = r1 << 2;
        jac_fx[K_tmp + r2] =
            ((ecefPosWithENUOrigin_idx_0 * p_est[r2] + d * p_est[r2 + 1]) +
             sinprev * p_est[r2 + 2]) +
            ecefPos_idx_1 * p_est[r2 + 3];
      }
    }
    memcpy(&p_est[0], &jac_fx[0], 16U * sizeof(double));
  } else {
    /*  without GPS measurement */
    x_est[0] = x_prd[0];
    x_est[1] = x_prd[1];
    x_est[2] = sinlambda;
    x_est[3] = N;
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
  d = 107.60981;
  b_sind(&d);
  a22 = cosphi * 800.0 - sinphi * x_est[1];
  enu_idx_0 = rho * ecefPosWithENUOrigin_idx_0 +
              (coslambda * a22 - sinlambda * x_est[0]);
  ecefPos_idx_1 = rho * d + (sinlambda * a22 + coslambda * x_est[0]);
  c_sinphi = (N * 0.99330562000985867 + 800.0) * b_sinphi +
             (sinphi * 800.0 + cosphi * x_est[1]);
  rho = rt_hypotd_snf(enu_idx_0, ecefPos_idx_1);
  b_N = 6.378137E+6 * rho;
  coslambda = 6.3567523142451793E+6 * c_sinphi *
              (42841.311513313565 / rt_hypotd_snf(rho, c_sinphi) + 1.0);
  a22 = b_N;
  if (!rtIsNaN(b_N)) {
    a22 = (b_N > 0.0);
  }
  sinlambda = a22 / rt_hypotd_snf(1.0, coslambda / b_N);
  a22 = coslambda;
  if (!rtIsNaN(coslambda)) {
    if (coslambda < 0.0) {
      a22 = -1.0;
    } else {
      a22 = (coslambda > 0.0);
    }
  }
  ecefPosWithENUOrigin_idx_1 = a22 / rt_hypotd_snf(1.0, b_N / coslambda);
  r1 = 0;
  iterate = true;
  while (iterate && (r1 < 5)) {
    N = sinlambda;
    sinprev = ecefPosWithENUOrigin_idx_1;
    b_N = rho - 42697.672707179969 * rt_powd_snf(sinlambda, 3.0);
    coslambda = c_sinphi + 42841.311513313565 *
                               rt_powd_snf(ecefPosWithENUOrigin_idx_1, 3.0);
    ecefPosWithENUOrigin_idx_1 = 6.378137E+6 * b_N;
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
        (rt_hypotd_snf(sinlambda - N, ecefPosWithENUOrigin_idx_1 - sinprev) >
         2.2204460492503131E-16);
    r1++;
  }
  ecefPosWithENUOrigin_idx_1 =
      57.295779513082323 * rt_atan2d_snf(coslambda, b_N);
  sinphi = ecefPosWithENUOrigin_idx_1;
  b_sind(&sinphi);
  ecefPosWithENUOrigin_idx_0 = ecefPosWithENUOrigin_idx_1;
  b_cosd(&ecefPosWithENUOrigin_idx_0);
  result_ekf[0] = ecefPosWithENUOrigin_idx_1;
  result_ekf[1] = 57.295779513082323 * rt_atan2d_snf(ecefPos_idx_1, enu_idx_0);
  result_ekf[2] = 57.295779513082323 * x_est[2];
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
