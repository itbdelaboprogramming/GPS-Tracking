/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ekf.c
 *
 * MATLAB Coder version            : 5.4
 * C/C++ source code generated on  : 08-Dec-2022 10:01:37
 */

/* Include Files */
#include "ekf.h"
#include "cosd.h"
#include "ekf_data.h"
#include "ekf_initialize.h"
#include "enu2lla.h"
#include "mrdivide_helper.h"
#include "rt_nonfinite.h"
#include "sind.h"
#include <math.h>
#include <string.h>

/* Type Definitions */
#ifndef typedef_captured_var
#define typedef_captured_var
typedef struct {
  double contents;
} captured_var;
#endif /* typedef_captured_var */

#ifndef typedef_rtString
#define typedef_rtString
typedef struct {
  char Value[7];
} rtString;
#endif /* typedef_rtString */

/* Variable Definitions */
static boolean_T x_est_not_empty;

static double cal;

static double status;

static double px_a[10];

static double py_a[10];

static double px_b[10];

static double py_b[10];

/* Function Definitions */
/*
 * KALMAN FILTER VARIABLE & TUNING PARAMETER
 *  dt = time increment [s]
 *  zx,zy = latitude,longitude GPS measurment [deg]
 *  psi_1dot = turning velocity [rad/s]
 *  V = linear velocity [m/s]
 *  V_1dot = linear acceleration [m/s2]
 *
 * Arguments    : double mode
 *                double dt
 *                double lat
 *                double lon
 *                double psi_1dot
 *                double V
 *                double V_1dot
 *                double result_ekf[4]
 * Return Type  : void
 */
void ekf(double mode, double dt, double lat, double lon, double psi_1dot,
         double V, double V_1dot, double result_ekf[4])
{
  static captured_var Cv;
  static captured_var V_1dot_std;
  static captured_var alpha;
  static captured_var psi_1dot_std;
  static rtString last;
  static const double lla0[3] = {-6.914744, 107.60981, 800.0};
  static double p_est[16];
  static double x_est[4];
  static const char b[7] = {'p', 'o', 'i', 'n', 't', ' ', 'B'};
  static const char b_b[7] = {'p', 'o', 'i', 'n', 't', ' ', 'A'};
  static const char cv[7] = {'p', 'o', 'i', 'n', 't', ' ', 'B'};
  static const char cv1[7] = {'p', 'o', 'i', 'n', 't', ' ', 'A'};
  double b_jac_fx[16];
  double b_x_prd[16];
  double jac_fx[16];
  double x_prd[4];
  double ecefPosWithENUOrigin[3];
  double enu[3];
  double N;
  double b_N;
  double b_rho;
  double b_sinphi;
  double c_sinphi;
  double coslambda;
  double cosphi;
  double d;
  double d1;
  double d2;
  double d3;
  double rho;
  double sinlambda;
  double sinphi;
  int i;
  int i1;
  int ret;
  if (!isInitialized_ekf) {
    ekf_initialize();
  }
  /*  EKF GPS-Tracking */
  /*  numerical jacobian perturbation increment */
  /*  GPS distance to mid section of tire [m] */
  /*  GPS datum (latitude [deg] longitude [deg] altitude [m] of Bandung) */
  /*  GPS coordinate conversion from latitude,longitude [deg] to east-x,north-y
   * [m] */
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
  b_rho = (b_N + 800.0) * d;
  d = lon;
  b_cosd(&d);
  d1 = lon;
  b_sind(&d1);
  d2 = 107.60981;
  b_cosd(&d2);
  d3 = 107.60981;
  b_sind(&d3);
  ecefPosWithENUOrigin[0] = rho * d - b_rho * d2;
  ecefPosWithENUOrigin[1] = rho * d1 - b_rho * d3;
  enu[0] = -sinlambda * ecefPosWithENUOrigin[0] +
           coslambda * ecefPosWithENUOrigin[1];
  enu[1] = -b_sinphi * (coslambda * ecefPosWithENUOrigin[0] +
                        sinlambda * ecefPosWithENUOrigin[1]) +
           cosphi * ((N * 0.99330562000985867 + 800.0) * sinphi -
                     (b_N * 0.99330562000985867 + 800.0) * c_sinphi);
  /*  Initial state & covariance (x_est = [Px; Py; psi; V]) */
  if (!x_est_not_empty) {
    x_est[0] = enu[0];
    x_est[1] = enu[1];
    x_est[2] = 1.0471975511965976;
    x_est[3] = V;
    x_est_not_empty = true;
    memset(&p_est[0], 0, 16U * sizeof(double));
    p_est[0] = 1.0;
    p_est[5] = 1.0;
    p_est[10] = 1.0;
    p_est[15] = 1.0;
    for (i = 0; i < 7; i++) {
      last.Value[i] = cv[i];
    }
  }
  /*  GPS meas standard deviation [m] */
  /*  linear velocity meas standard deviation [m] */
  /*  angular velocity input standard deviation [rad/s] */
  psi_1dot_std.contents = 1.5;
  /*  linear accel input standard deviation [m/s2] */
  V_1dot_std.contents = 1.5;
  if (status == 1.0) {
    /*  GPS meas standard deviation [m] */
    /*  linear velocity meas standard deviation [m] */
    /*  angular velocity input/meas standard deviation [rad/s] */
    psi_1dot_std.contents = 0.1;
    /*  linear accel input/meas standard deviation [m/s2] */
    V_1dot_std.contents = 0.5;
  }
  /*  Skid-steering parameter */
  if (psi_1dot * psi_1dot < 0.1) {
    Cv.contents = 1.0;
    alpha.contents = 0.0;
  } else {
    d = V * V;
    if ((d < 0.3) && (psi_1dot > 0.0)) {
      Cv.contents = 0.356;
      alpha.contents = 1.5707963267948966;
    } else if ((d < 0.3) && (psi_1dot < 0.0)) {
      Cv.contents = 0.356;
      alpha.contents = -1.5707963267948966;
    } else {
      sinphi = V / psi_1dot;
      Cv.contents = sqrt(sinphi * sinphi + 0.126736) / sinphi;
      alpha.contents = atan(0.356 / sinphi);
    }
  }
  /*  KALMAN FILTERING */
  /*  Predicted state */
  /*  PREDICTION STEP */
  N = dt * psi_1dot;
  sinphi = x_est[2] + N;
  /*   ~wrapAngle [to make sure -pi/2 < psi < pi/2] */
  if (sinphi > 6.2831853071795862) {
    c_sinphi = sinphi - 6.2831853071795862;
  } else if (sinphi < 0.0) {
    c_sinphi = sinphi + 6.2831853071795862;
  } else {
    c_sinphi = sinphi;
  }
  cosphi = x_est[2] - alpha.contents;
  b_sinphi = sin(cosphi);
  coslambda = cos(cosphi);
  sinlambda = dt * Cv.contents * x_est[3];
  x_prd[0] = x_est[0] + sinlambda * b_sinphi;
  x_prd[1] = x_est[1] + sinlambda * coslambda;
  x_prd[2] = c_sinphi;
  cosphi = dt * V_1dot;
  b_N = x_est[3] + cosphi;
  x_prd[3] = b_N;
  /*  Jacobian of system function (predicted state) */
  sinphi = (x_est[2] + 0.0001) + N;
  /*   ~wrapAngle [to make sure -pi/2 < psi < pi/2] */
  jac_fx[0] = (x_est[0] + 0.0001) +
              dt * Cv.contents * x_est[3] * sin(x_est[2] - alpha.contents);
  jac_fx[1] =
      x_est[1] + dt * Cv.contents * x_est[3] * cos(x_est[2] - alpha.contents);
  jac_fx[2] = c_sinphi;
  jac_fx[3] = b_N;
  jac_fx[4] =
      x_est[0] + dt * Cv.contents * x_est[3] * sin(x_est[2] - alpha.contents);
  jac_fx[5] = (x_est[1] + 0.0001) +
              dt * Cv.contents * x_est[3] * cos(x_est[2] - alpha.contents);
  jac_fx[6] = c_sinphi;
  jac_fx[7] = b_N;
  rho = (x_est[2] + 0.0001) - alpha.contents;
  jac_fx[8] = x_est[0] + sinlambda * sin(rho);
  jac_fx[9] = x_est[1] + sinlambda * cos(rho);
  if (sinphi > 6.2831853071795862) {
    jac_fx[10] = sinphi - 6.2831853071795862;
  } else if (sinphi < 0.0) {
    jac_fx[10] = sinphi + 6.2831853071795862;
  } else {
    jac_fx[10] = sinphi;
  }
  jac_fx[11] = b_N;
  jac_fx[12] = (x_est[0] + sinlambda) + 0.0001 * b_sinphi;
  jac_fx[13] = (x_est[1] + sinlambda) + 0.0001 * coslambda;
  jac_fx[14] = c_sinphi;
  jac_fx[15] = (x_est[3] + 0.0001) + cosphi;
  b_x_prd[0] = x_prd[0];
  b_x_prd[4] = x_prd[0];
  b_x_prd[8] = x_prd[0];
  b_x_prd[12] = x_prd[0];
  b_x_prd[1] = x_prd[1];
  b_x_prd[5] = x_prd[1];
  b_x_prd[9] = x_prd[1];
  b_x_prd[13] = x_prd[1];
  b_x_prd[2] = c_sinphi;
  b_x_prd[6] = c_sinphi;
  b_x_prd[10] = c_sinphi;
  b_x_prd[14] = c_sinphi;
  b_x_prd[3] = b_N;
  b_x_prd[7] = b_N;
  b_x_prd[11] = b_N;
  b_x_prd[15] = b_N;
  for (i = 0; i < 16; i++) {
    jac_fx[i] = (jac_fx[i] - b_x_prd[i]) / 0.0001;
  }
  /*  Predicted State Noise Covariance */
  sinphi = psi_1dot_std.contents * dt;
  b_sinphi = V_1dot_std.contents * dt;
  /*  Predicted State Total Covariance */
  for (i = 0; i < 4; i++) {
    d = jac_fx[i];
    d1 = jac_fx[i + 4];
    d2 = jac_fx[i + 8];
    d3 = jac_fx[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      ret = i1 << 2;
      b_x_prd[i + ret] =
          ((d * p_est[ret] + d1 * p_est[ret + 1]) + d2 * p_est[ret + 2]) +
          d3 * p_est[ret + 3];
    }
    d = b_x_prd[i];
    d1 = b_x_prd[i + 4];
    d2 = b_x_prd[i + 8];
    d3 = b_x_prd[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      b_jac_fx[i + (i1 << 2)] =
          ((d * jac_fx[i1] + d1 * jac_fx[i1 + 4]) + d2 * jac_fx[i1 + 8]) +
          d3 * jac_fx[i1 + 12];
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
  b_x_prd[10] = sinphi * sinphi;
  b_x_prd[14] = 0.0;
  b_x_prd[3] = 0.0;
  b_x_prd[7] = 0.0;
  b_x_prd[11] = 0.0;
  b_x_prd[15] = b_sinphi * b_sinphi;
  for (i = 0; i < 16; i++) {
    p_est[i] = b_jac_fx[i] + b_x_prd[i];
  }
  if (mode == 0.0) {
    /*  without GPS measurement */
    x_est[0] = x_prd[0];
    x_est[1] = x_prd[1];
    x_est[2] = c_sinphi;
    x_est[3] = b_N;
  } else {
    double K[12];
    double b_ecefPosWithENUOrigin[12];
    double jac_hx[12];
    double b_jac_hx[9];
    double dv[9];
    /*  with GPS measurement */
    /*  Measurement Innovation */
    /*  UPDATE STEP */
    /*  Jacobian of measurement function */
    jac_hx[0] = x_prd[0] + 0.0001;
    jac_hx[1] = x_prd[1];
    jac_hx[2] = b_N;
    jac_hx[3] = x_prd[0];
    jac_hx[4] = x_prd[1] + 0.0001;
    jac_hx[5] = b_N;
    jac_hx[6] = x_prd[0];
    jac_hx[7] = x_prd[1];
    jac_hx[8] = b_N;
    jac_hx[9] = x_prd[0];
    jac_hx[10] = x_prd[1];
    jac_hx[11] = b_N + 0.0001;
    b_ecefPosWithENUOrigin[0] = x_prd[0];
    b_ecefPosWithENUOrigin[3] = x_prd[0];
    b_ecefPosWithENUOrigin[6] = x_prd[0];
    b_ecefPosWithENUOrigin[9] = x_prd[0];
    b_ecefPosWithENUOrigin[1] = x_prd[1];
    b_ecefPosWithENUOrigin[4] = x_prd[1];
    b_ecefPosWithENUOrigin[7] = x_prd[1];
    b_ecefPosWithENUOrigin[10] = x_prd[1];
    b_ecefPosWithENUOrigin[2] = b_N;
    b_ecefPosWithENUOrigin[5] = b_N;
    b_ecefPosWithENUOrigin[8] = b_N;
    b_ecefPosWithENUOrigin[11] = b_N;
    for (i = 0; i < 12; i++) {
      jac_hx[i] = (jac_hx[i] - b_ecefPosWithENUOrigin[i]) / 0.0001;
    }
    /*  Measurement Noise Covariance */
    /*  Kalman Gain */
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < 4; i1++) {
        int K_tmp;
        K_tmp = i + 3 * i1;
        K[i1 + (i << 2)] = jac_hx[K_tmp];
        ret = i1 << 2;
        b_ecefPosWithENUOrigin[K_tmp] =
            ((jac_hx[i] * p_est[ret] + jac_hx[i + 3] * p_est[ret + 1]) +
             jac_hx[i + 6] * p_est[ret + 2]) +
            jac_hx[i + 9] * p_est[ret + 3];
      }
    }
    for (i = 0; i < 3; i++) {
      d = b_ecefPosWithENUOrigin[i];
      d1 = b_ecefPosWithENUOrigin[i + 3];
      d2 = b_ecefPosWithENUOrigin[i + 6];
      d3 = b_ecefPosWithENUOrigin[i + 9];
      for (i1 = 0; i1 < 3; i1++) {
        ret = i1 << 2;
        b_jac_hx[i + 3 * i1] =
            ((d * K[ret] + d1 * K[ret + 1]) + d2 * K[ret + 2]) +
            d3 * K[ret + 3];
      }
    }
    dv[0] = 81.0;
    dv[3] = 0.0;
    dv[6] = 0.0;
    dv[1] = 0.0;
    dv[4] = 81.0;
    dv[7] = 0.0;
    dv[2] = 0.0;
    dv[5] = 0.0;
    dv[8] = 2.25;
    for (i = 0; i < 4; i++) {
      d = p_est[i];
      d1 = p_est[i + 4];
      d2 = p_est[i + 8];
      d3 = p_est[i + 12];
      for (i1 = 0; i1 < 3; i1++) {
        ret = i1 << 2;
        b_ecefPosWithENUOrigin[i + ret] =
            ((d * K[ret] + d1 * K[ret + 1]) + d2 * K[ret + 2]) +
            d3 * K[ret + 3];
      }
    }
    for (i = 0; i < 9; i++) {
      b_jac_hx[i] += dv[i];
    }
    mrdiv(b_ecefPosWithENUOrigin, b_jac_hx, K);
    /*  Estimated State */
    N = enu[0] - x_prd[0];
    rho = enu[1] - x_prd[1];
    sinphi = V - b_N;
    for (i = 0; i < 4; i++) {
      x_est[i] = x_prd[i] + ((K[i] * N + K[i + 4] * rho) + K[i + 8] * sinphi);
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
      d2 = K[i + 8];
      for (i1 = 0; i1 < 4; i1++) {
        ret = i + (i1 << 2);
        b_x_prd[ret] =
            jac_fx[ret] - ((d * jac_hx[3 * i1] + d1 * jac_hx[3 * i1 + 1]) +
                           d2 * jac_hx[3 * i1 + 2]);
      }
      d = b_x_prd[i];
      d1 = b_x_prd[i + 4];
      d2 = b_x_prd[i + 8];
      d3 = b_x_prd[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        ret = i1 << 2;
        jac_fx[i + ret] =
            ((d * p_est[ret] + d1 * p_est[ret + 1]) + d2 * p_est[ret + 2]) +
            d3 * p_est[ret + 3];
      }
    }
    memcpy(&p_est[0], &jac_fx[0], 16U * sizeof(double));
    /*  heading callibration */
    if (mode == 2.0) {
      ret = memcmp(&last.Value[0], &b[0], 7);
      if (ret == 0) {
        cal = 1.0;
        for (i = 0; i < 7; i++) {
          last.Value[i] = cv1[i];
        }
        status = 0.0;
      } else {
        ret = memcmp(&last.Value[0], &b_b[0], 7);
        if (ret == 0) {
          cal++;
        }
      }
      px_a[(int)cal - 1] = x_est[0];
      py_a[(int)cal - 1] = x_est[1];
    } else if (mode == 3.0) {
      ret = memcmp(&last.Value[0], &b_b[0], 7);
      if (ret == 0) {
        cal = 1.0;
        for (i = 0; i < 7; i++) {
          last.Value[i] = cv[i];
        }
      } else {
        ret = memcmp(&last.Value[0], &b[0], 7);
        if (ret == 0) {
          cal++;
        }
      }
      px_b[(int)cal - 1] = x_est[0];
      py_b[(int)cal - 1] = x_est[1];
    } else if (mode == 4.0) {
      boolean_T guard1 = false;
      rho = px_b[0];
      N = py_b[0];
      b_sinphi = px_a[0];
      cosphi = py_a[0];
      coslambda = px_a[0];
      for (ret = 0; ret < 9; ret++) {
        rho += px_b[ret + 1];
        N += py_b[ret + 1];
        sinphi = px_a[ret + 1];
        b_sinphi += sinphi;
        cosphi += py_a[ret + 1];
        coslambda += sinphi;
      }
      rho /= 10.0;
      N /= 10.0;
      sinphi = (rho - b_sinphi / 10.0) / (N - cosphi / 10.0);
      d = rho - coslambda / 10.0;
      guard1 = false;
      if (d > 0.0) {
        rho = py_a[0];
        for (ret = 0; ret < 9; ret++) {
          rho += py_a[ret + 1];
        }
        if (N - rho / 10.0 < 0.0) {
          x_est[2] = atan(sinphi) + 3.1415926535897931;
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
      if (guard1) {
        if (d < 0.0) {
          rho = py_a[0];
          for (ret = 0; ret < 9; ret++) {
            rho += py_a[ret + 1];
          }
          if (N - rho / 10.0 < 0.0) {
            x_est[2] = atan(sinphi) + 3.1415926535897931;
          } else {
            x_est[2] = atan(sinphi);
          }
        } else {
          x_est[2] = atan(sinphi);
        }
      }
      status = 1.0;
    }
  }
  /*  Updated Measurements (from estimated state) */
  enu[0] = x_est[0];
  enu[1] = x_est[1];
  enu[2] = 800.0;
  enu2lla(enu, lla0, ecefPosWithENUOrigin);
  result_ekf[0] = ecefPosWithENUOrigin[0];
  result_ekf[1] = ecefPosWithENUOrigin[1];
  result_ekf[2] = 57.295779513082323 * x_est[2];
  result_ekf[3] = x_est[3];
}

/*
 * KALMAN FILTER VARIABLE & TUNING PARAMETER
 *  dt = time increment [s]
 *  zx,zy = latitude,longitude GPS measurment [deg]
 *  psi_1dot = turning velocity [rad/s]
 *  V = linear velocity [m/s]
 *  V_1dot = linear acceleration [m/s2]
 *
 * Arguments    : void
 * Return Type  : void
 */
void ekf_init(void)
{
  memset(&px_a[0], 0, 10U * sizeof(double));
  memset(&py_a[0], 0, 10U * sizeof(double));
  memset(&px_b[0], 0, 10U * sizeof(double));
  memset(&py_b[0], 0, 10U * sizeof(double));
  cal = 0.0;
  status = 0.0;
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
