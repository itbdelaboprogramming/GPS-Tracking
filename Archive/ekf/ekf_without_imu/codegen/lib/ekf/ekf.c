/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ekf.c
 *
 * MATLAB Coder version            : 5.4
 * C/C++ source code generated on  : 09-Dec-2022 11:36:23
 */

/* Include Files */
#include "ekf.h"
#include "cosd.h"
#include "ekf_data.h"
#include "ekf_initialize.h"
#include "enu2lla.h"
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

static double head_a;

static double head_b;

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
 *                double odo_VL
 *                double odo_VR
 *                double result_ekf[3]
 * Return Type  : void
 */
void ekf(double mode, double dt, double lat, double lon, double odo_VL,
         double odo_VR, double result_ekf[3])
{
  static captured_var Cv;
  static captured_var alpha;
  static captured_var gps_std;
  static captured_var odo_std;
  static rtString last;
  static const double lla0[3] = {-6.914744, 107.60981, 800.0};
  static double p_est[9];
  static double x_est[3];
  static const char b[7] = {'p', 'o', 'i', 'n', 't', ' ', 'A'};
  static const char b_b[7] = {'p', 'o', 'i', 'n', 't', ' ', 'B'};
  static const char cv[7] = {'p', 'o', 'i', 'n', 't', ' ', 'B'};
  static const char cv1[7] = {'p', 'o', 'i', 'n', 't', ' ', 'A'};
  double b_ecefPosWithENUOrigin[9];
  double b_jac_fx[9];
  double jac_fx[9];
  double ecefPosWithENUOrigin[3];
  double enu[3];
  double N;
  double a22;
  double b_N;
  double b_sinphi;
  double c_sinphi;
  double coslambda;
  double cosphi;
  double d;
  double d1;
  double d2;
  double d3;
  double d4;
  double odo_V_contents;
  double rho;
  double sinlambda;
  double sinphi;
  int k;
  int r1;
  int r2;
  if (!isInitialized_ekf) {
    ekf_initialize();
  }
  /*  EKF GPS-Tracking */
  /*  numerical jacobian perturbation increment */
  /*  GPS distance to mid section of tire [m] */
  /*  tire distance [m] */
  /*  odometry measurement */
  odo_V_contents = (odo_VL + odo_VR) / 2.0;
  d = (odo_VL - odo_VR) / 0.325;
  /*  GPS datum (latitude [deg] longitude [deg] altitude [m] of Bandung) */
  /*  GPS coordinate conversion from latitude,longitude [deg] to east-x,north-y
   * [m] */
  sinphi = lat;
  b_sind(&sinphi);
  N = 6.378137E+6 / sqrt(1.0 - 0.0066943799901413165 * (sinphi * sinphi));
  d1 = lat;
  b_cosd(&d1);
  a22 = (N + 800.0) * d1;
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
  d1 = -6.914744;
  b_cosd(&d1);
  rho = (b_N + 800.0) * d1;
  d1 = lon;
  b_cosd(&d1);
  d2 = lon;
  b_sind(&d2);
  d3 = 107.60981;
  b_cosd(&d3);
  d4 = 107.60981;
  b_sind(&d4);
  ecefPosWithENUOrigin[0] = a22 * d1 - rho * d3;
  ecefPosWithENUOrigin[1] = a22 * d2 - rho * d4;
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
    x_est[2] = 4.71238898038469;
    x_est_not_empty = true;
    memset(&p_est[0], 0, 9U * sizeof(double));
    p_est[0] = 1.0;
    p_est[4] = 1.0;
    p_est[8] = 1.0;
    for (r1 = 0; r1 < 7; r1++) {
      last.Value[r1] = cv[r1];
    }
  }
  /*  GPS meas standard deviation [m] */
  gps_std.contents = 5.0;
  /*  odometry linear velocity meas standard deviation [m/s] */
  odo_std.contents = 5.0;
  if (status == 1.0) {
    /*  GPS meas standard deviation [m] */
    gps_std.contents = 10.0;
    /*  odometry linear velocity meas standard deviation [m/s] */
    odo_std.contents = 0.1;
  }
  /*  Skid-steering parameter */
  if (d * d < 0.1) {
    Cv.contents = 1.0;
    alpha.contents = 0.0;
  } else {
    d1 = odo_V_contents * odo_V_contents;
    if ((d1 < 0.3) && (d > 0.0)) {
      Cv.contents = -0.356;
      alpha.contents = 1.5707963267948966;
    } else if ((d1 < 0.3) && (d < 0.0)) {
      Cv.contents = -0.356;
      alpha.contents = -1.5707963267948966;
    } else {
      sinphi = odo_V_contents / d;
      Cv.contents = sqrt(sinphi * sinphi + 0.126736) / sinphi;
      alpha.contents = atan(-0.356 / sinphi);
    }
  }
  /*  KALMAN FILTERING */
  /*  Predicted state */
  /*  PREDICTION STEP */
  b_N = dt * d;
  cosphi = x_est[2] + b_N;
  /*   ~wrapAngle [to make sure -pi/2 < psi < pi/2] */
  if (cosphi > 6.2831853071795862) {
    coslambda = cosphi - 6.2831853071795862;
  } else if (cosphi < 0.0) {
    coslambda = cosphi + 6.2831853071795862;
  } else {
    coslambda = cosphi;
  }
  c_sinphi = x_est[2] - alpha.contents;
  N = dt * Cv.contents * odo_V_contents;
  sinphi = N * sin(c_sinphi);
  sinlambda = x_est[0] + sinphi;
  ecefPosWithENUOrigin[0] = sinlambda;
  a22 = N * cos(c_sinphi);
  c_sinphi = x_est[1] + a22;
  ecefPosWithENUOrigin[1] = c_sinphi;
  ecefPosWithENUOrigin[2] = coslambda;
  /*  Jacobian of system function (predicted state) */
  cosphi = (x_est[2] + 0.0001) + b_N;
  /*   ~wrapAngle [to make sure -pi/2 < psi < pi/2] */
  jac_fx[0] = (x_est[0] + 0.0001) + sinphi;
  jac_fx[1] = c_sinphi;
  jac_fx[2] = coslambda;
  jac_fx[3] = sinlambda;
  jac_fx[4] = (x_est[1] + 0.0001) + a22;
  jac_fx[5] = coslambda;
  sinphi = (x_est[2] + 0.0001) - alpha.contents;
  jac_fx[6] = x_est[0] + N * sin(sinphi);
  jac_fx[7] = x_est[1] + N * cos(sinphi);
  if (cosphi > 6.2831853071795862) {
    jac_fx[8] = cosphi - 6.2831853071795862;
  } else if (cosphi < 0.0) {
    jac_fx[8] = cosphi + 6.2831853071795862;
  } else {
    jac_fx[8] = cosphi;
  }
  b_ecefPosWithENUOrigin[0] = sinlambda;
  b_ecefPosWithENUOrigin[3] = sinlambda;
  b_ecefPosWithENUOrigin[6] = sinlambda;
  b_ecefPosWithENUOrigin[1] = c_sinphi;
  b_ecefPosWithENUOrigin[4] = c_sinphi;
  b_ecefPosWithENUOrigin[7] = c_sinphi;
  b_ecefPosWithENUOrigin[2] = coslambda;
  b_ecefPosWithENUOrigin[5] = coslambda;
  b_ecefPosWithENUOrigin[8] = coslambda;
  for (r1 = 0; r1 < 9; r1++) {
    jac_fx[r1] = (jac_fx[r1] - b_ecefPosWithENUOrigin[r1]) / 0.0001;
  }
  /*  Predicted State Noise Covariance */
  sinphi = odo_std.contents * dt;
  b_sinphi = odo_std.contents / 0.325;
  /*  Predicted State Total Covariance */
  for (r1 = 0; r1 < 3; r1++) {
    d = jac_fx[r1];
    d1 = jac_fx[r1 + 3];
    d2 = jac_fx[r1 + 6];
    for (r2 = 0; r2 < 3; r2++) {
      b_ecefPosWithENUOrigin[r1 + 3 * r2] =
          (d * p_est[3 * r2] + d1 * p_est[3 * r2 + 1]) + d2 * p_est[3 * r2 + 2];
    }
    d = b_ecefPosWithENUOrigin[r1];
    d1 = b_ecefPosWithENUOrigin[r1 + 3];
    d2 = b_ecefPosWithENUOrigin[r1 + 6];
    for (r2 = 0; r2 < 3; r2++) {
      b_jac_fx[r1 + 3 * r2] =
          (d * jac_fx[r2] + d1 * jac_fx[r2 + 3]) + d2 * jac_fx[r2 + 6];
    }
  }
  N = sinphi * sinphi;
  b_ecefPosWithENUOrigin[0] = N;
  b_ecefPosWithENUOrigin[3] = 0.0;
  b_ecefPosWithENUOrigin[6] = 0.0;
  b_ecefPosWithENUOrigin[1] = 0.0;
  b_ecefPosWithENUOrigin[4] = N;
  b_ecefPosWithENUOrigin[7] = 0.0;
  b_ecefPosWithENUOrigin[2] = 0.0;
  b_ecefPosWithENUOrigin[5] = 0.0;
  b_ecefPosWithENUOrigin[8] = b_sinphi * b_sinphi;
  for (r1 = 0; r1 < 9; r1++) {
    p_est[r1] = b_jac_fx[r1] + b_ecefPosWithENUOrigin[r1];
  }
  if (mode == 0.0) {
    /*  without GPS measurement */
    x_est[0] = sinlambda;
    x_est[1] = c_sinphi;
    x_est[2] = coslambda;
  } else {
    double K[6];
    double jac_hx[6];
    double y[6];
    double z_prd[6];
    double b_jac_hx[4];
    int K_tmp;
    boolean_T guard1 = false;
    /*  with GPS measurement */
    /*  Measurement Innovation */
    /*  UPDATE STEP */
    /*  Jacobian of measurement function */
    jac_hx[0] = sinlambda + 0.0001;
    jac_hx[1] = c_sinphi;
    jac_hx[2] = sinlambda;
    jac_hx[3] = c_sinphi + 0.0001;
    jac_hx[4] = sinlambda;
    jac_hx[5] = c_sinphi;
    z_prd[0] = sinlambda;
    z_prd[2] = sinlambda;
    z_prd[4] = sinlambda;
    z_prd[1] = c_sinphi;
    z_prd[3] = c_sinphi;
    z_prd[5] = c_sinphi;
    for (r1 = 0; r1 < 6; r1++) {
      jac_hx[r1] = (jac_hx[r1] - z_prd[r1]) / 0.0001;
    }
    /*  Measurement Noise Covariance */
    /*  Kalman Gain */
    for (r1 = 0; r1 < 2; r1++) {
      K[3 * r1] = jac_hx[r1];
      K[3 * r1 + 1] = jac_hx[r1 + 2];
      K[3 * r1 + 2] = jac_hx[r1 + 4];
    }
    for (r1 = 0; r1 < 3; r1++) {
      d = p_est[r1];
      d1 = p_est[r1 + 3];
      d2 = p_est[r1 + 6];
      for (r2 = 0; r2 < 2; r2++) {
        y[r1 + 3 * r2] =
            (d * K[3 * r2] + d1 * K[3 * r2 + 1]) + d2 * K[3 * r2 + 2];
      }
    }
    for (r1 = 0; r1 < 2; r1++) {
      d = jac_hx[r1];
      d1 = jac_hx[r1 + 2];
      d2 = jac_hx[r1 + 4];
      for (r2 = 0; r2 < 3; r2++) {
        z_prd[r1 + (r2 << 1)] = (d * p_est[3 * r2] + d1 * p_est[3 * r2 + 1]) +
                                d2 * p_est[3 * r2 + 2];
      }
      d = z_prd[r1];
      d1 = z_prd[r1 + 2];
      d2 = z_prd[r1 + 4];
      for (r2 = 0; r2 < 2; r2++) {
        b_jac_hx[r1 + (r2 << 1)] =
            (d * K[3 * r2] + d1 * K[3 * r2 + 1]) + d2 * K[3 * r2 + 2];
      }
    }
    r1 = (int)((float)gps_std.contents * (float)gps_std.contents);
    b_jac_hx[0] += (double)r1;
    b_jac_hx[3] += (double)r1;
    if (fabs(b_jac_hx[1]) > fabs(b_jac_hx[0])) {
      r1 = 1;
      r2 = 0;
    } else {
      r1 = 0;
      r2 = 1;
    }
    N = b_jac_hx[r2] / b_jac_hx[r1];
    b_sinphi = b_jac_hx[r1 + 2];
    a22 = b_jac_hx[r2 + 2] - N * b_sinphi;
    /*  Estimated State */
    cosphi = enu[0] - sinlambda;
    sinphi = enu[1] - c_sinphi;
    for (k = 0; k < 3; k++) {
      int b_K_tmp;
      K_tmp = k + 3 * r1;
      K[K_tmp] = y[k] / b_jac_hx[r1];
      b_K_tmp = k + 3 * r2;
      K[b_K_tmp] = (y[k + 3] - K[K_tmp] * b_sinphi) / a22;
      K[K_tmp] -= K[b_K_tmp] * N;
      x_est[k] = ecefPosWithENUOrigin[k] + (K[k] * cosphi + K[k + 3] * sinphi);
    }
    /*  Estimated State Covariance */
    memset(&jac_fx[0], 0, 9U * sizeof(double));
    jac_fx[0] = 1.0;
    jac_fx[4] = 1.0;
    jac_fx[8] = 1.0;
    for (r1 = 0; r1 < 3; r1++) {
      d = K[r1];
      d1 = K[r1 + 3];
      for (r2 = 0; r2 < 3; r2++) {
        k = r2 << 1;
        K_tmp = r1 + 3 * r2;
        b_ecefPosWithENUOrigin[K_tmp] =
            jac_fx[K_tmp] - (d * jac_hx[k] + d1 * jac_hx[k + 1]);
      }
      d = b_ecefPosWithENUOrigin[r1];
      d1 = b_ecefPosWithENUOrigin[r1 + 3];
      d2 = b_ecefPosWithENUOrigin[r1 + 6];
      for (r2 = 0; r2 < 3; r2++) {
        jac_fx[r1 + 3 * r2] = (d * p_est[3 * r2] + d1 * p_est[3 * r2 + 1]) +
                              d2 * p_est[3 * r2 + 2];
      }
    }
    memcpy(&p_est[0], &jac_fx[0], 9U * sizeof(double));
    /*  heading callibration */
    guard1 = false;
    if (mode == 1.0) {
      r1 = memcmp(&last.Value[0], &b[0], 7);
      if (r1 == 0) {
        head_b += b_N;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      if (mode == 2.0) {
        r1 = memcmp(&last.Value[0], &b_b[0], 7);
        if (r1 == 0) {
          cal = 1.0;
          for (r1 = 0; r1 < 7; r1++) {
            last.Value[r1] = cv1[r1];
          }
          status = 0.0;
        } else {
          r1 = memcmp(&last.Value[0], &b[0], 7);
          if (r1 == 0) {
            cal++;
          }
        }
        px_a[(int)cal - 1] = x_est[0];
        py_a[(int)cal - 1] = x_est[1];
        head_a = x_est[2];
        head_b = head_a;
      } else if (mode == 3.0) {
        r1 = memcmp(&last.Value[0], &b[0], 7);
        if (r1 == 0) {
          cal = 1.0;
          for (r1 = 0; r1 < 7; r1++) {
            last.Value[r1] = cv[r1];
          }
        } else {
          r1 = memcmp(&last.Value[0], &b_b[0], 7);
          if (r1 == 0) {
            cal++;
          }
        }
        px_b[(int)cal - 1] = x_est[0];
        py_b[(int)cal - 1] = x_est[1];
        /* head_B = x_est(3,1); */
      } else if (mode == 4.0) {
        boolean_T guard2 = false;
        a22 = px_b[0];
        cosphi = py_b[0];
        N = px_a[0];
        b_sinphi = py_a[0];
        coslambda = px_a[0];
        for (k = 0; k < 9; k++) {
          a22 += px_b[k + 1];
          cosphi += py_b[k + 1];
          sinphi = px_a[k + 1];
          N += sinphi;
          b_sinphi += py_a[k + 1];
          coslambda += sinphi;
        }
        a22 /= 10.0;
        cosphi /= 10.0;
        sinphi = (a22 - N / 10.0) / (cosphi - b_sinphi / 10.0);
        d = a22 - coslambda / 10.0;
        guard2 = false;
        if (d > 0.0) {
          a22 = py_a[0];
          for (k = 0; k < 9; k++) {
            a22 += py_a[k + 1];
          }
          if (cosphi - a22 / 10.0 < 0.0) {
            x_est[2] = ((head_b - head_a) +
                        2.0 * (atan(sinphi) + 3.1415926535897931)) /
                       2.0;
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }
        if (guard2) {
          if (d < 0.0) {
            a22 = py_a[0];
            for (k = 0; k < 9; k++) {
              a22 += py_a[k + 1];
            }
            if (cosphi - a22 / 10.0 < 0.0) {
              x_est[2] = ((head_b - head_a) +
                          2.0 * (atan(sinphi) + 3.1415926535897931)) /
                         2.0;
            } else {
              x_est[2] = ((head_b - head_a) + 2.0 * atan(sinphi)) / 2.0;
            }
          } else {
            x_est[2] = ((head_b - head_a) + 2.0 * atan(sinphi)) / 2.0;
          }
        }
        status = 1.0;
      }
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
  head_a = 0.0;
  head_b = 0.0;
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
