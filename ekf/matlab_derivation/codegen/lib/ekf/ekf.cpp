//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ekf.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Nov-2022 22:36:16
//

// Include Files
#include "ekf.h"
#include "cosd.h"
#include "ekf_data.h"
#include "ekf_initialize.h"
#include "rt_nonfinite.h"
#include "sind.h"
#include "rt_defines.h"
#include <cmath>
#include <cstring>
#include <math.h>

// Variable Definitions
static boolean_T x_est_not_empty;

// Function Declarations
static double rt_atan2d_snf(double u0, double u1);

static double rt_hypotd_snf(double u0, double u1);

static double rt_powd_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else if (std::isinf(u0) && std::isinf(u1)) {
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
    y = std::atan2(static_cast<double>(b_u0), static_cast<double>(b_u1));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }
  return y;
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_hypotd_snf(double u0, double u1)
{
  double a;
  double y;
  a = std::abs(u0);
  y = std::abs(u1);
  if (a < y) {
    a /= y;
    y *= std::sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = a * std::sqrt(y * y + 1.0);
  } else if (!std::isnan(y)) {
    y = a * 1.4142135623730951;
  }
  return y;
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
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
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = std::pow(u0, u1);
    }
  }
  return y;
}

//
// Kalman Filter input
//  dt = time increment [s]
//  zx,zy = latitude,longitude GPS measurment [deg]
//  psi_1dot = turning velocity [rad/s]
//  V = linear velocity [m/s]
//  V_1dot = linear acceleration [m/s2]
//
// Arguments    : double dt
//                double lat
//                double lon
//                double psi_1dot
//                double V
//                double V_1dot
//                double P[4]
// Return Type  : void
//
void ekf(double dt, double lat, double lon, double psi_1dot, double V,
         double V_1dot, double P[4])
{
  static const double R[4]{0.1111111111111111, 0.0, 0.0, 0.1111111111111111};
  static double p_est[16];
  static double x_est[4];
  double b_x_prd[16];
  double jac_fx[16];
  double p_prd[16];
  double c_x_prd[8];
  double jac_hx[8];
  double klm_gain[8];
  double S[4];
  double x_prd[4];
  double N;
  double a22;
  double b_N;
  double b_sinphi;
  double c_sinphi;
  double cosphi;
  double d;
  double ecefPosWithENUOrigin_idx_0;
  double ecefPosWithENUOrigin_idx_1;
  double ecefPos_idx_0;
  double rho;
  double sinphi;
  double u;
  double v;
  int exponent;
  int i1;
  int klm_gain_tmp;
  int r1;
  int r2;
  boolean_T iterate;
  if (!isInitialized_ekf) {
    ekf_initialize();
  }
  //  EKF GPS-Tracking
  // Kalman Filter Tuning Parameter
  //  numerical jacobian perturbation increment
  //  GPS distance to mid section of tire [m]
  //  GPS meas standard deviation [m]
  //  turning velocity input/meas standard deviation [rad/s]
  //  linear accel input/meas standard deviation [m/s2]
  // GPS-coordinate conversion
  //  latitude [deg] longitude [deg] altitude [m] Bandung
  sinphi = lat;
  coder::b_sind(&sinphi);
  N = 6.378137E+6 / std::sqrt(1.0 - 0.0066943799901413165 * (sinphi * sinphi));
  d = lat;
  coder::b_cosd(&d);
  rho = (N + 800.0) * d;
  cosphi = -6.914744;
  coder::b_cosd(&cosphi);
  b_sinphi = -6.914744;
  coder::b_sind(&b_sinphi);
  u = 107.60981;
  coder::b_cosd(&u);
  v = 107.60981;
  coder::b_sind(&v);
  c_sinphi = -6.914744;
  coder::b_sind(&c_sinphi);
  b_N = 6.378137E+6 /
        std::sqrt(1.0 - 0.0066943799901413165 * (c_sinphi * c_sinphi));
  d = -6.914744;
  coder::b_cosd(&d);
  a22 = (b_N + 800.0) * d;
  d = lon;
  coder::b_cosd(&d);
  ecefPos_idx_0 = lon;
  coder::b_sind(&ecefPos_idx_0);
  ecefPosWithENUOrigin_idx_0 = rho * d - a22 * u;
  ecefPosWithENUOrigin_idx_1 = rho * ecefPos_idx_0 - a22 * v;
  //  change position from [deg] to [m]
  rho = -v * ecefPosWithENUOrigin_idx_0 + u * ecefPosWithENUOrigin_idx_1;
  sinphi = -b_sinphi * (u * ecefPosWithENUOrigin_idx_0 +
                        v * ecefPosWithENUOrigin_idx_1) +
           cosphi * ((N * 0.99330562000985867 + 800.0) * sinphi -
                     (b_N * 0.99330562000985867 + 800.0) * c_sinphi);
  //  east-x [m], north-y [m]
  //  Initial state conditions
  if (!x_est_not_empty) {
    x_est[0] = rho;
    x_est[1] = sinphi;
    x_est[2] = 0.0;
    x_est[3] = V;
    x_est_not_empty = true;
    //  x_est=[Px,Py,psi,V]'
    std::memset(&p_est[0], 0, 16U * sizeof(double));
    p_est[0] = 1.0;
    p_est[5] = 1.0;
    p_est[10] = 1.0;
    p_est[15] = 1.0;
  }
  //  Skid-steering parameter
  if (psi_1dot == 0.0) {
    ecefPosWithENUOrigin_idx_1 = 1.0;
    ecefPosWithENUOrigin_idx_0 = 0.0;
  } else if ((V == 0.0) && (psi_1dot > 0.0)) {
    ecefPosWithENUOrigin_idx_1 = 0.3;
    ecefPosWithENUOrigin_idx_0 = 1.5707963267948966;
  } else if ((V == 0.0) && (psi_1dot < 0.0)) {
    ecefPosWithENUOrigin_idx_1 = 0.3;
    ecefPosWithENUOrigin_idx_0 = -1.5707963267948966;
  } else {
    ecefPosWithENUOrigin_idx_0 = V / psi_1dot;
    ecefPosWithENUOrigin_idx_1 =
        std::sqrt(ecefPosWithENUOrigin_idx_0 * ecefPosWithENUOrigin_idx_0 +
                  0.09) /
        ecefPosWithENUOrigin_idx_0;
    ecefPosWithENUOrigin_idx_0 = std::atan(0.3 / ecefPosWithENUOrigin_idx_0);
  }
  //  Predicted state and covariance
  a22 = dt * psi_1dot;
  c_sinphi = x_est[2] + a22;
  //  wrapAngle to make sure -pi/2 < psi < pi/2
  if (c_sinphi >= 1.5707963267948966) {
    u = c_sinphi - 3.1415926535897931;
  } else if (c_sinphi < 1.5707963267948966) {
    u = c_sinphi + 3.1415926535897931;
  } else {
    u = rtNaN;
  }
  v = x_est[2] - ecefPosWithENUOrigin_idx_0;
  N = std::sin(v);
  cosphi = std::cos(v);
  b_sinphi = dt * ecefPosWithENUOrigin_idx_1 * x_est[3];
  x_prd[0] = x_est[0] + b_sinphi * N;
  x_prd[1] = x_est[1] + b_sinphi * cosphi;
  x_prd[2] = u;
  v = dt * V_1dot;
  b_N = x_est[3] + v;
  x_prd[3] = b_N;
  //     % [Px]
  //     % [Py]
  //                             % [psi]
  //  [V]
  c_sinphi = (x_est[2] + 0.01) + a22;
  //  wrapAngle to make sure -pi/2 < psi < pi/2
  jac_fx[0] =
      (x_est[0] + 0.01) + dt * ecefPosWithENUOrigin_idx_1 * x_est[3] *
                              std::sin(x_est[2] - ecefPosWithENUOrigin_idx_0);
  jac_fx[1] = x_est[1] + dt * ecefPosWithENUOrigin_idx_1 * x_est[3] *
                             std::cos(x_est[2] - ecefPosWithENUOrigin_idx_0);
  jac_fx[2] = u;
  jac_fx[3] = b_N;
  jac_fx[4] = x_est[0] + dt * ecefPosWithENUOrigin_idx_1 * x_est[3] *
                             std::sin(x_est[2] - ecefPosWithENUOrigin_idx_0);
  jac_fx[5] =
      (x_est[1] + 0.01) + dt * ecefPosWithENUOrigin_idx_1 * x_est[3] *
                              std::cos(x_est[2] - ecefPosWithENUOrigin_idx_0);
  jac_fx[6] = u;
  jac_fx[7] = b_N;
  ecefPosWithENUOrigin_idx_1 = (x_est[2] + 0.01) - ecefPosWithENUOrigin_idx_0;
  jac_fx[8] = x_est[0] + b_sinphi * std::sin(ecefPosWithENUOrigin_idx_1);
  jac_fx[9] = x_est[1] + b_sinphi * std::cos(ecefPosWithENUOrigin_idx_1);
  if (c_sinphi >= 1.5707963267948966) {
    jac_fx[10] = c_sinphi - 3.1415926535897931;
  } else if (c_sinphi < 1.5707963267948966) {
    jac_fx[10] = c_sinphi + 3.1415926535897931;
  } else {
    jac_fx[10] = rtNaN;
  }
  jac_fx[11] = b_N;
  jac_fx[12] = (x_est[0] + b_sinphi) + 0.01 * N;
  jac_fx[13] = (x_est[1] + b_sinphi) + 0.01 * cosphi;
  jac_fx[14] = u;
  jac_fx[15] = (x_est[3] + 0.01) + v;
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
  b_x_prd[3] = b_N;
  b_x_prd[7] = b_N;
  b_x_prd[11] = b_N;
  b_x_prd[15] = b_N;
  for (r1 = 0; r1 < 16; r1++) {
    jac_fx[r1] = (jac_fx[r1] - b_x_prd[r1]) / 0.01;
  }
  ecefPosWithENUOrigin_idx_1 = 10.0 * dt;
  ecefPosWithENUOrigin_idx_0 = 0.01 * dt;
  for (r1 = 0; r1 < 4; r1++) {
    d = jac_fx[r1];
    ecefPos_idx_0 = jac_fx[r1 + 4];
    a22 = jac_fx[r1 + 8];
    c_sinphi = jac_fx[r1 + 12];
    for (int i{0}; i < 4; i++) {
      i1 = i << 2;
      b_x_prd[r1 + i1] = ((d * p_est[i1] + ecefPos_idx_0 * p_est[i1 + 1]) +
                          a22 * p_est[i1 + 2]) +
                         c_sinphi * p_est[i1 + 3];
    }
    d = b_x_prd[r1];
    ecefPos_idx_0 = b_x_prd[r1 + 4];
    a22 = b_x_prd[r1 + 8];
    c_sinphi = b_x_prd[r1 + 12];
    for (int i{0}; i < 4; i++) {
      p_prd[r1 + (i << 2)] = ((d * jac_fx[i] + ecefPos_idx_0 * jac_fx[i + 4]) +
                              a22 * jac_fx[i + 8]) +
                             c_sinphi * jac_fx[i + 12];
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
  for (r1 = 0; r1 < 16; r1++) {
    p_prd[r1] += b_x_prd[r1];
  }
  //  Estimation
  //  measurement innovation
  jac_hx[0] = x_prd[0] + 0.01;
  jac_hx[1] = x_prd[1];
  jac_hx[2] = x_prd[0];
  jac_hx[3] = x_prd[1] + 0.01;
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
  for (r1 = 0; r1 < 8; r1++) {
    jac_hx[r1] = (jac_hx[r1] - c_x_prd[r1]) / 0.01;
  }
  //             % measurement covariance
  for (r1 = 0; r1 < 2; r1++) {
    for (int i{0}; i < 4; i++) {
      klm_gain_tmp = r1 + (i << 1);
      klm_gain[i + (r1 << 2)] = jac_hx[klm_gain_tmp];
      i1 = i << 2;
      c_x_prd[klm_gain_tmp] =
          ((jac_hx[r1] * p_prd[i1] + jac_hx[r1 + 2] * p_prd[i1 + 1]) +
           jac_hx[r1 + 4] * p_prd[i1 + 2]) +
          jac_hx[r1 + 6] * p_prd[i1 + 3];
    }
  }
  for (r1 = 0; r1 < 2; r1++) {
    d = c_x_prd[r1];
    ecefPos_idx_0 = c_x_prd[r1 + 2];
    a22 = c_x_prd[r1 + 4];
    c_sinphi = c_x_prd[r1 + 6];
    for (int i{0}; i < 2; i++) {
      i1 = i << 2;
      r2 = r1 + (i << 1);
      S[r2] = (((d * klm_gain[i1] + ecefPos_idx_0 * klm_gain[i1 + 1]) +
                a22 * klm_gain[i1 + 2]) +
               c_sinphi * klm_gain[i1 + 3]) +
              R[r2];
    }
  }
  for (r1 = 0; r1 < 4; r1++) {
    d = p_prd[r1];
    ecefPos_idx_0 = p_prd[r1 + 4];
    a22 = p_prd[r1 + 8];
    c_sinphi = p_prd[r1 + 12];
    for (int i{0}; i < 2; i++) {
      i1 = i << 2;
      c_x_prd[r1 + i1] =
          ((d * klm_gain[i1] + ecefPos_idx_0 * klm_gain[i1 + 1]) +
           a22 * klm_gain[i1 + 2]) +
          c_sinphi * klm_gain[i1 + 3];
    }
  }
  if (std::abs(S[1]) > std::abs(S[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }
  ecefPosWithENUOrigin_idx_0 = S[r2] / S[r1];
  ecefPosWithENUOrigin_idx_1 = S[r1 + 2];
  a22 = S[r2 + 2] - ecefPosWithENUOrigin_idx_0 * ecefPosWithENUOrigin_idx_1;
  klm_gain_tmp = r1 << 2;
  klm_gain[klm_gain_tmp] = c_x_prd[0] / S[r1];
  r2 <<= 2;
  klm_gain[r2] =
      (c_x_prd[4] - klm_gain[klm_gain_tmp] * ecefPosWithENUOrigin_idx_1) / a22;
  klm_gain[klm_gain_tmp] -= klm_gain[r2] * ecefPosWithENUOrigin_idx_0;
  klm_gain[klm_gain_tmp + 1] = c_x_prd[1] / S[r1];
  klm_gain[r2 + 1] =
      (c_x_prd[5] - klm_gain[klm_gain_tmp + 1] * ecefPosWithENUOrigin_idx_1) /
      a22;
  klm_gain[klm_gain_tmp + 1] -= klm_gain[r2 + 1] * ecefPosWithENUOrigin_idx_0;
  klm_gain[klm_gain_tmp + 2] = c_x_prd[2] / S[r1];
  klm_gain[r2 + 2] =
      (c_x_prd[6] - klm_gain[klm_gain_tmp + 2] * ecefPosWithENUOrigin_idx_1) /
      a22;
  klm_gain[klm_gain_tmp + 2] -= klm_gain[r2 + 2] * ecefPosWithENUOrigin_idx_0;
  klm_gain[klm_gain_tmp + 3] = c_x_prd[3] / S[r1];
  klm_gain[r2 + 3] =
      (c_x_prd[7] - klm_gain[klm_gain_tmp + 3] * ecefPosWithENUOrigin_idx_1) /
      a22;
  klm_gain[klm_gain_tmp + 3] -= klm_gain[r2 + 3] * ecefPosWithENUOrigin_idx_0;
  //  Estimated state and covariance
  ecefPosWithENUOrigin_idx_1 = rho - x_prd[0];
  ecefPosWithENUOrigin_idx_0 = sinphi - x_prd[1];
  for (r1 = 0; r1 < 4; r1++) {
    x_est[r1] = x_prd[r1] + (klm_gain[r1] * ecefPosWithENUOrigin_idx_1 +
                             klm_gain[r1 + 4] * ecefPosWithENUOrigin_idx_0);
  }
  std::memset(&jac_fx[0], 0, 16U * sizeof(double));
  jac_fx[0] = 1.0;
  jac_fx[5] = 1.0;
  jac_fx[10] = 1.0;
  jac_fx[15] = 1.0;
  for (r1 = 0; r1 < 4; r1++) {
    d = klm_gain[r1];
    ecefPos_idx_0 = klm_gain[r1 + 4];
    for (int i{0}; i < 4; i++) {
      i1 = i << 1;
      r2 = r1 + (i << 2);
      b_x_prd[r2] =
          jac_fx[r2] - (d * jac_hx[i1] + ecefPos_idx_0 * jac_hx[i1 + 1]);
    }
    d = b_x_prd[r1];
    ecefPos_idx_0 = b_x_prd[r1 + 4];
    a22 = b_x_prd[r1 + 8];
    c_sinphi = b_x_prd[r1 + 12];
    for (int i{0}; i < 4; i++) {
      i1 = i << 2;
      p_est[r1 + i1] = ((d * p_prd[i1] + ecefPos_idx_0 * p_prd[i1 + 1]) +
                        a22 * p_prd[i1 + 2]) +
                       c_sinphi * p_prd[i1 + 3];
    }
  }
  //  Compute the estimated measurements
  cosphi = -6.914744;
  coder::b_cosd(&cosphi);
  sinphi = -6.914744;
  coder::b_sind(&sinphi);
  u = 107.60981;
  coder::b_cosd(&u);
  v = 107.60981;
  coder::b_sind(&v);
  b_sinphi = -6.914744;
  coder::b_sind(&b_sinphi);
  N = 6.378137E+6 /
      std::sqrt(1.0 - 0.0066943799901413165 * (b_sinphi * b_sinphi));
  d = -6.914744;
  coder::b_cosd(&d);
  rho = (N + 800.0) * d;
  ecefPosWithENUOrigin_idx_1 = cosphi * 800.0 - sinphi * x_est[1];
  ecefPos_idx_0 = rho * u + (u * ecefPosWithENUOrigin_idx_1 - v * x_est[0]);
  b_N = rho * v + (v * ecefPosWithENUOrigin_idx_1 + u * x_est[0]);
  ecefPosWithENUOrigin_idx_0 = (N * 0.99330562000985867 + 800.0) * b_sinphi +
                               (sinphi * 800.0 + cosphi * x_est[1]);
  rho = rt_hypotd_snf(ecefPos_idx_0, b_N);
  u = 6.378137E+6 * rho;
  v = 6.3567523142451793E+6 * ecefPosWithENUOrigin_idx_0 *
      (42841.311513313565 / rt_hypotd_snf(rho, ecefPosWithENUOrigin_idx_0) +
       1.0);
  a22 = u;
  if (!std::isnan(u)) {
    a22 = (u > 0.0);
  }
  N = a22 / rt_hypotd_snf(1.0, v / u);
  a22 = v;
  if (!std::isnan(v)) {
    if (v < 0.0) {
      a22 = -1.0;
    } else {
      a22 = (v > 0.0);
    }
  }
  ecefPosWithENUOrigin_idx_1 = a22 / rt_hypotd_snf(1.0, u / v);
  r2 = 0;
  iterate = true;
  while (iterate && (r2 < 5)) {
    cosphi = N;
    b_sinphi = ecefPosWithENUOrigin_idx_1;
    u = rho - 42697.672707179969 * rt_powd_snf(N, 3.0);
    v = ecefPosWithENUOrigin_idx_0 +
        42841.311513313565 * rt_powd_snf(ecefPosWithENUOrigin_idx_1, 3.0);
    c_sinphi = 6.378137E+6 * u;
    ecefPosWithENUOrigin_idx_1 = 6.3567523142451793E+6 * v;
    a22 = c_sinphi;
    if (!std::isnan(c_sinphi)) {
      if (c_sinphi < 0.0) {
        a22 = -1.0;
      } else {
        a22 = (c_sinphi > 0.0);
      }
    }
    N = a22 / rt_hypotd_snf(1.0, ecefPosWithENUOrigin_idx_1 / c_sinphi);
    a22 = ecefPosWithENUOrigin_idx_1;
    if (!std::isnan(ecefPosWithENUOrigin_idx_1)) {
      if (ecefPosWithENUOrigin_idx_1 < 0.0) {
        a22 = -1.0;
      } else {
        a22 = (ecefPosWithENUOrigin_idx_1 > 0.0);
      }
    }
    ecefPosWithENUOrigin_idx_1 =
        a22 / rt_hypotd_snf(1.0, c_sinphi / ecefPosWithENUOrigin_idx_1);
    frexp(1.5707963267948966, &exponent);
    iterate =
        (rt_hypotd_snf(N - cosphi, ecefPosWithENUOrigin_idx_1 - b_sinphi) >
         2.2204460492503131E-16);
    r2++;
  }
  ecefPosWithENUOrigin_idx_1 = 57.295779513082323 * rt_atan2d_snf(v, u);
  sinphi = ecefPosWithENUOrigin_idx_1;
  coder::b_sind(&sinphi);
  d = ecefPosWithENUOrigin_idx_1;
  coder::b_cosd(&d);
  P[0] = ecefPosWithENUOrigin_idx_1;
  P[1] = 57.295779513082323 * rt_atan2d_snf(b_N, ecefPos_idx_0);
  P[2] = x_est[2];
  P[3] = x_est[3];
}

//
// Arguments    : void
// Return Type  : void
//
void x_est_not_empty_init()
{
  x_est_not_empty = false;
}

//
// File trailer for ekf.cpp
//
// [EOF]
//
