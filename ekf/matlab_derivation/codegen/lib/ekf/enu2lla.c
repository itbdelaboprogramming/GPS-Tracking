/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: enu2lla.c
 *
 * MATLAB Coder version            : 5.4
 * C/C++ source code generated on  : 08-Dec-2022 10:22:30
 */

/* Include Files */
#include "enu2lla.h"
#include "cosd.h"
#include "rt_nonfinite.h"
#include "sind.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

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
 * Arguments    : const double xyzENU[3]
 *                const double lla0[3]
 *                double lla[3]
 * Return Type  : void
 */
void enu2lla(const double xyzENU[3], const double lla0[3], double lla[3])
{
  double N;
  double b_sinphi;
  double bv;
  double coslambda;
  double cosphi;
  double d;
  double ecefPos_idx_0;
  double ecefPos_idx_1;
  double ecefPos_idx_2;
  double rho;
  double sinlambda;
  double sinphi;
  double tmp;
  int count;
  int exponent;
  boolean_T iterate;
  cosphi = lla0[0];
  b_cosd(&cosphi);
  sinphi = lla0[0];
  b_sind(&sinphi);
  coslambda = lla0[1];
  b_cosd(&coslambda);
  sinlambda = lla0[1];
  b_sind(&sinlambda);
  b_sinphi = lla0[0];
  b_sind(&b_sinphi);
  N = 6.378137E+6 / sqrt(1.0 - 0.0066943799901413165 * (b_sinphi * b_sinphi));
  d = lla0[0];
  b_cosd(&d);
  rho = (N + lla0[2]) * d;
  d = lla0[1];
  b_cosd(&d);
  bv = lla0[1];
  b_sind(&bv);
  tmp = cosphi * xyzENU[2] - sinphi * xyzENU[1];
  ecefPos_idx_0 = rho * d + (coslambda * tmp - sinlambda * xyzENU[0]);
  ecefPos_idx_1 = rho * bv + (sinlambda * tmp + coslambda * xyzENU[0]);
  ecefPos_idx_2 = (N * 0.99330562000985867 + lla0[2]) * b_sinphi +
                  (sinphi * xyzENU[2] + cosphi * xyzENU[1]);
  rho = rt_hypotd_snf(ecefPos_idx_0, ecefPos_idx_1);
  tmp = 6.378137E+6 * rho;
  cosphi = 6.3567523142451793E+6 * ecefPos_idx_2 *
           (42841.311513313565 / rt_hypotd_snf(rho, ecefPos_idx_2) + 1.0);
  coslambda = tmp;
  if (!rtIsNaN(tmp)) {
    coslambda = (tmp > 0.0);
  }
  b_sinphi = coslambda / rt_hypotd_snf(1.0, cosphi / tmp);
  coslambda = cosphi;
  if (!rtIsNaN(cosphi)) {
    if (cosphi < 0.0) {
      coslambda = -1.0;
    } else {
      coslambda = (cosphi > 0.0);
    }
  }
  coslambda /= rt_hypotd_snf(1.0, tmp / cosphi);
  count = 0;
  iterate = true;
  while (iterate && (count < 5)) {
    sinphi = b_sinphi;
    N = coslambda;
    tmp = rho - 42697.672707179969 * rt_powd_snf(b_sinphi, 3.0);
    cosphi = ecefPos_idx_2 + 42841.311513313565 * rt_powd_snf(coslambda, 3.0);
    sinlambda = 6.378137E+6 * tmp;
    bv = 6.3567523142451793E+6 * cosphi;
    coslambda = sinlambda;
    if (!rtIsNaN(sinlambda)) {
      if (sinlambda < 0.0) {
        coslambda = -1.0;
      } else {
        coslambda = (sinlambda > 0.0);
      }
    }
    b_sinphi = coslambda / rt_hypotd_snf(1.0, bv / sinlambda);
    coslambda = bv;
    if (!rtIsNaN(bv)) {
      if (bv < 0.0) {
        coslambda = -1.0;
      } else {
        coslambda = (bv > 0.0);
      }
    }
    coslambda /= rt_hypotd_snf(1.0, sinlambda / bv);
    frexp(1.5707963267948966, &exponent);
    iterate = (rt_hypotd_snf(b_sinphi - sinphi, coslambda - N) >
               2.2204460492503131E-16);
    count++;
  }
  coslambda = 57.295779513082323 * rt_atan2d_snf(cosphi, tmp);
  sinphi = coslambda;
  b_sind(&sinphi);
  N = 6.378137E+6 / sqrt(1.0 - 0.0066943799901413165 * (sinphi * sinphi));
  d = coslambda;
  b_cosd(&d);
  lla[0] = coslambda;
  lla[1] = 57.295779513082323 * rt_atan2d_snf(ecefPos_idx_1, ecefPos_idx_0);
  lla[2] = (rho * d +
            (ecefPos_idx_2 + 0.0066943799901413165 * N * sinphi) * sinphi) -
           N;
}

/*
 * File trailer for enu2lla.c
 *
 * [EOF]
 */
