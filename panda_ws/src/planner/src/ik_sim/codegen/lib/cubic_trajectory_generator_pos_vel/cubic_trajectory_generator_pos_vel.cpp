//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: cubic_trajectory_generator_pos_vel.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 16-May-2022 12:36:45
//

// Include Files
#include "cubic_trajectory_generator_pos_vel.h"
#include "ppval.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Type Definitions
struct cell_wrap_4 {
  double f1[6];
};

// Function Declarations
static double rt_powd_snf(double u0, double u1);

// Function Definitions
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
// Arguments    : const double wayPoints[6]
//                const double wayPointVels[6]
//                double traj_pos[60]
//                double traj_vel[60]
// Return Type  : void
//
void cubic_trajectory_generator_pos_vel(const double wayPoints[6],
                                        const double wayPointVels[6],
                                        double traj_pos[60],
                                        double traj_vel[60])
{
  static const double newBreaks[4]{-1.0, 0.0, 4.0, 5.0};
  cell_wrap_4 parsedResults;
  double dCoeffs[36];
  double newCoefs[36];
  double coefsWithFlatStart[24];
  double dv[20];
  double coefMat[12];
  double coeffMat[12];
  double b_wayPoints[6];
  double coeffVec[4];
  int b_i;
  int i;
  int i1;
  int wayPoints_tmp;
  for (i = 0; i < 2; i++) {
    b_wayPoints[3 * i] = wayPoints[i];
    parsedResults.f1[3 * i] = wayPointVels[i];
    wayPoints_tmp = 3 * i + 1;
    b_wayPoints[wayPoints_tmp] = wayPoints[i + 2];
    parsedResults.f1[wayPoints_tmp] = wayPointVels[i + 2];
    wayPoints_tmp = 3 * i + 2;
    b_wayPoints[wayPoints_tmp] = wayPoints[i + 4];
    parsedResults.f1[wayPoints_tmp] = wayPointVels[i + 4];
  }
  for (wayPoints_tmp = 0; wayPoints_tmp < 3; wayPoints_tmp++) {
    double wayPoints_idx_0;
    double wayPoints_idx_1;
    coeffVec[0] = b_wayPoints[wayPoints_tmp];
    coeffVec[1] = parsedResults.f1[wayPoints_tmp];
    wayPoints_idx_0 =
        b_wayPoints[wayPoints_tmp + 3] - (coeffVec[0] + 4.0 * coeffVec[1]);
    wayPoints_idx_1 =
        parsedResults.f1[wayPoints_tmp + 3] - (0.0 * coeffVec[0] + coeffVec[1]);
    coeffVec[3] = coeffVec[0];
    coefMat[wayPoints_tmp] =
        -0.03125 * wayPoints_idx_0 + 0.0625 * wayPoints_idx_1;
    coefMat[wayPoints_tmp + 3] =
        0.1875 * wayPoints_idx_0 + -0.25 * wayPoints_idx_1;
    coefMat[wayPoints_tmp + 6] = coeffVec[1];
    coefMat[wayPoints_tmp + 9] = coeffVec[3];
  }
  std::memset(&coeffMat[0], 0, 12U * sizeof(double));
  for (i = 0; i < 3; i++) {
    coeffMat[i + 9] =
        ((coefMat[i] * 0.0 + coefMat[i + 3] * 0.0) + coefMat[i + 6] * 0.0) +
        coefMat[i + 9];
  }
  std::memset(&coefsWithFlatStart[0], 0, 24U * sizeof(double));
  for (b_i = 0; b_i < 4; b_i++) {
    coefsWithFlatStart[6 * b_i] = coeffMat[3 * b_i];
    coefsWithFlatStart[6 * b_i + 3] = coefMat[3 * b_i];
    wayPoints_tmp = 3 * b_i + 1;
    coefsWithFlatStart[6 * b_i + 1] = coeffMat[wayPoints_tmp];
    coefsWithFlatStart[6 * b_i + 4] = coefMat[wayPoints_tmp];
    wayPoints_tmp = 3 * b_i + 2;
    coefsWithFlatStart[6 * b_i + 2] = coeffMat[wayPoints_tmp];
    coefsWithFlatStart[6 * b_i + 5] = coefMat[wayPoints_tmp];
    coeffVec[b_i] = rt_powd_snf(4.0, 4.0 - (static_cast<double>(b_i) + 1.0));
  }
  std::memset(&coeffMat[0], 0, 12U * sizeof(double));
  i = static_cast<int>(coeffVec[0]);
  wayPoints_tmp = static_cast<int>(coeffVec[1]);
  b_i = static_cast<int>(coeffVec[2]);
  i1 = static_cast<int>(coeffVec[3]);
  for (int i2{0}; i2 < 3; i2++) {
    coeffMat[i2 + 9] =
        ((coefsWithFlatStart[i2 + 3] * static_cast<double>(i) +
          coefsWithFlatStart[i2 + 9] * static_cast<double>(wayPoints_tmp)) +
         coefsWithFlatStart[i2 + 15] * static_cast<double>(b_i)) +
        coefsWithFlatStart[i2 + 21] * static_cast<double>(i1);
  }
  for (i = 0; i < 4; i++) {
    for (wayPoints_tmp = 0; wayPoints_tmp < 6; wayPoints_tmp++) {
      newCoefs[wayPoints_tmp + 9 * i] =
          coefsWithFlatStart[wayPoints_tmp + 6 * i];
    }
    newCoefs[9 * i + 6] = coeffMat[3 * i];
    newCoefs[9 * i + 7] = coeffMat[3 * i + 1];
    newCoefs[9 * i + 8] = coeffMat[3 * i + 2];
  }
  for (i = 0; i < 20; i++) {
    dv[i] = 0.21052631578947367 * static_cast<double>(i);
  }
  coder::ppval(newBreaks, newCoefs, dv, traj_pos);
  coeffVec[0] = -1.0;
  coeffVec[1] = 0.0;
  coeffVec[3] = 5.0;
  coeffVec[2] = 4.01;
  std::memset(&dCoeffs[0], 0, 36U * sizeof(double));
  for (b_i = 0; b_i < 3; b_i++) {
    for (i = 0; i < 9; i++) {
      dCoeffs[i + 9 * (b_i + 1)] =
          ((4.0 - (static_cast<double>(b_i) + 2.0)) + 1.0) *
          newCoefs[i + 9 * b_i];
    }
  }
  for (i = 0; i < 20; i++) {
    dv[i] = 0.21052631578947367 * static_cast<double>(i);
  }
  coder::ppval(coeffVec, dCoeffs, dv, traj_vel);
}

//
// File trailer for cubic_trajectory_generator_pos_vel.cpp
//
// [EOF]
//
