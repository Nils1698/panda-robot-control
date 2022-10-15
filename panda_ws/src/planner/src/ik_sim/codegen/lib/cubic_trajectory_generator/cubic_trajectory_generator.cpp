//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: cubic_trajectory_generator.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 12-May-2022 11:56:34
//

// Include Files
#include "cubic_trajectory_generator.h"
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
//                double trajectory[60]
// Return Type  : void
//
void cubic_trajectory_generator(const double wayPoints[6],
                                const double wayPointVels[6],
                                double trajectory[60])
{
  static const signed char pp_breaks[4]{-1, 0, 4, 5};
  cell_wrap_4 parsedResults;
  double newCoefs[36];
  double coefsWithFlatStart[24];
  double coefMat[12];
  double coeffMat[12];
  double b_wayPoints[6];
  double coeffVec[4];
  double wayPoints_idx_1;
  double xloc;
  int high_i;
  int ic0;
  int icp;
  int low_i;
  int low_ip1;
  for (low_i = 0; low_i < 2; low_i++) {
    b_wayPoints[3 * low_i] = wayPoints[low_i];
    parsedResults.f1[3 * low_i] = wayPointVels[low_i];
    low_ip1 = 3 * low_i + 1;
    b_wayPoints[low_ip1] = wayPoints[low_i + 2];
    parsedResults.f1[low_ip1] = wayPointVels[low_i + 2];
    low_ip1 = 3 * low_i + 2;
    b_wayPoints[low_ip1] = wayPoints[low_i + 4];
    parsedResults.f1[low_ip1] = wayPointVels[low_i + 4];
  }
  std::memset(&trajectory[0], 0, 60U * sizeof(double));
  for (low_ip1 = 0; low_ip1 < 3; low_ip1++) {
    coeffVec[0] = b_wayPoints[low_ip1];
    coeffVec[1] = parsedResults.f1[low_ip1];
    xloc = b_wayPoints[low_ip1 + 3] - (coeffVec[0] + 4.0 * coeffVec[1]);
    wayPoints_idx_1 =
        parsedResults.f1[low_ip1 + 3] - (0.0 * coeffVec[0] + coeffVec[1]);
    coeffVec[3] = coeffVec[0];
    coefMat[low_ip1] = -0.03125 * xloc + 0.0625 * wayPoints_idx_1;
    coefMat[low_ip1 + 3] = 0.1875 * xloc + -0.25 * wayPoints_idx_1;
    coefMat[low_ip1 + 6] = coeffVec[1];
    coefMat[low_ip1 + 9] = coeffVec[3];
  }
  std::memset(&coeffMat[0], 0, 12U * sizeof(double));
  for (low_i = 0; low_i < 3; low_i++) {
    coeffMat[low_i + 9] = ((coefMat[low_i] * 0.0 + coefMat[low_i + 3] * 0.0) +
                           coefMat[low_i + 6] * 0.0) +
                          coefMat[low_i + 9];
  }
  std::memset(&coefsWithFlatStart[0], 0, 24U * sizeof(double));
  for (low_ip1 = 0; low_ip1 < 4; low_ip1++) {
    coefsWithFlatStart[6 * low_ip1] = coeffMat[3 * low_ip1];
    coefsWithFlatStart[6 * low_ip1 + 3] = coefMat[3 * low_ip1];
    ic0 = 3 * low_ip1 + 1;
    coefsWithFlatStart[6 * low_ip1 + 1] = coeffMat[ic0];
    coefsWithFlatStart[6 * low_ip1 + 4] = coefMat[ic0];
    ic0 = 3 * low_ip1 + 2;
    coefsWithFlatStart[6 * low_ip1 + 2] = coeffMat[ic0];
    coefsWithFlatStart[6 * low_ip1 + 5] = coefMat[ic0];
    coeffVec[low_ip1] =
        rt_powd_snf(4.0, 4.0 - (static_cast<double>(low_ip1) + 1.0));
  }
  std::memset(&coeffMat[0], 0, 12U * sizeof(double));
  low_i = static_cast<int>(coeffVec[0]);
  low_ip1 = static_cast<int>(coeffVec[1]);
  ic0 = static_cast<int>(coeffVec[2]);
  high_i = static_cast<int>(coeffVec[3]);
  for (icp = 0; icp < 3; icp++) {
    coeffMat[icp + 9] =
        ((coefsWithFlatStart[icp + 3] * static_cast<double>(low_i) +
          coefsWithFlatStart[icp + 9] * static_cast<double>(low_ip1)) +
         coefsWithFlatStart[icp + 15] * static_cast<double>(ic0)) +
        coefsWithFlatStart[icp + 21] * static_cast<double>(high_i);
  }
  for (low_i = 0; low_i < 4; low_i++) {
    for (low_ip1 = 0; low_ip1 < 6; low_ip1++) {
      newCoefs[low_ip1 + 9 * low_i] = coefsWithFlatStart[low_ip1 + 6 * low_i];
    }
    newCoefs[9 * low_i + 6] = coeffMat[3 * low_i];
    newCoefs[9 * low_i + 7] = coeffMat[3 * low_i + 1];
    newCoefs[9 * low_i + 8] = coeffMat[3 * low_i + 2];
  }
  for (int ix{0}; ix < 20; ix++) {
    double d;
    double d1;
    int iv0;
    iv0 = ix * 3;
    low_i = 1;
    low_ip1 = 2;
    high_i = 4;
    while (high_i > low_ip1) {
      ic0 = (low_i + high_i) >> 1;
      if (0.21052631578947367 * static_cast<double>(ix) >= pp_breaks[ic0 - 1]) {
        low_i = ic0;
        low_ip1 = ic0 + 1;
      } else {
        high_i = ic0;
      }
    }
    icp = (low_i - 1) * 3;
    xloc = 0.21052631578947367 * static_cast<double>(ix) -
           static_cast<double>(pp_breaks[low_i - 1]);
    wayPoints_idx_1 = newCoefs[icp];
    d = newCoefs[icp + 1];
    d1 = newCoefs[icp + 2];
    for (low_ip1 = 0; low_ip1 < 3; low_ip1++) {
      ic0 = icp + (low_ip1 + 1) * 9;
      wayPoints_idx_1 = xloc * wayPoints_idx_1 + newCoefs[ic0];
      d = xloc * d + newCoefs[ic0 + 1];
      d1 = xloc * d1 + newCoefs[ic0 + 2];
    }
    trajectory[iv0 + 2] = d1;
    trajectory[iv0 + 1] = d;
    trajectory[iv0] = wayPoints_idx_1;
  }
  //  ik = robotics.InverseKinematics('RigidBodyTree',robot);
  //  weights = [0.1 0.1 0.1 1 1 1];
  //  initialguess = robot.homeConfiguration;
  //
  //  % Call inverse kinematics solver for every end-effector position using the
  //  % previous configuration as initial guess
  //  for idx = 1:size(trajectory,2)
  //      tform = trvec2tform(trajectory(:,idx)');
  //      configSoln(idx,:) = ik('panda_hand',tform,weights,initialguess);
  //      initialguess = configSoln(idx,:);
  //  end
}

//
// File trailer for cubic_trajectory_generator.cpp
//
// [EOF]
//
