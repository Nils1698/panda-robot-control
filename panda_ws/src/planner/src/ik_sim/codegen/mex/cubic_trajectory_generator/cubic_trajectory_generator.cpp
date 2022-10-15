//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// cubic_trajectory_generator.cpp
//
// Code generation for function 'cubic_trajectory_generator'
//

// Include files
#include "cubic_trajectory_generator.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <cstring>

// Type Definitions
struct cell_wrap_4 {
  real_T f1[6];
};

// Variable Definitions
static emlrtRSInfo emlrtRSI{
    9,                            // lineNo
    "cubic_trajectory_generator", // fcnName
    "F:\\DTU\\M.Sc\\5th_win_msc\\program\\inverse_kinematics_sim\\cubic_"
    "trajectory_generator.m" // pathName
};

static emlrtRSInfo b_emlrtRSI{
    62,                                                              // lineNo
    "cubicpolytraj",                                                 // fcnName
    "F:\\MATLAB\\R2021b\\toolbox\\shared\\polytraj\\cubicpolytraj.m" // pathName
};

static emlrtRSInfo c_emlrtRSI{
    75,                                                              // lineNo
    "cubicpolytraj",                                                 // fcnName
    "F:\\MATLAB\\R2021b\\toolbox\\shared\\polytraj\\cubicpolytraj.m" // pathName
};

static emlrtRSInfo d_emlrtRSI{
    76,                                                              // lineNo
    "cubicpolytraj",                                                 // fcnName
    "F:\\MATLAB\\R2021b\\toolbox\\shared\\polytraj\\cubicpolytraj.m" // pathName
};

static emlrtRSInfo e_emlrtRSI{
    77,                                                              // lineNo
    "cubicpolytraj",                                                 // fcnName
    "F:\\MATLAB\\R2021b\\toolbox\\shared\\polytraj\\cubicpolytraj.m" // pathName
};

static emlrtRSInfo f_emlrtRSI{
    103,                                                             // lineNo
    "cubicpolytraj",                                                 // fcnName
    "F:\\MATLAB\\R2021b\\toolbox\\shared\\polytraj\\cubicpolytraj.m" // pathName
};

static emlrtRSInfo g_emlrtRSI{
    113,                                                             // lineNo
    "cubicpolytraj",                                                 // fcnName
    "F:\\MATLAB\\R2021b\\toolbox\\shared\\polytraj\\cubicpolytraj.m" // pathName
};

static emlrtRSInfo h_emlrtRSI{
    117,                                                             // lineNo
    "cubicpolytraj",                                                 // fcnName
    "F:\\MATLAB\\R2021b\\toolbox\\shared\\polytraj\\cubicpolytraj.m" // pathName
};

static emlrtRSInfo
    i_emlrtRSI{
        76,                   // lineNo
        "validateattributes", // fcnName
        "F:"
        "\\MATLAB\\R2021b\\toolbox\\eml\\lib\\matlab\\lang\\validateattributes."
        "m" // pathName
    };

static emlrtRTEInfo emlrtRTEI{
    14,               // lineNo
    37,               // colNo
    "validatefinite", // fName
    "F:\\MATLAB\\R2021b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatefinite.m" // pName
};

// Function Definitions
void cubic_trajectory_generator(const emlrtStack *sp, const real_T wayPoints[6],
                                const real_T wayPointVels[6],
                                real_T trajectory[60])
{
  static const int8_T pp_breaks[4]{-1, 0, 4, 5};
  cell_wrap_4 parsedResults;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T newCoefs[36];
  real_T coefsWithFlatStart[24];
  real_T coefMat[12];
  real_T coeffMat[12];
  real_T b_wayPoints[6];
  real_T coeffVec_idx_0;
  real_T coeffVec_idx_1;
  real_T wayPoints_idx_0;
  real_T wayPoints_idx_1;
  int32_T high_i;
  int32_T low_ip1;
  boolean_T exitg1;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  for (high_i = 0; high_i < 2; high_i++) {
    b_wayPoints[3 * high_i] = wayPoints[high_i];
    b_wayPoints[3 * high_i + 1] = wayPoints[high_i + 2];
    b_wayPoints[3 * high_i + 2] = wayPoints[high_i + 4];
  }
  b_st.site = &b_emlrtRSI;
  c_st.site = &i_emlrtRSI;
  p = true;
  low_ip1 = 0;
  exitg1 = false;
  while ((!exitg1) && (low_ip1 < 6)) {
    if ((!muDoubleScalarIsInf(b_wayPoints[low_ip1])) &&
        (!muDoubleScalarIsNaN(b_wayPoints[low_ip1]))) {
      low_ip1++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:cubicpolytraj:expectedFinite", 3, 4, 9, "wayPoints");
  }
  b_st.site = &c_emlrtRSI;
  b_st.site = &d_emlrtRSI;
  for (high_i = 0; high_i < 2; high_i++) {
    parsedResults.f1[3 * high_i] = wayPointVels[high_i];
    parsedResults.f1[3 * high_i + 1] = wayPointVels[high_i + 2];
    parsedResults.f1[3 * high_i + 2] = wayPointVels[high_i + 4];
  }
  b_st.site = &e_emlrtRSI;
  std::memset(&trajectory[0], 0, 60U * sizeof(real_T));
  for (low_ip1 = 0; low_ip1 < 3; low_ip1++) {
    b_st.site = &f_emlrtRSI;
    coeffVec_idx_0 = b_wayPoints[low_ip1];
    coeffVec_idx_1 = parsedResults.f1[low_ip1];
    wayPoints_idx_0 =
        b_wayPoints[low_ip1 + 3] - (coeffVec_idx_0 + 4.0 * coeffVec_idx_1);
    wayPoints_idx_1 =
        parsedResults.f1[low_ip1 + 3] - (0.0 * coeffVec_idx_0 + coeffVec_idx_1);
    coefMat[low_ip1] = -0.03125 * wayPoints_idx_0 + 0.0625 * wayPoints_idx_1;
    coefMat[low_ip1 + 3] = 0.1875 * wayPoints_idx_0 + -0.25 * wayPoints_idx_1;
    coefMat[low_ip1 + 6] = coeffVec_idx_1;
    coefMat[low_ip1 + 9] = coeffVec_idx_0;
  }
  b_st.site = &g_emlrtRSI;
  std::memset(&coeffMat[0], 0, 12U * sizeof(real_T));
  for (high_i = 0; high_i < 3; high_i++) {
    coeffMat[high_i + 9] =
        ((coefMat[high_i] * 0.0 + coefMat[high_i + 3] * 0.0) +
         coefMat[high_i + 6] * 0.0) +
        coefMat[high_i + 9];
  }
  std::memset(&coefsWithFlatStart[0], 0, 24U * sizeof(real_T));
  for (high_i = 0; high_i < 4; high_i++) {
    coefsWithFlatStart[6 * high_i] = coeffMat[3 * high_i];
    coefsWithFlatStart[6 * high_i + 3] = coefMat[3 * high_i];
    low_ip1 = 3 * high_i + 1;
    coefsWithFlatStart[6 * high_i + 1] = coeffMat[low_ip1];
    coefsWithFlatStart[6 * high_i + 4] = coefMat[low_ip1];
    low_ip1 = 3 * high_i + 2;
    coefsWithFlatStart[6 * high_i + 2] = coeffMat[low_ip1];
    coefsWithFlatStart[6 * high_i + 5] = coefMat[low_ip1];
  }
  std::memset(&coeffMat[0], 0, 12U * sizeof(real_T));
  for (high_i = 0; high_i < 3; high_i++) {
    coeffMat[high_i + 9] = ((coefsWithFlatStart[high_i + 3] * 64.0 +
                             coefsWithFlatStart[high_i + 9] * 16.0) +
                            coefsWithFlatStart[high_i + 15] * 4.0) +
                           coefsWithFlatStart[high_i + 21];
  }
  for (high_i = 0; high_i < 4; high_i++) {
    for (low_ip1 = 0; low_ip1 < 6; low_ip1++) {
      newCoefs[low_ip1 + 9 * high_i] = coefsWithFlatStart[low_ip1 + 6 * high_i];
    }
    newCoefs[9 * high_i + 6] = coeffMat[3 * high_i];
    newCoefs[9 * high_i + 7] = coeffMat[3 * high_i + 1];
    newCoefs[9 * high_i + 8] = coeffMat[3 * high_i + 2];
  }
  b_st.site = &h_emlrtRSI;
  for (int32_T ix{0}; ix < 20; ix++) {
    int32_T iv0;
    int32_T low_i;
    int32_T mid_i;
    iv0 = ix * 3;
    low_i = 1;
    low_ip1 = 2;
    high_i = 4;
    while (high_i > low_ip1) {
      mid_i = (low_i + high_i) >> 1;
      if (0.21052631578947367 * static_cast<real_T>(ix) >=
          pp_breaks[mid_i - 1]) {
        low_i = mid_i;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }
    high_i = (low_i - 1) * 3;
    coeffVec_idx_0 = 0.21052631578947367 * static_cast<real_T>(ix) -
                     static_cast<real_T>(pp_breaks[low_i - 1]);
    coeffVec_idx_1 = newCoefs[high_i];
    wayPoints_idx_0 = newCoefs[high_i + 1];
    wayPoints_idx_1 = newCoefs[high_i + 2];
    for (mid_i = 0; mid_i < 3; mid_i++) {
      low_ip1 = high_i + (mid_i + 1) * 9;
      coeffVec_idx_1 = coeffVec_idx_0 * coeffVec_idx_1 + newCoefs[low_ip1];
      wayPoints_idx_0 =
          coeffVec_idx_0 * wayPoints_idx_0 + newCoefs[low_ip1 + 1];
      wayPoints_idx_1 =
          coeffVec_idx_0 * wayPoints_idx_1 + newCoefs[low_ip1 + 2];
    }
    trajectory[iv0 + 2] = wayPoints_idx_1;
    trajectory[iv0 + 1] = wayPoints_idx_0;
    trajectory[iv0] = coeffVec_idx_1;
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

// End of code generation (cubic_trajectory_generator.cpp)
