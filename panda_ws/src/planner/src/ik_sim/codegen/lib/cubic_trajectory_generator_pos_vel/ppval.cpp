//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ppval.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 16-May-2022 12:36:45
//

// Include Files
#include "ppval.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : const double pp_breaks[4]
//                const double pp_coefs[36]
//                const double x[20]
//                double v[60]
// Return Type  : void
//
namespace coder {
void ppval(const double pp_breaks[4], const double pp_coefs[36],
           const double x[20], double v[60])
{
  for (int ix{0}; ix < 20; ix++) {
    double d;
    double d1;
    double d2;
    double xloc;
    int high_i;
    int icp;
    int iv0;
    int low_i;
    int low_ip1;
    iv0 = ix * 3;
    low_i = 1;
    low_ip1 = 2;
    high_i = 4;
    while (high_i > low_ip1) {
      icp = (low_i + high_i) >> 1;
      if (x[ix] >= pp_breaks[icp - 1]) {
        low_i = icp;
        low_ip1 = icp + 1;
      } else {
        high_i = icp;
      }
    }
    icp = (low_i - 1) * 3;
    xloc = x[ix] - pp_breaks[low_i - 1];
    d = pp_coefs[icp];
    d1 = pp_coefs[icp + 1];
    d2 = pp_coefs[icp + 2];
    for (low_ip1 = 0; low_ip1 < 3; low_ip1++) {
      high_i = icp + (low_ip1 + 1) * 9;
      d = xloc * d + pp_coefs[high_i];
      d1 = xloc * d1 + pp_coefs[high_i + 1];
      d2 = xloc * d2 + pp_coefs[high_i + 2];
    }
    v[iv0 + 2] = d2;
    v[iv0 + 1] = d1;
    v[iv0] = d;
  }
}

} // namespace coder

//
// File trailer for ppval.cpp
//
// [EOF]
//
