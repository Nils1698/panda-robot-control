//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_cubic_trajectory_generator_pos_vel_mex.h
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 16-May-2022 12:36:45
//

#ifndef _CODER_CUBIC_TRAJECTORY_GENERATOR_POS_VEL_MEX_H
#define _CODER_CUBIC_TRAJECTORY_GENERATOR_POS_VEL_MEX_H

// Include Files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

// Function Declarations
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS();

void unsafe_cubic_trajectory_generator_pos_vel_mexFunction(
    int32_T nlhs, mxArray *plhs[2], int32_T nrhs, const mxArray *prhs[2]);

#endif
//
// File trailer for _coder_cubic_trajectory_generator_pos_vel_mex.h
//
// [EOF]
//
