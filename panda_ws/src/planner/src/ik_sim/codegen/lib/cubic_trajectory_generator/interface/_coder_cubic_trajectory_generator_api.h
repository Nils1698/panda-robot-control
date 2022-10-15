//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_cubic_trajectory_generator_api.h
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 12-May-2022 11:56:34
//

#ifndef _CODER_CUBIC_TRAJECTORY_GENERATOR_API_H
#define _CODER_CUBIC_TRAJECTORY_GENERATOR_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void cubic_trajectory_generator(real_T wayPoints[6], real_T wayPointVels[6],
                                real_T trajectory[60]);

void cubic_trajectory_generator_api(const mxArray *const prhs[2],
                                    const mxArray **plhs);

void cubic_trajectory_generator_atexit();

void cubic_trajectory_generator_initialize();

void cubic_trajectory_generator_terminate();

void cubic_trajectory_generator_xil_shutdown();

void cubic_trajectory_generator_xil_terminate();

#endif
//
// File trailer for _coder_cubic_trajectory_generator_api.h
//
// [EOF]
//
