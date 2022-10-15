//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_cubic_trajectory_generator_pos_vel_api.h
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 16-May-2022 12:36:45
//

#ifndef _CODER_CUBIC_TRAJECTORY_GENERATOR_POS_VEL_API_H
#define _CODER_CUBIC_TRAJECTORY_GENERATOR_POS_VEL_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void cubic_trajectory_generator_pos_vel(real_T wayPoints[6],
                                        real_T wayPointVels[6],
                                        real_T traj_pos[60],
                                        real_T traj_vel[60]);

void cubic_trajectory_generator_pos_vel_api(const mxArray *const prhs[2],
                                            int32_T nlhs,
                                            const mxArray *plhs[2]);

void cubic_trajectory_generator_pos_vel_atexit();

void cubic_trajectory_generator_pos_vel_initialize();

void cubic_trajectory_generator_pos_vel_terminate();

void cubic_trajectory_generator_pos_vel_xil_shutdown();

void cubic_trajectory_generator_pos_vel_xil_terminate();

#endif
//
// File trailer for _coder_cubic_trajectory_generator_pos_vel_api.h
//
// [EOF]
//
