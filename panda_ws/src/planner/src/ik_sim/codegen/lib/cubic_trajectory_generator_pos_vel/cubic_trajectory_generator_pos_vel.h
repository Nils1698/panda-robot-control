//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: cubic_trajectory_generator_pos_vel.h
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 16-May-2022 12:36:45
//

#ifndef CUBIC_TRAJECTORY_GENERATOR_POS_VEL_H
#define CUBIC_TRAJECTORY_GENERATOR_POS_VEL_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void cubic_trajectory_generator_pos_vel(const double wayPoints[6],
                                               const double wayPointVels[6],
                                               double traj_pos[60],
                                               double traj_vel[60]);

#endif
//
// File trailer for cubic_trajectory_generator_pos_vel.h
//
// [EOF]
//
