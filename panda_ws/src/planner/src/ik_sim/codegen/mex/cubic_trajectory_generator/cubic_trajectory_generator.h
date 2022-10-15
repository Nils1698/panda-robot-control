//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// cubic_trajectory_generator.h
//
// Code generation for function 'cubic_trajectory_generator'
//

#pragma once

// Include files
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// Function Declarations
void cubic_trajectory_generator(const emlrtStack *sp, const real_T wayPoints[6],
                                const real_T wayPointVels[6],
                                real_T trajectory[60]);

// End of code generation (cubic_trajectory_generator.h)
