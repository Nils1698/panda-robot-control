//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// cubic_trajectory_generator_terminate.cpp
//
// Code generation for function 'cubic_trajectory_generator_terminate'
//

// Include files
#include "cubic_trajectory_generator_terminate.h"
#include "_coder_cubic_trajectory_generator_mex.h"
#include "cubic_trajectory_generator_data.h"
#include "rt_nonfinite.h"

// Function Definitions
void cubic_trajectory_generator_atexit()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void cubic_trajectory_generator_terminate()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

// End of code generation (cubic_trajectory_generator_terminate.cpp)
