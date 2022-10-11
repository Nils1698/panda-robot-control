//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// cubic_trajectory_generator_initialize.cpp
//
// Code generation for function 'cubic_trajectory_generator_initialize'
//

// Include files
#include "cubic_trajectory_generator_initialize.h"
#include "_coder_cubic_trajectory_generator_mex.h"
#include "cubic_trajectory_generator_data.h"
#include "rt_nonfinite.h"

// Function Definitions
void cubic_trajectory_generator_initialize()
{
  static const volatile char_T *emlrtBreakCheckR2012bFlagVar{nullptr};
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mex_InitInfAndNan();
  mexFunctionCreateRootTLS();
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

// End of code generation (cubic_trajectory_generator_initialize.cpp)
