//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_cubic_trajectory_generator_mex.cpp
//
// Code generation for function '_coder_cubic_trajectory_generator_mex'
//

// Include files
#include "_coder_cubic_trajectory_generator_mex.h"
#include "_coder_cubic_trajectory_generator_api.h"
#include "cubic_trajectory_generator_data.h"
#include "cubic_trajectory_generator_initialize.h"
#include "cubic_trajectory_generator_terminate.h"
#include "rt_nonfinite.h"
#include <stdexcept>

void emlrtExceptionBridge();
void emlrtExceptionBridge()
{
  throw std::runtime_error("");
}
// Function Definitions
void cubic_trajectory_generator_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                            int32_T nrhs,
                                            const mxArray *prhs[2])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 2, 4,
                        26, "cubic_trajectory_generator");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 26,
                        "cubic_trajectory_generator");
  }
  // Call the function.
  cubic_trajectory_generator_api(prhs, &outputs);
  // Copy over outputs to the caller.
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&cubic_trajectory_generator_atexit);
  // Module initialization.
  cubic_trajectory_generator_initialize();
  try {
    emlrtShouldCleanupOnError((emlrtCTX *)emlrtRootTLSGlobal, false);
    // Dispatch the entry-point.
    cubic_trajectory_generator_mexFunction(nlhs, plhs, nrhs, prhs);
    // Module termination.
    cubic_trajectory_generator_terminate();
  } catch (...) {
    emlrtCleanupOnException((emlrtCTX *)emlrtRootTLSGlobal);
    throw;
  }
}

emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2021a(&emlrtRootTLSGlobal, &emlrtContextGlobal, nullptr, 1,
                           (void *)&emlrtExceptionBridge);
  return emlrtRootTLSGlobal;
}

// End of code generation (_coder_cubic_trajectory_generator_mex.cpp)
