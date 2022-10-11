//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_cubic_trajectory_generator_pos_vel_api.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 16-May-2022 12:36:45
//

// Include Files
#include "_coder_cubic_trajectory_generator_pos_vel_api.h"
#include "_coder_cubic_trajectory_generator_pos_vel_mex.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131611U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "cubic_trajectory_generator_pos_vel",                 // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

// Function Declarations
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[6];

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *wayPoints,
                                 const char_T *identifier))[6];

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[6];

static const mxArray *emlrt_marshallOut(const real_T u[60]);

// Function Definitions
//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[6]
//
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[6]
{
  static const int32_T dims[2]{2, 3};
  real_T(*ret)[6];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  ret = (real_T(*)[6])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *wayPoints
//                const char_T *identifier
// Return Type  : real_T (*)[6]
//
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *wayPoints,
                                 const char_T *identifier))[6]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[6];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = emlrt_marshallIn(sp, emlrtAlias(wayPoints), &thisId);
  emlrtDestroyArray(&wayPoints);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[6]
//
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[6]
{
  real_T(*y)[6];
  y = b_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const real_T u[60]
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const real_T u[60])
{
  static const int32_T iv[2]{0, 0};
  static const int32_T iv1[2]{3, 20};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const mxArray * const prhs[2]
//                int32_T nlhs
//                const mxArray *plhs[2]
// Return Type  : void
//
void cubic_trajectory_generator_pos_vel_api(const mxArray *const prhs[2],
                                            int32_T nlhs,
                                            const mxArray *plhs[2])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  real_T(*traj_pos)[60];
  real_T(*traj_vel)[60];
  real_T(*wayPointVels)[6];
  real_T(*wayPoints)[6];
  st.tls = emlrtRootTLSGlobal;
  traj_pos = (real_T(*)[60])mxMalloc(sizeof(real_T[60]));
  traj_vel = (real_T(*)[60])mxMalloc(sizeof(real_T[60]));
  // Marshall function inputs
  wayPoints = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "wayPoints");
  wayPointVels = emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "wayPointVels");
  // Invoke the target function
  cubic_trajectory_generator_pos_vel(*wayPoints, *wayPointVels, *traj_pos,
                                     *traj_vel);
  // Marshall function outputs
  plhs[0] = emlrt_marshallOut(*traj_pos);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(*traj_vel);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void cubic_trajectory_generator_pos_vel_atexit()
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
  cubic_trajectory_generator_pos_vel_xil_terminate();
  cubic_trajectory_generator_pos_vel_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void cubic_trajectory_generator_pos_vel_initialize()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void cubic_trajectory_generator_pos_vel_terminate()
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

//
// File trailer for _coder_cubic_trajectory_generator_pos_vel_api.cpp
//
// [EOF]
//
