//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_cubic_trajectory_generator_api.cpp
//
// Code generation for function '_coder_cubic_trajectory_generator_api'
//

// Include files
#include "_coder_cubic_trajectory_generator_api.h"
#include "cubic_trajectory_generator.h"
#include "cubic_trajectory_generator_data.h"
#include "rt_nonfinite.h"

// Function Declarations
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[6];

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *wayPoints,
                                 const char_T *identifier))[6];

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[6];

static const mxArray *emlrt_marshallOut(const real_T u[60]);

// Function Definitions
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

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[6]
{
  real_T(*y)[6];
  y = b_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

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

void cubic_trajectory_generator_api(const mxArray *const prhs[2],
                                    const mxArray **plhs)
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  real_T(*trajectory)[60];
  real_T(*wayPointVels)[6];
  real_T(*wayPoints)[6];
  st.tls = emlrtRootTLSGlobal;
  trajectory = (real_T(*)[60])mxMalloc(sizeof(real_T[60]));
  // Marshall function inputs
  wayPoints = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "wayPoints");
  wayPointVels = emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "wayPointVels");
  // Invoke the target function
  cubic_trajectory_generator(&st, *wayPoints, *wayPointVels, *trajectory);
  // Marshall function outputs
  *plhs = emlrt_marshallOut(*trajectory);
}

// End of code generation (_coder_cubic_trajectory_generator_api.cpp)
