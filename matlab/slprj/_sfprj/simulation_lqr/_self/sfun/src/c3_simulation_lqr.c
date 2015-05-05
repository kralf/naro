/* Include files */

#include <stddef.h>
#include "blas.h"
#include "simulation_lqr_sfun.h"
#include "c3_simulation_lqr.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "simulation_lqr_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c3_debug_family_names[5] = { "nargin", "nargout", "h",
  "h_old", "y" };

/* Function Declarations */
static void initialize_c3_simulation_lqr(SFc3_simulation_lqrInstanceStruct
  *chartInstance);
static void initialize_params_c3_simulation_lqr
  (SFc3_simulation_lqrInstanceStruct *chartInstance);
static void enable_c3_simulation_lqr(SFc3_simulation_lqrInstanceStruct
  *chartInstance);
static void disable_c3_simulation_lqr(SFc3_simulation_lqrInstanceStruct
  *chartInstance);
static void c3_update_debugger_state_c3_simulation_lqr
  (SFc3_simulation_lqrInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c3_simulation_lqr
  (SFc3_simulation_lqrInstanceStruct *chartInstance);
static void set_sim_state_c3_simulation_lqr(SFc3_simulation_lqrInstanceStruct
  *chartInstance, const mxArray *c3_st);
static void finalize_c3_simulation_lqr(SFc3_simulation_lqrInstanceStruct
  *chartInstance);
static void sf_gateway_c3_simulation_lqr(SFc3_simulation_lqrInstanceStruct
  *chartInstance);
static void mdl_start_c3_simulation_lqr(SFc3_simulation_lqrInstanceStruct
  *chartInstance);
static void initSimStructsc3_simulation_lqr(SFc3_simulation_lqrInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber, uint32_T c3_instanceNumber);
static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData);
static real_T c3_emlrt_marshallIn(SFc3_simulation_lqrInstanceStruct
  *chartInstance, const mxArray *c3_b_y, const char_T *c3_identifier);
static real_T c3_b_emlrt_marshallIn(SFc3_simulation_lqrInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static int32_T c3_c_emlrt_marshallIn(SFc3_simulation_lqrInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static uint8_T c3_d_emlrt_marshallIn(SFc3_simulation_lqrInstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_simulation_lqr, const char_T *
  c3_identifier);
static uint8_T c3_e_emlrt_marshallIn(SFc3_simulation_lqrInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void init_dsm_address_info(SFc3_simulation_lqrInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc3_simulation_lqrInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c3_simulation_lqr(SFc3_simulation_lqrInstanceStruct
  *chartInstance)
{
  chartInstance->c3_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c3_is_active_c3_simulation_lqr = 0U;
}

static void initialize_params_c3_simulation_lqr
  (SFc3_simulation_lqrInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c3_simulation_lqr(SFc3_simulation_lqrInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c3_simulation_lqr(SFc3_simulation_lqrInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c3_update_debugger_state_c3_simulation_lqr
  (SFc3_simulation_lqrInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c3_simulation_lqr
  (SFc3_simulation_lqrInstanceStruct *chartInstance)
{
  const mxArray *c3_st;
  const mxArray *c3_b_y = NULL;
  real_T c3_hoistedGlobal;
  real_T c3_u;
  const mxArray *c3_c_y = NULL;
  uint8_T c3_b_hoistedGlobal;
  uint8_T c3_b_u;
  const mxArray *c3_d_y = NULL;
  c3_st = NULL;
  c3_st = NULL;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_createcellmatrix(2, 1), false);
  c3_hoistedGlobal = *chartInstance->c3_y;
  c3_u = c3_hoistedGlobal;
  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_b_y, 0, c3_c_y);
  c3_b_hoistedGlobal = chartInstance->c3_is_active_c3_simulation_lqr;
  c3_b_u = c3_b_hoistedGlobal;
  c3_d_y = NULL;
  sf_mex_assign(&c3_d_y, sf_mex_create("y", &c3_b_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_b_y, 1, c3_d_y);
  sf_mex_assign(&c3_st, c3_b_y, false);
  return c3_st;
}

static void set_sim_state_c3_simulation_lqr(SFc3_simulation_lqrInstanceStruct
  *chartInstance, const mxArray *c3_st)
{
  const mxArray *c3_u;
  chartInstance->c3_doneDoubleBufferReInit = true;
  c3_u = sf_mex_dup(c3_st);
  *chartInstance->c3_y = c3_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 0)), "y");
  chartInstance->c3_is_active_c3_simulation_lqr = c3_d_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 1)),
     "is_active_c3_simulation_lqr");
  sf_mex_destroy(&c3_u);
  c3_update_debugger_state_c3_simulation_lqr(chartInstance);
  sf_mex_destroy(&c3_st);
}

static void finalize_c3_simulation_lqr(SFc3_simulation_lqrInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c3_simulation_lqr(SFc3_simulation_lqrInstanceStruct
  *chartInstance)
{
  real_T c3_hoistedGlobal;
  real_T c3_b_hoistedGlobal;
  real_T c3_b_h;
  real_T c3_b_h_old;
  uint32_T c3_debug_family_var_map[5];
  real_T c3_nargin = 2.0;
  real_T c3_nargout = 1.0;
  real_T c3_b_y;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_h, 0U);
  chartInstance->c3_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  c3_hoistedGlobal = *chartInstance->c3_h;
  c3_b_hoistedGlobal = *chartInstance->c3_h_old;
  c3_b_h = c3_hoistedGlobal;
  c3_b_h_old = c3_b_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c3_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 0U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 1U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_b_h, 2U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_b_h_old, 3U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_y, 4U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 4);
  if (CV_EML_IF(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 0, c3_b_h_old, c3_b_h, -1,
        4U, c3_b_h_old > c3_b_h))) {
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 5);
    c3_b_y = c3_b_h_old + 0.1;
  } else {
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 6);
    if (CV_EML_IF(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 1, c3_b_h_old, c3_b_h, -1,
          2U, c3_b_h_old < c3_b_h))) {
      _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 7);
      c3_b_y = c3_b_h_old - 0.1;
    } else {
      _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 9);
      c3_b_y = c3_b_h;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, -9);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c3_y = c3_b_y;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_simulation_lqrMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_y, 1U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_h_old, 2U);
}

static void mdl_start_c3_simulation_lqr(SFc3_simulation_lqrInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void initSimStructsc3_simulation_lqr(SFc3_simulation_lqrInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber, uint32_T c3_instanceNumber)
{
  (void)c3_machineNumber;
  (void)c3_chartNumber;
  (void)c3_instanceNumber;
}

static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_b_y = NULL;
  SFc3_simulation_lqrInstanceStruct *chartInstance;
  chartInstance = (SFc3_simulation_lqrInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_b_y, false);
  return c3_mxArrayOutData;
}

static real_T c3_emlrt_marshallIn(SFc3_simulation_lqrInstanceStruct
  *chartInstance, const mxArray *c3_b_y, const char_T *c3_identifier)
{
  real_T c3_c_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_c_y = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_y), &c3_thisId);
  sf_mex_destroy(&c3_b_y);
  return c3_c_y;
}

static real_T c3_b_emlrt_marshallIn(SFc3_simulation_lqrInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_b_y;
  real_T c3_d0;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d0, 1, 0, 0U, 0, 0U, 0);
  c3_b_y = c3_d0;
  sf_mex_destroy(&c3_u);
  return c3_b_y;
}

static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_y;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_c_y;
  SFc3_simulation_lqrInstanceStruct *chartInstance;
  chartInstance = (SFc3_simulation_lqrInstanceStruct *)chartInstanceVoid;
  c3_b_y = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_c_y = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_y), &c3_thisId);
  sf_mex_destroy(&c3_b_y);
  *(real_T *)c3_outData = c3_c_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

const mxArray *sf_c3_simulation_lqr_get_eml_resolved_functions_info(void)
{
  const mxArray *c3_nameCaptureInfo = NULL;
  c3_nameCaptureInfo = NULL;
  sf_mex_assign(&c3_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c3_nameCaptureInfo;
}

static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_u;
  const mxArray *c3_b_y = NULL;
  SFc3_simulation_lqrInstanceStruct *chartInstance;
  chartInstance = (SFc3_simulation_lqrInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(int32_T *)c3_inData;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_b_y, false);
  return c3_mxArrayOutData;
}

static int32_T c3_c_emlrt_marshallIn(SFc3_simulation_lqrInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  int32_T c3_b_y;
  int32_T c3_i0;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_i0, 1, 6, 0U, 0, 0U, 0);
  c3_b_y = c3_i0;
  sf_mex_destroy(&c3_u);
  return c3_b_y;
}

static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_sfEvent;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  int32_T c3_b_y;
  SFc3_simulation_lqrInstanceStruct *chartInstance;
  chartInstance = (SFc3_simulation_lqrInstanceStruct *)chartInstanceVoid;
  c3_b_sfEvent = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_b_y = c3_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_sfEvent),
    &c3_thisId);
  sf_mex_destroy(&c3_b_sfEvent);
  *(int32_T *)c3_outData = c3_b_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static uint8_T c3_d_emlrt_marshallIn(SFc3_simulation_lqrInstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_simulation_lqr, const char_T *
  c3_identifier)
{
  uint8_T c3_b_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_b_y = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_b_is_active_c3_simulation_lqr), &c3_thisId);
  sf_mex_destroy(&c3_b_is_active_c3_simulation_lqr);
  return c3_b_y;
}

static uint8_T c3_e_emlrt_marshallIn(SFc3_simulation_lqrInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  uint8_T c3_b_y;
  uint8_T c3_u0;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_u0, 1, 3, 0U, 0, 0U, 0);
  c3_b_y = c3_u0;
  sf_mex_destroy(&c3_u);
  return c3_b_y;
}

static void init_dsm_address_info(SFc3_simulation_lqrInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc3_simulation_lqrInstanceStruct
  *chartInstance)
{
  chartInstance->c3_h = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    0);
  chartInstance->c3_y = (real_T *)ssGetOutputPortSignal_wrapper(chartInstance->S,
    1);
  chartInstance->c3_h_old = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c3_simulation_lqr_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4055788221U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2745244213U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1770778195U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(992261866U);
}

mxArray* sf_c3_simulation_lqr_get_post_codegen_info(void);
mxArray *sf_c3_simulation_lqr_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("QEbgBLxyf0AeChfOgmjQxE");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c3_simulation_lqr_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c3_simulation_lqr_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c3_simulation_lqr_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "incompatibleSymbol", };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 3, infoFields);
  mxArray *fallbackReason = mxCreateString("feature_off");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxArray *fallbackType = mxCreateString("early");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c3_simulation_lqr_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c3_simulation_lqr_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c3_simulation_lqr(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"y\",},{M[8],M[0],T\"is_active_c3_simulation_lqr\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c3_simulation_lqr_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc3_simulation_lqrInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc3_simulation_lqrInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _simulation_lqrMachineNumber_,
           3,
           1,
           1,
           0,
           3,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_simulation_lqrMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_simulation_lqrMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _simulation_lqrMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"h");
          _SFD_SET_DATA_PROPS(1,2,0,1,"y");
          _SFD_SET_DATA_PROPS(2,1,1,0,"h_old");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,2,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,128);
        _SFD_CV_INIT_EML_IF(0,1,0,38,51,72,126);
        _SFD_CV_INIT_EML_IF(0,1,1,72,87,107,126);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,41,50,-1,4);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,1,79,86,-1,2);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)c3_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c3_h);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c3_y);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c3_h_old);
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _simulation_lqrMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "YmD5Stvpk9961bb7rIHwtD";
}

static void sf_opaque_initialize_c3_simulation_lqr(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc3_simulation_lqrInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c3_simulation_lqr((SFc3_simulation_lqrInstanceStruct*)
    chartInstanceVar);
  initialize_c3_simulation_lqr((SFc3_simulation_lqrInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c3_simulation_lqr(void *chartInstanceVar)
{
  enable_c3_simulation_lqr((SFc3_simulation_lqrInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c3_simulation_lqr(void *chartInstanceVar)
{
  disable_c3_simulation_lqr((SFc3_simulation_lqrInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c3_simulation_lqr(void *chartInstanceVar)
{
  sf_gateway_c3_simulation_lqr((SFc3_simulation_lqrInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c3_simulation_lqr(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c3_simulation_lqr((SFc3_simulation_lqrInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c3_simulation_lqr(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c3_simulation_lqr((SFc3_simulation_lqrInstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c3_simulation_lqr(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc3_simulation_lqrInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_simulation_lqr_optimization_info();
    }

    finalize_c3_simulation_lqr((SFc3_simulation_lqrInstanceStruct*)
      chartInstanceVar);
    utFree(chartInstanceVar);
    if (crtInfo != NULL) {
      utFree(crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc3_simulation_lqr((SFc3_simulation_lqrInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c3_simulation_lqr(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c3_simulation_lqr((SFc3_simulation_lqrInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c3_simulation_lqr(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_simulation_lqr_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,3);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,3,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,3,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,3);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,3,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,3,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 2; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,3);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1799194307U));
  ssSetChecksum1(S,(3100460631U));
  ssSetChecksum2(S,(1640439921U));
  ssSetChecksum3(S,(2728340757U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c3_simulation_lqr(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c3_simulation_lqr(SimStruct *S)
{
  SFc3_simulation_lqrInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc3_simulation_lqrInstanceStruct *)utMalloc(sizeof
    (SFc3_simulation_lqrInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc3_simulation_lqrInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c3_simulation_lqr;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c3_simulation_lqr;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c3_simulation_lqr;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c3_simulation_lqr;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c3_simulation_lqr;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c3_simulation_lqr;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c3_simulation_lqr;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c3_simulation_lqr;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c3_simulation_lqr;
  chartInstance->chartInfo.mdlStart = mdlStart_c3_simulation_lqr;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c3_simulation_lqr;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->checksum = SF_RUNTIME_INFO_CHECKSUM;
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  crtInfo->compiledInfo = NULL;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c3_simulation_lqr_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c3_simulation_lqr(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c3_simulation_lqr(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c3_simulation_lqr(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c3_simulation_lqr_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
