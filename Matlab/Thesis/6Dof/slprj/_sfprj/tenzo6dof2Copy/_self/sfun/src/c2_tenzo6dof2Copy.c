/* Include files */

#include "blascompat32.h"
#include "tenzo6dof2Copy_sfun.h"
#include "c2_tenzo6dof2Copy.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "tenzo6dof2Copy_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c2_debug_family_names[5] = { "Cmis", "nargin", "nargout",
  "X", "ymis" };

/* Function Declarations */
static void initialize_c2_tenzo6dof2Copy(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance);
static void initialize_params_c2_tenzo6dof2Copy
  (SFc2_tenzo6dof2CopyInstanceStruct *chartInstance);
static void enable_c2_tenzo6dof2Copy(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance);
static void disable_c2_tenzo6dof2Copy(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance);
static void c2_update_debugger_state_c2_tenzo6dof2Copy
  (SFc2_tenzo6dof2CopyInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_tenzo6dof2Copy
  (SFc2_tenzo6dof2CopyInstanceStruct *chartInstance);
static void set_sim_state_c2_tenzo6dof2Copy(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance, const mxArray *c2_st);
static void finalize_c2_tenzo6dof2Copy(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance);
static void sf_c2_tenzo6dof2Copy(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance);
static void c2_chartstep_c2_tenzo6dof2Copy(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance);
static void initSimStructsc2_tenzo6dof2Copy(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static void c2_emlrt_marshallIn(SFc2_tenzo6dof2CopyInstanceStruct *chartInstance,
  const mxArray *c2_ymis, const char_T *c2_identifier, real_T c2_y[4]);
static void c2_b_emlrt_marshallIn(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4]);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_c_emlrt_marshallIn(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_eml_scalar_eg(SFc2_tenzo6dof2CopyInstanceStruct *chartInstance);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_d_emlrt_marshallIn(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_e_emlrt_marshallIn(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_tenzo6dof2Copy, const char_T *
  c2_identifier);
static uint8_T c2_f_emlrt_marshallIn(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void init_dsm_address_info(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_tenzo6dof2Copy(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_is_active_c2_tenzo6dof2Copy = 0U;
}

static void initialize_params_c2_tenzo6dof2Copy
  (SFc2_tenzo6dof2CopyInstanceStruct *chartInstance)
{
}

static void enable_c2_tenzo6dof2Copy(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_tenzo6dof2Copy(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_tenzo6dof2Copy
  (SFc2_tenzo6dof2CopyInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c2_tenzo6dof2Copy
  (SFc2_tenzo6dof2CopyInstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  int32_T c2_i0;
  real_T c2_u[4];
  const mxArray *c2_b_y = NULL;
  uint8_T c2_hoistedGlobal;
  uint8_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  real_T (*c2_ymis)[4];
  c2_ymis = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(2), FALSE);
  for (c2_i0 = 0; c2_i0 < 4; c2_i0++) {
    c2_u[c2_i0] = (*c2_ymis)[c2_i0];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_hoistedGlobal = chartInstance->c2_is_active_c2_tenzo6dof2Copy;
  c2_b_u = c2_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  sf_mex_assign(&c2_st, c2_y, FALSE);
  return c2_st;
}

static void set_sim_state_c2_tenzo6dof2Copy(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[4];
  int32_T c2_i1;
  real_T (*c2_ymis)[4];
  c2_ymis = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)), "ymis",
                      c2_dv0);
  for (c2_i1 = 0; c2_i1 < 4; c2_i1++) {
    (*c2_ymis)[c2_i1] = c2_dv0[c2_i1];
  }

  chartInstance->c2_is_active_c2_tenzo6dof2Copy = c2_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 1)),
     "is_active_c2_tenzo6dof2Copy");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_tenzo6dof2Copy(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_tenzo6dof2Copy(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance)
{
}

static void sf_c2_tenzo6dof2Copy(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance)
{
  int32_T c2_i2;
  int32_T c2_i3;
  real_T (*c2_ymis)[4];
  real_T (*c2_X)[12];
  c2_ymis = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_X = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  for (c2_i2 = 0; c2_i2 < 12; c2_i2++) {
    _SFD_DATA_RANGE_CHECK((*c2_X)[c2_i2], 0U);
  }

  for (c2_i3 = 0; c2_i3 < 4; c2_i3++) {
    _SFD_DATA_RANGE_CHECK((*c2_ymis)[c2_i3], 1U);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_tenzo6dof2Copy(chartInstance);
  sf_debug_check_for_state_inconsistency(_tenzo6dof2CopyMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c2_chartstep_c2_tenzo6dof2Copy(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance)
{
  int32_T c2_i4;
  real_T c2_X[12];
  uint32_T c2_debug_family_var_map[5];
  real_T c2_Cmis[48];
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  real_T c2_ymis[4];
  int32_T c2_i5;
  static real_T c2_a[48] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T c2_i6;
  real_T c2_b[12];
  int32_T c2_i7;
  int32_T c2_i8;
  int32_T c2_i9;
  real_T c2_C[4];
  int32_T c2_i10;
  int32_T c2_i11;
  int32_T c2_i12;
  int32_T c2_i13;
  int32_T c2_i14;
  int32_T c2_i15;
  int32_T c2_i16;
  real_T (*c2_b_ymis)[4];
  real_T (*c2_b_X)[12];
  c2_b_ymis = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_b_X = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  for (c2_i4 = 0; c2_i4 < 12; c2_i4++) {
    c2_X[c2_i4] = (*c2_b_X)[c2_i4];
  }

  sf_debug_symbol_scope_push_eml(0U, 5U, 5U, c2_debug_family_names,
    c2_debug_family_var_map);
  sf_debug_symbol_scope_add_eml(c2_Cmis, 0U, c2_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargin, 1U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargout, 2U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c2_X, 3U, c2_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c2_ymis, 4U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 4);
  for (c2_i5 = 0; c2_i5 < 48; c2_i5++) {
    c2_Cmis[c2_i5] = c2_a[c2_i5];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 8);
  for (c2_i6 = 0; c2_i6 < 12; c2_i6++) {
    c2_b[c2_i6] = c2_X[c2_i6];
  }

  c2_eml_scalar_eg(chartInstance);
  c2_eml_scalar_eg(chartInstance);
  for (c2_i7 = 0; c2_i7 < 4; c2_i7++) {
    c2_ymis[c2_i7] = 0.0;
  }

  for (c2_i8 = 0; c2_i8 < 4; c2_i8++) {
    c2_ymis[c2_i8] = 0.0;
  }

  for (c2_i9 = 0; c2_i9 < 4; c2_i9++) {
    c2_C[c2_i9] = c2_ymis[c2_i9];
  }

  for (c2_i10 = 0; c2_i10 < 4; c2_i10++) {
    c2_ymis[c2_i10] = c2_C[c2_i10];
  }

  for (c2_i11 = 0; c2_i11 < 4; c2_i11++) {
    c2_C[c2_i11] = c2_ymis[c2_i11];
  }

  for (c2_i12 = 0; c2_i12 < 4; c2_i12++) {
    c2_ymis[c2_i12] = c2_C[c2_i12];
  }

  for (c2_i13 = 0; c2_i13 < 4; c2_i13++) {
    c2_ymis[c2_i13] = 0.0;
    c2_i14 = 0;
    for (c2_i15 = 0; c2_i15 < 12; c2_i15++) {
      c2_ymis[c2_i13] += c2_a[c2_i14 + c2_i13] * c2_b[c2_i15];
      c2_i14 += 4;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -8);
  sf_debug_symbol_scope_pop();
  for (c2_i16 = 0; c2_i16 < 4; c2_i16++) {
    (*c2_b_ymis)[c2_i16] = c2_ymis[c2_i16];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_tenzo6dof2Copy(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance)
{
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i17;
  real_T c2_b_inData[4];
  int32_T c2_i18;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_tenzo6dof2CopyInstanceStruct *chartInstance;
  chartInstance = (SFc2_tenzo6dof2CopyInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i17 = 0; c2_i17 < 4; c2_i17++) {
    c2_b_inData[c2_i17] = (*(real_T (*)[4])c2_inData)[c2_i17];
  }

  for (c2_i18 = 0; c2_i18 < 4; c2_i18++) {
    c2_u[c2_i18] = c2_b_inData[c2_i18];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_emlrt_marshallIn(SFc2_tenzo6dof2CopyInstanceStruct *chartInstance,
  const mxArray *c2_ymis, const char_T *c2_identifier, real_T c2_y[4])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_ymis), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_ymis);
}

static void c2_b_emlrt_marshallIn(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4])
{
  real_T c2_dv1[4];
  int32_T c2_i19;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv1, 1, 0, 0U, 1, 0U, 1, 4);
  for (c2_i19 = 0; c2_i19 < 4; c2_i19++) {
    c2_y[c2_i19] = c2_dv1[c2_i19];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_ymis;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i20;
  SFc2_tenzo6dof2CopyInstanceStruct *chartInstance;
  chartInstance = (SFc2_tenzo6dof2CopyInstanceStruct *)chartInstanceVoid;
  c2_ymis = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_ymis), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_ymis);
  for (c2_i20 = 0; c2_i20 < 4; c2_i20++) {
    (*(real_T (*)[4])c2_outData)[c2_i20] = c2_y[c2_i20];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i21;
  real_T c2_b_inData[12];
  int32_T c2_i22;
  real_T c2_u[12];
  const mxArray *c2_y = NULL;
  SFc2_tenzo6dof2CopyInstanceStruct *chartInstance;
  chartInstance = (SFc2_tenzo6dof2CopyInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i21 = 0; c2_i21 < 12; c2_i21++) {
    c2_b_inData[c2_i21] = (*(real_T (*)[12])c2_inData)[c2_i21];
  }

  for (c2_i22 = 0; c2_i22 < 12; c2_i22++) {
    c2_u[c2_i22] = c2_b_inData[c2_i22];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 12), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_tenzo6dof2CopyInstanceStruct *chartInstance;
  chartInstance = (SFc2_tenzo6dof2CopyInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_c_emlrt_marshallIn(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_nargout;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_tenzo6dof2CopyInstanceStruct *chartInstance;
  chartInstance = (SFc2_tenzo6dof2CopyInstanceStruct *)chartInstanceVoid;
  c2_nargout = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_nargout), &c2_thisId);
  sf_mex_destroy(&c2_nargout);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i23;
  int32_T c2_i24;
  int32_T c2_i25;
  real_T c2_b_inData[48];
  int32_T c2_i26;
  int32_T c2_i27;
  int32_T c2_i28;
  real_T c2_u[48];
  const mxArray *c2_y = NULL;
  SFc2_tenzo6dof2CopyInstanceStruct *chartInstance;
  chartInstance = (SFc2_tenzo6dof2CopyInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i23 = 0;
  for (c2_i24 = 0; c2_i24 < 12; c2_i24++) {
    for (c2_i25 = 0; c2_i25 < 4; c2_i25++) {
      c2_b_inData[c2_i25 + c2_i23] = (*(real_T (*)[48])c2_inData)[c2_i25 +
        c2_i23];
    }

    c2_i23 += 4;
  }

  c2_i26 = 0;
  for (c2_i27 = 0; c2_i27 < 12; c2_i27++) {
    for (c2_i28 = 0; c2_i28 < 4; c2_i28++) {
      c2_u[c2_i28 + c2_i26] = c2_b_inData[c2_i28 + c2_i26];
    }

    c2_i26 += 4;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 4, 12), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

const mxArray *sf_c2_tenzo6dof2Copy_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo;
  c2_ResolvedFunctionInfo c2_info[8];
  c2_ResolvedFunctionInfo (*c2_b_info)[8];
  const mxArray *c2_m0 = NULL;
  int32_T c2_i29;
  c2_ResolvedFunctionInfo *c2_r0;
  c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  c2_b_info = (c2_ResolvedFunctionInfo (*)[8])c2_info;
  (*c2_b_info)[0].context = "";
  (*c2_b_info)[0].name = "mtimes";
  (*c2_b_info)[0].dominantType = "double";
  (*c2_b_info)[0].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c2_b_info)[0].fileTimeLo = 1289519692U;
  (*c2_b_info)[0].fileTimeHi = 0U;
  (*c2_b_info)[0].mFileTimeLo = 0U;
  (*c2_b_info)[0].mFileTimeHi = 0U;
  (*c2_b_info)[1].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c2_b_info)[1].name = "eml_index_class";
  (*c2_b_info)[1].dominantType = "";
  (*c2_b_info)[1].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  (*c2_b_info)[1].fileTimeLo = 1286818778U;
  (*c2_b_info)[1].fileTimeHi = 0U;
  (*c2_b_info)[1].mFileTimeLo = 0U;
  (*c2_b_info)[1].mFileTimeHi = 0U;
  (*c2_b_info)[2].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c2_b_info)[2].name = "eml_scalar_eg";
  (*c2_b_info)[2].dominantType = "double";
  (*c2_b_info)[2].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  (*c2_b_info)[2].fileTimeLo = 1286818796U;
  (*c2_b_info)[2].fileTimeHi = 0U;
  (*c2_b_info)[2].mFileTimeLo = 0U;
  (*c2_b_info)[2].mFileTimeHi = 0U;
  (*c2_b_info)[3].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c2_b_info)[3].name = "eml_xgemm";
  (*c2_b_info)[3].dominantType = "int32";
  (*c2_b_info)[3].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  (*c2_b_info)[3].fileTimeLo = 1299076772U;
  (*c2_b_info)[3].fileTimeHi = 0U;
  (*c2_b_info)[3].mFileTimeLo = 0U;
  (*c2_b_info)[3].mFileTimeHi = 0U;
  (*c2_b_info)[4].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  (*c2_b_info)[4].name = "eml_blas_inline";
  (*c2_b_info)[4].dominantType = "";
  (*c2_b_info)[4].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  (*c2_b_info)[4].fileTimeLo = 1299076768U;
  (*c2_b_info)[4].fileTimeHi = 0U;
  (*c2_b_info)[4].mFileTimeLo = 0U;
  (*c2_b_info)[4].mFileTimeHi = 0U;
  (*c2_b_info)[5].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  (*c2_b_info)[5].name = "mtimes";
  (*c2_b_info)[5].dominantType = "double";
  (*c2_b_info)[5].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c2_b_info)[5].fileTimeLo = 1289519692U;
  (*c2_b_info)[5].fileTimeHi = 0U;
  (*c2_b_info)[5].mFileTimeLo = 0U;
  (*c2_b_info)[5].mFileTimeHi = 0U;
  (*c2_b_info)[6].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  (*c2_b_info)[6].name = "eml_scalar_eg";
  (*c2_b_info)[6].dominantType = "double";
  (*c2_b_info)[6].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  (*c2_b_info)[6].fileTimeLo = 1286818796U;
  (*c2_b_info)[6].fileTimeHi = 0U;
  (*c2_b_info)[6].mFileTimeLo = 0U;
  (*c2_b_info)[6].mFileTimeHi = 0U;
  (*c2_b_info)[7].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  (*c2_b_info)[7].name = "eml_refblas_xgemm";
  (*c2_b_info)[7].dominantType = "int32";
  (*c2_b_info)[7].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  (*c2_b_info)[7].fileTimeLo = 1299076774U;
  (*c2_b_info)[7].fileTimeHi = 0U;
  (*c2_b_info)[7].mFileTimeLo = 0U;
  (*c2_b_info)[7].mFileTimeHi = 0U;
  sf_mex_assign(&c2_m0, sf_mex_createstruct("nameCaptureInfo", 1, 8), FALSE);
  for (c2_i29 = 0; c2_i29 < 8; c2_i29++) {
    c2_r0 = &c2_info[c2_i29];
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->context)), "context", "nameCaptureInfo",
                    c2_i29);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r0->name)), "name", "nameCaptureInfo", c2_i29);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c2_i29);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->resolved)), "resolved", "nameCaptureInfo",
                    c2_i29);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c2_i29);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c2_i29);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c2_i29);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c2_i29);
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_eml_scalar_eg(SFc2_tenzo6dof2CopyInstanceStruct *chartInstance)
{
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_tenzo6dof2CopyInstanceStruct *chartInstance;
  chartInstance = (SFc2_tenzo6dof2CopyInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static int32_T c2_d_emlrt_marshallIn(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i30;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i30, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i30;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_tenzo6dof2CopyInstanceStruct *chartInstance;
  chartInstance = (SFc2_tenzo6dof2CopyInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_e_emlrt_marshallIn(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_tenzo6dof2Copy, const char_T *
  c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_tenzo6dof2Copy), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_tenzo6dof2Copy);
  return c2_y;
}

static uint8_T c2_f_emlrt_marshallIn(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void init_dsm_address_info(SFc2_tenzo6dof2CopyInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c2_tenzo6dof2Copy_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1366388365U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1219449510U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3008131929U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(568844505U);
}

mxArray *sf_c2_tenzo6dof2Copy_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("r7rvY7a8r5GIrRf6Vq6qpD");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(12);
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
      pr[0] = (double)(4);
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

  return(mxAutoinheritanceInfo);
}

static const mxArray *sf_get_sim_state_info_c2_tenzo6dof2Copy(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"ymis\",},{M[8],M[0],T\"is_active_c2_tenzo6dof2Copy\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_tenzo6dof2Copy_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_tenzo6dof2CopyInstanceStruct *chartInstance;
    chartInstance = (SFc2_tenzo6dof2CopyInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_tenzo6dof2CopyMachineNumber_,
           2,
           1,
           1,
           2,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_tenzo6dof2CopyMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (_tenzo6dof2CopyMachineNumber_,chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(_tenzo6dof2CopyMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"X");
          _SFD_SET_DATA_PROPS(1,2,0,1,"ymis");
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
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,182);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }

        {
          real_T (*c2_X)[12];
          real_T (*c2_ymis)[4];
          c2_ymis = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
          c2_X = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c2_X);
          _SFD_SET_DATA_VALUE_PTR(1U, *c2_ymis);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(_tenzo6dof2CopyMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization()
{
  return "A8u29sI3YjQIzTWl6PD6iC";
}

static void sf_opaque_initialize_c2_tenzo6dof2Copy(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_tenzo6dof2CopyInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c2_tenzo6dof2Copy((SFc2_tenzo6dof2CopyInstanceStruct*)
    chartInstanceVar);
  initialize_c2_tenzo6dof2Copy((SFc2_tenzo6dof2CopyInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c2_tenzo6dof2Copy(void *chartInstanceVar)
{
  enable_c2_tenzo6dof2Copy((SFc2_tenzo6dof2CopyInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_tenzo6dof2Copy(void *chartInstanceVar)
{
  disable_c2_tenzo6dof2Copy((SFc2_tenzo6dof2CopyInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c2_tenzo6dof2Copy(void *chartInstanceVar)
{
  sf_c2_tenzo6dof2Copy((SFc2_tenzo6dof2CopyInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_tenzo6dof2Copy(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_tenzo6dof2Copy
    ((SFc2_tenzo6dof2CopyInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_tenzo6dof2Copy();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c2_tenzo6dof2Copy(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_tenzo6dof2Copy();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_tenzo6dof2Copy((SFc2_tenzo6dof2CopyInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_tenzo6dof2Copy(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_tenzo6dof2Copy(S);
}

static void sf_opaque_set_sim_state_c2_tenzo6dof2Copy(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c2_tenzo6dof2Copy(S, st);
}

static void sf_opaque_terminate_c2_tenzo6dof2Copy(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_tenzo6dof2CopyInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c2_tenzo6dof2Copy((SFc2_tenzo6dof2CopyInstanceStruct*)
      chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_tenzo6dof2Copy_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_tenzo6dof2Copy((SFc2_tenzo6dof2CopyInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_tenzo6dof2Copy(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_tenzo6dof2Copy((SFc2_tenzo6dof2CopyInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_tenzo6dof2Copy(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_tenzo6dof2Copy_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,2,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,1);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,1);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1874684280U));
  ssSetChecksum1(S,(2096360006U));
  ssSetChecksum2(S,(3731038U));
  ssSetChecksum3(S,(1450798895U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c2_tenzo6dof2Copy(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_tenzo6dof2Copy(SimStruct *S)
{
  SFc2_tenzo6dof2CopyInstanceStruct *chartInstance;
  chartInstance = (SFc2_tenzo6dof2CopyInstanceStruct *)malloc(sizeof
    (SFc2_tenzo6dof2CopyInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_tenzo6dof2CopyInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_tenzo6dof2Copy;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_tenzo6dof2Copy;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_tenzo6dof2Copy;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_tenzo6dof2Copy;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_tenzo6dof2Copy;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_tenzo6dof2Copy;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_tenzo6dof2Copy;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_tenzo6dof2Copy;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_tenzo6dof2Copy;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_tenzo6dof2Copy;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_tenzo6dof2Copy;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo)));/* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c2_tenzo6dof2Copy_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_tenzo6dof2Copy(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_tenzo6dof2Copy(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_tenzo6dof2Copy(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_tenzo6dof2Copy_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
