/* Include files */

#include "blascompat32.h"
#include "TenzoDecV3DeltaP_sfun.h"
#include "c1_TenzoDecV3DeltaP.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "TenzoDecV3DeltaP_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c1_debug_family_names[38] = { "g", "rho", "mq", "lx", "ly",
  "lz", "dcg", "If", "cp", "ct", "rp", "Km", "Kt", "Ktm", "k1", "k2", "k3", "k4",
  "tc", "dz1", "dz2", "dz3", "dz4", "PWM_max", "w0c", "Ft0", "WTF", "T",
  "nargin", "nargout", "taux", "tauy", "tauz", "Thrust", "w1", "w2", "w3", "w4"
};

/* Function Declarations */
static void initialize_c1_TenzoDecV3DeltaP(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance);
static void initialize_params_c1_TenzoDecV3DeltaP
  (SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance);
static void enable_c1_TenzoDecV3DeltaP(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance);
static void disable_c1_TenzoDecV3DeltaP(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance);
static void c1_update_debugger_state_c1_TenzoDecV3DeltaP
  (SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c1_TenzoDecV3DeltaP
  (SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance);
static void set_sim_state_c1_TenzoDecV3DeltaP
  (SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance, const mxArray *c1_st);
static void finalize_c1_TenzoDecV3DeltaP(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance);
static void sf_c1_TenzoDecV3DeltaP(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance);
static void c1_chartstep_c1_TenzoDecV3DeltaP(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance);
static void initSimStructsc1_TenzoDecV3DeltaP
  (SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static real_T c1_emlrt_marshallIn(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c1_w4, const char_T *c1_identifier);
static real_T c1_b_emlrt_marshallIn(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_c_emlrt_marshallIn(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[4]);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_info_helper(c1_ResolvedFunctionInfo c1_info[109]);
static void c1_b_info_helper(c1_ResolvedFunctionInfo c1_info[109]);
static void c1_eml_error(SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance);
static void c1_eml_lusolve(SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance,
  real_T c1_B[4], real_T c1_X[4]);
static void c1_eml_int_forloop_overflow_check
  (SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance);
static void c1_b_eml_int_forloop_overflow_check
  (SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance, int32_T c1_a, int32_T
   c1_b);
static void c1_eml_xtrsm(SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance,
  real_T c1_A[16], real_T c1_B[4], real_T c1_b_B[4]);
static void c1_below_threshold(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance);
static void c1_eml_scalar_eg(SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance);
static void c1_c_eml_int_forloop_overflow_check
  (SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance);
static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_d_emlrt_marshallIn(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static uint8_T c1_e_emlrt_marshallIn(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_TenzoDecV3DeltaP, const
  char_T *c1_identifier);
static uint8_T c1_f_emlrt_marshallIn(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_eml_xtrsm(SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance,
  real_T c1_A[16], real_T c1_B[4]);
static void init_dsm_address_info(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c1_TenzoDecV3DeltaP(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance)
{
  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c1_is_active_c1_TenzoDecV3DeltaP = 0U;
}

static void initialize_params_c1_TenzoDecV3DeltaP
  (SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance)
{
}

static void enable_c1_TenzoDecV3DeltaP(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c1_TenzoDecV3DeltaP(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c1_update_debugger_state_c1_TenzoDecV3DeltaP
  (SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c1_TenzoDecV3DeltaP
  (SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_y = NULL;
  real_T c1_hoistedGlobal;
  real_T c1_u;
  const mxArray *c1_b_y = NULL;
  real_T c1_b_hoistedGlobal;
  real_T c1_b_u;
  const mxArray *c1_c_y = NULL;
  real_T c1_c_hoistedGlobal;
  real_T c1_c_u;
  const mxArray *c1_d_y = NULL;
  real_T c1_d_hoistedGlobal;
  real_T c1_d_u;
  const mxArray *c1_e_y = NULL;
  uint8_T c1_e_hoistedGlobal;
  uint8_T c1_e_u;
  const mxArray *c1_f_y = NULL;
  real_T *c1_w1;
  real_T *c1_w2;
  real_T *c1_w3;
  real_T *c1_w4;
  c1_w4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c1_w3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c1_w2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_w1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_st = NULL;
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellarray(5), FALSE);
  c1_hoistedGlobal = *c1_w1;
  c1_u = c1_hoistedGlobal;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 0, c1_b_y);
  c1_b_hoistedGlobal = *c1_w2;
  c1_b_u = c1_b_hoistedGlobal;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 1, c1_c_y);
  c1_c_hoistedGlobal = *c1_w3;
  c1_c_u = c1_c_hoistedGlobal;
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", &c1_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 2, c1_d_y);
  c1_d_hoistedGlobal = *c1_w4;
  c1_d_u = c1_d_hoistedGlobal;
  c1_e_y = NULL;
  sf_mex_assign(&c1_e_y, sf_mex_create("y", &c1_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 3, c1_e_y);
  c1_e_hoistedGlobal = chartInstance->c1_is_active_c1_TenzoDecV3DeltaP;
  c1_e_u = c1_e_hoistedGlobal;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", &c1_e_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 4, c1_f_y);
  sf_mex_assign(&c1_st, c1_y, FALSE);
  return c1_st;
}

static void set_sim_state_c1_TenzoDecV3DeltaP
  (SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance, const mxArray *c1_st)
{
  const mxArray *c1_u;
  real_T *c1_w1;
  real_T *c1_w2;
  real_T *c1_w3;
  real_T *c1_w4;
  c1_w4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c1_w3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c1_w2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_w1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c1_doneDoubleBufferReInit = TRUE;
  c1_u = sf_mex_dup(c1_st);
  *c1_w1 = c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 0)),
    "w1");
  *c1_w2 = c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 1)),
    "w2");
  *c1_w3 = c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 2)),
    "w3");
  *c1_w4 = c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 3)),
    "w4");
  chartInstance->c1_is_active_c1_TenzoDecV3DeltaP = c1_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 4)),
     "is_active_c1_TenzoDecV3DeltaP");
  sf_mex_destroy(&c1_u);
  c1_update_debugger_state_c1_TenzoDecV3DeltaP(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_TenzoDecV3DeltaP(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance)
{
}

static void sf_c1_TenzoDecV3DeltaP(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance)
{
  real_T *c1_taux;
  real_T *c1_tauy;
  real_T *c1_tauz;
  real_T *c1_Thrust;
  real_T *c1_w1;
  real_T *c1_w2;
  real_T *c1_w3;
  real_T *c1_w4;
  c1_w4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c1_w3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c1_w2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_w1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_Thrust = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c1_tauz = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c1_tauy = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c1_taux = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c1_taux, 0U);
  _SFD_DATA_RANGE_CHECK(*c1_tauy, 1U);
  _SFD_DATA_RANGE_CHECK(*c1_tauz, 2U);
  _SFD_DATA_RANGE_CHECK(*c1_Thrust, 3U);
  _SFD_DATA_RANGE_CHECK(*c1_w1, 4U);
  _SFD_DATA_RANGE_CHECK(*c1_w2, 5U);
  _SFD_DATA_RANGE_CHECK(*c1_w3, 6U);
  _SFD_DATA_RANGE_CHECK(*c1_w4, 7U);
  chartInstance->c1_sfEvent = CALL_EVENT;
  c1_chartstep_c1_TenzoDecV3DeltaP(chartInstance);
  sf_debug_check_for_state_inconsistency(_TenzoDecV3DeltaPMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c1_chartstep_c1_TenzoDecV3DeltaP(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance)
{
  real_T c1_hoistedGlobal;
  real_T c1_b_hoistedGlobal;
  real_T c1_c_hoistedGlobal;
  real_T c1_d_hoistedGlobal;
  real_T c1_taux;
  real_T c1_tauy;
  real_T c1_tauz;
  real_T c1_Thrust;
  uint32_T c1_debug_family_var_map[38];
  real_T c1_g;
  real_T c1_rho;
  real_T c1_mq;
  real_T c1_lx;
  real_T c1_ly;
  real_T c1_lz;
  real_T c1_dcg;
  real_T c1_If;
  real_T c1_cp;
  real_T c1_ct;
  real_T c1_rp;
  real_T c1_Km;
  real_T c1_Kt;
  real_T c1_Ktm;
  real_T c1_k1;
  real_T c1_k2;
  real_T c1_k3;
  real_T c1_k4;
  real_T c1_tc;
  real_T c1_dz1;
  real_T c1_dz2;
  real_T c1_dz3;
  real_T c1_dz4;
  real_T c1_PWM_max;
  real_T c1_w0c;
  real_T c1_Ft0;
  real_T c1_WTF[16];
  real_T c1_T[4];
  real_T c1_nargin = 4.0;
  real_T c1_nargout = 4.0;
  real_T c1_w1;
  real_T c1_w2;
  real_T c1_w3;
  real_T c1_w4;
  real_T c1_x;
  int32_T c1_i0;
  static real_T c1_dv0[16] = { 0.0, 8.0834517164645556E-6, 4.3325606937667151E-7,
    -0.00028067540682168596, -8.0834517164645556E-6, 0.0, -4.3325606937667151E-7,
    -0.00028067540682168596, 0.0, -8.0834517164645556E-6, 4.3325606937667151E-7,
    -0.00028067540682168596, 8.0834517164645556E-6, 0.0, -4.3325606937667151E-7,
    -0.00028067540682168596 };

  real_T c1_B[4];
  int32_T c1_i1;
  real_T c1_b_B[4];
  real_T c1_dv1[4];
  int32_T c1_i2;
  real_T *c1_b_taux;
  real_T *c1_b_tauy;
  real_T *c1_b_tauz;
  real_T *c1_b_Thrust;
  real_T *c1_b_w1;
  real_T *c1_b_w2;
  real_T *c1_b_w3;
  real_T *c1_b_w4;
  c1_b_w4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c1_b_w3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c1_b_w2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_b_w1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_b_Thrust = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c1_b_tauz = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c1_b_tauy = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c1_b_taux = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  c1_hoistedGlobal = *c1_b_taux;
  c1_b_hoistedGlobal = *c1_b_tauy;
  c1_c_hoistedGlobal = *c1_b_tauz;
  c1_d_hoistedGlobal = *c1_b_Thrust;
  c1_taux = c1_hoistedGlobal;
  c1_tauy = c1_b_hoistedGlobal;
  c1_tauz = c1_c_hoistedGlobal;
  c1_Thrust = c1_d_hoistedGlobal;
  sf_debug_symbol_scope_push_eml(0U, 38U, 38U, c1_debug_family_names,
    c1_debug_family_var_map);
  sf_debug_symbol_scope_add_eml(&c1_g, 0U, c1_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c1_rho, 1U, c1_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c1_mq, 2U, c1_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c1_lx, 3U, c1_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c1_ly, 4U, c1_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c1_lz, 5U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_dcg, 6U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_If, 7U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c1_cp, 8U, c1_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c1_ct, 9U, c1_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c1_rp, 10U, c1_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c1_Km, 11U, c1_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c1_Kt, 12U, c1_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c1_Ktm, 13U, c1_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c1_k1, 14U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_k2, 15U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_k3, 16U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_k4, 17U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_tc, 18U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_dz1, 19U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_dz2, 20U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_dz3, 21U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_dz4, 22U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_PWM_max, 23U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_w0c, 24U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_Ft0, 25U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c1_WTF, 26U, c1_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c1_T, 27U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_nargin, 28U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_nargout, 29U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c1_taux, 30U, c1_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c1_tauy, 31U, c1_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c1_tauz, 32U, c1_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c1_Thrust, 33U, c1_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c1_w1, 34U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_w2, 35U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_w3, 36U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_w4, 37U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 4);
  c1_g = 9.81;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 5);
  c1_rho = 1.2;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 10);
  c1_mq = 0.82;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 12);
  c1_lx = 0.0288;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 14);
  c1_ly = 0.0288;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 16);
  c1_lz = 0.026;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 21);
  c1_dcg = 0.288;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 25);
  c1_If = -0.3559;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 28);
  c1_cp = 0.0743;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 29);
  c1_ct = 0.1154;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 32);
  c1_rp = 0.0254;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 35);
  c1_Km = 1.2160432353026188E-10;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 39);
  c1_Kt = 0.00028067540682168596;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 41);
  c1_Ktm = 4.3325606937667151E-7;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 42);
  c1_k1 = 2.028;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 43);
  c1_k2 = 1.869;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 44);
  c1_k3 = 2.002;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 45);
  c1_k4 = 1.996;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 46);
  c1_tc = 0.00436;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 48);
  c1_dz1 = 1247.4;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 50);
  c1_dz2 = 1247.8;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 52);
  c1_dz3 = 1246.4;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 54);
  c1_dz4 = 1247.9;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 63);
  c1_PWM_max = 2200.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 65);
  c1_w0c = 28660.152633574013;
  if (c1_w0c < 0.0) {
    c1_eml_error(chartInstance);
  }

  c1_x = c1_w0c;
  c1_w0c = c1_x;
  c1_w0c = muDoubleScalarSqrt(c1_w0c);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 67);
  c1_Ft0 = 8.0442;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 80);
  for (c1_i0 = 0; c1_i0 < 16; c1_i0++) {
    c1_WTF[c1_i0] = c1_dv0[c1_i0];
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 84);
  c1_B[0] = c1_taux;
  c1_B[1] = c1_tauy;
  c1_B[2] = c1_tauz;
  c1_B[3] = c1_Thrust;
  for (c1_i1 = 0; c1_i1 < 4; c1_i1++) {
    c1_b_B[c1_i1] = c1_B[c1_i1];
  }

  c1_eml_lusolve(chartInstance, c1_b_B, c1_dv1);
  for (c1_i2 = 0; c1_i2 < 4; c1_i2++) {
    c1_T[c1_i2] = c1_dv1[c1_i2];
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 88);
  c1_w1 = c1_T[0];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 89);
  c1_w2 = c1_T[1];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 90);
  c1_w3 = c1_T[2];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 91);
  c1_w4 = c1_T[3];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -91);
  sf_debug_symbol_scope_pop();
  *c1_b_w1 = c1_w1;
  *c1_b_w2 = c1_w2;
  *c1_b_w3 = c1_w3;
  *c1_b_w4 = c1_w4;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
}

static void initSimStructsc1_TenzoDecV3DeltaP
  (SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber)
{
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance;
  chartInstance = (SFc1_TenzoDecV3DeltaPInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static real_T c1_emlrt_marshallIn(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c1_w4, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_w4), &c1_thisId);
  sf_mex_destroy(&c1_w4);
  return c1_y;
}

static real_T c1_b_emlrt_marshallIn(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d0;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d0, 1, 0, 0U, 0, 0U, 0);
  c1_y = c1_d0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_w4;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance;
  chartInstance = (SFc1_TenzoDecV3DeltaPInstanceStruct *)chartInstanceVoid;
  c1_w4 = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_w4), &c1_thisId);
  sf_mex_destroy(&c1_w4);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i3;
  real_T c1_b_inData[4];
  int32_T c1_i4;
  real_T c1_u[4];
  const mxArray *c1_y = NULL;
  SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance;
  chartInstance = (SFc1_TenzoDecV3DeltaPInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i3 = 0; c1_i3 < 4; c1_i3++) {
    c1_b_inData[c1_i3] = (*(real_T (*)[4])c1_inData)[c1_i3];
  }

  for (c1_i4 = 0; c1_i4 < 4; c1_i4++) {
    c1_u[c1_i4] = c1_b_inData[c1_i4];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static void c1_c_emlrt_marshallIn(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[4])
{
  real_T c1_dv2[4];
  int32_T c1_i5;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv2, 1, 0, 0U, 1, 0U, 1, 4);
  for (c1_i5 = 0; c1_i5 < 4; c1_i5++) {
    c1_y[c1_i5] = c1_dv2[c1_i5];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_T;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[4];
  int32_T c1_i6;
  SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance;
  chartInstance = (SFc1_TenzoDecV3DeltaPInstanceStruct *)chartInstanceVoid;
  c1_T = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_T), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_T);
  for (c1_i6 = 0; c1_i6 < 4; c1_i6++) {
    (*(real_T (*)[4])c1_outData)[c1_i6] = c1_y[c1_i6];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i7;
  int32_T c1_i8;
  int32_T c1_i9;
  real_T c1_b_inData[16];
  int32_T c1_i10;
  int32_T c1_i11;
  int32_T c1_i12;
  real_T c1_u[16];
  const mxArray *c1_y = NULL;
  SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance;
  chartInstance = (SFc1_TenzoDecV3DeltaPInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_i7 = 0;
  for (c1_i8 = 0; c1_i8 < 4; c1_i8++) {
    for (c1_i9 = 0; c1_i9 < 4; c1_i9++) {
      c1_b_inData[c1_i9 + c1_i7] = (*(real_T (*)[16])c1_inData)[c1_i9 + c1_i7];
    }

    c1_i7 += 4;
  }

  c1_i10 = 0;
  for (c1_i11 = 0; c1_i11 < 4; c1_i11++) {
    for (c1_i12 = 0; c1_i12 < 4; c1_i12++) {
      c1_u[c1_i12 + c1_i10] = c1_b_inData[c1_i12 + c1_i10];
    }

    c1_i10 += 4;
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

const mxArray *sf_c1_TenzoDecV3DeltaP_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo;
  c1_ResolvedFunctionInfo c1_info[109];
  const mxArray *c1_m0 = NULL;
  int32_T c1_i13;
  c1_ResolvedFunctionInfo *c1_r0;
  c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  c1_info_helper(c1_info);
  c1_b_info_helper(c1_info);
  sf_mex_assign(&c1_m0, sf_mex_createstruct("nameCaptureInfo", 1, 109), FALSE);
  for (c1_i13 = 0; c1_i13 < 109; c1_i13++) {
    c1_r0 = &c1_info[c1_i13];
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c1_r0->context)), "context", "nameCaptureInfo",
                    c1_i13);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c1_r0->name)), "name", "nameCaptureInfo", c1_i13);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c1_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c1_i13);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c1_r0->resolved)), "resolved", "nameCaptureInfo",
                    c1_i13);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c1_i13);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c1_i13);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c1_i13);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c1_i13);
  }

  sf_mex_assign(&c1_nameCaptureInfo, c1_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c1_nameCaptureInfo);
  return c1_nameCaptureInfo;
}

static void c1_info_helper(c1_ResolvedFunctionInfo c1_info[109])
{
  c1_info[0].context = "";
  c1_info[0].name = "mtimes";
  c1_info[0].dominantType = "double";
  c1_info[0].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  c1_info[0].fileTimeLo = 1289519692U;
  c1_info[0].fileTimeHi = 0U;
  c1_info[0].mFileTimeLo = 0U;
  c1_info[0].mFileTimeHi = 0U;
  c1_info[1].context = "";
  c1_info[1].name = "mpower";
  c1_info[1].dominantType = "double";
  c1_info[1].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mpower.m";
  c1_info[1].fileTimeLo = 1286818842U;
  c1_info[1].fileTimeHi = 0U;
  c1_info[1].mFileTimeLo = 0U;
  c1_info[1].mFileTimeHi = 0U;
  c1_info[2].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mpower.m";
  c1_info[2].name = "power";
  c1_info[2].dominantType = "double";
  c1_info[2].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/power.m";
  c1_info[2].fileTimeLo = 1307651240U;
  c1_info[2].fileTimeHi = 0U;
  c1_info[2].mFileTimeLo = 0U;
  c1_info[2].mFileTimeHi = 0U;
  c1_info[3].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/power.m";
  c1_info[3].name = "eml_scalar_eg";
  c1_info[3].dominantType = "double";
  c1_info[3].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[3].fileTimeLo = 1286818796U;
  c1_info[3].fileTimeHi = 0U;
  c1_info[3].mFileTimeLo = 0U;
  c1_info[3].mFileTimeHi = 0U;
  c1_info[4].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/power.m";
  c1_info[4].name = "eml_scalexp_alloc";
  c1_info[4].dominantType = "double";
  c1_info[4].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c1_info[4].fileTimeLo = 1286818796U;
  c1_info[4].fileTimeHi = 0U;
  c1_info[4].mFileTimeLo = 0U;
  c1_info[4].mFileTimeHi = 0U;
  c1_info[5].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/power.m";
  c1_info[5].name = "eml_scalar_floor";
  c1_info[5].dominantType = "double";
  c1_info[5].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c1_info[5].fileTimeLo = 1286818726U;
  c1_info[5].fileTimeHi = 0U;
  c1_info[5].mFileTimeLo = 0U;
  c1_info[5].mFileTimeHi = 0U;
  c1_info[6].context = "";
  c1_info[6].name = "mrdivide";
  c1_info[6].dominantType = "double";
  c1_info[6].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c1_info[6].fileTimeLo = 1325124138U;
  c1_info[6].fileTimeHi = 0U;
  c1_info[6].mFileTimeLo = 1319729966U;
  c1_info[6].mFileTimeHi = 0U;
  c1_info[7].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c1_info[7].name = "rdivide";
  c1_info[7].dominantType = "double";
  c1_info[7].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/rdivide.m";
  c1_info[7].fileTimeLo = 1286818844U;
  c1_info[7].fileTimeHi = 0U;
  c1_info[7].mFileTimeLo = 0U;
  c1_info[7].mFileTimeHi = 0U;
  c1_info[8].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/rdivide.m";
  c1_info[8].name = "eml_div";
  c1_info[8].dominantType = "double";
  c1_info[8].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_div.m";
  c1_info[8].fileTimeLo = 1313347810U;
  c1_info[8].fileTimeHi = 0U;
  c1_info[8].mFileTimeLo = 0U;
  c1_info[8].mFileTimeHi = 0U;
  c1_info[9].context = "";
  c1_info[9].name = "sqrt";
  c1_info[9].dominantType = "double";
  c1_info[9].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c1_info[9].fileTimeLo = 1286818752U;
  c1_info[9].fileTimeHi = 0U;
  c1_info[9].mFileTimeLo = 0U;
  c1_info[9].mFileTimeHi = 0U;
  c1_info[10].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c1_info[10].name = "eml_error";
  c1_info[10].dominantType = "char";
  c1_info[10].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_error.m";
  c1_info[10].fileTimeLo = 1305318000U;
  c1_info[10].fileTimeHi = 0U;
  c1_info[10].mFileTimeLo = 0U;
  c1_info[10].mFileTimeHi = 0U;
  c1_info[11].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c1_info[11].name = "eml_scalar_sqrt";
  c1_info[11].dominantType = "double";
  c1_info[11].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c1_info[11].fileTimeLo = 1286818738U;
  c1_info[11].fileTimeHi = 0U;
  c1_info[11].mFileTimeLo = 0U;
  c1_info[11].mFileTimeHi = 0U;
  c1_info[12].context = "";
  c1_info[12].name = "mldivide";
  c1_info[12].dominantType = "double";
  c1_info[12].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mldivide.p";
  c1_info[12].fileTimeLo = 1325124138U;
  c1_info[12].fileTimeHi = 0U;
  c1_info[12].mFileTimeLo = 1319729966U;
  c1_info[12].mFileTimeHi = 0U;
  c1_info[13].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mldivide.p";
  c1_info[13].name = "eml_lusolve";
  c1_info[13].dominantType = "double";
  c1_info[13].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c1_info[13].fileTimeLo = 1309451196U;
  c1_info[13].fileTimeHi = 0U;
  c1_info[13].mFileTimeLo = 0U;
  c1_info[13].mFileTimeHi = 0U;
  c1_info[14].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c1_info[14].name = "eml_index_class";
  c1_info[14].dominantType = "";
  c1_info[14].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[14].fileTimeLo = 1286818778U;
  c1_info[14].fileTimeHi = 0U;
  c1_info[14].mFileTimeLo = 0U;
  c1_info[14].mFileTimeHi = 0U;
  c1_info[15].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c1_info[15].name = "eml_index_class";
  c1_info[15].dominantType = "";
  c1_info[15].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[15].fileTimeLo = 1286818778U;
  c1_info[15].fileTimeHi = 0U;
  c1_info[15].mFileTimeLo = 0U;
  c1_info[15].mFileTimeHi = 0U;
  c1_info[16].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c1_info[16].name = "eml_xgetrf";
  c1_info[16].dominantType = "int32";
  c1_info[16].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c1_info[16].fileTimeLo = 1286818806U;
  c1_info[16].fileTimeHi = 0U;
  c1_info[16].mFileTimeLo = 0U;
  c1_info[16].mFileTimeHi = 0U;
  c1_info[17].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c1_info[17].name = "eml_lapack_xgetrf";
  c1_info[17].dominantType = "int32";
  c1_info[17].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c1_info[17].fileTimeLo = 1286818810U;
  c1_info[17].fileTimeHi = 0U;
  c1_info[17].mFileTimeLo = 0U;
  c1_info[17].mFileTimeHi = 0U;
  c1_info[18].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c1_info[18].name = "eml_matlab_zgetrf";
  c1_info[18].dominantType = "int32";
  c1_info[18].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c1_info[18].fileTimeLo = 1302688994U;
  c1_info[18].fileTimeHi = 0U;
  c1_info[18].mFileTimeLo = 0U;
  c1_info[18].mFileTimeHi = 0U;
  c1_info[19].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c1_info[19].name = "realmin";
  c1_info[19].dominantType = "char";
  c1_info[19].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/realmin.m";
  c1_info[19].fileTimeLo = 1307651242U;
  c1_info[19].fileTimeHi = 0U;
  c1_info[19].mFileTimeLo = 0U;
  c1_info[19].mFileTimeHi = 0U;
  c1_info[20].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/realmin.m";
  c1_info[20].name = "eml_realmin";
  c1_info[20].dominantType = "char";
  c1_info[20].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c1_info[20].fileTimeLo = 1307651244U;
  c1_info[20].fileTimeHi = 0U;
  c1_info[20].mFileTimeLo = 0U;
  c1_info[20].mFileTimeHi = 0U;
  c1_info[21].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c1_info[21].name = "eml_float_model";
  c1_info[21].dominantType = "char";
  c1_info[21].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c1_info[21].fileTimeLo = 1307651242U;
  c1_info[21].fileTimeHi = 0U;
  c1_info[21].mFileTimeLo = 0U;
  c1_info[21].mFileTimeHi = 0U;
  c1_info[22].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c1_info[22].name = "eps";
  c1_info[22].dominantType = "char";
  c1_info[22].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/eps.m";
  c1_info[22].fileTimeLo = 1307651240U;
  c1_info[22].fileTimeHi = 0U;
  c1_info[22].mFileTimeLo = 0U;
  c1_info[22].mFileTimeHi = 0U;
  c1_info[23].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/eps.m";
  c1_info[23].name = "eml_is_float_class";
  c1_info[23].dominantType = "char";
  c1_info[23].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c1_info[23].fileTimeLo = 1286818782U;
  c1_info[23].fileTimeHi = 0U;
  c1_info[23].mFileTimeLo = 0U;
  c1_info[23].mFileTimeHi = 0U;
  c1_info[24].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/eps.m";
  c1_info[24].name = "eml_eps";
  c1_info[24].dominantType = "char";
  c1_info[24].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c1_info[24].fileTimeLo = 1307651240U;
  c1_info[24].fileTimeHi = 0U;
  c1_info[24].mFileTimeLo = 0U;
  c1_info[24].mFileTimeHi = 0U;
  c1_info[25].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c1_info[25].name = "eml_float_model";
  c1_info[25].dominantType = "char";
  c1_info[25].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c1_info[25].fileTimeLo = 1307651242U;
  c1_info[25].fileTimeHi = 0U;
  c1_info[25].mFileTimeLo = 0U;
  c1_info[25].mFileTimeHi = 0U;
  c1_info[26].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c1_info[26].name = "min";
  c1_info[26].dominantType = "int32";
  c1_info[26].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/datafun/min.m";
  c1_info[26].fileTimeLo = 1311255318U;
  c1_info[26].fileTimeHi = 0U;
  c1_info[26].mFileTimeLo = 0U;
  c1_info[26].mFileTimeHi = 0U;
  c1_info[27].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/datafun/min.m";
  c1_info[27].name = "eml_min_or_max";
  c1_info[27].dominantType = "int32";
  c1_info[27].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c1_info[27].fileTimeLo = 1303146212U;
  c1_info[27].fileTimeHi = 0U;
  c1_info[27].mFileTimeLo = 0U;
  c1_info[27].mFileTimeHi = 0U;
  c1_info[28].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c1_info[28].name = "eml_scalar_eg";
  c1_info[28].dominantType = "int32";
  c1_info[28].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[28].fileTimeLo = 1286818796U;
  c1_info[28].fileTimeHi = 0U;
  c1_info[28].mFileTimeLo = 0U;
  c1_info[28].mFileTimeHi = 0U;
  c1_info[29].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c1_info[29].name = "eml_scalexp_alloc";
  c1_info[29].dominantType = "int32";
  c1_info[29].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c1_info[29].fileTimeLo = 1286818796U;
  c1_info[29].fileTimeHi = 0U;
  c1_info[29].mFileTimeLo = 0U;
  c1_info[29].mFileTimeHi = 0U;
  c1_info[30].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c1_info[30].name = "eml_index_class";
  c1_info[30].dominantType = "";
  c1_info[30].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[30].fileTimeLo = 1286818778U;
  c1_info[30].fileTimeHi = 0U;
  c1_info[30].mFileTimeLo = 0U;
  c1_info[30].mFileTimeHi = 0U;
  c1_info[31].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c1_info[31].name = "eml_scalar_eg";
  c1_info[31].dominantType = "int32";
  c1_info[31].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[31].fileTimeLo = 1286818796U;
  c1_info[31].fileTimeHi = 0U;
  c1_info[31].mFileTimeLo = 0U;
  c1_info[31].mFileTimeHi = 0U;
  c1_info[32].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c1_info[32].name = "colon";
  c1_info[32].dominantType = "int32";
  c1_info[32].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m";
  c1_info[32].fileTimeLo = 1311255318U;
  c1_info[32].fileTimeHi = 0U;
  c1_info[32].mFileTimeLo = 0U;
  c1_info[32].mFileTimeHi = 0U;
  c1_info[33].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m";
  c1_info[33].name = "colon";
  c1_info[33].dominantType = "int32";
  c1_info[33].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m";
  c1_info[33].fileTimeLo = 1311255318U;
  c1_info[33].fileTimeHi = 0U;
  c1_info[33].mFileTimeLo = 0U;
  c1_info[33].mFileTimeHi = 0U;
  c1_info[34].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m";
  c1_info[34].name = "floor";
  c1_info[34].dominantType = "double";
  c1_info[34].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/floor.m";
  c1_info[34].fileTimeLo = 1286818742U;
  c1_info[34].fileTimeHi = 0U;
  c1_info[34].mFileTimeLo = 0U;
  c1_info[34].mFileTimeHi = 0U;
  c1_info[35].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/floor.m";
  c1_info[35].name = "eml_scalar_floor";
  c1_info[35].dominantType = "double";
  c1_info[35].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c1_info[35].fileTimeLo = 1286818726U;
  c1_info[35].fileTimeHi = 0U;
  c1_info[35].mFileTimeLo = 0U;
  c1_info[35].mFileTimeHi = 0U;
  c1_info[36].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c1_info[36].name = "intmin";
  c1_info[36].dominantType = "char";
  c1_info[36].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/intmin.m";
  c1_info[36].fileTimeLo = 1311255318U;
  c1_info[36].fileTimeHi = 0U;
  c1_info[36].mFileTimeLo = 0U;
  c1_info[36].mFileTimeHi = 0U;
  c1_info[37].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c1_info[37].name = "intmax";
  c1_info[37].dominantType = "char";
  c1_info[37].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/intmax.m";
  c1_info[37].fileTimeLo = 1311255316U;
  c1_info[37].fileTimeHi = 0U;
  c1_info[37].mFileTimeLo = 0U;
  c1_info[37].mFileTimeHi = 0U;
  c1_info[38].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c1_info[38].name = "intmin";
  c1_info[38].dominantType = "char";
  c1_info[38].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/intmin.m";
  c1_info[38].fileTimeLo = 1311255318U;
  c1_info[38].fileTimeHi = 0U;
  c1_info[38].mFileTimeLo = 0U;
  c1_info[38].mFileTimeHi = 0U;
  c1_info[39].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c1_info[39].name = "intmax";
  c1_info[39].dominantType = "char";
  c1_info[39].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/intmax.m";
  c1_info[39].fileTimeLo = 1311255316U;
  c1_info[39].fileTimeHi = 0U;
  c1_info[39].mFileTimeLo = 0U;
  c1_info[39].mFileTimeHi = 0U;
  c1_info[40].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c1_info[40].name = "eml_isa_uint";
  c1_info[40].dominantType = "int32";
  c1_info[40].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c1_info[40].fileTimeLo = 1286818784U;
  c1_info[40].fileTimeHi = 0U;
  c1_info[40].mFileTimeLo = 0U;
  c1_info[40].mFileTimeHi = 0U;
  c1_info[41].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c1_info[41].name = "eml_unsigned_class";
  c1_info[41].dominantType = "char";
  c1_info[41].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c1_info[41].fileTimeLo = 1286818800U;
  c1_info[41].fileTimeHi = 0U;
  c1_info[41].mFileTimeLo = 0U;
  c1_info[41].mFileTimeHi = 0U;
  c1_info[42].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c1_info[42].name = "eml_index_class";
  c1_info[42].dominantType = "";
  c1_info[42].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[42].fileTimeLo = 1286818778U;
  c1_info[42].fileTimeHi = 0U;
  c1_info[42].mFileTimeLo = 0U;
  c1_info[42].mFileTimeHi = 0U;
  c1_info[43].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c1_info[43].name = "intmax";
  c1_info[43].dominantType = "char";
  c1_info[43].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/intmax.m";
  c1_info[43].fileTimeLo = 1311255316U;
  c1_info[43].fileTimeHi = 0U;
  c1_info[43].mFileTimeLo = 0U;
  c1_info[43].mFileTimeHi = 0U;
  c1_info[44].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c1_info[44].name = "eml_isa_uint";
  c1_info[44].dominantType = "int32";
  c1_info[44].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c1_info[44].fileTimeLo = 1286818784U;
  c1_info[44].fileTimeHi = 0U;
  c1_info[44].mFileTimeLo = 0U;
  c1_info[44].mFileTimeHi = 0U;
  c1_info[45].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c1_info[45].name = "eml_index_plus";
  c1_info[45].dominantType = "int32";
  c1_info[45].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c1_info[45].fileTimeLo = 1286818778U;
  c1_info[45].fileTimeHi = 0U;
  c1_info[45].mFileTimeLo = 0U;
  c1_info[45].mFileTimeHi = 0U;
  c1_info[46].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c1_info[46].name = "eml_index_class";
  c1_info[46].dominantType = "";
  c1_info[46].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[46].fileTimeLo = 1286818778U;
  c1_info[46].fileTimeHi = 0U;
  c1_info[46].mFileTimeLo = 0U;
  c1_info[46].mFileTimeHi = 0U;
  c1_info[47].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon";
  c1_info[47].name = "eml_int_forloop_overflow_check";
  c1_info[47].dominantType = "";
  c1_info[47].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c1_info[47].fileTimeLo = 1311255316U;
  c1_info[47].fileTimeHi = 0U;
  c1_info[47].mFileTimeLo = 0U;
  c1_info[47].mFileTimeHi = 0U;
  c1_info[48].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c1_info[48].name = "intmax";
  c1_info[48].dominantType = "char";
  c1_info[48].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/intmax.m";
  c1_info[48].fileTimeLo = 1311255316U;
  c1_info[48].fileTimeHi = 0U;
  c1_info[48].mFileTimeLo = 0U;
  c1_info[48].mFileTimeHi = 0U;
  c1_info[49].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c1_info[49].name = "eml_index_class";
  c1_info[49].dominantType = "";
  c1_info[49].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[49].fileTimeLo = 1286818778U;
  c1_info[49].fileTimeHi = 0U;
  c1_info[49].mFileTimeLo = 0U;
  c1_info[49].mFileTimeHi = 0U;
  c1_info[50].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c1_info[50].name = "eml_index_plus";
  c1_info[50].dominantType = "int32";
  c1_info[50].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c1_info[50].fileTimeLo = 1286818778U;
  c1_info[50].fileTimeHi = 0U;
  c1_info[50].mFileTimeLo = 0U;
  c1_info[50].mFileTimeHi = 0U;
  c1_info[51].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c1_info[51].name = "eml_int_forloop_overflow_check";
  c1_info[51].dominantType = "";
  c1_info[51].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c1_info[51].fileTimeLo = 1311255316U;
  c1_info[51].fileTimeHi = 0U;
  c1_info[51].mFileTimeLo = 0U;
  c1_info[51].mFileTimeHi = 0U;
  c1_info[52].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c1_info[52].name = "eml_index_minus";
  c1_info[52].dominantType = "int32";
  c1_info[52].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c1_info[52].fileTimeLo = 1286818778U;
  c1_info[52].fileTimeHi = 0U;
  c1_info[52].mFileTimeLo = 0U;
  c1_info[52].mFileTimeHi = 0U;
  c1_info[53].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c1_info[53].name = "eml_index_class";
  c1_info[53].dominantType = "";
  c1_info[53].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[53].fileTimeLo = 1286818778U;
  c1_info[53].fileTimeHi = 0U;
  c1_info[53].mFileTimeLo = 0U;
  c1_info[53].mFileTimeHi = 0U;
  c1_info[54].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c1_info[54].name = "eml_index_times";
  c1_info[54].dominantType = "int32";
  c1_info[54].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c1_info[54].fileTimeLo = 1286818780U;
  c1_info[54].fileTimeHi = 0U;
  c1_info[54].mFileTimeLo = 0U;
  c1_info[54].mFileTimeHi = 0U;
  c1_info[55].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c1_info[55].name = "eml_index_class";
  c1_info[55].dominantType = "";
  c1_info[55].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[55].fileTimeLo = 1286818778U;
  c1_info[55].fileTimeHi = 0U;
  c1_info[55].mFileTimeLo = 0U;
  c1_info[55].mFileTimeHi = 0U;
  c1_info[56].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c1_info[56].name = "eml_ixamax";
  c1_info[56].dominantType = "int32";
  c1_info[56].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c1_info[56].fileTimeLo = 1299076770U;
  c1_info[56].fileTimeHi = 0U;
  c1_info[56].mFileTimeLo = 0U;
  c1_info[56].mFileTimeHi = 0U;
  c1_info[57].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c1_info[57].name = "eml_blas_inline";
  c1_info[57].dominantType = "";
  c1_info[57].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c1_info[57].fileTimeLo = 1299076768U;
  c1_info[57].fileTimeHi = 0U;
  c1_info[57].mFileTimeLo = 0U;
  c1_info[57].mFileTimeHi = 0U;
  c1_info[58].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m!below_threshold";
  c1_info[58].name = "length";
  c1_info[58].dominantType = "double";
  c1_info[58].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/length.m";
  c1_info[58].fileTimeLo = 1303146206U;
  c1_info[58].fileTimeHi = 0U;
  c1_info[58].mFileTimeLo = 0U;
  c1_info[58].mFileTimeHi = 0U;
  c1_info[59].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c1_info[59].name = "eml_index_class";
  c1_info[59].dominantType = "";
  c1_info[59].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[59].fileTimeLo = 1286818778U;
  c1_info[59].fileTimeHi = 0U;
  c1_info[59].mFileTimeLo = 0U;
  c1_info[59].mFileTimeHi = 0U;
  c1_info[60].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c1_info[60].name = "eml_refblas_ixamax";
  c1_info[60].dominantType = "int32";
  c1_info[60].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c1_info[60].fileTimeLo = 1299076770U;
  c1_info[60].fileTimeHi = 0U;
  c1_info[60].mFileTimeLo = 0U;
  c1_info[60].mFileTimeHi = 0U;
  c1_info[61].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c1_info[61].name = "eml_index_class";
  c1_info[61].dominantType = "";
  c1_info[61].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[61].fileTimeLo = 1286818778U;
  c1_info[61].fileTimeHi = 0U;
  c1_info[61].mFileTimeLo = 0U;
  c1_info[61].mFileTimeHi = 0U;
  c1_info[62].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c1_info[62].name = "eml_xcabs1";
  c1_info[62].dominantType = "double";
  c1_info[62].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c1_info[62].fileTimeLo = 1286818706U;
  c1_info[62].fileTimeHi = 0U;
  c1_info[62].mFileTimeLo = 0U;
  c1_info[62].mFileTimeHi = 0U;
  c1_info[63].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c1_info[63].name = "abs";
  c1_info[63].dominantType = "double";
  c1_info[63].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/abs.m";
  c1_info[63].fileTimeLo = 1286818694U;
  c1_info[63].fileTimeHi = 0U;
  c1_info[63].mFileTimeLo = 0U;
  c1_info[63].mFileTimeHi = 0U;
}

static void c1_b_info_helper(c1_ResolvedFunctionInfo c1_info[109])
{
  c1_info[64].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/abs.m";
  c1_info[64].name = "eml_scalar_abs";
  c1_info[64].dominantType = "double";
  c1_info[64].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c1_info[64].fileTimeLo = 1286818712U;
  c1_info[64].fileTimeHi = 0U;
  c1_info[64].mFileTimeLo = 0U;
  c1_info[64].mFileTimeHi = 0U;
  c1_info[65].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c1_info[65].name = "eml_int_forloop_overflow_check";
  c1_info[65].dominantType = "";
  c1_info[65].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c1_info[65].fileTimeLo = 1311255316U;
  c1_info[65].fileTimeHi = 0U;
  c1_info[65].mFileTimeLo = 0U;
  c1_info[65].mFileTimeHi = 0U;
  c1_info[66].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c1_info[66].name = "eml_index_plus";
  c1_info[66].dominantType = "int32";
  c1_info[66].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c1_info[66].fileTimeLo = 1286818778U;
  c1_info[66].fileTimeHi = 0U;
  c1_info[66].mFileTimeLo = 0U;
  c1_info[66].mFileTimeHi = 0U;
  c1_info[67].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c1_info[67].name = "eml_xswap";
  c1_info[67].dominantType = "int32";
  c1_info[67].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c1_info[67].fileTimeLo = 1299076778U;
  c1_info[67].fileTimeHi = 0U;
  c1_info[67].mFileTimeLo = 0U;
  c1_info[67].mFileTimeHi = 0U;
  c1_info[68].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c1_info[68].name = "eml_blas_inline";
  c1_info[68].dominantType = "";
  c1_info[68].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c1_info[68].fileTimeLo = 1299076768U;
  c1_info[68].fileTimeHi = 0U;
  c1_info[68].mFileTimeLo = 0U;
  c1_info[68].mFileTimeHi = 0U;
  c1_info[69].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c1_info[69].name = "eml_refblas_xswap";
  c1_info[69].dominantType = "int32";
  c1_info[69].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c1_info[69].fileTimeLo = 1299076786U;
  c1_info[69].fileTimeHi = 0U;
  c1_info[69].mFileTimeLo = 0U;
  c1_info[69].mFileTimeHi = 0U;
  c1_info[70].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c1_info[70].name = "eml_index_class";
  c1_info[70].dominantType = "";
  c1_info[70].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[70].fileTimeLo = 1286818778U;
  c1_info[70].fileTimeHi = 0U;
  c1_info[70].mFileTimeLo = 0U;
  c1_info[70].mFileTimeHi = 0U;
  c1_info[71].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c1_info[71].name = "abs";
  c1_info[71].dominantType = "int32";
  c1_info[71].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/abs.m";
  c1_info[71].fileTimeLo = 1286818694U;
  c1_info[71].fileTimeHi = 0U;
  c1_info[71].mFileTimeLo = 0U;
  c1_info[71].mFileTimeHi = 0U;
  c1_info[72].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/abs.m";
  c1_info[72].name = "eml_scalar_abs";
  c1_info[72].dominantType = "int32";
  c1_info[72].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c1_info[72].fileTimeLo = 1286818712U;
  c1_info[72].fileTimeHi = 0U;
  c1_info[72].mFileTimeLo = 0U;
  c1_info[72].mFileTimeHi = 0U;
  c1_info[73].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c1_info[73].name = "eml_int_forloop_overflow_check";
  c1_info[73].dominantType = "";
  c1_info[73].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c1_info[73].fileTimeLo = 1311255316U;
  c1_info[73].fileTimeHi = 0U;
  c1_info[73].mFileTimeLo = 0U;
  c1_info[73].mFileTimeHi = 0U;
  c1_info[74].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c1_info[74].name = "eml_index_plus";
  c1_info[74].dominantType = "int32";
  c1_info[74].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c1_info[74].fileTimeLo = 1286818778U;
  c1_info[74].fileTimeHi = 0U;
  c1_info[74].mFileTimeLo = 0U;
  c1_info[74].mFileTimeHi = 0U;
  c1_info[75].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c1_info[75].name = "eml_div";
  c1_info[75].dominantType = "double";
  c1_info[75].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_div.m";
  c1_info[75].fileTimeLo = 1313347810U;
  c1_info[75].fileTimeHi = 0U;
  c1_info[75].mFileTimeLo = 0U;
  c1_info[75].mFileTimeHi = 0U;
  c1_info[76].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c1_info[76].name = "eml_xgeru";
  c1_info[76].dominantType = "int32";
  c1_info[76].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c1_info[76].fileTimeLo = 1299076774U;
  c1_info[76].fileTimeHi = 0U;
  c1_info[76].mFileTimeLo = 0U;
  c1_info[76].mFileTimeHi = 0U;
  c1_info[77].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c1_info[77].name = "eml_blas_inline";
  c1_info[77].dominantType = "";
  c1_info[77].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c1_info[77].fileTimeLo = 1299076768U;
  c1_info[77].fileTimeHi = 0U;
  c1_info[77].mFileTimeLo = 0U;
  c1_info[77].mFileTimeHi = 0U;
  c1_info[78].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c1_info[78].name = "eml_xger";
  c1_info[78].dominantType = "int32";
  c1_info[78].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c1_info[78].fileTimeLo = 1299076774U;
  c1_info[78].fileTimeHi = 0U;
  c1_info[78].mFileTimeLo = 0U;
  c1_info[78].mFileTimeHi = 0U;
  c1_info[79].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c1_info[79].name = "eml_blas_inline";
  c1_info[79].dominantType = "";
  c1_info[79].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c1_info[79].fileTimeLo = 1299076768U;
  c1_info[79].fileTimeHi = 0U;
  c1_info[79].mFileTimeLo = 0U;
  c1_info[79].mFileTimeHi = 0U;
  c1_info[80].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c1_info[80].name = "intmax";
  c1_info[80].dominantType = "char";
  c1_info[80].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/intmax.m";
  c1_info[80].fileTimeLo = 1311255316U;
  c1_info[80].fileTimeHi = 0U;
  c1_info[80].mFileTimeLo = 0U;
  c1_info[80].mFileTimeHi = 0U;
  c1_info[81].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c1_info[81].name = "min";
  c1_info[81].dominantType = "double";
  c1_info[81].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/datafun/min.m";
  c1_info[81].fileTimeLo = 1311255318U;
  c1_info[81].fileTimeHi = 0U;
  c1_info[81].mFileTimeLo = 0U;
  c1_info[81].mFileTimeHi = 0U;
  c1_info[82].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/datafun/min.m";
  c1_info[82].name = "eml_min_or_max";
  c1_info[82].dominantType = "char";
  c1_info[82].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c1_info[82].fileTimeLo = 1303146212U;
  c1_info[82].fileTimeHi = 0U;
  c1_info[82].mFileTimeLo = 0U;
  c1_info[82].mFileTimeHi = 0U;
  c1_info[83].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c1_info[83].name = "eml_scalar_eg";
  c1_info[83].dominantType = "double";
  c1_info[83].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[83].fileTimeLo = 1286818796U;
  c1_info[83].fileTimeHi = 0U;
  c1_info[83].mFileTimeLo = 0U;
  c1_info[83].mFileTimeHi = 0U;
  c1_info[84].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c1_info[84].name = "eml_scalexp_alloc";
  c1_info[84].dominantType = "double";
  c1_info[84].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c1_info[84].fileTimeLo = 1286818796U;
  c1_info[84].fileTimeHi = 0U;
  c1_info[84].mFileTimeLo = 0U;
  c1_info[84].mFileTimeHi = 0U;
  c1_info[85].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c1_info[85].name = "eml_scalar_eg";
  c1_info[85].dominantType = "double";
  c1_info[85].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[85].fileTimeLo = 1286818796U;
  c1_info[85].fileTimeHi = 0U;
  c1_info[85].mFileTimeLo = 0U;
  c1_info[85].mFileTimeHi = 0U;
  c1_info[86].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c1_info[86].name = "mtimes";
  c1_info[86].dominantType = "double";
  c1_info[86].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  c1_info[86].fileTimeLo = 1289519692U;
  c1_info[86].fileTimeHi = 0U;
  c1_info[86].mFileTimeLo = 0U;
  c1_info[86].mFileTimeHi = 0U;
  c1_info[87].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c1_info[87].name = "eml_refblas_xger";
  c1_info[87].dominantType = "int32";
  c1_info[87].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c1_info[87].fileTimeLo = 1299076776U;
  c1_info[87].fileTimeHi = 0U;
  c1_info[87].mFileTimeLo = 0U;
  c1_info[87].mFileTimeHi = 0U;
  c1_info[88].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c1_info[88].name = "eml_refblas_xgerx";
  c1_info[88].dominantType = "int32";
  c1_info[88].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c1_info[88].fileTimeLo = 1299076778U;
  c1_info[88].fileTimeHi = 0U;
  c1_info[88].mFileTimeLo = 0U;
  c1_info[88].mFileTimeHi = 0U;
  c1_info[89].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c1_info[89].name = "eml_index_class";
  c1_info[89].dominantType = "";
  c1_info[89].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[89].fileTimeLo = 1286818778U;
  c1_info[89].fileTimeHi = 0U;
  c1_info[89].mFileTimeLo = 0U;
  c1_info[89].mFileTimeHi = 0U;
  c1_info[90].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c1_info[90].name = "abs";
  c1_info[90].dominantType = "int32";
  c1_info[90].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/abs.m";
  c1_info[90].fileTimeLo = 1286818694U;
  c1_info[90].fileTimeHi = 0U;
  c1_info[90].mFileTimeLo = 0U;
  c1_info[90].mFileTimeHi = 0U;
  c1_info[91].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c1_info[91].name = "eml_index_minus";
  c1_info[91].dominantType = "int32";
  c1_info[91].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c1_info[91].fileTimeLo = 1286818778U;
  c1_info[91].fileTimeHi = 0U;
  c1_info[91].mFileTimeLo = 0U;
  c1_info[91].mFileTimeHi = 0U;
  c1_info[92].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c1_info[92].name = "eml_int_forloop_overflow_check";
  c1_info[92].dominantType = "";
  c1_info[92].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c1_info[92].fileTimeLo = 1311255316U;
  c1_info[92].fileTimeHi = 0U;
  c1_info[92].mFileTimeLo = 0U;
  c1_info[92].mFileTimeHi = 0U;
  c1_info[93].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c1_info[93].name = "eml_index_plus";
  c1_info[93].dominantType = "int32";
  c1_info[93].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c1_info[93].fileTimeLo = 1286818778U;
  c1_info[93].fileTimeHi = 0U;
  c1_info[93].mFileTimeLo = 0U;
  c1_info[93].mFileTimeHi = 0U;
  c1_info[94].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c1_info[94].name = "eml_scalar_eg";
  c1_info[94].dominantType = "double";
  c1_info[94].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[94].fileTimeLo = 1286818796U;
  c1_info[94].fileTimeHi = 0U;
  c1_info[94].mFileTimeLo = 0U;
  c1_info[94].mFileTimeHi = 0U;
  c1_info[95].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c1_info[95].name = "eml_int_forloop_overflow_check";
  c1_info[95].dominantType = "";
  c1_info[95].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c1_info[95].fileTimeLo = 1311255316U;
  c1_info[95].fileTimeHi = 0U;
  c1_info[95].mFileTimeLo = 0U;
  c1_info[95].mFileTimeHi = 0U;
  c1_info[96].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c1_info[96].name = "eml_xtrsm";
  c1_info[96].dominantType = "int32";
  c1_info[96].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c1_info[96].fileTimeLo = 1299076778U;
  c1_info[96].fileTimeHi = 0U;
  c1_info[96].mFileTimeLo = 0U;
  c1_info[96].mFileTimeHi = 0U;
  c1_info[97].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c1_info[97].name = "eml_blas_inline";
  c1_info[97].dominantType = "";
  c1_info[97].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c1_info[97].fileTimeLo = 1299076768U;
  c1_info[97].fileTimeHi = 0U;
  c1_info[97].mFileTimeLo = 0U;
  c1_info[97].mFileTimeHi = 0U;
  c1_info[98].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m!below_threshold";
  c1_info[98].name = "mtimes";
  c1_info[98].dominantType = "double";
  c1_info[98].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  c1_info[98].fileTimeLo = 1289519692U;
  c1_info[98].fileTimeHi = 0U;
  c1_info[98].mFileTimeLo = 0U;
  c1_info[98].mFileTimeHi = 0U;
  c1_info[99].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c1_info[99].name = "eml_scalar_eg";
  c1_info[99].dominantType = "double";
  c1_info[99].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[99].fileTimeLo = 1286818796U;
  c1_info[99].fileTimeHi = 0U;
  c1_info[99].mFileTimeLo = 0U;
  c1_info[99].mFileTimeHi = 0U;
  c1_info[100].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c1_info[100].name = "eml_refblas_xtrsm";
  c1_info[100].dominantType = "int32";
  c1_info[100].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c1_info[100].fileTimeLo = 1299076786U;
  c1_info[100].fileTimeHi = 0U;
  c1_info[100].mFileTimeLo = 0U;
  c1_info[100].mFileTimeHi = 0U;
  c1_info[101].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c1_info[101].name = "eml_scalar_eg";
  c1_info[101].dominantType = "double";
  c1_info[101].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[101].fileTimeLo = 1286818796U;
  c1_info[101].fileTimeHi = 0U;
  c1_info[101].mFileTimeLo = 0U;
  c1_info[101].mFileTimeHi = 0U;
  c1_info[102].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c1_info[102].name = "eml_index_minus";
  c1_info[102].dominantType = "int32";
  c1_info[102].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c1_info[102].fileTimeLo = 1286818778U;
  c1_info[102].fileTimeHi = 0U;
  c1_info[102].mFileTimeLo = 0U;
  c1_info[102].mFileTimeHi = 0U;
  c1_info[103].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c1_info[103].name = "eml_index_class";
  c1_info[103].dominantType = "";
  c1_info[103].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[103].fileTimeLo = 1286818778U;
  c1_info[103].fileTimeHi = 0U;
  c1_info[103].mFileTimeLo = 0U;
  c1_info[103].mFileTimeHi = 0U;
  c1_info[104].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c1_info[104].name = "eml_index_times";
  c1_info[104].dominantType = "int32";
  c1_info[104].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c1_info[104].fileTimeLo = 1286818780U;
  c1_info[104].fileTimeHi = 0U;
  c1_info[104].mFileTimeLo = 0U;
  c1_info[104].mFileTimeHi = 0U;
  c1_info[105].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c1_info[105].name = "eml_index_plus";
  c1_info[105].dominantType = "int32";
  c1_info[105].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c1_info[105].fileTimeLo = 1286818778U;
  c1_info[105].fileTimeHi = 0U;
  c1_info[105].mFileTimeLo = 0U;
  c1_info[105].mFileTimeHi = 0U;
  c1_info[106].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c1_info[106].name = "eml_int_forloop_overflow_check";
  c1_info[106].dominantType = "";
  c1_info[106].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c1_info[106].fileTimeLo = 1311255316U;
  c1_info[106].fileTimeHi = 0U;
  c1_info[106].mFileTimeLo = 0U;
  c1_info[106].mFileTimeHi = 0U;
  c1_info[107].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c1_info[107].name = "intmin";
  c1_info[107].dominantType = "char";
  c1_info[107].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/intmin.m";
  c1_info[107].fileTimeLo = 1311255318U;
  c1_info[107].fileTimeHi = 0U;
  c1_info[107].mFileTimeLo = 0U;
  c1_info[107].mFileTimeHi = 0U;
  c1_info[108].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c1_info[108].name = "eml_div";
  c1_info[108].dominantType = "double";
  c1_info[108].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_div.m";
  c1_info[108].fileTimeLo = 1313347810U;
  c1_info[108].fileTimeHi = 0U;
  c1_info[108].mFileTimeLo = 0U;
  c1_info[108].mFileTimeHi = 0U;
}

static void c1_eml_error(SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance)
{
  int32_T c1_i14;
  static char_T c1_varargin_1[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o',
    'o', 'l', 'b', 'o', 'x', ':', 's', 'q', 'r', 't', '_', 'd', 'o', 'm', 'a',
    'i', 'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c1_u[30];
  const mxArray *c1_y = NULL;
  for (c1_i14 = 0; c1_i14 < 30; c1_i14++) {
    c1_u[c1_i14] = c1_varargin_1[c1_i14];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c1_y));
}

static void c1_eml_lusolve(SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance,
  real_T c1_B[4], real_T c1_X[4])
{
  int32_T c1_i15;
  int32_T c1_i;
  int32_T c1_b_i;
  static int32_T c1_iv0[4] = { 4, 2, 4, 4 };

  int32_T c1_ip;
  real_T c1_temp;
  int32_T c1_i16;
  static real_T c1_A[16] = { -0.00028067540682168596, -0.0288, -0.0,
    -0.0015436196362295488, -0.00028067540682168596, -8.0834517164645556E-6, 1.0,
    0.10719580807149645, -0.00028067540682168596, -1.6166903432929111E-5,
    1.6166903432929111E-5, 0.10719580807149645, -0.00028067540682168596,
    -8.0834517164645556E-6, 1.6166903432929111E-5, -1.733024277506686E-6 };

  real_T c1_b_A[16];
  int32_T c1_k;
  int32_T c1_b_k;
  int32_T c1_a;
  int32_T c1_c;
  int32_T c1_b;
  int32_T c1_b_c;
  int32_T c1_b_b;
  int32_T c1_kAcol;
  int32_T c1_b_a;
  int32_T c1_c_c;
  int32_T c1_c_a;
  int32_T c1_d_c;
  int32_T c1_d_a;
  int32_T c1_e_c;
  int32_T c1_e_a;
  int32_T c1_c_b;
  int32_T c1_f_c;
  real_T c1_x;
  static real_T c1_dv3[16] = { -0.00028067540682168596, -0.0288, -0.0,
    -0.0015436196362295488, -0.00028067540682168596, -8.0834517164645556E-6, 1.0,
    0.10719580807149645, -0.00028067540682168596, -1.6166903432929111E-5,
    1.6166903432929111E-5, 0.10719580807149645, -0.00028067540682168596,
    -8.0834517164645556E-6, 1.6166903432929111E-5, -1.733024277506686E-6 };

  real_T c1_y;
  real_T c1_z;
  int32_T c1_f_a;
  int32_T c1_i17;
  int32_T c1_c_i;
  int32_T c1_d_i;
  int32_T c1_g_a;
  int32_T c1_g_c;
  int32_T c1_h_a;
  int32_T c1_h_c;
  int32_T c1_i_a;
  int32_T c1_i_c;
  int32_T c1_j_a;
  int32_T c1_d_b;
  int32_T c1_j_c;
  for (c1_i15 = 0; c1_i15 < 4; c1_i15++) {
    c1_X[c1_i15] = c1_B[c1_i15];
  }

  c1_eml_int_forloop_overflow_check(chartInstance);
  for (c1_i = 1; c1_i < 5; c1_i++) {
    c1_b_i = c1_i;
    if (c1_iv0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c1_b_i), 1, 4, 1, 0) - 1] != c1_b_i) {
      c1_ip = c1_iv0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
        ("", (real_T)c1_b_i), 1, 4, 1, 0) - 1];
      c1_temp = c1_X[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
        ("", (real_T)c1_b_i), 1, 4, 1, 0) - 1];
      c1_X[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c1_b_i), 1, 4, 1, 0) - 1] = c1_X[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c1_ip), 1, 4, 1, 0) - 1];
      c1_X[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c1_ip), 1, 4, 1, 0) - 1] = c1_temp;
    }
  }

  for (c1_i16 = 0; c1_i16 < 16; c1_i16++) {
    c1_b_A[c1_i16] = c1_A[c1_i16];
  }

  c1_b_eml_xtrsm(chartInstance, c1_b_A, c1_X);
  c1_below_threshold(chartInstance);
  c1_eml_scalar_eg(chartInstance);
  c1_c_eml_int_forloop_overflow_check(chartInstance);
  for (c1_k = 4; c1_k > 0; c1_k--) {
    c1_b_k = c1_k;
    c1_a = c1_b_k;
    c1_c = c1_a;
    c1_b = c1_c - 1;
    c1_b_c = c1_b << 2;
    c1_b_b = c1_b_c;
    c1_kAcol = c1_b_b;
    c1_b_a = c1_b_k;
    c1_c_c = c1_b_a;
    if (c1_X[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c1_c_c), 1, 4, 1, 0) - 1] != 0.0) {
      c1_c_a = c1_b_k;
      c1_d_c = c1_c_a;
      c1_d_a = c1_b_k;
      c1_e_c = c1_d_a;
      c1_e_a = c1_b_k;
      c1_c_b = c1_kAcol;
      c1_f_c = c1_e_a + c1_c_b;
      c1_x = c1_X[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c1_e_c), 1, 4, 1, 0) - 1];
      c1_y = c1_dv3[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c1_f_c), 1, 16, 1, 0) - 1];
      c1_z = c1_x / c1_y;
      c1_X[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c1_d_c), 1, 4, 1, 0) - 1] = c1_z;
      c1_f_a = c1_b_k - 1;
      c1_i17 = c1_f_a;
      c1_b_eml_int_forloop_overflow_check(chartInstance, 1, c1_i17);
      for (c1_c_i = 1; c1_c_i <= c1_i17; c1_c_i++) {
        c1_d_i = c1_c_i;
        c1_g_a = c1_d_i;
        c1_g_c = c1_g_a;
        c1_h_a = c1_d_i;
        c1_h_c = c1_h_a;
        c1_i_a = c1_b_k;
        c1_i_c = c1_i_a;
        c1_j_a = c1_d_i;
        c1_d_b = c1_kAcol;
        c1_j_c = c1_j_a + c1_d_b;
        c1_X[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c1_g_c), 1, 4, 1, 0) - 1] = c1_X[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c1_h_c), 1, 4, 1, 0) - 1]
          - c1_X[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c1_i_c), 1, 4, 1, 0) - 1] * c1_dv3[_SFD_EML_ARRAY_BOUNDS_CHECK
          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c1_j_c), 1, 16, 1, 0) - 1];
      }
    }
  }
}

static void c1_eml_int_forloop_overflow_check
  (SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance)
{
}

static void c1_b_eml_int_forloop_overflow_check
  (SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance, int32_T c1_a, int32_T
   c1_b)
{
  int32_T c1_b_a;
  int32_T c1_b_b;
  boolean_T c1_overflow;
  boolean_T c1_safe;
  int32_T c1_i18;
  static char_T c1_cv0[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c1_u[34];
  const mxArray *c1_y = NULL;
  int32_T c1_i19;
  static char_T c1_cv1[5] = { 'i', 'n', 't', '3', '2' };

  char_T c1_b_u[5];
  const mxArray *c1_b_y = NULL;
  c1_b_a = c1_a;
  c1_b_b = c1_b;
  if (c1_b_a > c1_b_b) {
    c1_overflow = FALSE;
  } else {
    c1_overflow = (c1_b_b > 2147483646);
  }

  c1_safe = !c1_overflow;
  if (c1_safe) {
  } else {
    for (c1_i18 = 0; c1_i18 < 34; c1_i18++) {
      c1_u[c1_i18] = c1_cv0[c1_i18];
    }

    c1_y = NULL;
    sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c1_i19 = 0; c1_i19 < 5; c1_i19++) {
      c1_b_u[c1_i19] = c1_cv1[c1_i19];
    }

    c1_b_y = NULL;
    sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_b_u, 10, 0U, 1U, 0U, 2, 1, 5),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c1_y, 14, c1_b_y));
  }
}

static void c1_eml_xtrsm(SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance,
  real_T c1_A[16], real_T c1_B[4], real_T c1_b_B[4])
{
  int32_T c1_i20;
  int32_T c1_i21;
  real_T c1_b_A[16];
  for (c1_i20 = 0; c1_i20 < 4; c1_i20++) {
    c1_b_B[c1_i20] = c1_B[c1_i20];
  }

  for (c1_i21 = 0; c1_i21 < 16; c1_i21++) {
    c1_b_A[c1_i21] = c1_A[c1_i21];
  }

  c1_b_eml_xtrsm(chartInstance, c1_b_A, c1_b_B);
}

static void c1_below_threshold(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance)
{
}

static void c1_eml_scalar_eg(SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance)
{
}

static void c1_c_eml_int_forloop_overflow_check
  (SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance)
{
}

static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance;
  chartInstance = (SFc1_TenzoDecV3DeltaPInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static int32_T c1_d_emlrt_marshallIn(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_y;
  int32_T c1_i22;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i22, 1, 6, 0U, 0, 0U, 0);
  c1_y = c1_i22;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y;
  SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance;
  chartInstance = (SFc1_TenzoDecV3DeltaPInstanceStruct *)chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static uint8_T c1_e_emlrt_marshallIn(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_TenzoDecV3DeltaP, const
  char_T *c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_is_active_c1_TenzoDecV3DeltaP), &c1_thisId);
  sf_mex_destroy(&c1_b_is_active_c1_TenzoDecV3DeltaP);
  return c1_y;
}

static uint8_T c1_f_emlrt_marshallIn(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u0;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_b_eml_xtrsm(SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance,
  real_T c1_A[16], real_T c1_B[4])
{
  int32_T c1_k;
  int32_T c1_b_k;
  int32_T c1_a;
  int32_T c1_c;
  int32_T c1_b;
  int32_T c1_b_c;
  int32_T c1_b_b;
  int32_T c1_kAcol;
  int32_T c1_b_a;
  int32_T c1_c_c;
  int32_T c1_c_a;
  int32_T c1_i23;
  int32_T c1_i;
  int32_T c1_b_i;
  int32_T c1_d_a;
  int32_T c1_d_c;
  int32_T c1_e_a;
  int32_T c1_e_c;
  int32_T c1_f_a;
  int32_T c1_f_c;
  int32_T c1_g_a;
  int32_T c1_c_b;
  int32_T c1_g_c;
  c1_below_threshold(chartInstance);
  c1_eml_scalar_eg(chartInstance);
  c1_eml_int_forloop_overflow_check(chartInstance);
  for (c1_k = 1; c1_k < 5; c1_k++) {
    c1_b_k = c1_k;
    c1_a = c1_b_k;
    c1_c = c1_a;
    c1_b = c1_c - 1;
    c1_b_c = c1_b << 2;
    c1_b_b = c1_b_c;
    c1_kAcol = c1_b_b;
    c1_b_a = c1_b_k;
    c1_c_c = c1_b_a;
    if (c1_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c1_c_c), 1, 4, 1, 0) - 1] != 0.0) {
      c1_c_a = c1_b_k + 1;
      c1_i23 = c1_c_a;
      c1_b_eml_int_forloop_overflow_check(chartInstance, c1_i23, 4);
      for (c1_i = c1_i23; c1_i < 5; c1_i++) {
        c1_b_i = c1_i;
        c1_d_a = c1_b_i;
        c1_d_c = c1_d_a;
        c1_e_a = c1_b_i;
        c1_e_c = c1_e_a;
        c1_f_a = c1_b_k;
        c1_f_c = c1_f_a;
        c1_g_a = c1_b_i;
        c1_c_b = c1_kAcol;
        c1_g_c = c1_g_a + c1_c_b;
        c1_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c1_d_c), 1, 4, 1, 0) - 1] = c1_B[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c1_e_c), 1, 4, 1, 0) - 1]
          - c1_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c1_f_c), 1, 4, 1, 0) - 1] * c1_A[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c1_g_c), 1, 16, 1, 0) - 1];
      }
    }
  }
}

static void init_dsm_address_info(SFc1_TenzoDecV3DeltaPInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c1_TenzoDecV3DeltaP_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3985442010U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2171431412U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3049227239U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1458161001U);
}

mxArray *sf_c1_TenzoDecV3DeltaP_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("K6Gd2iO7erbAbN7IKtUtJC");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

static const mxArray *sf_get_sim_state_info_c1_TenzoDecV3DeltaP(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x5'type','srcId','name','auxInfo'{{M[1],M[5],T\"w1\",},{M[1],M[9],T\"w2\",},{M[1],M[10],T\"w3\",},{M[1],M[11],T\"w4\",},{M[8],M[0],T\"is_active_c1_TenzoDecV3DeltaP\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 5, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_TenzoDecV3DeltaP_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance;
    chartInstance = (SFc1_TenzoDecV3DeltaPInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_TenzoDecV3DeltaPMachineNumber_,
           1,
           1,
           1,
           8,
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
          init_script_number_translation(_TenzoDecV3DeltaPMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (_TenzoDecV3DeltaPMachineNumber_,chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(_TenzoDecV3DeltaPMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"taux");
          _SFD_SET_DATA_PROPS(1,1,1,0,"tauy");
          _SFD_SET_DATA_PROPS(2,1,1,0,"tauz");
          _SFD_SET_DATA_PROPS(3,1,1,0,"Thrust");
          _SFD_SET_DATA_PROPS(4,2,0,1,"w1");
          _SFD_SET_DATA_PROPS(5,2,0,1,"w2");
          _SFD_SET_DATA_PROPS(6,2,0,1,"w3");
          _SFD_SET_DATA_PROPS(7,2,0,1,"w4");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,2739);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);

        {
          real_T *c1_taux;
          real_T *c1_tauy;
          real_T *c1_tauz;
          real_T *c1_Thrust;
          real_T *c1_w1;
          real_T *c1_w2;
          real_T *c1_w3;
          real_T *c1_w4;
          c1_w4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
          c1_w3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
          c1_w2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c1_w1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c1_Thrust = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c1_tauz = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c1_tauy = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c1_taux = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c1_taux);
          _SFD_SET_DATA_VALUE_PTR(1U, c1_tauy);
          _SFD_SET_DATA_VALUE_PTR(2U, c1_tauz);
          _SFD_SET_DATA_VALUE_PTR(3U, c1_Thrust);
          _SFD_SET_DATA_VALUE_PTR(4U, c1_w1);
          _SFD_SET_DATA_VALUE_PTR(5U, c1_w2);
          _SFD_SET_DATA_VALUE_PTR(6U, c1_w3);
          _SFD_SET_DATA_VALUE_PTR(7U, c1_w4);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(_TenzoDecV3DeltaPMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization()
{
  return "gJdQEn19CGL0AR5pXWjWVG";
}

static void sf_opaque_initialize_c1_TenzoDecV3DeltaP(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_TenzoDecV3DeltaPInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c1_TenzoDecV3DeltaP((SFc1_TenzoDecV3DeltaPInstanceStruct*)
    chartInstanceVar);
  initialize_c1_TenzoDecV3DeltaP((SFc1_TenzoDecV3DeltaPInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c1_TenzoDecV3DeltaP(void *chartInstanceVar)
{
  enable_c1_TenzoDecV3DeltaP((SFc1_TenzoDecV3DeltaPInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c1_TenzoDecV3DeltaP(void *chartInstanceVar)
{
  disable_c1_TenzoDecV3DeltaP((SFc1_TenzoDecV3DeltaPInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c1_TenzoDecV3DeltaP(void *chartInstanceVar)
{
  sf_c1_TenzoDecV3DeltaP((SFc1_TenzoDecV3DeltaPInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c1_TenzoDecV3DeltaP(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c1_TenzoDecV3DeltaP
    ((SFc1_TenzoDecV3DeltaPInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_TenzoDecV3DeltaP();/* state var info */
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

extern void sf_internal_set_sim_state_c1_TenzoDecV3DeltaP(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_TenzoDecV3DeltaP();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c1_TenzoDecV3DeltaP((SFc1_TenzoDecV3DeltaPInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c1_TenzoDecV3DeltaP(SimStruct* S)
{
  return sf_internal_get_sim_state_c1_TenzoDecV3DeltaP(S);
}

static void sf_opaque_set_sim_state_c1_TenzoDecV3DeltaP(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c1_TenzoDecV3DeltaP(S, st);
}

static void sf_opaque_terminate_c1_TenzoDecV3DeltaP(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_TenzoDecV3DeltaPInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c1_TenzoDecV3DeltaP((SFc1_TenzoDecV3DeltaPInstanceStruct*)
      chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_TenzoDecV3DeltaP_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_TenzoDecV3DeltaP((SFc1_TenzoDecV3DeltaPInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_TenzoDecV3DeltaP(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c1_TenzoDecV3DeltaP((SFc1_TenzoDecV3DeltaPInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c1_TenzoDecV3DeltaP(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_TenzoDecV3DeltaP_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,1,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,1,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,4);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1628554583U));
  ssSetChecksum1(S,(421262345U));
  ssSetChecksum2(S,(2364745737U));
  ssSetChecksum3(S,(4024565787U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c1_TenzoDecV3DeltaP(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_TenzoDecV3DeltaP(SimStruct *S)
{
  SFc1_TenzoDecV3DeltaPInstanceStruct *chartInstance;
  chartInstance = (SFc1_TenzoDecV3DeltaPInstanceStruct *)malloc(sizeof
    (SFc1_TenzoDecV3DeltaPInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc1_TenzoDecV3DeltaPInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c1_TenzoDecV3DeltaP;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c1_TenzoDecV3DeltaP;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c1_TenzoDecV3DeltaP;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c1_TenzoDecV3DeltaP;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c1_TenzoDecV3DeltaP;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c1_TenzoDecV3DeltaP;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c1_TenzoDecV3DeltaP;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c1_TenzoDecV3DeltaP;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_TenzoDecV3DeltaP;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_TenzoDecV3DeltaP;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c1_TenzoDecV3DeltaP;
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

void c1_TenzoDecV3DeltaP_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_TenzoDecV3DeltaP(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_TenzoDecV3DeltaP(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_TenzoDecV3DeltaP(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_TenzoDecV3DeltaP_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
