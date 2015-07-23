/* Include files */

#include "blascompat32.h"
#include "TenzoDecV3DeltaP_sfun.h"
#include "c5_TenzoDecV3DeltaP.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "TenzoDecV3DeltaP_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c5_debug_family_names[39] = { "g", "rho", "mq", "lx", "ly",
  "lz", "dcg", "If", "cp", "ct", "rp", "Km", "Kt", "Ktm", "k1", "k2", "k3", "k4",
  "tc", "dz1", "dz2", "dz3", "dz4", "w0c", "Ft0", "WTF", "w", "wSign", "T", "w2",
  "nargin", "nargout", "w1", "w3", "w4", "taux", "tauy", "tauz", "Thrust" };

/* Function Declarations */
static void initialize_c5_TenzoDecV3DeltaP(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance);
static void initialize_params_c5_TenzoDecV3DeltaP
  (SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance);
static void enable_c5_TenzoDecV3DeltaP(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance);
static void disable_c5_TenzoDecV3DeltaP(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance);
static void c5_update_debugger_state_c5_TenzoDecV3DeltaP
  (SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c5_TenzoDecV3DeltaP
  (SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance);
static void set_sim_state_c5_TenzoDecV3DeltaP
  (SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance, const mxArray *c5_st);
static void finalize_c5_TenzoDecV3DeltaP(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance);
static void sf_c5_TenzoDecV3DeltaP(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance);
static void c5_chartstep_c5_TenzoDecV3DeltaP(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance);
static void initSimStructsc5_TenzoDecV3DeltaP
  (SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber);
static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData);
static real_T c5_emlrt_marshallIn(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c5_Thrust, const char_T *c5_identifier);
static real_T c5_b_emlrt_marshallIn(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static void c5_c_emlrt_marshallIn(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[4]);
static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static void c5_info_helper(c5_ResolvedFunctionInfo c5_info[22]);
static real_T c5_sqrt(SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance, real_T
                      c5_x);
static void c5_eml_error(SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance);
static void c5_sign(SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance, real_T
                    c5_x[4], real_T c5_b_x[4]);
static void c5_power(SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance, real_T
                     c5_a[4], real_T c5_y[4]);
static void c5_eml_scalar_eg(SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance);
static const mxArray *c5_d_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static int32_T c5_d_emlrt_marshallIn(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static uint8_T c5_e_emlrt_marshallIn(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c5_b_is_active_c5_TenzoDecV3DeltaP, const
  char_T *c5_identifier);
static uint8_T c5_f_emlrt_marshallIn(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_b_sqrt(SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance, real_T
                      *c5_x);
static void c5_b_sign(SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance, real_T
                      c5_x[4]);
static void init_dsm_address_info(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c5_TenzoDecV3DeltaP(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance)
{
  chartInstance->c5_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c5_is_active_c5_TenzoDecV3DeltaP = 0U;
}

static void initialize_params_c5_TenzoDecV3DeltaP
  (SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance)
{
}

static void enable_c5_TenzoDecV3DeltaP(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c5_TenzoDecV3DeltaP(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c5_update_debugger_state_c5_TenzoDecV3DeltaP
  (SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c5_TenzoDecV3DeltaP
  (SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance)
{
  const mxArray *c5_st;
  const mxArray *c5_y = NULL;
  real_T c5_hoistedGlobal;
  real_T c5_u;
  const mxArray *c5_b_y = NULL;
  real_T c5_b_hoistedGlobal;
  real_T c5_b_u;
  const mxArray *c5_c_y = NULL;
  real_T c5_c_hoistedGlobal;
  real_T c5_c_u;
  const mxArray *c5_d_y = NULL;
  real_T c5_d_hoistedGlobal;
  real_T c5_d_u;
  const mxArray *c5_e_y = NULL;
  uint8_T c5_e_hoistedGlobal;
  uint8_T c5_e_u;
  const mxArray *c5_f_y = NULL;
  real_T *c5_Thrust;
  real_T *c5_taux;
  real_T *c5_tauy;
  real_T *c5_tauz;
  c5_Thrust = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c5_tauz = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c5_tauy = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c5_taux = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c5_st = NULL;
  c5_st = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_createcellarray(5), FALSE);
  c5_hoistedGlobal = *c5_Thrust;
  c5_u = c5_hoistedGlobal;
  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", &c5_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c5_y, 0, c5_b_y);
  c5_b_hoistedGlobal = *c5_taux;
  c5_b_u = c5_b_hoistedGlobal;
  c5_c_y = NULL;
  sf_mex_assign(&c5_c_y, sf_mex_create("y", &c5_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c5_y, 1, c5_c_y);
  c5_c_hoistedGlobal = *c5_tauy;
  c5_c_u = c5_c_hoistedGlobal;
  c5_d_y = NULL;
  sf_mex_assign(&c5_d_y, sf_mex_create("y", &c5_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c5_y, 2, c5_d_y);
  c5_d_hoistedGlobal = *c5_tauz;
  c5_d_u = c5_d_hoistedGlobal;
  c5_e_y = NULL;
  sf_mex_assign(&c5_e_y, sf_mex_create("y", &c5_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c5_y, 3, c5_e_y);
  c5_e_hoistedGlobal = chartInstance->c5_is_active_c5_TenzoDecV3DeltaP;
  c5_e_u = c5_e_hoistedGlobal;
  c5_f_y = NULL;
  sf_mex_assign(&c5_f_y, sf_mex_create("y", &c5_e_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c5_y, 4, c5_f_y);
  sf_mex_assign(&c5_st, c5_y, FALSE);
  return c5_st;
}

static void set_sim_state_c5_TenzoDecV3DeltaP
  (SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance, const mxArray *c5_st)
{
  const mxArray *c5_u;
  real_T *c5_Thrust;
  real_T *c5_taux;
  real_T *c5_tauy;
  real_T *c5_tauz;
  c5_Thrust = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c5_tauz = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c5_tauy = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c5_taux = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c5_doneDoubleBufferReInit = TRUE;
  c5_u = sf_mex_dup(c5_st);
  *c5_Thrust = c5_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u,
    0)), "Thrust");
  *c5_taux = c5_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u,
    1)), "taux");
  *c5_tauy = c5_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u,
    2)), "tauy");
  *c5_tauz = c5_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u,
    3)), "tauz");
  chartInstance->c5_is_active_c5_TenzoDecV3DeltaP = c5_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 4)),
     "is_active_c5_TenzoDecV3DeltaP");
  sf_mex_destroy(&c5_u);
  c5_update_debugger_state_c5_TenzoDecV3DeltaP(chartInstance);
  sf_mex_destroy(&c5_st);
}

static void finalize_c5_TenzoDecV3DeltaP(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance)
{
}

static void sf_c5_TenzoDecV3DeltaP(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance)
{
  real_T *c5_taux;
  real_T *c5_tauy;
  real_T *c5_tauz;
  real_T *c5_Thrust;
  real_T *c5_w1;
  real_T *c5_w2;
  real_T *c5_w3;
  real_T *c5_w4;
  c5_w4 = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c5_w3 = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c5_w2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c5_w1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  c5_Thrust = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c5_tauz = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c5_tauy = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c5_taux = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 4U, chartInstance->c5_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c5_taux, 0U);
  _SFD_DATA_RANGE_CHECK(*c5_tauy, 1U);
  _SFD_DATA_RANGE_CHECK(*c5_tauz, 2U);
  _SFD_DATA_RANGE_CHECK(*c5_Thrust, 3U);
  _SFD_DATA_RANGE_CHECK(*c5_w1, 4U);
  _SFD_DATA_RANGE_CHECK(*c5_w2, 5U);
  _SFD_DATA_RANGE_CHECK(*c5_w3, 6U);
  _SFD_DATA_RANGE_CHECK(*c5_w4, 7U);
  chartInstance->c5_sfEvent = CALL_EVENT;
  c5_chartstep_c5_TenzoDecV3DeltaP(chartInstance);
  sf_debug_check_for_state_inconsistency(_TenzoDecV3DeltaPMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c5_chartstep_c5_TenzoDecV3DeltaP(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance)
{
  real_T c5_hoistedGlobal;
  real_T c5_b_hoistedGlobal;
  real_T c5_c_hoistedGlobal;
  real_T c5_d_hoistedGlobal;
  real_T c5_w1;
  real_T c5_w2;
  real_T c5_w3;
  real_T c5_w4;
  uint32_T c5_debug_family_var_map[39];
  real_T c5_g;
  real_T c5_rho;
  real_T c5_mq;
  real_T c5_lx;
  real_T c5_ly;
  real_T c5_lz;
  real_T c5_dcg;
  real_T c5_If;
  real_T c5_cp;
  real_T c5_ct;
  real_T c5_rp;
  real_T c5_Km;
  real_T c5_Kt;
  real_T c5_Ktm;
  real_T c5_k1;
  real_T c5_k2;
  real_T c5_k3;
  real_T c5_k4;
  real_T c5_tc;
  real_T c5_dz1;
  real_T c5_dz2;
  real_T c5_dz3;
  real_T c5_dz4;
  real_T c5_w0c;
  real_T c5_Ft0;
  real_T c5_WTF[16];
  real_T c5_w[4];
  real_T c5_wSign[4];
  real_T c5_T[4];
  real_T c5_b_w2[4];
  real_T c5_nargin = 4.0;
  real_T c5_nargout = 4.0;
  real_T c5_taux;
  real_T c5_tauy;
  real_T c5_tauz;
  real_T c5_Thrust;
  int32_T c5_i0;
  static real_T c5_a[16] = { 0.0, -5.21511970939427E-9, 0.00067154825063034244,
    -1.8108054546507884E-7, -5.21511970939427E-9, 0.0, -0.00067154825063034244,
    -1.8108054546507884E-7, 0.0, 5.21511970939427E-9, 0.00067154825063034244,
    -1.8108054546507884E-7, 5.21511970939427E-9, 0.0, -0.00067154825063034244,
    -1.8108054546507884E-7 };

  int32_T c5_i1;
  int32_T c5_i2;
  real_T c5_b_w[4];
  real_T c5_b[4];
  int32_T c5_i3;
  int32_T c5_i4;
  int32_T c5_i5;
  int32_T c5_i6;
  int32_T c5_i7;
  real_T c5_C[4];
  int32_T c5_i8;
  int32_T c5_i9;
  int32_T c5_i10;
  int32_T c5_i11;
  int32_T c5_i12;
  int32_T c5_i13;
  real_T *c5_b_w1;
  real_T *c5_c_w2;
  real_T *c5_b_w3;
  real_T *c5_b_w4;
  real_T *c5_b_taux;
  real_T *c5_b_tauy;
  real_T *c5_b_tauz;
  real_T *c5_b_Thrust;
  c5_b_w4 = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c5_b_w3 = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c5_c_w2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c5_b_w1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  c5_b_Thrust = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c5_b_tauz = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c5_b_tauy = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c5_b_taux = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 4U, chartInstance->c5_sfEvent);
  c5_hoistedGlobal = *c5_b_w1;
  c5_b_hoistedGlobal = *c5_c_w2;
  c5_c_hoistedGlobal = *c5_b_w3;
  c5_d_hoistedGlobal = *c5_b_w4;
  c5_w1 = c5_hoistedGlobal;
  c5_w2 = c5_b_hoistedGlobal;
  c5_w3 = c5_c_hoistedGlobal;
  c5_w4 = c5_d_hoistedGlobal;
  sf_debug_symbol_scope_push_eml(0U, 39U, 40U, c5_debug_family_names,
    c5_debug_family_var_map);
  sf_debug_symbol_scope_add_eml(&c5_g, 0U, c5_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c5_rho, 1U, c5_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c5_mq, 2U, c5_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c5_lx, 3U, c5_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c5_ly, 4U, c5_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c5_lz, 5U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c5_dcg, 6U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c5_If, 7U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c5_cp, 8U, c5_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c5_ct, 9U, c5_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c5_rp, 10U, c5_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c5_Km, 11U, c5_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c5_Kt, 12U, c5_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c5_Ktm, 13U, c5_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c5_k1, 14U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c5_k2, 15U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c5_k3, 16U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c5_k4, 17U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c5_tc, 18U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c5_dz1, 19U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c5_dz2, 20U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c5_dz3, 21U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c5_dz4, 22U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c5_w0c, 23U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c5_Ft0, 24U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c5_WTF, 25U, c5_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c5_w, 26U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c5_wSign, 27U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c5_T, 28U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c5_b_w2, MAX_uint32_T,
    c5_b_sf_marshallOut, c5_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c5_nargin, 30U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c5_nargout, 31U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c5_w1, 32U, c5_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c5_w2, 29U, c5_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c5_w3, 33U, c5_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c5_w4, 34U, c5_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c5_taux, 35U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c5_tauy, 36U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c5_tauz, 37U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c5_Thrust, 38U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 4);
  c5_g = 9.81;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 5);
  c5_rho = 1.2;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 10);
  c5_mq = 0.82;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 12);
  c5_lx = 0.0288;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 14);
  c5_ly = 0.0288;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 16);
  c5_lz = 0.026;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 21);
  c5_dcg = 0.288;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 25);
  c5_If = -0.3559;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 28);
  c5_cp = 0.0743;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 29);
  c5_ct = 0.1154;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 32);
  c5_rp = 0.0254;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 35);
  c5_Km = 1.2160432353026188E-10;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 39);
  c5_Kt = 1.8108054546507884E-7;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 41);
  c5_Ktm = 0.00067154825063034244;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 42);
  c5_k1 = 2.028;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 43);
  c5_k2 = 1.869;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 44);
  c5_k3 = 2.002;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 45);
  c5_k4 = 1.996;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 46);
  c5_tc = 0.00436;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 48);
  c5_dz1 = 1247.4;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 50);
  c5_dz2 = 1247.8;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 52);
  c5_dz3 = 1246.4;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 54);
  c5_dz4 = 1247.9;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 64);
  c5_w0c = 4.442332542869059E+7;
  c5_b_sqrt(chartInstance, &c5_w0c);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 66);
  c5_Ft0 = 8.0442;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 79);
  for (c5_i0 = 0; c5_i0 < 16; c5_i0++) {
    c5_WTF[c5_i0] = c5_a[c5_i0];
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 84);
  c5_w[0] = c5_w1;
  c5_w[1] = c5_w2;
  c5_w[2] = c5_w3;
  c5_w[3] = c5_w4;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 85);
  for (c5_i1 = 0; c5_i1 < 4; c5_i1++) {
    c5_wSign[c5_i1] = c5_w[c5_i1];
  }

  c5_b_sign(chartInstance, c5_wSign);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 86);
  for (c5_i2 = 0; c5_i2 < 4; c5_i2++) {
    c5_b_w[c5_i2] = c5_w[c5_i2];
  }

  c5_power(chartInstance, c5_b_w, c5_b);
  for (c5_i3 = 0; c5_i3 < 4; c5_i3++) {
    c5_b_w2[c5_i3] = c5_wSign[c5_i3] * c5_b[c5_i3];
  }

  sf_debug_symbol_switch(29U, 29U);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 87);
  for (c5_i4 = 0; c5_i4 < 4; c5_i4++) {
    c5_b[c5_i4] = c5_b_w2[c5_i4];
  }

  c5_eml_scalar_eg(chartInstance);
  c5_eml_scalar_eg(chartInstance);
  for (c5_i5 = 0; c5_i5 < 4; c5_i5++) {
    c5_T[c5_i5] = 0.0;
  }

  for (c5_i6 = 0; c5_i6 < 4; c5_i6++) {
    c5_T[c5_i6] = 0.0;
  }

  for (c5_i7 = 0; c5_i7 < 4; c5_i7++) {
    c5_C[c5_i7] = c5_T[c5_i7];
  }

  for (c5_i8 = 0; c5_i8 < 4; c5_i8++) {
    c5_T[c5_i8] = c5_C[c5_i8];
  }

  for (c5_i9 = 0; c5_i9 < 4; c5_i9++) {
    c5_C[c5_i9] = c5_T[c5_i9];
  }

  for (c5_i10 = 0; c5_i10 < 4; c5_i10++) {
    c5_T[c5_i10] = c5_C[c5_i10];
  }

  for (c5_i11 = 0; c5_i11 < 4; c5_i11++) {
    c5_T[c5_i11] = 0.0;
    c5_i12 = 0;
    for (c5_i13 = 0; c5_i13 < 4; c5_i13++) {
      c5_T[c5_i11] += c5_a[c5_i12 + c5_i11] * c5_b[c5_i13];
      c5_i12 += 4;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 91);
  c5_taux = c5_T[0];
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 92);
  c5_tauy = c5_T[1];
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 93);
  c5_tauz = c5_T[2];
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 94);
  c5_Thrust = c5_T[3];
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, -94);
  sf_debug_symbol_scope_pop();
  *c5_b_taux = c5_taux;
  *c5_b_tauy = c5_tauy;
  *c5_b_tauz = c5_tauz;
  *c5_b_Thrust = c5_Thrust;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 4U, chartInstance->c5_sfEvent);
}

static void initSimStructsc5_TenzoDecV3DeltaP
  (SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber)
{
}

static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  real_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance;
  chartInstance = (SFc5_TenzoDecV3DeltaPInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(real_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

static real_T c5_emlrt_marshallIn(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c5_Thrust, const char_T *c5_identifier)
{
  real_T c5_y;
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_Thrust), &c5_thisId);
  sf_mex_destroy(&c5_Thrust);
  return c5_y;
}

static real_T c5_b_emlrt_marshallIn(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  real_T c5_y;
  real_T c5_d0;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_d0, 1, 0, 0U, 0, 0U, 0);
  c5_y = c5_d0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_Thrust;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y;
  SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance;
  chartInstance = (SFc5_TenzoDecV3DeltaPInstanceStruct *)chartInstanceVoid;
  c5_Thrust = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_Thrust), &c5_thisId);
  sf_mex_destroy(&c5_Thrust);
  *(real_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i14;
  real_T c5_b_inData[4];
  int32_T c5_i15;
  real_T c5_u[4];
  const mxArray *c5_y = NULL;
  SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance;
  chartInstance = (SFc5_TenzoDecV3DeltaPInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i14 = 0; c5_i14 < 4; c5_i14++) {
    c5_b_inData[c5_i14] = (*(real_T (*)[4])c5_inData)[c5_i14];
  }

  for (c5_i15 = 0; c5_i15 < 4; c5_i15++) {
    c5_u[c5_i15] = c5_b_inData[c5_i15];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

static void c5_c_emlrt_marshallIn(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[4])
{
  real_T c5_dv0[4];
  int32_T c5_i16;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_dv0, 1, 0, 0U, 1, 0U, 1, 4);
  for (c5_i16 = 0; c5_i16 < 4; c5_i16++) {
    c5_y[c5_i16] = c5_dv0[c5_i16];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_w2;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y[4];
  int32_T c5_i17;
  SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance;
  chartInstance = (SFc5_TenzoDecV3DeltaPInstanceStruct *)chartInstanceVoid;
  c5_w2 = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_w2), &c5_thisId, c5_y);
  sf_mex_destroy(&c5_w2);
  for (c5_i17 = 0; c5_i17 < 4; c5_i17++) {
    (*(real_T (*)[4])c5_outData)[c5_i17] = c5_y[c5_i17];
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i18;
  int32_T c5_i19;
  int32_T c5_i20;
  real_T c5_b_inData[16];
  int32_T c5_i21;
  int32_T c5_i22;
  int32_T c5_i23;
  real_T c5_u[16];
  const mxArray *c5_y = NULL;
  SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance;
  chartInstance = (SFc5_TenzoDecV3DeltaPInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_i18 = 0;
  for (c5_i19 = 0; c5_i19 < 4; c5_i19++) {
    for (c5_i20 = 0; c5_i20 < 4; c5_i20++) {
      c5_b_inData[c5_i20 + c5_i18] = (*(real_T (*)[16])c5_inData)[c5_i20 +
        c5_i18];
    }

    c5_i18 += 4;
  }

  c5_i21 = 0;
  for (c5_i22 = 0; c5_i22 < 4; c5_i22++) {
    for (c5_i23 = 0; c5_i23 < 4; c5_i23++) {
      c5_u[c5_i23 + c5_i21] = c5_b_inData[c5_i23 + c5_i21];
    }

    c5_i21 += 4;
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

const mxArray *sf_c5_TenzoDecV3DeltaP_get_eml_resolved_functions_info(void)
{
  const mxArray *c5_nameCaptureInfo;
  c5_ResolvedFunctionInfo c5_info[22];
  const mxArray *c5_m0 = NULL;
  int32_T c5_i24;
  c5_ResolvedFunctionInfo *c5_r0;
  c5_nameCaptureInfo = NULL;
  c5_nameCaptureInfo = NULL;
  c5_info_helper(c5_info);
  sf_mex_assign(&c5_m0, sf_mex_createstruct("nameCaptureInfo", 1, 22), FALSE);
  for (c5_i24 = 0; c5_i24 < 22; c5_i24++) {
    c5_r0 = &c5_info[c5_i24];
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c5_r0->context)), "context", "nameCaptureInfo",
                    c5_i24);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c5_r0->name)), "name", "nameCaptureInfo", c5_i24);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c5_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c5_i24);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c5_r0->resolved)), "resolved", "nameCaptureInfo",
                    c5_i24);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", &c5_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c5_i24);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", &c5_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c5_i24);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", &c5_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c5_i24);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", &c5_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c5_i24);
  }

  sf_mex_assign(&c5_nameCaptureInfo, c5_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c5_nameCaptureInfo);
  return c5_nameCaptureInfo;
}

static void c5_info_helper(c5_ResolvedFunctionInfo c5_info[22])
{
  c5_info[0].context = "";
  c5_info[0].name = "mtimes";
  c5_info[0].dominantType = "double";
  c5_info[0].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[0].fileTimeLo = 1289519692U;
  c5_info[0].fileTimeHi = 0U;
  c5_info[0].mFileTimeLo = 0U;
  c5_info[0].mFileTimeHi = 0U;
  c5_info[1].context = "";
  c5_info[1].name = "mpower";
  c5_info[1].dominantType = "double";
  c5_info[1].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mpower.m";
  c5_info[1].fileTimeLo = 1286818842U;
  c5_info[1].fileTimeHi = 0U;
  c5_info[1].mFileTimeLo = 0U;
  c5_info[1].mFileTimeHi = 0U;
  c5_info[2].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mpower.m";
  c5_info[2].name = "power";
  c5_info[2].dominantType = "double";
  c5_info[2].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/power.m";
  c5_info[2].fileTimeLo = 1307651240U;
  c5_info[2].fileTimeHi = 0U;
  c5_info[2].mFileTimeLo = 0U;
  c5_info[2].mFileTimeHi = 0U;
  c5_info[3].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/power.m";
  c5_info[3].name = "eml_scalar_eg";
  c5_info[3].dominantType = "double";
  c5_info[3].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c5_info[3].fileTimeLo = 1286818796U;
  c5_info[3].fileTimeHi = 0U;
  c5_info[3].mFileTimeLo = 0U;
  c5_info[3].mFileTimeHi = 0U;
  c5_info[4].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/power.m";
  c5_info[4].name = "eml_scalexp_alloc";
  c5_info[4].dominantType = "double";
  c5_info[4].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c5_info[4].fileTimeLo = 1286818796U;
  c5_info[4].fileTimeHi = 0U;
  c5_info[4].mFileTimeLo = 0U;
  c5_info[4].mFileTimeHi = 0U;
  c5_info[5].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/power.m";
  c5_info[5].name = "eml_scalar_floor";
  c5_info[5].dominantType = "double";
  c5_info[5].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c5_info[5].fileTimeLo = 1286818726U;
  c5_info[5].fileTimeHi = 0U;
  c5_info[5].mFileTimeLo = 0U;
  c5_info[5].mFileTimeHi = 0U;
  c5_info[6].context = "";
  c5_info[6].name = "mrdivide";
  c5_info[6].dominantType = "double";
  c5_info[6].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c5_info[6].fileTimeLo = 1325124138U;
  c5_info[6].fileTimeHi = 0U;
  c5_info[6].mFileTimeLo = 1319729966U;
  c5_info[6].mFileTimeHi = 0U;
  c5_info[7].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c5_info[7].name = "rdivide";
  c5_info[7].dominantType = "double";
  c5_info[7].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/rdivide.m";
  c5_info[7].fileTimeLo = 1286818844U;
  c5_info[7].fileTimeHi = 0U;
  c5_info[7].mFileTimeLo = 0U;
  c5_info[7].mFileTimeHi = 0U;
  c5_info[8].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/rdivide.m";
  c5_info[8].name = "eml_div";
  c5_info[8].dominantType = "double";
  c5_info[8].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_div.m";
  c5_info[8].fileTimeLo = 1313347810U;
  c5_info[8].fileTimeHi = 0U;
  c5_info[8].mFileTimeLo = 0U;
  c5_info[8].mFileTimeHi = 0U;
  c5_info[9].context = "";
  c5_info[9].name = "sqrt";
  c5_info[9].dominantType = "double";
  c5_info[9].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c5_info[9].fileTimeLo = 1286818752U;
  c5_info[9].fileTimeHi = 0U;
  c5_info[9].mFileTimeLo = 0U;
  c5_info[9].mFileTimeHi = 0U;
  c5_info[10].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c5_info[10].name = "eml_error";
  c5_info[10].dominantType = "char";
  c5_info[10].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_error.m";
  c5_info[10].fileTimeLo = 1305318000U;
  c5_info[10].fileTimeHi = 0U;
  c5_info[10].mFileTimeLo = 0U;
  c5_info[10].mFileTimeHi = 0U;
  c5_info[11].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c5_info[11].name = "eml_scalar_sqrt";
  c5_info[11].dominantType = "double";
  c5_info[11].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c5_info[11].fileTimeLo = 1286818738U;
  c5_info[11].fileTimeHi = 0U;
  c5_info[11].mFileTimeLo = 0U;
  c5_info[11].mFileTimeHi = 0U;
  c5_info[12].context = "";
  c5_info[12].name = "sign";
  c5_info[12].dominantType = "double";
  c5_info[12].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/sign.m";
  c5_info[12].fileTimeLo = 1286818750U;
  c5_info[12].fileTimeHi = 0U;
  c5_info[12].mFileTimeLo = 0U;
  c5_info[12].mFileTimeHi = 0U;
  c5_info[13].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/sign.m";
  c5_info[13].name = "eml_scalar_sign";
  c5_info[13].dominantType = "double";
  c5_info[13].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/eml_scalar_sign.m";
  c5_info[13].fileTimeLo = 1307651238U;
  c5_info[13].fileTimeHi = 0U;
  c5_info[13].mFileTimeLo = 0U;
  c5_info[13].mFileTimeHi = 0U;
  c5_info[14].context = "";
  c5_info[14].name = "power";
  c5_info[14].dominantType = "double";
  c5_info[14].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/power.m";
  c5_info[14].fileTimeLo = 1307651240U;
  c5_info[14].fileTimeHi = 0U;
  c5_info[14].mFileTimeLo = 0U;
  c5_info[14].mFileTimeHi = 0U;
  c5_info[15].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[15].name = "eml_index_class";
  c5_info[15].dominantType = "";
  c5_info[15].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c5_info[15].fileTimeLo = 1286818778U;
  c5_info[15].fileTimeHi = 0U;
  c5_info[15].mFileTimeLo = 0U;
  c5_info[15].mFileTimeHi = 0U;
  c5_info[16].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[16].name = "eml_scalar_eg";
  c5_info[16].dominantType = "double";
  c5_info[16].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c5_info[16].fileTimeLo = 1286818796U;
  c5_info[16].fileTimeHi = 0U;
  c5_info[16].mFileTimeLo = 0U;
  c5_info[16].mFileTimeHi = 0U;
  c5_info[17].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[17].name = "eml_xgemm";
  c5_info[17].dominantType = "int32";
  c5_info[17].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c5_info[17].fileTimeLo = 1299076772U;
  c5_info[17].fileTimeHi = 0U;
  c5_info[17].mFileTimeLo = 0U;
  c5_info[17].mFileTimeHi = 0U;
  c5_info[18].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c5_info[18].name = "eml_blas_inline";
  c5_info[18].dominantType = "";
  c5_info[18].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c5_info[18].fileTimeLo = 1299076768U;
  c5_info[18].fileTimeHi = 0U;
  c5_info[18].mFileTimeLo = 0U;
  c5_info[18].mFileTimeHi = 0U;
  c5_info[19].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c5_info[19].name = "mtimes";
  c5_info[19].dominantType = "double";
  c5_info[19].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[19].fileTimeLo = 1289519692U;
  c5_info[19].fileTimeHi = 0U;
  c5_info[19].mFileTimeLo = 0U;
  c5_info[19].mFileTimeHi = 0U;
  c5_info[20].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c5_info[20].name = "eml_scalar_eg";
  c5_info[20].dominantType = "double";
  c5_info[20].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c5_info[20].fileTimeLo = 1286818796U;
  c5_info[20].fileTimeHi = 0U;
  c5_info[20].mFileTimeLo = 0U;
  c5_info[20].mFileTimeHi = 0U;
  c5_info[21].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c5_info[21].name = "eml_refblas_xgemm";
  c5_info[21].dominantType = "int32";
  c5_info[21].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c5_info[21].fileTimeLo = 1299076774U;
  c5_info[21].fileTimeHi = 0U;
  c5_info[21].mFileTimeLo = 0U;
  c5_info[21].mFileTimeHi = 0U;
}

static real_T c5_sqrt(SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance, real_T
                      c5_x)
{
  real_T c5_b_x;
  c5_b_x = c5_x;
  c5_b_sqrt(chartInstance, &c5_b_x);
  return c5_b_x;
}

static void c5_eml_error(SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance)
{
  int32_T c5_i25;
  static char_T c5_varargin_1[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o',
    'o', 'l', 'b', 'o', 'x', ':', 's', 'q', 'r', 't', '_', 'd', 'o', 'm', 'a',
    'i', 'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c5_u[30];
  const mxArray *c5_y = NULL;
  for (c5_i25 = 0; c5_i25 < 30; c5_i25++) {
    c5_u[c5_i25] = c5_varargin_1[c5_i25];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c5_y));
}

static void c5_sign(SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance, real_T
                    c5_x[4], real_T c5_b_x[4])
{
  int32_T c5_i26;
  for (c5_i26 = 0; c5_i26 < 4; c5_i26++) {
    c5_b_x[c5_i26] = c5_x[c5_i26];
  }

  c5_b_sign(chartInstance, c5_b_x);
}

static void c5_power(SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance, real_T
                     c5_a[4], real_T c5_y[4])
{
  int32_T c5_k;
  real_T c5_b_k;
  real_T c5_ak;
  for (c5_k = 0; c5_k < 4; c5_k++) {
    c5_b_k = 1.0 + (real_T)c5_k;
    c5_ak = c5_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c5_b_k), 1, 4, 1, 0) - 1];
    c5_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c5_b_k),
      1, 4, 1, 0) - 1] = muDoubleScalarPower(c5_ak, 2.0);
  }
}

static void c5_eml_scalar_eg(SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance)
{
}

static const mxArray *c5_d_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance;
  chartInstance = (SFc5_TenzoDecV3DeltaPInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(int32_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

static int32_T c5_d_emlrt_marshallIn(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  int32_T c5_y;
  int32_T c5_i27;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_i27, 1, 6, 0U, 0, 0U, 0);
  c5_y = c5_i27;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_b_sfEvent;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  int32_T c5_y;
  SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance;
  chartInstance = (SFc5_TenzoDecV3DeltaPInstanceStruct *)chartInstanceVoid;
  c5_b_sfEvent = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_b_sfEvent),
    &c5_thisId);
  sf_mex_destroy(&c5_b_sfEvent);
  *(int32_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static uint8_T c5_e_emlrt_marshallIn(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c5_b_is_active_c5_TenzoDecV3DeltaP, const
  char_T *c5_identifier)
{
  uint8_T c5_y;
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c5_b_is_active_c5_TenzoDecV3DeltaP), &c5_thisId);
  sf_mex_destroy(&c5_b_is_active_c5_TenzoDecV3DeltaP);
  return c5_y;
}

static uint8_T c5_f_emlrt_marshallIn(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  uint8_T c5_y;
  uint8_T c5_u0;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_u0, 1, 3, 0U, 0, 0U, 0);
  c5_y = c5_u0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_b_sqrt(SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance, real_T
                      *c5_x)
{
  if (*c5_x < 0.0) {
    c5_eml_error(chartInstance);
  }

  *c5_x = muDoubleScalarSqrt(*c5_x);
}

static void c5_b_sign(SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance, real_T
                      c5_x[4])
{
  int32_T c5_k;
  real_T c5_b_k;
  real_T c5_b_x;
  real_T c5_c_x;
  for (c5_k = 0; c5_k < 4; c5_k++) {
    c5_b_k = 1.0 + (real_T)c5_k;
    c5_b_x = c5_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c5_b_k), 1, 4, 1, 0) - 1];
    c5_c_x = c5_b_x;
    c5_c_x = muDoubleScalarSign(c5_c_x);
    c5_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c5_b_k),
      1, 4, 1, 0) - 1] = c5_c_x;
  }
}

static void init_dsm_address_info(SFc5_TenzoDecV3DeltaPInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c5_TenzoDecV3DeltaP_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1564522948U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2073605207U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2156105772U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2996627944U);
}

mxArray *sf_c5_TenzoDecV3DeltaP_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("E5jkJ1eUXmSVidqrkSwYrF");
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

static const mxArray *sf_get_sim_state_info_c5_TenzoDecV3DeltaP(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x5'type','srcId','name','auxInfo'{{M[1],M[4],T\"Thrust\",},{M[1],M[6],T\"taux\",},{M[1],M[7],T\"tauy\",},{M[1],M[8],T\"tauz\",},{M[8],M[0],T\"is_active_c5_TenzoDecV3DeltaP\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 5, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c5_TenzoDecV3DeltaP_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance;
    chartInstance = (SFc5_TenzoDecV3DeltaPInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_TenzoDecV3DeltaPMachineNumber_,
           5,
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
          _SFD_SET_DATA_PROPS(0,2,0,1,"taux");
          _SFD_SET_DATA_PROPS(1,2,0,1,"tauy");
          _SFD_SET_DATA_PROPS(2,2,0,1,"tauz");
          _SFD_SET_DATA_PROPS(3,2,0,1,"Thrust");
          _SFD_SET_DATA_PROPS(4,1,1,0,"w1");
          _SFD_SET_DATA_PROPS(5,1,1,0,"w2");
          _SFD_SET_DATA_PROPS(6,1,1,0,"w3");
          _SFD_SET_DATA_PROPS(7,1,1,0,"w4");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,2722);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c5_taux;
          real_T *c5_tauy;
          real_T *c5_tauz;
          real_T *c5_Thrust;
          real_T *c5_w1;
          real_T *c5_w2;
          real_T *c5_w3;
          real_T *c5_w4;
          c5_w4 = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c5_w3 = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c5_w2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c5_w1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          c5_Thrust = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
          c5_tauz = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
          c5_tauy = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c5_taux = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          _SFD_SET_DATA_VALUE_PTR(0U, c5_taux);
          _SFD_SET_DATA_VALUE_PTR(1U, c5_tauy);
          _SFD_SET_DATA_VALUE_PTR(2U, c5_tauz);
          _SFD_SET_DATA_VALUE_PTR(3U, c5_Thrust);
          _SFD_SET_DATA_VALUE_PTR(4U, c5_w1);
          _SFD_SET_DATA_VALUE_PTR(5U, c5_w2);
          _SFD_SET_DATA_VALUE_PTR(6U, c5_w3);
          _SFD_SET_DATA_VALUE_PTR(7U, c5_w4);
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
  return "Rd5lrvCJ9NxmrEoHKSNsoE";
}

static void sf_opaque_initialize_c5_TenzoDecV3DeltaP(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc5_TenzoDecV3DeltaPInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c5_TenzoDecV3DeltaP((SFc5_TenzoDecV3DeltaPInstanceStruct*)
    chartInstanceVar);
  initialize_c5_TenzoDecV3DeltaP((SFc5_TenzoDecV3DeltaPInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c5_TenzoDecV3DeltaP(void *chartInstanceVar)
{
  enable_c5_TenzoDecV3DeltaP((SFc5_TenzoDecV3DeltaPInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c5_TenzoDecV3DeltaP(void *chartInstanceVar)
{
  disable_c5_TenzoDecV3DeltaP((SFc5_TenzoDecV3DeltaPInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c5_TenzoDecV3DeltaP(void *chartInstanceVar)
{
  sf_c5_TenzoDecV3DeltaP((SFc5_TenzoDecV3DeltaPInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c5_TenzoDecV3DeltaP(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c5_TenzoDecV3DeltaP
    ((SFc5_TenzoDecV3DeltaPInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c5_TenzoDecV3DeltaP();/* state var info */
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

extern void sf_internal_set_sim_state_c5_TenzoDecV3DeltaP(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c5_TenzoDecV3DeltaP();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c5_TenzoDecV3DeltaP((SFc5_TenzoDecV3DeltaPInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c5_TenzoDecV3DeltaP(SimStruct* S)
{
  return sf_internal_get_sim_state_c5_TenzoDecV3DeltaP(S);
}

static void sf_opaque_set_sim_state_c5_TenzoDecV3DeltaP(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c5_TenzoDecV3DeltaP(S, st);
}

static void sf_opaque_terminate_c5_TenzoDecV3DeltaP(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc5_TenzoDecV3DeltaPInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c5_TenzoDecV3DeltaP((SFc5_TenzoDecV3DeltaPInstanceStruct*)
      chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_TenzoDecV3DeltaP_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc5_TenzoDecV3DeltaP((SFc5_TenzoDecV3DeltaPInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c5_TenzoDecV3DeltaP(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c5_TenzoDecV3DeltaP((SFc5_TenzoDecV3DeltaPInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c5_TenzoDecV3DeltaP(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_TenzoDecV3DeltaP_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      5);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,5,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,5,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,5,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,5,4);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,5);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3845986172U));
  ssSetChecksum1(S,(612150537U));
  ssSetChecksum2(S,(3858348162U));
  ssSetChecksum3(S,(344823699U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c5_TenzoDecV3DeltaP(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c5_TenzoDecV3DeltaP(SimStruct *S)
{
  SFc5_TenzoDecV3DeltaPInstanceStruct *chartInstance;
  chartInstance = (SFc5_TenzoDecV3DeltaPInstanceStruct *)malloc(sizeof
    (SFc5_TenzoDecV3DeltaPInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc5_TenzoDecV3DeltaPInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c5_TenzoDecV3DeltaP;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c5_TenzoDecV3DeltaP;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c5_TenzoDecV3DeltaP;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c5_TenzoDecV3DeltaP;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c5_TenzoDecV3DeltaP;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c5_TenzoDecV3DeltaP;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c5_TenzoDecV3DeltaP;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c5_TenzoDecV3DeltaP;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c5_TenzoDecV3DeltaP;
  chartInstance->chartInfo.mdlStart = mdlStart_c5_TenzoDecV3DeltaP;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c5_TenzoDecV3DeltaP;
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

void c5_TenzoDecV3DeltaP_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c5_TenzoDecV3DeltaP(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c5_TenzoDecV3DeltaP(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c5_TenzoDecV3DeltaP(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c5_TenzoDecV3DeltaP_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
