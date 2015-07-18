/* Include files */

#include "blascompat32.h"
#include "progetto3TenzoDecoupleV2_sfun.h"
#include "c2_progetto3TenzoDecoupleV2.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "progetto3TenzoDecoupleV2_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c2_debug_family_names[38] = { "g", "rho", "mq", "lx", "ly",
  "lz", "dcg", "If", "cp", "ct", "rp", "Km", "Kt", "Ktm", "k1", "k2", "k3", "k4",
  "tc", "dz1", "dz2", "dz3", "dz4", "PWM_max", "w0c", "Ft0", "WTF", "T",
  "nargin", "nargout", "taux", "tauy", "tauz", "Thrust", "w1", "w2", "w3", "w4"
};

/* Function Declarations */
static void initialize_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance);
static void initialize_params_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance);
static void enable_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance);
static void disable_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance);
static void set_sim_state_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance, const mxArray
   *c2_st);
static void finalize_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance);
static void sf_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance);
static void c2_chartstep_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance);
static void initSimStructsc2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static real_T c2_emlrt_marshallIn(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, const mxArray *c2_w4, const char_T *c2_identifier);
static real_T c2_b_emlrt_marshallIn(SFc2_progetto3TenzoDecoupleV2InstanceStruct *
  chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_c_emlrt_marshallIn(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4]);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[136]);
static void c2_b_info_helper(c2_ResolvedFunctionInfo c2_info[136]);
static void c2_c_info_helper(c2_ResolvedFunctionInfo c2_info[136]);
static real_T c2_sqrt(SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance,
                      real_T c2_x);
static void c2_eml_error(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance);
static void c2_eml_scalar_eg(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance);
static void c2_b_eml_scalar_eg(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance);
static void c2_realmin(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance);
static void c2_eps(SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance);
static void c2_eml_int_forloop_overflow_check
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance);
static void c2_mpower(SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance,
                      real_T c2_c[16]);
static void c2_matrix_to_integer_power
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance, real_T c2_c[16]);
static void c2_invNxN(SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance,
                      real_T c2_x[16], real_T c2_y[16]);
static void c2_eml_matlab_zgetrf(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_b_A[16], int32_T c2_ipiv[4],
  int32_T *c2_info);
static void c2_b_eml_int_forloop_overflow_check
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance, int32_T c2_b);
static void c2_c_eml_int_forloop_overflow_check
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance);
static void c2_d_eml_int_forloop_overflow_check
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance, int32_T c2_a,
   int32_T c2_b);
static void c2_eml_xgeru(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0,
  int32_T c2_iy0, real_T c2_A[16], int32_T c2_ia0, real_T c2_b_A[16]);
static void c2_eml_xtrsm(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_B[16], real_T c2_b_B[16]);
static void c2_e_eml_int_forloop_overflow_check
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance);
static real_T c2_norm(SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance,
                      real_T c2_x[16]);
static void c2_eml_warning(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance);
static void c2_b_eml_warning(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, char_T c2_varargin_2[14]);
static void c2_c_eml_scalar_eg(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance);
static void c2_d_emlrt_marshallIn(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, const mxArray *c2_sprintf, const char_T *c2_identifier, char_T
  c2_y[14]);
static void c2_e_emlrt_marshallIn(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  char_T c2_y[14]);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_f_emlrt_marshallIn(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_g_emlrt_marshallIn(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_progetto3TenzoDecoupleV2,
  const char_T *c2_identifier);
static uint8_T c2_h_emlrt_marshallIn(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sqrt(SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance,
                      real_T *c2_x);
static void c2_b_eml_matlab_zgetrf(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, real_T c2_A[16], int32_T c2_ipiv[4], int32_T *c2_info);
static void c2_b_eml_xgeru(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0,
  int32_T c2_iy0, real_T c2_A[16], int32_T c2_ia0);
static void c2_b_eml_xtrsm(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_B[16]);
static void init_dsm_address_info(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_is_active_c2_progetto3TenzoDecoupleV2 = 0U;
}

static void initialize_params_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance)
{
}

static void enable_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  real_T c2_hoistedGlobal;
  real_T c2_u;
  const mxArray *c2_b_y = NULL;
  real_T c2_b_hoistedGlobal;
  real_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  real_T c2_c_hoistedGlobal;
  real_T c2_c_u;
  const mxArray *c2_d_y = NULL;
  real_T c2_d_hoistedGlobal;
  real_T c2_d_u;
  const mxArray *c2_e_y = NULL;
  uint8_T c2_e_hoistedGlobal;
  uint8_T c2_e_u;
  const mxArray *c2_f_y = NULL;
  real_T *c2_w1;
  real_T *c2_w2;
  real_T *c2_w3;
  real_T *c2_w4;
  c2_w4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_w3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_w2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_w1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(5), FALSE);
  c2_hoistedGlobal = *c2_w1;
  c2_u = c2_hoistedGlobal;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_b_hoistedGlobal = *c2_w2;
  c2_b_u = c2_b_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_c_hoistedGlobal = *c2_w3;
  c2_c_u = c2_c_hoistedGlobal;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 2, c2_d_y);
  c2_d_hoistedGlobal = *c2_w4;
  c2_d_u = c2_d_hoistedGlobal;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 3, c2_e_y);
  c2_e_hoistedGlobal = chartInstance->c2_is_active_c2_progetto3TenzoDecoupleV2;
  c2_e_u = c2_e_hoistedGlobal;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_e_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 4, c2_f_y);
  sf_mex_assign(&c2_st, c2_y, FALSE);
  return c2_st;
}

static void set_sim_state_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance, const mxArray
   *c2_st)
{
  const mxArray *c2_u;
  real_T *c2_w1;
  real_T *c2_w2;
  real_T *c2_w3;
  real_T *c2_w4;
  c2_w4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_w3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_w2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_w1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  *c2_w1 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)),
    "w1");
  *c2_w2 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 1)),
    "w2");
  *c2_w3 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 2)),
    "w3");
  *c2_w4 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 3)),
    "w4");
  chartInstance->c2_is_active_c2_progetto3TenzoDecoupleV2 =
    c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 4)),
    "is_active_c2_progetto3TenzoDecoupleV2");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_progetto3TenzoDecoupleV2(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance)
{
}

static void sf_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance)
{
  real_T *c2_taux;
  real_T *c2_tauy;
  real_T *c2_tauz;
  real_T *c2_Thrust;
  real_T *c2_w1;
  real_T *c2_w2;
  real_T *c2_w3;
  real_T *c2_w4;
  c2_w4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_w3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_w2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_w1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_Thrust = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c2_tauz = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c2_tauy = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_taux = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c2_taux, 0U);
  _SFD_DATA_RANGE_CHECK(*c2_tauy, 1U);
  _SFD_DATA_RANGE_CHECK(*c2_tauz, 2U);
  _SFD_DATA_RANGE_CHECK(*c2_Thrust, 3U);
  _SFD_DATA_RANGE_CHECK(*c2_w1, 4U);
  _SFD_DATA_RANGE_CHECK(*c2_w2, 5U);
  _SFD_DATA_RANGE_CHECK(*c2_w3, 6U);
  _SFD_DATA_RANGE_CHECK(*c2_w4, 7U);
  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_progetto3TenzoDecoupleV2(chartInstance);
  sf_debug_check_for_state_inconsistency(_progetto3TenzoDecoupleV2MachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c2_chartstep_c2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance)
{
  real_T c2_hoistedGlobal;
  real_T c2_b_hoistedGlobal;
  real_T c2_c_hoistedGlobal;
  real_T c2_d_hoistedGlobal;
  real_T c2_taux;
  real_T c2_tauy;
  real_T c2_tauz;
  real_T c2_Thrust;
  uint32_T c2_debug_family_var_map[38];
  real_T c2_g;
  real_T c2_rho;
  real_T c2_mq;
  real_T c2_lx;
  real_T c2_ly;
  real_T c2_lz;
  real_T c2_dcg;
  real_T c2_If;
  real_T c2_cp;
  real_T c2_ct;
  real_T c2_rp;
  real_T c2_Km;
  real_T c2_Kt;
  real_T c2_Ktm;
  real_T c2_k1;
  real_T c2_k2;
  real_T c2_k3;
  real_T c2_k4;
  real_T c2_tc;
  real_T c2_dz1;
  real_T c2_dz2;
  real_T c2_dz3;
  real_T c2_dz4;
  real_T c2_PWM_max;
  real_T c2_w0c;
  real_T c2_Ft0;
  real_T c2_WTF[16];
  real_T c2_T[4];
  real_T c2_nargin = 4.0;
  real_T c2_nargout = 4.0;
  real_T c2_w1;
  real_T c2_w2;
  real_T c2_w3;
  real_T c2_w4;
  int32_T c2_i0;
  static real_T c2_dv0[16] = { 0.0, 8.0834517164645556E-6, 4.3325606937667151E-7,
    -0.00028067540682168596, -8.0834517164645556E-6, 0.0, -4.3325606937667151E-7,
    -0.00028067540682168596, 0.0, -8.0834517164645556E-6, 4.3325606937667151E-7,
    -0.00028067540682168596, 8.0834517164645556E-6, 0.0, -4.3325606937667151E-7,
    -0.00028067540682168596 };

  real_T c2_a[16];
  real_T c2_b[4];
  int32_T c2_i1;
  int32_T c2_i2;
  int32_T c2_i3;
  real_T c2_C[4];
  int32_T c2_i4;
  int32_T c2_i5;
  int32_T c2_i6;
  int32_T c2_i7;
  int32_T c2_i8;
  int32_T c2_i9;
  real_T *c2_b_taux;
  real_T *c2_b_tauy;
  real_T *c2_b_tauz;
  real_T *c2_b_Thrust;
  real_T *c2_b_w1;
  real_T *c2_b_w2;
  real_T *c2_b_w3;
  real_T *c2_b_w4;
  c2_b_w4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_b_w3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_b_w2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_b_w1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_b_Thrust = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c2_b_tauz = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c2_b_tauy = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_b_taux = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  c2_hoistedGlobal = *c2_b_taux;
  c2_b_hoistedGlobal = *c2_b_tauy;
  c2_c_hoistedGlobal = *c2_b_tauz;
  c2_d_hoistedGlobal = *c2_b_Thrust;
  c2_taux = c2_hoistedGlobal;
  c2_tauy = c2_b_hoistedGlobal;
  c2_tauz = c2_c_hoistedGlobal;
  c2_Thrust = c2_d_hoistedGlobal;
  sf_debug_symbol_scope_push_eml(0U, 38U, 38U, c2_debug_family_names,
    c2_debug_family_var_map);
  sf_debug_symbol_scope_add_eml(&c2_g, 0U, c2_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c2_rho, 1U, c2_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c2_mq, 2U, c2_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c2_lx, 3U, c2_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c2_ly, 4U, c2_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c2_lz, 5U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_dcg, 6U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_If, 7U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c2_cp, 8U, c2_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c2_ct, 9U, c2_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c2_rp, 10U, c2_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c2_Km, 11U, c2_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c2_Kt, 12U, c2_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c2_Ktm, 13U, c2_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c2_k1, 14U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_k2, 15U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_k3, 16U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_k4, 17U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_tc, 18U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_dz1, 19U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_dz2, 20U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_dz3, 21U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_dz4, 22U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_PWM_max, 23U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_w0c, 24U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_Ft0, 25U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c2_WTF, 26U, c2_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c2_T, 27U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargin, 28U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargout, 29U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c2_taux, 30U, c2_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c2_tauy, 31U, c2_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c2_tauz, 32U, c2_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c2_Thrust, 33U, c2_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c2_w1, 34U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_w2, 35U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_w3, 36U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_w4, 37U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 4);
  c2_g = 9.81;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 5);
  c2_rho = 1.2;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 10);
  c2_mq = 0.82;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 12);
  c2_lx = 0.0288;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 14);
  c2_ly = 0.0288;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 16);
  c2_lz = 0.026;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 21);
  c2_dcg = 0.288;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 25);
  c2_If = -0.3559;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 28);
  c2_cp = 0.0743;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 29);
  c2_ct = 0.1154;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 32);
  c2_rp = 0.0254;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 35);
  c2_Km = 1.2160432353026188E-10;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 39);
  c2_Kt = 0.00028067540682168596;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 41);
  c2_Ktm = 4.3325606937667151E-7;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 42);
  c2_k1 = 2.028;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 43);
  c2_k2 = 1.869;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 44);
  c2_k3 = 2.002;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 45);
  c2_k4 = 1.996;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 46);
  c2_tc = 0.436;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 48);
  c2_dz1 = 1247.4;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 50);
  c2_dz2 = 1247.8;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 52);
  c2_dz3 = 1246.4;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 54);
  c2_dz4 = 1247.9;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 63);
  c2_PWM_max = 2200.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 65);
  c2_w0c = 28660.152633574013;
  c2_b_sqrt(chartInstance, &c2_w0c);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 67);
  c2_Ft0 = 8.0442;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 80);
  for (c2_i0 = 0; c2_i0 < 16; c2_i0++) {
    c2_WTF[c2_i0] = c2_dv0[c2_i0];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 86);
  c2_mpower(chartInstance, c2_a);
  c2_b[0] = c2_taux;
  c2_b[1] = c2_tauy;
  c2_b[2] = c2_tauz;
  c2_b[3] = c2_Thrust;
  c2_c_eml_scalar_eg(chartInstance);
  c2_c_eml_scalar_eg(chartInstance);
  for (c2_i1 = 0; c2_i1 < 4; c2_i1++) {
    c2_T[c2_i1] = 0.0;
  }

  for (c2_i2 = 0; c2_i2 < 4; c2_i2++) {
    c2_T[c2_i2] = 0.0;
  }

  for (c2_i3 = 0; c2_i3 < 4; c2_i3++) {
    c2_C[c2_i3] = c2_T[c2_i3];
  }

  for (c2_i4 = 0; c2_i4 < 4; c2_i4++) {
    c2_T[c2_i4] = c2_C[c2_i4];
  }

  for (c2_i5 = 0; c2_i5 < 4; c2_i5++) {
    c2_C[c2_i5] = c2_T[c2_i5];
  }

  for (c2_i6 = 0; c2_i6 < 4; c2_i6++) {
    c2_T[c2_i6] = c2_C[c2_i6];
  }

  for (c2_i7 = 0; c2_i7 < 4; c2_i7++) {
    c2_T[c2_i7] = 0.0;
    c2_i8 = 0;
    for (c2_i9 = 0; c2_i9 < 4; c2_i9++) {
      c2_T[c2_i7] += c2_a[c2_i8 + c2_i7] * c2_b[c2_i9];
      c2_i8 += 4;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 90);
  c2_w1 = c2_T[0];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 91);
  c2_w2 = c2_T[1];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 92);
  c2_w3 = c2_T[2];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 93);
  c2_w4 = c2_T[3];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -93);
  sf_debug_symbol_scope_pop();
  *c2_b_w1 = c2_w1;
  *c2_b_w2 = c2_w2;
  *c2_b_w3 = c2_w3;
  *c2_b_w4 = c2_w4;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_progetto3TenzoDecoupleV2
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance;
  chartInstance = (SFc2_progetto3TenzoDecoupleV2InstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_emlrt_marshallIn(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, const mxArray *c2_w4, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_w4), &c2_thisId);
  sf_mex_destroy(&c2_w4);
  return c2_y;
}

static real_T c2_b_emlrt_marshallIn(SFc2_progetto3TenzoDecoupleV2InstanceStruct *
  chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_w4;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance;
  chartInstance = (SFc2_progetto3TenzoDecoupleV2InstanceStruct *)
    chartInstanceVoid;
  c2_w4 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_w4), &c2_thisId);
  sf_mex_destroy(&c2_w4);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i10;
  real_T c2_b_inData[4];
  int32_T c2_i11;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance;
  chartInstance = (SFc2_progetto3TenzoDecoupleV2InstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i10 = 0; c2_i10 < 4; c2_i10++) {
    c2_b_inData[c2_i10] = (*(real_T (*)[4])c2_inData)[c2_i10];
  }

  for (c2_i11 = 0; c2_i11 < 4; c2_i11++) {
    c2_u[c2_i11] = c2_b_inData[c2_i11];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_c_emlrt_marshallIn(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4])
{
  real_T c2_dv1[4];
  int32_T c2_i12;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv1, 1, 0, 0U, 1, 0U, 1, 4);
  for (c2_i12 = 0; c2_i12 < 4; c2_i12++) {
    c2_y[c2_i12] = c2_dv1[c2_i12];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_T;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i13;
  SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance;
  chartInstance = (SFc2_progetto3TenzoDecoupleV2InstanceStruct *)
    chartInstanceVoid;
  c2_T = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_T), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_T);
  for (c2_i13 = 0; c2_i13 < 4; c2_i13++) {
    (*(real_T (*)[4])c2_outData)[c2_i13] = c2_y[c2_i13];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i14;
  int32_T c2_i15;
  int32_T c2_i16;
  real_T c2_b_inData[16];
  int32_T c2_i17;
  int32_T c2_i18;
  int32_T c2_i19;
  real_T c2_u[16];
  const mxArray *c2_y = NULL;
  SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance;
  chartInstance = (SFc2_progetto3TenzoDecoupleV2InstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i14 = 0;
  for (c2_i15 = 0; c2_i15 < 4; c2_i15++) {
    for (c2_i16 = 0; c2_i16 < 4; c2_i16++) {
      c2_b_inData[c2_i16 + c2_i14] = (*(real_T (*)[16])c2_inData)[c2_i16 +
        c2_i14];
    }

    c2_i14 += 4;
  }

  c2_i17 = 0;
  for (c2_i18 = 0; c2_i18 < 4; c2_i18++) {
    for (c2_i19 = 0; c2_i19 < 4; c2_i19++) {
      c2_u[c2_i19 + c2_i17] = c2_b_inData[c2_i19 + c2_i17];
    }

    c2_i17 += 4;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

const mxArray *sf_c2_progetto3TenzoDecoupleV2_get_eml_resolved_functions_info
  (void)
{
  const mxArray *c2_nameCaptureInfo;
  c2_ResolvedFunctionInfo c2_info[136];
  const mxArray *c2_m0 = NULL;
  int32_T c2_i20;
  c2_ResolvedFunctionInfo *c2_r0;
  c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  c2_info_helper(c2_info);
  c2_b_info_helper(c2_info);
  c2_c_info_helper(c2_info);
  sf_mex_assign(&c2_m0, sf_mex_createstruct("nameCaptureInfo", 1, 136), FALSE);
  for (c2_i20 = 0; c2_i20 < 136; c2_i20++) {
    c2_r0 = &c2_info[c2_i20];
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->context)), "context", "nameCaptureInfo",
                    c2_i20);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r0->name)), "name", "nameCaptureInfo", c2_i20);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c2_i20);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->resolved)), "resolved", "nameCaptureInfo",
                    c2_i20);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c2_i20);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c2_i20);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c2_i20);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c2_i20);
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[136])
{
  c2_info[0].context = "";
  c2_info[0].name = "mtimes";
  c2_info[0].dominantType = "double";
  c2_info[0].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[0].fileTimeLo = 1289519692U;
  c2_info[0].fileTimeHi = 0U;
  c2_info[0].mFileTimeLo = 0U;
  c2_info[0].mFileTimeHi = 0U;
  c2_info[1].context = "";
  c2_info[1].name = "mpower";
  c2_info[1].dominantType = "double";
  c2_info[1].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[1].fileTimeLo = 1286818842U;
  c2_info[1].fileTimeHi = 0U;
  c2_info[1].mFileTimeLo = 0U;
  c2_info[1].mFileTimeHi = 0U;
  c2_info[2].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[2].name = "power";
  c2_info[2].dominantType = "double";
  c2_info[2].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[2].fileTimeLo = 1307651240U;
  c2_info[2].fileTimeHi = 0U;
  c2_info[2].mFileTimeLo = 0U;
  c2_info[2].mFileTimeHi = 0U;
  c2_info[3].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[3].name = "eml_scalar_eg";
  c2_info[3].dominantType = "double";
  c2_info[3].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[3].fileTimeLo = 1286818796U;
  c2_info[3].fileTimeHi = 0U;
  c2_info[3].mFileTimeLo = 0U;
  c2_info[3].mFileTimeHi = 0U;
  c2_info[4].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[4].name = "eml_scalexp_alloc";
  c2_info[4].dominantType = "double";
  c2_info[4].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[4].fileTimeLo = 1286818796U;
  c2_info[4].fileTimeHi = 0U;
  c2_info[4].mFileTimeLo = 0U;
  c2_info[4].mFileTimeHi = 0U;
  c2_info[5].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[5].name = "eml_scalar_floor";
  c2_info[5].dominantType = "double";
  c2_info[5].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c2_info[5].fileTimeLo = 1286818726U;
  c2_info[5].fileTimeHi = 0U;
  c2_info[5].mFileTimeLo = 0U;
  c2_info[5].mFileTimeHi = 0U;
  c2_info[6].context = "";
  c2_info[6].name = "mrdivide";
  c2_info[6].dominantType = "double";
  c2_info[6].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[6].fileTimeLo = 1325124138U;
  c2_info[6].fileTimeHi = 0U;
  c2_info[6].mFileTimeLo = 1319729966U;
  c2_info[6].mFileTimeHi = 0U;
  c2_info[7].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[7].name = "rdivide";
  c2_info[7].dominantType = "double";
  c2_info[7].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[7].fileTimeLo = 1286818844U;
  c2_info[7].fileTimeHi = 0U;
  c2_info[7].mFileTimeLo = 0U;
  c2_info[7].mFileTimeHi = 0U;
  c2_info[8].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[8].name = "eml_div";
  c2_info[8].dominantType = "double";
  c2_info[8].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[8].fileTimeLo = 1313347810U;
  c2_info[8].fileTimeHi = 0U;
  c2_info[8].mFileTimeLo = 0U;
  c2_info[8].mFileTimeHi = 0U;
  c2_info[9].context = "";
  c2_info[9].name = "sqrt";
  c2_info[9].dominantType = "double";
  c2_info[9].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[9].fileTimeLo = 1286818752U;
  c2_info[9].fileTimeHi = 0U;
  c2_info[9].mFileTimeLo = 0U;
  c2_info[9].mFileTimeHi = 0U;
  c2_info[10].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[10].name = "eml_error";
  c2_info[10].dominantType = "char";
  c2_info[10].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[10].fileTimeLo = 1305318000U;
  c2_info[10].fileTimeHi = 0U;
  c2_info[10].mFileTimeLo = 0U;
  c2_info[10].mFileTimeHi = 0U;
  c2_info[11].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[11].name = "eml_scalar_sqrt";
  c2_info[11].dominantType = "double";
  c2_info[11].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c2_info[11].fileTimeLo = 1286818738U;
  c2_info[11].fileTimeHi = 0U;
  c2_info[11].mFileTimeLo = 0U;
  c2_info[11].mFileTimeHi = 0U;
  c2_info[12].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[12].name = "eml_scalar_floor";
  c2_info[12].dominantType = "double";
  c2_info[12].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c2_info[12].fileTimeLo = 1286818726U;
  c2_info[12].fileTimeHi = 0U;
  c2_info[12].mFileTimeLo = 0U;
  c2_info[12].mFileTimeHi = 0U;
  c2_info[13].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c2_info[13].name = "eml_index_class";
  c2_info[13].dominantType = "";
  c2_info[13].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[13].fileTimeLo = 1286818778U;
  c2_info[13].fileTimeHi = 0U;
  c2_info[13].mFileTimeLo = 0U;
  c2_info[13].mFileTimeHi = 0U;
  c2_info[14].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c2_info[14].name = "eml_scalar_eg";
  c2_info[14].dominantType = "double";
  c2_info[14].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[14].fileTimeLo = 1286818796U;
  c2_info[14].fileTimeHi = 0U;
  c2_info[14].mFileTimeLo = 0U;
  c2_info[14].mFileTimeHi = 0U;
  c2_info[15].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c2_info[15].name = "eml_scalar_abs";
  c2_info[15].dominantType = "double";
  c2_info[15].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[15].fileTimeLo = 1286818712U;
  c2_info[15].fileTimeHi = 0U;
  c2_info[15].mFileTimeLo = 0U;
  c2_info[15].mFileTimeHi = 0U;
  c2_info[16].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c2_info[16].name = "eml_scalar_floor";
  c2_info[16].dominantType = "double";
  c2_info[16].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c2_info[16].fileTimeLo = 1286818726U;
  c2_info[16].fileTimeHi = 0U;
  c2_info[16].mFileTimeLo = 0U;
  c2_info[16].mFileTimeHi = 0U;
  c2_info[17].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c2_info[17].name = "mtimes";
  c2_info[17].dominantType = "double";
  c2_info[17].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[17].fileTimeLo = 1289519692U;
  c2_info[17].fileTimeHi = 0U;
  c2_info[17].mFileTimeLo = 0U;
  c2_info[17].mFileTimeHi = 0U;
  c2_info[18].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[18].name = "eml_index_class";
  c2_info[18].dominantType = "";
  c2_info[18].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[18].fileTimeLo = 1286818778U;
  c2_info[18].fileTimeHi = 0U;
  c2_info[18].mFileTimeLo = 0U;
  c2_info[18].mFileTimeHi = 0U;
  c2_info[19].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[19].name = "eml_scalar_eg";
  c2_info[19].dominantType = "double";
  c2_info[19].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[19].fileTimeLo = 1286818796U;
  c2_info[19].fileTimeHi = 0U;
  c2_info[19].mFileTimeLo = 0U;
  c2_info[19].mFileTimeHi = 0U;
  c2_info[20].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[20].name = "eml_xgemm";
  c2_info[20].dominantType = "int32";
  c2_info[20].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[20].fileTimeLo = 1299076772U;
  c2_info[20].fileTimeHi = 0U;
  c2_info[20].mFileTimeLo = 0U;
  c2_info[20].mFileTimeHi = 0U;
  c2_info[21].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[21].name = "eml_blas_inline";
  c2_info[21].dominantType = "";
  c2_info[21].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[21].fileTimeLo = 1299076768U;
  c2_info[21].fileTimeHi = 0U;
  c2_info[21].mFileTimeLo = 0U;
  c2_info[21].mFileTimeHi = 0U;
  c2_info[22].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c2_info[22].name = "mtimes";
  c2_info[22].dominantType = "double";
  c2_info[22].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[22].fileTimeLo = 1289519692U;
  c2_info[22].fileTimeHi = 0U;
  c2_info[22].mFileTimeLo = 0U;
  c2_info[22].mFileTimeHi = 0U;
  c2_info[23].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[23].name = "eml_scalar_eg";
  c2_info[23].dominantType = "double";
  c2_info[23].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[23].fileTimeLo = 1286818796U;
  c2_info[23].fileTimeHi = 0U;
  c2_info[23].mFileTimeLo = 0U;
  c2_info[23].mFileTimeHi = 0U;
  c2_info[24].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[24].name = "eml_refblas_xgemm";
  c2_info[24].dominantType = "int32";
  c2_info[24].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[24].fileTimeLo = 1299076774U;
  c2_info[24].fileTimeHi = 0U;
  c2_info[24].mFileTimeLo = 0U;
  c2_info[24].mFileTimeHi = 0U;
  c2_info[25].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c2_info[25].name = "inv";
  c2_info[25].dominantType = "double";
  c2_info[25].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/matfun/inv.m";
  c2_info[25].fileTimeLo = 1305318000U;
  c2_info[25].fileTimeHi = 0U;
  c2_info[25].mFileTimeLo = 0U;
  c2_info[25].mFileTimeHi = 0U;
  c2_info[26].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c2_info[26].name = "eml_index_class";
  c2_info[26].dominantType = "";
  c2_info[26].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[26].fileTimeLo = 1286818778U;
  c2_info[26].fileTimeHi = 0U;
  c2_info[26].mFileTimeLo = 0U;
  c2_info[26].mFileTimeHi = 0U;
  c2_info[27].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c2_info[27].name = "eml_xgetrf";
  c2_info[27].dominantType = "int32";
  c2_info[27].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c2_info[27].fileTimeLo = 1286818806U;
  c2_info[27].fileTimeHi = 0U;
  c2_info[27].mFileTimeLo = 0U;
  c2_info[27].mFileTimeHi = 0U;
  c2_info[28].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c2_info[28].name = "eml_lapack_xgetrf";
  c2_info[28].dominantType = "int32";
  c2_info[28].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c2_info[28].fileTimeLo = 1286818810U;
  c2_info[28].fileTimeHi = 0U;
  c2_info[28].mFileTimeLo = 0U;
  c2_info[28].mFileTimeHi = 0U;
  c2_info[29].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c2_info[29].name = "eml_matlab_zgetrf";
  c2_info[29].dominantType = "int32";
  c2_info[29].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[29].fileTimeLo = 1302688994U;
  c2_info[29].fileTimeHi = 0U;
  c2_info[29].mFileTimeLo = 0U;
  c2_info[29].mFileTimeHi = 0U;
  c2_info[30].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[30].name = "realmin";
  c2_info[30].dominantType = "char";
  c2_info[30].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[30].fileTimeLo = 1307651242U;
  c2_info[30].fileTimeHi = 0U;
  c2_info[30].mFileTimeLo = 0U;
  c2_info[30].mFileTimeHi = 0U;
  c2_info[31].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[31].name = "eml_realmin";
  c2_info[31].dominantType = "char";
  c2_info[31].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c2_info[31].fileTimeLo = 1307651244U;
  c2_info[31].fileTimeHi = 0U;
  c2_info[31].mFileTimeLo = 0U;
  c2_info[31].mFileTimeHi = 0U;
  c2_info[32].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c2_info[32].name = "eml_float_model";
  c2_info[32].dominantType = "char";
  c2_info[32].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c2_info[32].fileTimeLo = 1307651242U;
  c2_info[32].fileTimeHi = 0U;
  c2_info[32].mFileTimeLo = 0U;
  c2_info[32].mFileTimeHi = 0U;
  c2_info[33].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[33].name = "eps";
  c2_info[33].dominantType = "char";
  c2_info[33].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[33].fileTimeLo = 1307651240U;
  c2_info[33].fileTimeHi = 0U;
  c2_info[33].mFileTimeLo = 0U;
  c2_info[33].mFileTimeHi = 0U;
  c2_info[34].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[34].name = "eml_is_float_class";
  c2_info[34].dominantType = "char";
  c2_info[34].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c2_info[34].fileTimeLo = 1286818782U;
  c2_info[34].fileTimeHi = 0U;
  c2_info[34].mFileTimeLo = 0U;
  c2_info[34].mFileTimeHi = 0U;
  c2_info[35].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[35].name = "eml_eps";
  c2_info[35].dominantType = "char";
  c2_info[35].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c2_info[35].fileTimeLo = 1307651240U;
  c2_info[35].fileTimeHi = 0U;
  c2_info[35].mFileTimeLo = 0U;
  c2_info[35].mFileTimeHi = 0U;
  c2_info[36].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c2_info[36].name = "eml_float_model";
  c2_info[36].dominantType = "char";
  c2_info[36].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c2_info[36].fileTimeLo = 1307651242U;
  c2_info[36].fileTimeHi = 0U;
  c2_info[36].mFileTimeLo = 0U;
  c2_info[36].mFileTimeHi = 0U;
  c2_info[37].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[37].name = "min";
  c2_info[37].dominantType = "int32";
  c2_info[37].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[37].fileTimeLo = 1311255318U;
  c2_info[37].fileTimeHi = 0U;
  c2_info[37].mFileTimeLo = 0U;
  c2_info[37].mFileTimeHi = 0U;
  c2_info[38].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[38].name = "eml_min_or_max";
  c2_info[38].dominantType = "int32";
  c2_info[38].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c2_info[38].fileTimeLo = 1303146212U;
  c2_info[38].fileTimeHi = 0U;
  c2_info[38].mFileTimeLo = 0U;
  c2_info[38].mFileTimeHi = 0U;
  c2_info[39].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[39].name = "eml_scalar_eg";
  c2_info[39].dominantType = "int32";
  c2_info[39].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[39].fileTimeLo = 1286818796U;
  c2_info[39].fileTimeHi = 0U;
  c2_info[39].mFileTimeLo = 0U;
  c2_info[39].mFileTimeHi = 0U;
  c2_info[40].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[40].name = "eml_scalexp_alloc";
  c2_info[40].dominantType = "int32";
  c2_info[40].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[40].fileTimeLo = 1286818796U;
  c2_info[40].fileTimeHi = 0U;
  c2_info[40].mFileTimeLo = 0U;
  c2_info[40].mFileTimeHi = 0U;
  c2_info[41].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[41].name = "eml_index_class";
  c2_info[41].dominantType = "";
  c2_info[41].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[41].fileTimeLo = 1286818778U;
  c2_info[41].fileTimeHi = 0U;
  c2_info[41].mFileTimeLo = 0U;
  c2_info[41].mFileTimeHi = 0U;
  c2_info[42].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[42].name = "eml_scalar_eg";
  c2_info[42].dominantType = "int32";
  c2_info[42].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[42].fileTimeLo = 1286818796U;
  c2_info[42].fileTimeHi = 0U;
  c2_info[42].mFileTimeLo = 0U;
  c2_info[42].mFileTimeHi = 0U;
  c2_info[43].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[43].name = "colon";
  c2_info[43].dominantType = "int32";
  c2_info[43].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[43].fileTimeLo = 1311255318U;
  c2_info[43].fileTimeHi = 0U;
  c2_info[43].mFileTimeLo = 0U;
  c2_info[43].mFileTimeHi = 0U;
  c2_info[44].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[44].name = "colon";
  c2_info[44].dominantType = "int32";
  c2_info[44].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[44].fileTimeLo = 1311255318U;
  c2_info[44].fileTimeHi = 0U;
  c2_info[44].mFileTimeLo = 0U;
  c2_info[44].mFileTimeHi = 0U;
  c2_info[45].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[45].name = "floor";
  c2_info[45].dominantType = "double";
  c2_info[45].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[45].fileTimeLo = 1286818742U;
  c2_info[45].fileTimeHi = 0U;
  c2_info[45].mFileTimeLo = 0U;
  c2_info[45].mFileTimeHi = 0U;
  c2_info[46].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[46].name = "eml_scalar_floor";
  c2_info[46].dominantType = "double";
  c2_info[46].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c2_info[46].fileTimeLo = 1286818726U;
  c2_info[46].fileTimeHi = 0U;
  c2_info[46].mFileTimeLo = 0U;
  c2_info[46].mFileTimeHi = 0U;
  c2_info[47].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c2_info[47].name = "intmin";
  c2_info[47].dominantType = "char";
  c2_info[47].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[47].fileTimeLo = 1311255318U;
  c2_info[47].fileTimeHi = 0U;
  c2_info[47].mFileTimeLo = 0U;
  c2_info[47].mFileTimeHi = 0U;
  c2_info[48].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c2_info[48].name = "intmax";
  c2_info[48].dominantType = "char";
  c2_info[48].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[48].fileTimeLo = 1311255316U;
  c2_info[48].fileTimeHi = 0U;
  c2_info[48].mFileTimeLo = 0U;
  c2_info[48].mFileTimeHi = 0U;
  c2_info[49].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c2_info[49].name = "intmin";
  c2_info[49].dominantType = "char";
  c2_info[49].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[49].fileTimeLo = 1311255318U;
  c2_info[49].fileTimeHi = 0U;
  c2_info[49].mFileTimeLo = 0U;
  c2_info[49].mFileTimeHi = 0U;
  c2_info[50].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c2_info[50].name = "intmax";
  c2_info[50].dominantType = "char";
  c2_info[50].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[50].fileTimeLo = 1311255316U;
  c2_info[50].fileTimeHi = 0U;
  c2_info[50].mFileTimeLo = 0U;
  c2_info[50].mFileTimeHi = 0U;
  c2_info[51].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c2_info[51].name = "eml_isa_uint";
  c2_info[51].dominantType = "int32";
  c2_info[51].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c2_info[51].fileTimeLo = 1286818784U;
  c2_info[51].fileTimeHi = 0U;
  c2_info[51].mFileTimeLo = 0U;
  c2_info[51].mFileTimeHi = 0U;
  c2_info[52].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[52].name = "eml_unsigned_class";
  c2_info[52].dominantType = "char";
  c2_info[52].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c2_info[52].fileTimeLo = 1286818800U;
  c2_info[52].fileTimeHi = 0U;
  c2_info[52].mFileTimeLo = 0U;
  c2_info[52].mFileTimeHi = 0U;
  c2_info[53].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[53].name = "eml_index_class";
  c2_info[53].dominantType = "";
  c2_info[53].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[53].fileTimeLo = 1286818778U;
  c2_info[53].fileTimeHi = 0U;
  c2_info[53].mFileTimeLo = 0U;
  c2_info[53].mFileTimeHi = 0U;
  c2_info[54].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[54].name = "intmax";
  c2_info[54].dominantType = "char";
  c2_info[54].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[54].fileTimeLo = 1311255316U;
  c2_info[54].fileTimeHi = 0U;
  c2_info[54].mFileTimeLo = 0U;
  c2_info[54].mFileTimeHi = 0U;
  c2_info[55].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[55].name = "eml_isa_uint";
  c2_info[55].dominantType = "int32";
  c2_info[55].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c2_info[55].fileTimeLo = 1286818784U;
  c2_info[55].fileTimeHi = 0U;
  c2_info[55].mFileTimeLo = 0U;
  c2_info[55].mFileTimeHi = 0U;
  c2_info[56].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[56].name = "eml_index_plus";
  c2_info[56].dominantType = "int32";
  c2_info[56].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[56].fileTimeLo = 1286818778U;
  c2_info[56].fileTimeHi = 0U;
  c2_info[56].mFileTimeLo = 0U;
  c2_info[56].mFileTimeHi = 0U;
  c2_info[57].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[57].name = "eml_index_class";
  c2_info[57].dominantType = "";
  c2_info[57].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[57].fileTimeLo = 1286818778U;
  c2_info[57].fileTimeHi = 0U;
  c2_info[57].mFileTimeLo = 0U;
  c2_info[57].mFileTimeHi = 0U;
  c2_info[58].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon";
  c2_info[58].name = "eml_int_forloop_overflow_check";
  c2_info[58].dominantType = "";
  c2_info[58].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[58].fileTimeLo = 1311255316U;
  c2_info[58].fileTimeHi = 0U;
  c2_info[58].mFileTimeLo = 0U;
  c2_info[58].mFileTimeHi = 0U;
  c2_info[59].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c2_info[59].name = "intmax";
  c2_info[59].dominantType = "char";
  c2_info[59].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[59].fileTimeLo = 1311255316U;
  c2_info[59].fileTimeHi = 0U;
  c2_info[59].mFileTimeLo = 0U;
  c2_info[59].mFileTimeHi = 0U;
  c2_info[60].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[60].name = "eml_index_class";
  c2_info[60].dominantType = "";
  c2_info[60].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[60].fileTimeLo = 1286818778U;
  c2_info[60].fileTimeHi = 0U;
  c2_info[60].mFileTimeLo = 0U;
  c2_info[60].mFileTimeHi = 0U;
  c2_info[61].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[61].name = "eml_index_plus";
  c2_info[61].dominantType = "int32";
  c2_info[61].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[61].fileTimeLo = 1286818778U;
  c2_info[61].fileTimeHi = 0U;
  c2_info[61].mFileTimeLo = 0U;
  c2_info[61].mFileTimeHi = 0U;
  c2_info[62].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[62].name = "eml_int_forloop_overflow_check";
  c2_info[62].dominantType = "";
  c2_info[62].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[62].fileTimeLo = 1311255316U;
  c2_info[62].fileTimeHi = 0U;
  c2_info[62].mFileTimeLo = 0U;
  c2_info[62].mFileTimeHi = 0U;
  c2_info[63].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[63].name = "eml_index_minus";
  c2_info[63].dominantType = "int32";
  c2_info[63].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[63].fileTimeLo = 1286818778U;
  c2_info[63].fileTimeHi = 0U;
  c2_info[63].mFileTimeLo = 0U;
  c2_info[63].mFileTimeHi = 0U;
}

static void c2_b_info_helper(c2_ResolvedFunctionInfo c2_info[136])
{
  c2_info[64].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[64].name = "eml_index_class";
  c2_info[64].dominantType = "";
  c2_info[64].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[64].fileTimeLo = 1286818778U;
  c2_info[64].fileTimeHi = 0U;
  c2_info[64].mFileTimeLo = 0U;
  c2_info[64].mFileTimeHi = 0U;
  c2_info[65].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[65].name = "eml_index_times";
  c2_info[65].dominantType = "int32";
  c2_info[65].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[65].fileTimeLo = 1286818780U;
  c2_info[65].fileTimeHi = 0U;
  c2_info[65].mFileTimeLo = 0U;
  c2_info[65].mFileTimeHi = 0U;
  c2_info[66].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[66].name = "eml_index_class";
  c2_info[66].dominantType = "";
  c2_info[66].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[66].fileTimeLo = 1286818778U;
  c2_info[66].fileTimeHi = 0U;
  c2_info[66].mFileTimeLo = 0U;
  c2_info[66].mFileTimeHi = 0U;
  c2_info[67].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[67].name = "eml_ixamax";
  c2_info[67].dominantType = "int32";
  c2_info[67].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c2_info[67].fileTimeLo = 1299076770U;
  c2_info[67].fileTimeHi = 0U;
  c2_info[67].mFileTimeLo = 0U;
  c2_info[67].mFileTimeHi = 0U;
  c2_info[68].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c2_info[68].name = "eml_blas_inline";
  c2_info[68].dominantType = "";
  c2_info[68].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[68].fileTimeLo = 1299076768U;
  c2_info[68].fileTimeHi = 0U;
  c2_info[68].mFileTimeLo = 0U;
  c2_info[68].mFileTimeHi = 0U;
  c2_info[69].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m!below_threshold";
  c2_info[69].name = "length";
  c2_info[69].dominantType = "double";
  c2_info[69].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[69].fileTimeLo = 1303146206U;
  c2_info[69].fileTimeHi = 0U;
  c2_info[69].mFileTimeLo = 0U;
  c2_info[69].mFileTimeHi = 0U;
  c2_info[70].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c2_info[70].name = "eml_index_class";
  c2_info[70].dominantType = "";
  c2_info[70].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[70].fileTimeLo = 1286818778U;
  c2_info[70].fileTimeHi = 0U;
  c2_info[70].mFileTimeLo = 0U;
  c2_info[70].mFileTimeHi = 0U;
  c2_info[71].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c2_info[71].name = "eml_refblas_ixamax";
  c2_info[71].dominantType = "int32";
  c2_info[71].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[71].fileTimeLo = 1299076770U;
  c2_info[71].fileTimeHi = 0U;
  c2_info[71].mFileTimeLo = 0U;
  c2_info[71].mFileTimeHi = 0U;
  c2_info[72].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[72].name = "eml_index_class";
  c2_info[72].dominantType = "";
  c2_info[72].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[72].fileTimeLo = 1286818778U;
  c2_info[72].fileTimeHi = 0U;
  c2_info[72].mFileTimeLo = 0U;
  c2_info[72].mFileTimeHi = 0U;
  c2_info[73].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[73].name = "eml_xcabs1";
  c2_info[73].dominantType = "double";
  c2_info[73].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c2_info[73].fileTimeLo = 1286818706U;
  c2_info[73].fileTimeHi = 0U;
  c2_info[73].mFileTimeLo = 0U;
  c2_info[73].mFileTimeHi = 0U;
  c2_info[74].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c2_info[74].name = "abs";
  c2_info[74].dominantType = "double";
  c2_info[74].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[74].fileTimeLo = 1286818694U;
  c2_info[74].fileTimeHi = 0U;
  c2_info[74].mFileTimeLo = 0U;
  c2_info[74].mFileTimeHi = 0U;
  c2_info[75].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[75].name = "eml_scalar_abs";
  c2_info[75].dominantType = "double";
  c2_info[75].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[75].fileTimeLo = 1286818712U;
  c2_info[75].fileTimeHi = 0U;
  c2_info[75].mFileTimeLo = 0U;
  c2_info[75].mFileTimeHi = 0U;
  c2_info[76].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[76].name = "eml_int_forloop_overflow_check";
  c2_info[76].dominantType = "";
  c2_info[76].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[76].fileTimeLo = 1311255316U;
  c2_info[76].fileTimeHi = 0U;
  c2_info[76].mFileTimeLo = 0U;
  c2_info[76].mFileTimeHi = 0U;
  c2_info[77].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[77].name = "eml_index_plus";
  c2_info[77].dominantType = "int32";
  c2_info[77].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[77].fileTimeLo = 1286818778U;
  c2_info[77].fileTimeHi = 0U;
  c2_info[77].mFileTimeLo = 0U;
  c2_info[77].mFileTimeHi = 0U;
  c2_info[78].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[78].name = "eml_xswap";
  c2_info[78].dominantType = "int32";
  c2_info[78].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c2_info[78].fileTimeLo = 1299076778U;
  c2_info[78].fileTimeHi = 0U;
  c2_info[78].mFileTimeLo = 0U;
  c2_info[78].mFileTimeHi = 0U;
  c2_info[79].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c2_info[79].name = "eml_blas_inline";
  c2_info[79].dominantType = "";
  c2_info[79].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[79].fileTimeLo = 1299076768U;
  c2_info[79].fileTimeHi = 0U;
  c2_info[79].mFileTimeLo = 0U;
  c2_info[79].mFileTimeHi = 0U;
  c2_info[80].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c2_info[80].name = "eml_refblas_xswap";
  c2_info[80].dominantType = "int32";
  c2_info[80].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[80].fileTimeLo = 1299076786U;
  c2_info[80].fileTimeHi = 0U;
  c2_info[80].mFileTimeLo = 0U;
  c2_info[80].mFileTimeHi = 0U;
  c2_info[81].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[81].name = "eml_index_class";
  c2_info[81].dominantType = "";
  c2_info[81].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[81].fileTimeLo = 1286818778U;
  c2_info[81].fileTimeHi = 0U;
  c2_info[81].mFileTimeLo = 0U;
  c2_info[81].mFileTimeHi = 0U;
  c2_info[82].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[82].name = "abs";
  c2_info[82].dominantType = "int32";
  c2_info[82].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[82].fileTimeLo = 1286818694U;
  c2_info[82].fileTimeHi = 0U;
  c2_info[82].mFileTimeLo = 0U;
  c2_info[82].mFileTimeHi = 0U;
  c2_info[83].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[83].name = "eml_scalar_abs";
  c2_info[83].dominantType = "int32";
  c2_info[83].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[83].fileTimeLo = 1286818712U;
  c2_info[83].fileTimeHi = 0U;
  c2_info[83].mFileTimeLo = 0U;
  c2_info[83].mFileTimeHi = 0U;
  c2_info[84].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[84].name = "eml_int_forloop_overflow_check";
  c2_info[84].dominantType = "";
  c2_info[84].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[84].fileTimeLo = 1311255316U;
  c2_info[84].fileTimeHi = 0U;
  c2_info[84].mFileTimeLo = 0U;
  c2_info[84].mFileTimeHi = 0U;
  c2_info[85].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[85].name = "eml_index_plus";
  c2_info[85].dominantType = "int32";
  c2_info[85].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[85].fileTimeLo = 1286818778U;
  c2_info[85].fileTimeHi = 0U;
  c2_info[85].mFileTimeLo = 0U;
  c2_info[85].mFileTimeHi = 0U;
  c2_info[86].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[86].name = "eml_div";
  c2_info[86].dominantType = "double";
  c2_info[86].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[86].fileTimeLo = 1313347810U;
  c2_info[86].fileTimeHi = 0U;
  c2_info[86].mFileTimeLo = 0U;
  c2_info[86].mFileTimeHi = 0U;
  c2_info[87].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[87].name = "eml_xgeru";
  c2_info[87].dominantType = "int32";
  c2_info[87].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c2_info[87].fileTimeLo = 1299076774U;
  c2_info[87].fileTimeHi = 0U;
  c2_info[87].mFileTimeLo = 0U;
  c2_info[87].mFileTimeHi = 0U;
  c2_info[88].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c2_info[88].name = "eml_blas_inline";
  c2_info[88].dominantType = "";
  c2_info[88].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[88].fileTimeLo = 1299076768U;
  c2_info[88].fileTimeHi = 0U;
  c2_info[88].mFileTimeLo = 0U;
  c2_info[88].mFileTimeHi = 0U;
  c2_info[89].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c2_info[89].name = "eml_xger";
  c2_info[89].dominantType = "int32";
  c2_info[89].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c2_info[89].fileTimeLo = 1299076774U;
  c2_info[89].fileTimeHi = 0U;
  c2_info[89].mFileTimeLo = 0U;
  c2_info[89].mFileTimeHi = 0U;
  c2_info[90].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c2_info[90].name = "eml_blas_inline";
  c2_info[90].dominantType = "";
  c2_info[90].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[90].fileTimeLo = 1299076768U;
  c2_info[90].fileTimeHi = 0U;
  c2_info[90].mFileTimeLo = 0U;
  c2_info[90].mFileTimeHi = 0U;
  c2_info[91].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c2_info[91].name = "intmax";
  c2_info[91].dominantType = "char";
  c2_info[91].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[91].fileTimeLo = 1311255316U;
  c2_info[91].fileTimeHi = 0U;
  c2_info[91].mFileTimeLo = 0U;
  c2_info[91].mFileTimeHi = 0U;
  c2_info[92].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c2_info[92].name = "min";
  c2_info[92].dominantType = "double";
  c2_info[92].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[92].fileTimeLo = 1311255318U;
  c2_info[92].fileTimeHi = 0U;
  c2_info[92].mFileTimeLo = 0U;
  c2_info[92].mFileTimeHi = 0U;
  c2_info[93].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[93].name = "eml_min_or_max";
  c2_info[93].dominantType = "char";
  c2_info[93].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c2_info[93].fileTimeLo = 1303146212U;
  c2_info[93].fileTimeHi = 0U;
  c2_info[93].mFileTimeLo = 0U;
  c2_info[93].mFileTimeHi = 0U;
  c2_info[94].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[94].name = "eml_scalar_eg";
  c2_info[94].dominantType = "double";
  c2_info[94].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[94].fileTimeLo = 1286818796U;
  c2_info[94].fileTimeHi = 0U;
  c2_info[94].mFileTimeLo = 0U;
  c2_info[94].mFileTimeHi = 0U;
  c2_info[95].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[95].name = "eml_scalexp_alloc";
  c2_info[95].dominantType = "double";
  c2_info[95].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[95].fileTimeLo = 1286818796U;
  c2_info[95].fileTimeHi = 0U;
  c2_info[95].mFileTimeLo = 0U;
  c2_info[95].mFileTimeHi = 0U;
  c2_info[96].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[96].name = "eml_scalar_eg";
  c2_info[96].dominantType = "double";
  c2_info[96].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[96].fileTimeLo = 1286818796U;
  c2_info[96].fileTimeHi = 0U;
  c2_info[96].mFileTimeLo = 0U;
  c2_info[96].mFileTimeHi = 0U;
  c2_info[97].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c2_info[97].name = "mtimes";
  c2_info[97].dominantType = "double";
  c2_info[97].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[97].fileTimeLo = 1289519692U;
  c2_info[97].fileTimeHi = 0U;
  c2_info[97].mFileTimeLo = 0U;
  c2_info[97].mFileTimeHi = 0U;
  c2_info[98].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c2_info[98].name = "eml_refblas_xger";
  c2_info[98].dominantType = "int32";
  c2_info[98].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c2_info[98].fileTimeLo = 1299076776U;
  c2_info[98].fileTimeHi = 0U;
  c2_info[98].mFileTimeLo = 0U;
  c2_info[98].mFileTimeHi = 0U;
  c2_info[99].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c2_info[99].name = "eml_refblas_xgerx";
  c2_info[99].dominantType = "int32";
  c2_info[99].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[99].fileTimeLo = 1299076778U;
  c2_info[99].fileTimeHi = 0U;
  c2_info[99].mFileTimeLo = 0U;
  c2_info[99].mFileTimeHi = 0U;
  c2_info[100].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[100].name = "eml_index_class";
  c2_info[100].dominantType = "";
  c2_info[100].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[100].fileTimeLo = 1286818778U;
  c2_info[100].fileTimeHi = 0U;
  c2_info[100].mFileTimeLo = 0U;
  c2_info[100].mFileTimeHi = 0U;
  c2_info[101].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[101].name = "abs";
  c2_info[101].dominantType = "int32";
  c2_info[101].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[101].fileTimeLo = 1286818694U;
  c2_info[101].fileTimeHi = 0U;
  c2_info[101].mFileTimeLo = 0U;
  c2_info[101].mFileTimeHi = 0U;
  c2_info[102].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[102].name = "eml_index_minus";
  c2_info[102].dominantType = "int32";
  c2_info[102].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[102].fileTimeLo = 1286818778U;
  c2_info[102].fileTimeHi = 0U;
  c2_info[102].mFileTimeLo = 0U;
  c2_info[102].mFileTimeHi = 0U;
  c2_info[103].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[103].name = "eml_int_forloop_overflow_check";
  c2_info[103].dominantType = "";
  c2_info[103].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[103].fileTimeLo = 1311255316U;
  c2_info[103].fileTimeHi = 0U;
  c2_info[103].mFileTimeLo = 0U;
  c2_info[103].mFileTimeHi = 0U;
  c2_info[104].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[104].name = "eml_index_plus";
  c2_info[104].dominantType = "int32";
  c2_info[104].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[104].fileTimeLo = 1286818778U;
  c2_info[104].fileTimeHi = 0U;
  c2_info[104].mFileTimeLo = 0U;
  c2_info[104].mFileTimeHi = 0U;
  c2_info[105].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c2_info[105].name = "eml_ipiv2perm";
  c2_info[105].dominantType = "int32";
  c2_info[105].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_ipiv2perm.m";
  c2_info[105].fileTimeLo = 1286818782U;
  c2_info[105].fileTimeHi = 0U;
  c2_info[105].mFileTimeLo = 0U;
  c2_info[105].mFileTimeHi = 0U;
  c2_info[106].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_ipiv2perm.m";
  c2_info[106].name = "colon";
  c2_info[106].dominantType = "int32";
  c2_info[106].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[106].fileTimeLo = 1311255318U;
  c2_info[106].fileTimeHi = 0U;
  c2_info[106].mFileTimeLo = 0U;
  c2_info[106].mFileTimeHi = 0U;
  c2_info[107].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_ipiv2perm.m";
  c2_info[107].name = "eml_index_class";
  c2_info[107].dominantType = "";
  c2_info[107].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[107].fileTimeLo = 1286818778U;
  c2_info[107].fileTimeHi = 0U;
  c2_info[107].mFileTimeLo = 0U;
  c2_info[107].mFileTimeHi = 0U;
  c2_info[108].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c2_info[108].name = "eml_int_forloop_overflow_check";
  c2_info[108].dominantType = "";
  c2_info[108].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[108].fileTimeLo = 1311255316U;
  c2_info[108].fileTimeHi = 0U;
  c2_info[108].mFileTimeLo = 0U;
  c2_info[108].mFileTimeHi = 0U;
  c2_info[109].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c2_info[109].name = "eml_index_plus";
  c2_info[109].dominantType = "int32";
  c2_info[109].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[109].fileTimeLo = 1286818778U;
  c2_info[109].fileTimeHi = 0U;
  c2_info[109].mFileTimeLo = 0U;
  c2_info[109].mFileTimeHi = 0U;
  c2_info[110].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c2_info[110].name = "mtimes";
  c2_info[110].dominantType = "double";
  c2_info[110].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[110].fileTimeLo = 1289519692U;
  c2_info[110].fileTimeHi = 0U;
  c2_info[110].mFileTimeLo = 0U;
  c2_info[110].mFileTimeHi = 0U;
  c2_info[111].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c2_info[111].name = "eml_scalar_eg";
  c2_info[111].dominantType = "double";
  c2_info[111].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[111].fileTimeLo = 1286818796U;
  c2_info[111].fileTimeHi = 0U;
  c2_info[111].mFileTimeLo = 0U;
  c2_info[111].mFileTimeHi = 0U;
  c2_info[112].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c2_info[112].name = "eml_xtrsm";
  c2_info[112].dominantType = "int32";
  c2_info[112].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c2_info[112].fileTimeLo = 1299076778U;
  c2_info[112].fileTimeHi = 0U;
  c2_info[112].mFileTimeLo = 0U;
  c2_info[112].mFileTimeHi = 0U;
  c2_info[113].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c2_info[113].name = "eml_blas_inline";
  c2_info[113].dominantType = "";
  c2_info[113].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[113].fileTimeLo = 1299076768U;
  c2_info[113].fileTimeHi = 0U;
  c2_info[113].mFileTimeLo = 0U;
  c2_info[113].mFileTimeHi = 0U;
  c2_info[114].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m!below_threshold";
  c2_info[114].name = "mtimes";
  c2_info[114].dominantType = "double";
  c2_info[114].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[114].fileTimeLo = 1289519692U;
  c2_info[114].fileTimeHi = 0U;
  c2_info[114].mFileTimeLo = 0U;
  c2_info[114].mFileTimeHi = 0U;
  c2_info[115].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c2_info[115].name = "eml_scalar_eg";
  c2_info[115].dominantType = "double";
  c2_info[115].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[115].fileTimeLo = 1286818796U;
  c2_info[115].fileTimeHi = 0U;
  c2_info[115].mFileTimeLo = 0U;
  c2_info[115].mFileTimeHi = 0U;
  c2_info[116].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c2_info[116].name = "eml_refblas_xtrsm";
  c2_info[116].dominantType = "int32";
  c2_info[116].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[116].fileTimeLo = 1299076786U;
  c2_info[116].fileTimeHi = 0U;
  c2_info[116].mFileTimeLo = 0U;
  c2_info[116].mFileTimeHi = 0U;
  c2_info[117].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[117].name = "eml_scalar_eg";
  c2_info[117].dominantType = "double";
  c2_info[117].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[117].fileTimeLo = 1286818796U;
  c2_info[117].fileTimeHi = 0U;
  c2_info[117].mFileTimeLo = 0U;
  c2_info[117].mFileTimeHi = 0U;
  c2_info[118].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[118].name = "eml_index_minus";
  c2_info[118].dominantType = "int32";
  c2_info[118].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[118].fileTimeLo = 1286818778U;
  c2_info[118].fileTimeHi = 0U;
  c2_info[118].mFileTimeLo = 0U;
  c2_info[118].mFileTimeHi = 0U;
  c2_info[119].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[119].name = "eml_index_class";
  c2_info[119].dominantType = "";
  c2_info[119].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[119].fileTimeLo = 1286818778U;
  c2_info[119].fileTimeHi = 0U;
  c2_info[119].mFileTimeLo = 0U;
  c2_info[119].mFileTimeHi = 0U;
  c2_info[120].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[120].name = "eml_int_forloop_overflow_check";
  c2_info[120].dominantType = "";
  c2_info[120].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[120].fileTimeLo = 1311255316U;
  c2_info[120].fileTimeHi = 0U;
  c2_info[120].mFileTimeLo = 0U;
  c2_info[120].mFileTimeHi = 0U;
  c2_info[121].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[121].name = "eml_index_times";
  c2_info[121].dominantType = "int32";
  c2_info[121].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[121].fileTimeLo = 1286818780U;
  c2_info[121].fileTimeHi = 0U;
  c2_info[121].mFileTimeLo = 0U;
  c2_info[121].mFileTimeHi = 0U;
  c2_info[122].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[122].name = "eml_index_plus";
  c2_info[122].dominantType = "int32";
  c2_info[122].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[122].fileTimeLo = 1286818778U;
  c2_info[122].fileTimeHi = 0U;
  c2_info[122].mFileTimeLo = 0U;
  c2_info[122].mFileTimeHi = 0U;
  c2_info[123].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c2_info[123].name = "intmin";
  c2_info[123].dominantType = "char";
  c2_info[123].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[123].fileTimeLo = 1311255318U;
  c2_info[123].fileTimeHi = 0U;
  c2_info[123].mFileTimeLo = 0U;
  c2_info[123].mFileTimeHi = 0U;
  c2_info[124].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[124].name = "eml_div";
  c2_info[124].dominantType = "double";
  c2_info[124].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[124].fileTimeLo = 1313347810U;
  c2_info[124].fileTimeHi = 0U;
  c2_info[124].mFileTimeLo = 0U;
  c2_info[124].mFileTimeHi = 0U;
  c2_info[125].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c2_info[125].name = "norm";
  c2_info[125].dominantType = "double";
  c2_info[125].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/matfun/norm.m";
  c2_info[125].fileTimeLo = 1286818826U;
  c2_info[125].fileTimeHi = 0U;
  c2_info[125].mFileTimeLo = 0U;
  c2_info[125].mFileTimeHi = 0U;
  c2_info[126].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm";
  c2_info[126].name = "abs";
  c2_info[126].dominantType = "double";
  c2_info[126].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[126].fileTimeLo = 1286818694U;
  c2_info[126].fileTimeHi = 0U;
  c2_info[126].mFileTimeLo = 0U;
  c2_info[126].mFileTimeHi = 0U;
  c2_info[127].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm";
  c2_info[127].name = "isnan";
  c2_info[127].dominantType = "double";
  c2_info[127].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/isnan.m";
  c2_info[127].fileTimeLo = 1286818760U;
  c2_info[127].fileTimeHi = 0U;
  c2_info[127].mFileTimeLo = 0U;
  c2_info[127].mFileTimeHi = 0U;
}

static void c2_c_info_helper(c2_ResolvedFunctionInfo c2_info[136])
{
  c2_info[128].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm";
  c2_info[128].name = "eml_guarded_nan";
  c2_info[128].dominantType = "char";
  c2_info[128].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c2_info[128].fileTimeLo = 1286818776U;
  c2_info[128].fileTimeHi = 0U;
  c2_info[128].mFileTimeLo = 0U;
  c2_info[128].mFileTimeHi = 0U;
  c2_info[129].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c2_info[129].name = "eml_is_float_class";
  c2_info[129].dominantType = "char";
  c2_info[129].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c2_info[129].fileTimeLo = 1286818782U;
  c2_info[129].fileTimeHi = 0U;
  c2_info[129].mFileTimeLo = 0U;
  c2_info[129].mFileTimeHi = 0U;
  c2_info[130].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c2_info[130].name = "mtimes";
  c2_info[130].dominantType = "double";
  c2_info[130].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[130].fileTimeLo = 1289519692U;
  c2_info[130].fileTimeHi = 0U;
  c2_info[130].mFileTimeLo = 0U;
  c2_info[130].mFileTimeHi = 0U;
  c2_info[131].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c2_info[131].name = "eml_warning";
  c2_info[131].dominantType = "char";
  c2_info[131].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c2_info[131].fileTimeLo = 1286818802U;
  c2_info[131].fileTimeHi = 0U;
  c2_info[131].mFileTimeLo = 0U;
  c2_info[131].mFileTimeHi = 0U;
  c2_info[132].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c2_info[132].name = "isnan";
  c2_info[132].dominantType = "double";
  c2_info[132].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/isnan.m";
  c2_info[132].fileTimeLo = 1286818760U;
  c2_info[132].fileTimeHi = 0U;
  c2_info[132].mFileTimeLo = 0U;
  c2_info[132].mFileTimeHi = 0U;
  c2_info[133].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c2_info[133].name = "eps";
  c2_info[133].dominantType = "char";
  c2_info[133].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[133].fileTimeLo = 1307651240U;
  c2_info[133].fileTimeHi = 0U;
  c2_info[133].mFileTimeLo = 0U;
  c2_info[133].mFileTimeHi = 0U;
  c2_info[134].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c2_info[134].name = "eml_flt2str";
  c2_info[134].dominantType = "double";
  c2_info[134].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c2_info[134].fileTimeLo = 1309451196U;
  c2_info[134].fileTimeHi = 0U;
  c2_info[134].mFileTimeLo = 0U;
  c2_info[134].mFileTimeHi = 0U;
  c2_info[135].context =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c2_info[135].name = "char";
  c2_info[135].dominantType = "double";
  c2_info[135].resolved =
    "[ILXE]/home/userk/Programs/Matlab/toolbox/eml/lib/matlab/strfun/char.m";
  c2_info[135].fileTimeLo = 1319729968U;
  c2_info[135].fileTimeHi = 0U;
  c2_info[135].mFileTimeLo = 0U;
  c2_info[135].mFileTimeHi = 0U;
}

static real_T c2_sqrt(SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance,
                      real_T c2_x)
{
  real_T c2_b_x;
  c2_b_x = c2_x;
  c2_b_sqrt(chartInstance, &c2_b_x);
  return c2_b_x;
}

static void c2_eml_error(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance)
{
  int32_T c2_i21;
  static char_T c2_varargin_1[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o',
    'o', 'l', 'b', 'o', 'x', ':', 's', 'q', 'r', 't', '_', 'd', 'o', 'm', 'a',
    'i', 'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c2_u[30];
  const mxArray *c2_y = NULL;
  for (c2_i21 = 0; c2_i21 < 30; c2_i21++) {
    c2_u[c2_i21] = c2_varargin_1[c2_i21];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c2_y));
}

static void c2_eml_scalar_eg(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance)
{
}

static void c2_b_eml_scalar_eg(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance)
{
}

static void c2_realmin(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance)
{
}

static void c2_eps(SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance)
{
}

static void c2_eml_int_forloop_overflow_check
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance)
{
}

static void c2_mpower(SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance,
                      real_T c2_c[16])
{
  c2_matrix_to_integer_power(chartInstance, c2_c);
}

static void c2_matrix_to_integer_power
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance, real_T c2_c[16])
{
  int32_T c2_i22;
  static real_T c2_dv2[16] = { 0.0, 8.0834517164645556E-6, 4.3325606937667151E-7,
    -0.00028067540682168596, -8.0834517164645556E-6, 0.0, -4.3325606937667151E-7,
    -0.00028067540682168596, 0.0, -8.0834517164645556E-6, 4.3325606937667151E-7,
    -0.00028067540682168596, 8.0834517164645556E-6, 0.0, -4.3325606937667151E-7,
    -0.00028067540682168596 };

  real_T c2_a[16];
  real_T c2_e;
  boolean_T c2_firstmult;
  real_T c2_x;
  real_T c2_ed2;
  real_T c2_b;
  real_T c2_y;
  int32_T c2_i23;
  int32_T c2_i24;
  real_T c2_A[16];
  int32_T c2_i25;
  int32_T c2_i26;
  int32_T c2_i27;
  int32_T c2_i28;
  int32_T c2_i29;
  int32_T c2_i30;
  int32_T c2_i31;
  real_T c2_b_A[16];
  int32_T c2_i32;
  real_T c2_c_A[16];
  real_T c2_n1x;
  int32_T c2_i33;
  real_T c2_b_c[16];
  real_T c2_n1xinv;
  real_T c2_b_a;
  real_T c2_b_b;
  real_T c2_b_y;
  real_T c2_rc;
  real_T c2_b_x;
  boolean_T c2_c_b;
  real_T c2_c_x;
  int32_T c2_i34;
  static char_T c2_cv0[8] = { '%', '%', '%', 'd', '.', '%', 'd', 'e' };

  char_T c2_u[8];
  const mxArray *c2_c_y = NULL;
  real_T c2_b_u;
  const mxArray *c2_d_y = NULL;
  real_T c2_c_u;
  const mxArray *c2_e_y = NULL;
  real_T c2_d_u;
  const mxArray *c2_f_y = NULL;
  char_T c2_str[14];
  int32_T c2_i35;
  char_T c2_b_str[14];
  int32_T c2_i36;
  int32_T c2_i37;
  int32_T c2_i38;
  int32_T c2_i39;
  int32_T c2_i40;
  int32_T c2_i41;
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  int32_T exitg1;
  for (c2_i22 = 0; c2_i22 < 16; c2_i22++) {
    c2_a[c2_i22] = c2_dv2[c2_i22];
  }

  c2_eml_scalar_eg(chartInstance);
  c2_e = 1.0;
  c2_firstmult = TRUE;
  do {
    exitg1 = 0;
    c2_x = c2_e / 2.0;
    c2_ed2 = c2_x;
    c2_ed2 = muDoubleScalarFloor(c2_ed2);
    c2_b = c2_ed2;
    c2_y = 2.0 * c2_b;
    if (c2_y != c2_e) {
      if (c2_firstmult) {
        for (c2_i23 = 0; c2_i23 < 16; c2_i23++) {
          c2_c[c2_i23] = c2_a[c2_i23];
        }

        c2_firstmult = FALSE;
      } else {
        c2_b_eml_scalar_eg(chartInstance);
        c2_b_eml_scalar_eg(chartInstance);
        for (c2_i24 = 0; c2_i24 < 16; c2_i24++) {
          c2_A[c2_i24] = c2_c[c2_i24];
        }

        for (c2_i25 = 0; c2_i25 < 4; c2_i25++) {
          c2_i26 = 0;
          for (c2_i27 = 0; c2_i27 < 4; c2_i27++) {
            c2_c[c2_i26 + c2_i25] = 0.0;
            c2_i28 = 0;
            for (c2_i29 = 0; c2_i29 < 4; c2_i29++) {
              c2_c[c2_i26 + c2_i25] += c2_A[c2_i28 + c2_i25] * c2_a[c2_i29 +
                c2_i26];
              c2_i28 += 4;
            }

            c2_i26 += 4;
          }
        }
      }
    }

    if (c2_ed2 == 0.0) {
      exitg1 = 1;
    } else {
      c2_e = c2_ed2;
      c2_b_eml_scalar_eg(chartInstance);
      c2_b_eml_scalar_eg(chartInstance);
      for (c2_i36 = 0; c2_i36 < 16; c2_i36++) {
        c2_A[c2_i36] = c2_a[c2_i36];
      }

      for (c2_i37 = 0; c2_i37 < 4; c2_i37++) {
        c2_i38 = 0;
        for (c2_i39 = 0; c2_i39 < 4; c2_i39++) {
          c2_a[c2_i38 + c2_i37] = 0.0;
          c2_i40 = 0;
          for (c2_i41 = 0; c2_i41 < 4; c2_i41++) {
            c2_a[c2_i38 + c2_i37] += c2_A[c2_i40 + c2_i37] * c2_A[c2_i41 +
              c2_i38];
            c2_i40 += 4;
          }

          c2_i38 += 4;
        }
      }
    }
  } while (exitg1 == 0U);

  for (c2_i30 = 0; c2_i30 < 16; c2_i30++) {
    c2_A[c2_i30] = c2_c[c2_i30];
  }

  for (c2_i31 = 0; c2_i31 < 16; c2_i31++) {
    c2_b_A[c2_i31] = c2_A[c2_i31];
  }

  c2_invNxN(chartInstance, c2_b_A, c2_c);
  for (c2_i32 = 0; c2_i32 < 16; c2_i32++) {
    c2_c_A[c2_i32] = c2_A[c2_i32];
  }

  c2_n1x = c2_norm(chartInstance, c2_c_A);
  for (c2_i33 = 0; c2_i33 < 16; c2_i33++) {
    c2_b_c[c2_i33] = c2_c[c2_i33];
  }

  c2_n1xinv = c2_norm(chartInstance, c2_b_c);
  c2_b_a = c2_n1x;
  c2_b_b = c2_n1xinv;
  c2_b_y = c2_b_a * c2_b_b;
  c2_rc = 1.0 / c2_b_y;
  guard1 = FALSE;
  guard2 = FALSE;
  if (c2_n1x == 0.0) {
    guard2 = TRUE;
  } else if (c2_n1xinv == 0.0) {
    guard2 = TRUE;
  } else if (c2_rc == 0.0) {
    guard1 = TRUE;
  } else {
    c2_b_x = c2_rc;
    c2_c_b = muDoubleScalarIsNaN(c2_b_x);
    guard3 = FALSE;
    if (c2_c_b) {
      guard3 = TRUE;
    } else {
      c2_eps(chartInstance);
      if (c2_rc < 2.2204460492503131E-16) {
        guard3 = TRUE;
      }
    }

    if (guard3 == TRUE) {
      c2_c_x = c2_rc;
      for (c2_i34 = 0; c2_i34 < 8; c2_i34++) {
        c2_u[c2_i34] = c2_cv0[c2_i34];
      }

      c2_c_y = NULL;
      sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 8),
                    FALSE);
      c2_b_u = 14.0;
      c2_d_y = NULL;
      sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c2_c_u = 6.0;
      c2_e_y = NULL;
      sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c2_d_u = c2_c_x;
      c2_f_y = NULL;
      sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c2_d_emlrt_marshallIn(chartInstance, sf_mex_call_debug("sprintf", 1U, 2U,
        14, sf_mex_call_debug("sprintf", 1U, 3U, 14, c2_c_y, 14, c2_d_y, 14,
        c2_e_y), 14, c2_f_y), "sprintf", c2_str);
      for (c2_i35 = 0; c2_i35 < 14; c2_i35++) {
        c2_b_str[c2_i35] = c2_str[c2_i35];
      }

      c2_b_eml_warning(chartInstance, c2_b_str);
    }
  }

  if (guard2 == TRUE) {
    guard1 = TRUE;
  }

  if (guard1 == TRUE) {
    c2_eml_warning(chartInstance);
  }
}

static void c2_invNxN(SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance,
                      real_T c2_x[16], real_T c2_y[16])
{
  int32_T c2_i42;
  int32_T c2_info;
  int32_T c2_ipiv[4];
  int32_T c2_i43;
  int32_T c2_p[4];
  int32_T c2_k;
  real_T c2_b_k;
  int32_T c2_ipk;
  int32_T c2_pipk;
  int32_T c2_c_k;
  int32_T c2_d_k;
  int32_T c2_c;
  int32_T c2_e_k;
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_a;
  int32_T c2_i44;
  int32_T c2_i;
  int32_T c2_b_i;
  real_T c2_b_a;
  real_T c2_b;
  real_T c2_b_y;
  int32_T c2_i45;
  real_T c2_b_x[16];
  for (c2_i42 = 0; c2_i42 < 16; c2_i42++) {
    c2_y[c2_i42] = 0.0;
  }

  c2_b_eml_matlab_zgetrf(chartInstance, c2_x, c2_ipiv, &c2_info);
  for (c2_i43 = 0; c2_i43 < 4; c2_i43++) {
    c2_p[c2_i43] = 1 + c2_i43;
  }

  for (c2_k = 0; c2_k < 3; c2_k++) {
    c2_b_k = 1.0 + (real_T)c2_k;
    c2_ipk = c2_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
      ("", c2_b_k), 1, 4, 1, 0) - 1];
    if ((real_T)c2_ipk > c2_b_k) {
      c2_pipk = c2_p[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
        ("", (real_T)c2_ipk), 1, 4, 1, 0) - 1];
      c2_p[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_ipk), 1, 4, 1, 0) - 1] = c2_p[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", c2_b_k), 1, 4, 1, 0) - 1];
      c2_p[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        c2_b_k), 1, 4, 1, 0) - 1] = c2_pipk;
    }
  }

  c2_c_eml_int_forloop_overflow_check(chartInstance);
  for (c2_c_k = 1; c2_c_k < 5; c2_c_k++) {
    c2_d_k = c2_c_k;
    c2_c = c2_p[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_d_k), 1, 4, 1, 0) - 1];
    c2_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_d_k), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_c), 1, 4, 2, 0) - 1) <<
           2)) - 1] = 1.0;
    c2_e_k = c2_d_k;
    c2_d_eml_int_forloop_overflow_check(chartInstance, c2_e_k, 4);
    for (c2_j = c2_e_k; c2_j < 5; c2_j++) {
      c2_b_j = c2_j;
      if (c2_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c2_b_j), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_c), 1, 4, 2, 0) - 1) <<
            2)) - 1] != 0.0) {
        c2_a = c2_b_j + 1;
        c2_i44 = c2_a;
        c2_d_eml_int_forloop_overflow_check(chartInstance, c2_i44, 4);
        for (c2_i = c2_i44; c2_i < 5; c2_i++) {
          c2_b_i = c2_i;
          c2_b_a = c2_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 4, 1, 0) +
                         ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_c), 1, 4, 2, 0) - 1) << 2)) - 1];
          c2_b = c2_x[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 4, 1, 0) +
                       ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 4, 2, 0) - 1) << 2)) - 1];
          c2_b_y = c2_b_a * c2_b;
          c2_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c2_b_i), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK(
                   "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_c), 1, 4, 2, 0)
                  - 1) << 2)) - 1] = c2_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 4, 1, 0) +
            ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_c), 1, 4, 2, 0) - 1) << 2)) - 1] - c2_b_y;
        }
      }
    }
  }

  for (c2_i45 = 0; c2_i45 < 16; c2_i45++) {
    c2_b_x[c2_i45] = c2_x[c2_i45];
  }

  c2_b_eml_xtrsm(chartInstance, c2_b_x, c2_y);
}

static void c2_eml_matlab_zgetrf(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_b_A[16], int32_T c2_ipiv[4],
  int32_T *c2_info)
{
  int32_T c2_i46;
  for (c2_i46 = 0; c2_i46 < 16; c2_i46++) {
    c2_b_A[c2_i46] = c2_A[c2_i46];
  }

  c2_b_eml_matlab_zgetrf(chartInstance, c2_b_A, c2_ipiv, c2_info);
}

static void c2_b_eml_int_forloop_overflow_check
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance, int32_T c2_b)
{
  int32_T c2_b_b;
  boolean_T c2_overflow;
  boolean_T c2_safe;
  int32_T c2_i47;
  static char_T c2_cv1[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c2_u[34];
  const mxArray *c2_y = NULL;
  int32_T c2_i48;
  static char_T c2_cv2[5] = { 'i', 'n', 't', '3', '2' };

  char_T c2_b_u[5];
  const mxArray *c2_b_y = NULL;
  c2_b_b = c2_b;
  if (2 > c2_b_b) {
    c2_overflow = FALSE;
  } else {
    c2_overflow = (c2_b_b > 2147483646);
  }

  c2_safe = !c2_overflow;
  if (c2_safe) {
  } else {
    for (c2_i47 = 0; c2_i47 < 34; c2_i47++) {
      c2_u[c2_i47] = c2_cv1[c2_i47];
    }

    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c2_i48 = 0; c2_i48 < 5; c2_i48++) {
      c2_b_u[c2_i48] = c2_cv2[c2_i48];
    }

    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 5),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c2_y, 14, c2_b_y));
  }
}

static void c2_c_eml_int_forloop_overflow_check
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance)
{
}

static void c2_d_eml_int_forloop_overflow_check
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance, int32_T c2_a,
   int32_T c2_b)
{
  int32_T c2_b_a;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  boolean_T c2_safe;
  int32_T c2_i49;
  static char_T c2_cv3[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c2_u[34];
  const mxArray *c2_y = NULL;
  int32_T c2_i50;
  static char_T c2_cv4[5] = { 'i', 'n', 't', '3', '2' };

  char_T c2_b_u[5];
  const mxArray *c2_b_y = NULL;
  c2_b_a = c2_a;
  c2_b_b = c2_b;
  if (c2_b_a > c2_b_b) {
    c2_overflow = FALSE;
  } else {
    c2_overflow = (c2_b_b > 2147483646);
  }

  c2_safe = !c2_overflow;
  if (c2_safe) {
  } else {
    for (c2_i49 = 0; c2_i49 < 34; c2_i49++) {
      c2_u[c2_i49] = c2_cv3[c2_i49];
    }

    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c2_i50 = 0; c2_i50 < 5; c2_i50++) {
      c2_b_u[c2_i50] = c2_cv4[c2_i50];
    }

    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 5),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c2_y, 14, c2_b_y));
  }
}

static void c2_eml_xgeru(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0,
  int32_T c2_iy0, real_T c2_A[16], int32_T c2_ia0, real_T c2_b_A[16])
{
  int32_T c2_i51;
  for (c2_i51 = 0; c2_i51 < 16; c2_i51++) {
    c2_b_A[c2_i51] = c2_A[c2_i51];
  }

  c2_b_eml_xgeru(chartInstance, c2_m, c2_n, c2_alpha1, c2_ix0, c2_iy0, c2_b_A,
                 c2_ia0);
}

static void c2_eml_xtrsm(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_B[16], real_T c2_b_B[16])
{
  int32_T c2_i52;
  int32_T c2_i53;
  real_T c2_b_A[16];
  for (c2_i52 = 0; c2_i52 < 16; c2_i52++) {
    c2_b_B[c2_i52] = c2_B[c2_i52];
  }

  for (c2_i53 = 0; c2_i53 < 16; c2_i53++) {
    c2_b_A[c2_i53] = c2_A[c2_i53];
  }

  c2_b_eml_xtrsm(chartInstance, c2_b_A, c2_b_B);
}

static void c2_e_eml_int_forloop_overflow_check
  (SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance)
{
}

static real_T c2_norm(SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance,
                      real_T c2_x[16])
{
  real_T c2_y;
  int32_T c2_j;
  real_T c2_b_j;
  real_T c2_s;
  int32_T c2_i;
  real_T c2_b_i;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_b_y;
  real_T c2_d_x;
  boolean_T c2_b;
  boolean_T exitg1;
  c2_y = 0.0;
  c2_j = 0;
  exitg1 = FALSE;
  while ((exitg1 == 0U) && (c2_j < 4)) {
    c2_b_j = 1.0 + (real_T)c2_j;
    c2_s = 0.0;
    for (c2_i = 0; c2_i < 4; c2_i++) {
      c2_b_i = 1.0 + (real_T)c2_i;
      c2_b_x = c2_x[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", c2_b_i), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", c2_b_j), 1, 4, 2, 0) - 1) << 2)) - 1];
      c2_c_x = c2_b_x;
      c2_b_y = muDoubleScalarAbs(c2_c_x);
      c2_s += c2_b_y;
    }

    c2_d_x = c2_s;
    c2_b = muDoubleScalarIsNaN(c2_d_x);
    if (c2_b) {
      c2_y = rtNaN;
      exitg1 = TRUE;
    } else {
      if (c2_s > c2_y) {
        c2_y = c2_s;
      }

      c2_j++;
    }
  }

  return c2_y;
}

static void c2_eml_warning(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance)
{
  int32_T c2_i54;
  static char_T c2_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c2_u[27];
  const mxArray *c2_y = NULL;
  for (c2_i54 = 0; c2_i54 < 27; c2_i54++) {
    c2_u[c2_i54] = c2_varargin_1[c2_i54];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 27), FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c2_y));
}

static void c2_b_eml_warning(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, char_T c2_varargin_2[14])
{
  int32_T c2_i55;
  static char_T c2_varargin_1[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'i', 'l', 'l', 'C', 'o', 'n', 'd', 'i', 't', 'i',
    'o', 'n', 'e', 'd', 'M', 'a', 't', 'r', 'i', 'x' };

  char_T c2_u[33];
  const mxArray *c2_y = NULL;
  int32_T c2_i56;
  char_T c2_b_u[14];
  const mxArray *c2_b_y = NULL;
  for (c2_i55 = 0; c2_i55 < 33; c2_i55++) {
    c2_u[c2_i55] = c2_varargin_1[c2_i55];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 33), FALSE);
  for (c2_i56 = 0; c2_i56 < 14; c2_i56++) {
    c2_b_u[c2_i56] = c2_varargin_2[c2_i56];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 14),
                FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
    14, c2_y, 14, c2_b_y));
}

static void c2_c_eml_scalar_eg(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance)
{
}

static void c2_d_emlrt_marshallIn(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, const mxArray *c2_sprintf, const char_T *c2_identifier, char_T
  c2_y[14])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_sprintf), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_sprintf);
}

static void c2_e_emlrt_marshallIn(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  char_T c2_y[14])
{
  char_T c2_cv5[14];
  int32_T c2_i57;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_cv5, 1, 10, 0U, 1, 0U, 2, 1,
                14);
  for (c2_i57 = 0; c2_i57 < 14; c2_i57++) {
    c2_y[c2_i57] = c2_cv5[c2_i57];
  }

  sf_mex_destroy(&c2_u);
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance;
  chartInstance = (SFc2_progetto3TenzoDecoupleV2InstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static int32_T c2_f_emlrt_marshallIn(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i58;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i58, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i58;
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
  SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance;
  chartInstance = (SFc2_progetto3TenzoDecoupleV2InstanceStruct *)
    chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_g_emlrt_marshallIn(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_progetto3TenzoDecoupleV2,
  const char_T *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_progetto3TenzoDecoupleV2), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_progetto3TenzoDecoupleV2);
  return c2_y;
}

static uint8_T c2_h_emlrt_marshallIn(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sqrt(SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance,
                      real_T *c2_x)
{
  if (*c2_x < 0.0) {
    c2_eml_error(chartInstance);
  }

  *c2_x = muDoubleScalarSqrt(*c2_x);
}

static void c2_b_eml_matlab_zgetrf(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, real_T c2_A[16], int32_T c2_ipiv[4], int32_T *c2_info)
{
  int32_T c2_i59;
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_a;
  int32_T c2_jm1;
  int32_T c2_b;
  int32_T c2_mmj;
  int32_T c2_b_a;
  int32_T c2_c;
  int32_T c2_b_b;
  int32_T c2_jj;
  int32_T c2_c_a;
  int32_T c2_jp1j;
  int32_T c2_d_a;
  int32_T c2_b_c;
  int32_T c2_n;
  int32_T c2_ix0;
  int32_T c2_b_n;
  int32_T c2_b_ix0;
  int32_T c2_c_n;
  int32_T c2_c_ix0;
  int32_T c2_idxmax;
  int32_T c2_ix;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_y;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_b_y;
  real_T c2_smax;
  int32_T c2_loop_ub;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_e_a;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_c_y;
  real_T c2_i_x;
  real_T c2_j_x;
  real_T c2_d_y;
  real_T c2_s;
  int32_T c2_f_a;
  int32_T c2_jpiv_offset;
  int32_T c2_g_a;
  int32_T c2_c_b;
  int32_T c2_jpiv;
  int32_T c2_h_a;
  int32_T c2_d_b;
  int32_T c2_c_c;
  int32_T c2_e_b;
  int32_T c2_jrow;
  int32_T c2_i_a;
  int32_T c2_f_b;
  int32_T c2_jprow;
  int32_T c2_d_ix0;
  int32_T c2_iy0;
  int32_T c2_e_ix0;
  int32_T c2_b_iy0;
  int32_T c2_f_ix0;
  int32_T c2_c_iy0;
  int32_T c2_b_ix;
  int32_T c2_iy;
  int32_T c2_c_k;
  real_T c2_temp;
  int32_T c2_j_a;
  int32_T c2_k_a;
  int32_T c2_b_jp1j;
  int32_T c2_l_a;
  int32_T c2_d_c;
  int32_T c2_m_a;
  int32_T c2_g_b;
  int32_T c2_i60;
  int32_T c2_i;
  int32_T c2_b_i;
  real_T c2_k_x;
  real_T c2_e_y;
  real_T c2_z;
  int32_T c2_h_b;
  int32_T c2_e_c;
  int32_T c2_n_a;
  int32_T c2_f_c;
  int32_T c2_o_a;
  int32_T c2_g_c;
  real_T c2_d1;
  c2_realmin(chartInstance);
  c2_eps(chartInstance);
  for (c2_i59 = 0; c2_i59 < 4; c2_i59++) {
    c2_ipiv[c2_i59] = 1 + c2_i59;
  }

  *c2_info = 0;
  c2_eml_int_forloop_overflow_check(chartInstance);
  for (c2_j = 1; c2_j < 4; c2_j++) {
    c2_b_j = c2_j;
    c2_a = c2_b_j - 1;
    c2_jm1 = c2_a;
    c2_b = c2_b_j;
    c2_mmj = 4 - c2_b;
    c2_b_a = c2_jm1;
    c2_c = c2_b_a * 5;
    c2_b_b = c2_c + 1;
    c2_jj = c2_b_b;
    c2_c_a = c2_jj + 1;
    c2_jp1j = c2_c_a;
    c2_d_a = c2_mmj;
    c2_b_c = c2_d_a;
    c2_n = c2_b_c + 1;
    c2_ix0 = c2_jj;
    c2_b_n = c2_n;
    c2_b_ix0 = c2_ix0;
    c2_c_n = c2_b_n;
    c2_c_ix0 = c2_b_ix0;
    if ((real_T)c2_c_n < 1.0) {
      c2_idxmax = 0;
    } else {
      c2_idxmax = 1;
      if ((real_T)c2_c_n > 1.0) {
        c2_ix = c2_c_ix0;
        c2_x = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_ix), 1, 16, 1, 0) - 1];
        c2_b_x = c2_x;
        c2_c_x = c2_b_x;
        c2_y = muDoubleScalarAbs(c2_c_x);
        c2_d_x = 0.0;
        c2_e_x = c2_d_x;
        c2_b_y = muDoubleScalarAbs(c2_e_x);
        c2_smax = c2_y + c2_b_y;
        c2_b_eml_int_forloop_overflow_check(chartInstance, c2_c_n);
        c2_loop_ub = c2_c_n;
        for (c2_k = 2; c2_k <= c2_loop_ub; c2_k++) {
          c2_b_k = c2_k;
          c2_e_a = c2_ix + 1;
          c2_ix = c2_e_a;
          c2_f_x = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_ix), 1, 16, 1, 0) - 1];
          c2_g_x = c2_f_x;
          c2_h_x = c2_g_x;
          c2_c_y = muDoubleScalarAbs(c2_h_x);
          c2_i_x = 0.0;
          c2_j_x = c2_i_x;
          c2_d_y = muDoubleScalarAbs(c2_j_x);
          c2_s = c2_c_y + c2_d_y;
          if (c2_s > c2_smax) {
            c2_idxmax = c2_b_k;
            c2_smax = c2_s;
          }
        }
      }
    }

    c2_f_a = c2_idxmax - 1;
    c2_jpiv_offset = c2_f_a;
    c2_g_a = c2_jj;
    c2_c_b = c2_jpiv_offset;
    c2_jpiv = c2_g_a + c2_c_b;
    if (c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_jpiv), 1, 16, 1, 0) - 1] != 0.0) {
      if ((real_T)c2_jpiv_offset != 0.0) {
        c2_h_a = c2_b_j;
        c2_d_b = c2_jpiv_offset;
        c2_c_c = c2_h_a + c2_d_b;
        c2_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_j), 1, 4, 1, 0) - 1] = c2_c_c;
        c2_e_b = c2_jm1 + 1;
        c2_jrow = c2_e_b;
        c2_i_a = c2_jrow;
        c2_f_b = c2_jpiv_offset;
        c2_jprow = c2_i_a + c2_f_b;
        c2_d_ix0 = c2_jrow;
        c2_iy0 = c2_jprow;
        c2_e_ix0 = c2_d_ix0;
        c2_b_iy0 = c2_iy0;
        c2_f_ix0 = c2_e_ix0;
        c2_c_iy0 = c2_b_iy0;
        c2_b_ix = c2_f_ix0;
        c2_iy = c2_c_iy0;
        c2_c_eml_int_forloop_overflow_check(chartInstance);
        for (c2_c_k = 1; c2_c_k < 5; c2_c_k++) {
          c2_temp = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_ix), 1, 16, 1, 0) - 1];
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ix), 1, 16, 1, 0) - 1] =
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_iy), 1, 16, 1, 0) - 1];
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_iy), 1, 16, 1, 0) - 1] = c2_temp;
          c2_j_a = c2_b_ix + 4;
          c2_b_ix = c2_j_a;
          c2_k_a = c2_iy + 4;
          c2_iy = c2_k_a;
        }
      }

      c2_b_jp1j = c2_jp1j;
      c2_l_a = c2_mmj;
      c2_d_c = c2_l_a;
      c2_m_a = c2_jp1j;
      c2_g_b = c2_d_c - 1;
      c2_i60 = c2_m_a + c2_g_b;
      c2_d_eml_int_forloop_overflow_check(chartInstance, c2_b_jp1j, c2_i60);
      for (c2_i = c2_b_jp1j; c2_i <= c2_i60; c2_i++) {
        c2_b_i = c2_i;
        c2_k_x = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 16, 1, 0) - 1];
        c2_e_y = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_jj), 1, 16, 1, 0) - 1];
        c2_z = c2_k_x / c2_e_y;
        c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_i), 1, 16, 1, 0) - 1] = c2_z;
      }
    } else {
      *c2_info = c2_b_j;
    }

    c2_h_b = c2_b_j;
    c2_e_c = 4 - c2_h_b;
    c2_n_a = c2_jj;
    c2_f_c = c2_n_a;
    c2_o_a = c2_jj;
    c2_g_c = c2_o_a;
    c2_d1 = -1.0;
    c2_b_eml_xgeru(chartInstance, c2_mmj, c2_e_c, c2_d1, c2_jp1j, c2_f_c + 4,
                   c2_A, c2_g_c + 5);
  }

  if ((real_T)*c2_info == 0.0) {
    if (!(c2_A[15] != 0.0)) {
      *c2_info = 4;
    }
  }
}

static void c2_b_eml_xgeru(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0,
  int32_T c2_iy0, real_T c2_A[16], int32_T c2_ia0)
{
  int32_T c2_b_m;
  int32_T c2_b_n;
  real_T c2_b_alpha1;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_b_ia0;
  int32_T c2_c_m;
  int32_T c2_c_n;
  real_T c2_c_alpha1;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_c_ia0;
  int32_T c2_d_m;
  int32_T c2_d_n;
  real_T c2_d_alpha1;
  int32_T c2_d_ix0;
  int32_T c2_d_iy0;
  int32_T c2_d_ia0;
  int32_T c2_e_m;
  int32_T c2_e_n;
  real_T c2_e_alpha1;
  int32_T c2_e_ix0;
  int32_T c2_e_iy0;
  int32_T c2_e_ia0;
  int32_T c2_ixstart;
  int32_T c2_a;
  int32_T c2_jA;
  int32_T c2_jy;
  int32_T c2_loop_ub;
  int32_T c2_j;
  real_T c2_yjy;
  real_T c2_temp;
  int32_T c2_ix;
  int32_T c2_b;
  int32_T c2_i61;
  int32_T c2_b_a;
  int32_T c2_b_b;
  int32_T c2_i62;
  int32_T c2_ijA;
  int32_T c2_b_ijA;
  int32_T c2_c_a;
  int32_T c2_d_a;
  int32_T c2_e_a;
  c2_b_m = c2_m;
  c2_b_n = c2_n;
  c2_b_alpha1 = c2_alpha1;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_b_ia0 = c2_ia0;
  c2_c_m = c2_b_m;
  c2_c_n = c2_b_n;
  c2_c_alpha1 = c2_b_alpha1;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  c2_c_ia0 = c2_b_ia0;
  c2_d_m = c2_c_m;
  c2_d_n = c2_c_n;
  c2_d_alpha1 = c2_c_alpha1;
  c2_d_ix0 = c2_c_ix0;
  c2_d_iy0 = c2_c_iy0;
  c2_d_ia0 = c2_c_ia0;
  c2_e_m = c2_d_m;
  c2_e_n = c2_d_n;
  c2_e_alpha1 = c2_d_alpha1;
  c2_e_ix0 = c2_d_ix0;
  c2_e_iy0 = c2_d_iy0;
  c2_e_ia0 = c2_d_ia0;
  if (c2_e_alpha1 == 0.0) {
  } else {
    c2_ixstart = c2_e_ix0;
    c2_a = c2_e_ia0 - 1;
    c2_jA = c2_a;
    c2_jy = c2_e_iy0;
    c2_d_eml_int_forloop_overflow_check(chartInstance, 1, c2_e_n);
    c2_loop_ub = c2_e_n;
    for (c2_j = 1; c2_j <= c2_loop_ub; c2_j++) {
      c2_yjy = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_jy), 1, 16, 1, 0) - 1];
      if (c2_yjy != 0.0) {
        c2_temp = c2_yjy * c2_e_alpha1;
        c2_ix = c2_ixstart;
        c2_b = c2_jA + 1;
        c2_i61 = c2_b;
        c2_b_a = c2_e_m;
        c2_b_b = c2_jA;
        c2_i62 = c2_b_a + c2_b_b;
        c2_d_eml_int_forloop_overflow_check(chartInstance, c2_i61, c2_i62);
        for (c2_ijA = c2_i61; c2_ijA <= c2_i62; c2_ijA++) {
          c2_b_ijA = c2_ijA;
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ijA), 1, 16, 1, 0) - 1] =
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ijA), 1, 16, 1, 0) - 1] +
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_ix), 1, 16, 1, 0) - 1] * c2_temp;
          c2_c_a = c2_ix + 1;
          c2_ix = c2_c_a;
        }
      }

      c2_d_a = c2_jy + 4;
      c2_jy = c2_d_a;
      c2_e_a = c2_jA + 4;
      c2_jA = c2_e_a;
    }
  }
}

static void c2_b_eml_xtrsm(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_B[16])
{
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_a;
  int32_T c2_c;
  int32_T c2_b;
  int32_T c2_b_c;
  int32_T c2_b_b;
  int32_T c2_jBcol;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_b_a;
  int32_T c2_c_c;
  int32_T c2_c_b;
  int32_T c2_d_c;
  int32_T c2_d_b;
  int32_T c2_kAcol;
  int32_T c2_c_a;
  int32_T c2_e_b;
  int32_T c2_e_c;
  int32_T c2_d_a;
  int32_T c2_f_b;
  int32_T c2_f_c;
  int32_T c2_e_a;
  int32_T c2_g_b;
  int32_T c2_g_c;
  int32_T c2_f_a;
  int32_T c2_h_b;
  int32_T c2_h_c;
  real_T c2_x;
  real_T c2_y;
  real_T c2_z;
  int32_T c2_g_a;
  int32_T c2_i63;
  int32_T c2_i;
  int32_T c2_b_i;
  int32_T c2_h_a;
  int32_T c2_i_b;
  int32_T c2_i_c;
  int32_T c2_i_a;
  int32_T c2_j_b;
  int32_T c2_j_c;
  int32_T c2_j_a;
  int32_T c2_k_b;
  int32_T c2_k_c;
  int32_T c2_k_a;
  int32_T c2_l_b;
  int32_T c2_l_c;
  c2_c_eml_int_forloop_overflow_check(chartInstance);
  for (c2_j = 1; c2_j < 5; c2_j++) {
    c2_b_j = c2_j;
    c2_a = c2_b_j;
    c2_c = c2_a;
    c2_b = c2_c - 1;
    c2_b_c = c2_b << 2;
    c2_b_b = c2_b_c;
    c2_jBcol = c2_b_b;
    c2_e_eml_int_forloop_overflow_check(chartInstance);
    for (c2_k = 4; c2_k > 0; c2_k--) {
      c2_b_k = c2_k;
      c2_b_a = c2_b_k;
      c2_c_c = c2_b_a;
      c2_c_b = c2_c_c - 1;
      c2_d_c = c2_c_b << 2;
      c2_d_b = c2_d_c;
      c2_kAcol = c2_d_b;
      c2_c_a = c2_b_k;
      c2_e_b = c2_jBcol;
      c2_e_c = c2_c_a + c2_e_b;
      if (c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_e_c), 1, 16, 1, 0) - 1] != 0.0) {
        c2_d_a = c2_b_k;
        c2_f_b = c2_jBcol;
        c2_f_c = c2_d_a + c2_f_b;
        c2_e_a = c2_b_k;
        c2_g_b = c2_jBcol;
        c2_g_c = c2_e_a + c2_g_b;
        c2_f_a = c2_b_k;
        c2_h_b = c2_kAcol;
        c2_h_c = c2_f_a + c2_h_b;
        c2_x = c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_g_c), 1, 16, 1, 0) - 1];
        c2_y = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_h_c), 1, 16, 1, 0) - 1];
        c2_z = c2_x / c2_y;
        c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_f_c), 1, 16, 1, 0) - 1] = c2_z;
        c2_g_a = c2_b_k - 1;
        c2_i63 = c2_g_a;
        c2_d_eml_int_forloop_overflow_check(chartInstance, 1, c2_i63);
        for (c2_i = 1; c2_i <= c2_i63; c2_i++) {
          c2_b_i = c2_i;
          c2_h_a = c2_b_i;
          c2_i_b = c2_jBcol;
          c2_i_c = c2_h_a + c2_i_b;
          c2_i_a = c2_b_i;
          c2_j_b = c2_jBcol;
          c2_j_c = c2_i_a + c2_j_b;
          c2_j_a = c2_b_k;
          c2_k_b = c2_jBcol;
          c2_k_c = c2_j_a + c2_k_b;
          c2_k_a = c2_b_i;
          c2_l_b = c2_kAcol;
          c2_l_c = c2_k_a + c2_l_b;
          c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_i_c), 1, 16, 1, 0) - 1] =
            c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_j_c), 1, 16, 1, 0) - 1] -
            c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_k_c), 1, 16, 1, 0) - 1] *
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_l_c), 1, 16, 1, 0) - 1];
        }
      }
    }
  }
}

static void init_dsm_address_info(SFc2_progetto3TenzoDecoupleV2InstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c2_progetto3TenzoDecoupleV2_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2990908933U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3835117659U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1199222035U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(711565208U);
}

mxArray *sf_c2_progetto3TenzoDecoupleV2_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("DLhoKkMbF7P4lfAkEI0cfB");
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

static const mxArray *sf_get_sim_state_info_c2_progetto3TenzoDecoupleV2(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x5'type','srcId','name','auxInfo'{{M[1],M[5],T\"w1\",},{M[1],M[9],T\"w2\",},{M[1],M[10],T\"w3\",},{M[1],M[11],T\"w4\",},{M[8],M[0],T\"is_active_c2_progetto3TenzoDecoupleV2\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 5, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_progetto3TenzoDecoupleV2_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance;
    chartInstance = (SFc2_progetto3TenzoDecoupleV2InstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_progetto3TenzoDecoupleV2MachineNumber_,
           2,
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
          init_script_number_translation(_progetto3TenzoDecoupleV2MachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (_progetto3TenzoDecoupleV2MachineNumber_,chartInstance->chartNumber,
             1);
          sf_debug_set_chart_event_thresholds
            (_progetto3TenzoDecoupleV2MachineNumber_,
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
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);

        {
          real_T *c2_taux;
          real_T *c2_tauy;
          real_T *c2_tauz;
          real_T *c2_Thrust;
          real_T *c2_w1;
          real_T *c2_w2;
          real_T *c2_w3;
          real_T *c2_w4;
          c2_w4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
          c2_w3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
          c2_w2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c2_w1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c2_Thrust = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c2_tauz = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c2_tauy = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c2_taux = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c2_taux);
          _SFD_SET_DATA_VALUE_PTR(1U, c2_tauy);
          _SFD_SET_DATA_VALUE_PTR(2U, c2_tauz);
          _SFD_SET_DATA_VALUE_PTR(3U, c2_Thrust);
          _SFD_SET_DATA_VALUE_PTR(4U, c2_w1);
          _SFD_SET_DATA_VALUE_PTR(5U, c2_w2);
          _SFD_SET_DATA_VALUE_PTR(6U, c2_w3);
          _SFD_SET_DATA_VALUE_PTR(7U, c2_w4);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration
        (_progetto3TenzoDecoupleV2MachineNumber_,chartInstance->chartNumber,
         chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization()
{
  return "hk9ijbk27lKMvdVp76OTME";
}

static void sf_opaque_initialize_c2_progetto3TenzoDecoupleV2(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_progetto3TenzoDecoupleV2InstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c2_progetto3TenzoDecoupleV2
    ((SFc2_progetto3TenzoDecoupleV2InstanceStruct*) chartInstanceVar);
  initialize_c2_progetto3TenzoDecoupleV2
    ((SFc2_progetto3TenzoDecoupleV2InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_progetto3TenzoDecoupleV2(void *chartInstanceVar)
{
  enable_c2_progetto3TenzoDecoupleV2
    ((SFc2_progetto3TenzoDecoupleV2InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_progetto3TenzoDecoupleV2(void *chartInstanceVar)
{
  disable_c2_progetto3TenzoDecoupleV2
    ((SFc2_progetto3TenzoDecoupleV2InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_progetto3TenzoDecoupleV2(void *chartInstanceVar)
{
  sf_c2_progetto3TenzoDecoupleV2((SFc2_progetto3TenzoDecoupleV2InstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_progetto3TenzoDecoupleV2
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_progetto3TenzoDecoupleV2
    ((SFc2_progetto3TenzoDecoupleV2InstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_progetto3TenzoDecoupleV2();/* state var info */
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

extern void sf_internal_set_sim_state_c2_progetto3TenzoDecoupleV2(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_progetto3TenzoDecoupleV2();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_progetto3TenzoDecoupleV2
    ((SFc2_progetto3TenzoDecoupleV2InstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_progetto3TenzoDecoupleV2
  (SimStruct* S)
{
  return sf_internal_get_sim_state_c2_progetto3TenzoDecoupleV2(S);
}

static void sf_opaque_set_sim_state_c2_progetto3TenzoDecoupleV2(SimStruct* S,
  const mxArray *st)
{
  sf_internal_set_sim_state_c2_progetto3TenzoDecoupleV2(S, st);
}

static void sf_opaque_terminate_c2_progetto3TenzoDecoupleV2(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_progetto3TenzoDecoupleV2InstanceStruct*)
                    chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c2_progetto3TenzoDecoupleV2
      ((SFc2_progetto3TenzoDecoupleV2InstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_progetto3TenzoDecoupleV2_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_progetto3TenzoDecoupleV2
    ((SFc2_progetto3TenzoDecoupleV2InstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_progetto3TenzoDecoupleV2(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_progetto3TenzoDecoupleV2
      ((SFc2_progetto3TenzoDecoupleV2InstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_progetto3TenzoDecoupleV2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_progetto3TenzoDecoupleV2_optimization_info();
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
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,4);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1746115500U));
  ssSetChecksum1(S,(2445436000U));
  ssSetChecksum2(S,(30819630U));
  ssSetChecksum3(S,(3361784933U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c2_progetto3TenzoDecoupleV2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_progetto3TenzoDecoupleV2(SimStruct *S)
{
  SFc2_progetto3TenzoDecoupleV2InstanceStruct *chartInstance;
  chartInstance = (SFc2_progetto3TenzoDecoupleV2InstanceStruct *)malloc(sizeof
    (SFc2_progetto3TenzoDecoupleV2InstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_progetto3TenzoDecoupleV2InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_progetto3TenzoDecoupleV2;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_progetto3TenzoDecoupleV2;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_progetto3TenzoDecoupleV2;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c2_progetto3TenzoDecoupleV2;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c2_progetto3TenzoDecoupleV2;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_progetto3TenzoDecoupleV2;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_progetto3TenzoDecoupleV2;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_progetto3TenzoDecoupleV2;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_progetto3TenzoDecoupleV2;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_progetto3TenzoDecoupleV2;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_progetto3TenzoDecoupleV2;
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

void c2_progetto3TenzoDecoupleV2_method_dispatcher(SimStruct *S, int_T method,
  void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_progetto3TenzoDecoupleV2(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_progetto3TenzoDecoupleV2(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_progetto3TenzoDecoupleV2(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_progetto3TenzoDecoupleV2_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
