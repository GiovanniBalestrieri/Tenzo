#include "__cf_HinfTenzo.h"
#ifndef RTW_HEADER_HinfTenzo_h_
#define RTW_HEADER_HinfTenzo_h_
#include <stddef.h>
#include <string.h>
#include "rtw_modelmap.h"
#ifndef HinfTenzo_COMMON_INCLUDES_
#define HinfTenzo_COMMON_INCLUDES_
#include <stdlib.h>
#include "rtwtypes.h"
#include "simtarget/slSimTgtSigstreamRTW.h"
#include "simtarget/slSimTgtSlioCoreRTW.h"
#include "simtarget/slSimTgtSlioClientsRTW.h"
#include "sigstream_rtw.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "raccel.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "rt_logging.h"
#include "dt_info.h"
#include "ext_work.h"
#endif
#include "HinfTenzo_types.h"
#include "multiword_types.h"
#include "mwmathutil.h"
#include "rt_defines.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#define MODEL_NAME HinfTenzo
#define NSAMPLE_TIMES (3) 
#define NINPUTS (0)       
#define NOUTPUTS (0)     
#define NBLOCKIO (31) 
#define NUM_ZC_EVENTS (0) 
#ifndef NCSTATES
#define NCSTATES (49)   
#elif NCSTATES != 49
#error Invalid specification of NCSTATES defined in compiler command
#endif
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm) (*rt_dataMapInfoPtr)
#endif
#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val) (rt_dataMapInfoPtr = &val)
#endif
#ifndef IN_RACCEL_MAIN
#endif
typedef struct { real_T mikmmv5pjy ; real_T iglrdpeo0u ; real_T cuhucxnwbl [
4 ] ; real_T ndvxwhqhzh ; real_T kohnzyllz4 ; real_T lywwonuzni ; real_T
cgqon1bp5k ; real_T ccnrqukp0p [ 2 ] ; real_T o2ra02ecjo ; real_T g3cxppgcad
[ 4 ] ; real_T nq0m4x3haw ; real_T ffcuwotdjl ; real_T h0k2fzanuq ; real_T
l3ibipirjv ; real_T o4bvr14r1c ; real_T iklusr35kc ; real_T elhmsr12bw ;
real_T epfm0vmgr0 ; real_T dx0ibnxabm ; real_T ahq3gaiq3g ; real_T jfilnvyqtk
; real_T bnmysexqkh ; real_T afiufrufpa ; real_T eivqzdcosm ; real_T
bnmtoyr0ou ; real_T ji3tio223s ; real_T jf1eral2ve [ 4 ] ; real_T ngbxrb2p33
[ 4 ] ; real_T c33unhaobk ; real_T l5s1awmxqx ; real_T nue240oiht ; } B ;
typedef struct { real_T ox0fllt2el ; struct { void * LoggedData ; }
dbuidsfmth ; struct { void * LoggedData ; } gm2yjyanxh ; struct { void *
LoggedData ; } pd1kq3bjlk ; struct { void * LoggedData ; } ifwd1amcve ;
struct { void * LoggedData ; } fcc0uv3oo0 ; struct { void * LoggedData ; }
fet3nngant ; struct { void * LoggedData ; } hwrxgbpfob ; struct { void *
LoggedData ; } nnbpbghk5r ; struct { void * LoggedData ; } pvfh3hvin4 ;
struct { void * LoggedData ; } ed21y4p5jz ; struct { void * LoggedData ; }
fnl2bzwupn ; struct { void * LoggedData ; } euqmzld2p5 ; struct { void *
TimePtr ; void * DataPtr ; void * RSimInfoPtr ; } lmen2irgnh ; struct { void
* TimePtr ; void * DataPtr ; void * RSimInfoPtr ; } kxuk4bilk3 ; struct {
void * TimePtr ; void * DataPtr ; void * RSimInfoPtr ; } jmmcgd4r4a ; struct
{ void * TimePtr ; void * DataPtr ; void * RSimInfoPtr ; } mzmo2dtgp1 ;
struct { void * LoggedData ; } kbnckqutms ; uint32_T ic2yhbr32w ; int_T
ihm14vuvou [ 2 ] ; int_T mizp5pcuu3 ; struct { int_T PrevIndex ; } oosdo3rh02
; int_T fth52qijjp [ 2 ] ; int_T j5kfgeryef ; struct { int_T PrevIndex ; }
l2pbteie2e ; int_T pshgckdv2q [ 2 ] ; int_T nm2quutkdt ; struct { int_T
PrevIndex ; } agvnfrpxdp ; int_T mx32usueux [ 4 ] ; int_T i3sszjlhmx ; struct
{ int_T PrevIndex ; } n0unfq1ojc ; int_T dbgdqimhbs [ 4 ] ; boolean_T
nx03g0oqvx ; boolean_T k2y005jslo ; boolean_T lsikoznc1f ; boolean_T
pvf1npkacw ; boolean_T p424nko4vw ; } DW ; typedef struct { real_T ltfask1ptj
[ 2 ] ; real_T k5wabiqsuk [ 2 ] ; real_T b11juaaqeh [ 8 ] ; real_T oacfijplwi
[ 2 ] ; real_T njhzavtx5o [ 2 ] ; real_T hudxhgdxi2 [ 2 ] ; real_T l2dleorvkm
; real_T phhupngpjs [ 28 ] ; real_T inruainrus [ 2 ] ; } X ; typedef struct {
real_T ltfask1ptj [ 2 ] ; real_T k5wabiqsuk [ 2 ] ; real_T b11juaaqeh [ 8 ] ;
real_T oacfijplwi [ 2 ] ; real_T njhzavtx5o [ 2 ] ; real_T hudxhgdxi2 [ 2 ] ;
real_T l2dleorvkm ; real_T phhupngpjs [ 28 ] ; real_T inruainrus [ 2 ] ; }
XDot ; typedef struct { boolean_T ltfask1ptj [ 2 ] ; boolean_T k5wabiqsuk [ 2
] ; boolean_T b11juaaqeh [ 8 ] ; boolean_T oacfijplwi [ 2 ] ; boolean_T
njhzavtx5o [ 2 ] ; boolean_T hudxhgdxi2 [ 2 ] ; boolean_T l2dleorvkm ;
boolean_T phhupngpjs [ 28 ] ; boolean_T inruainrus [ 2 ] ; } XDis ; typedef
struct { real_T ltfask1ptj [ 2 ] ; real_T k5wabiqsuk [ 2 ] ; real_T
b11juaaqeh [ 8 ] ; real_T oacfijplwi [ 2 ] ; real_T njhzavtx5o [ 2 ] ; real_T
hudxhgdxi2 [ 2 ] ; real_T l2dleorvkm ; real_T phhupngpjs [ 28 ] ; real_T
inruainrus [ 2 ] ; } CStateAbsTol ; typedef struct { real_T l3ghldah3m ;
real_T g4rwffgs43 ; real_T fw4rhgfac4 ; real_T lefgzjm3hy ; real_T hfjunh22va
; real_T kc2m3kgcvp ; real_T m1bxttvtml ; real_T msymzkqbvl ; real_T
donjsqodtq ; real_T huw2sg4ptv [ 4 ] ; real_T mykym5a1ll [ 4 ] ; } ZCV ;
typedef struct { rtwCAPI_ModelMappingInfo mmi ; } DataMapInfo ; struct P_ {
real_T A0 [ 64 ] ; real_T B0 [ 32 ] ; real_T C0 [ 32 ] ; real_T
amplitudeNoise ; real_T amplitudePertOutZ ; real_T amplitudePertOutptp ;
real_T cstPertOut ; real_T omegaNoise ; real_T omegaPertOut ; real_T
randomAmpNoise ; real_T Ps1_A [ 2 ] ; real_T Ps1_C [ 2 ] ; real_T
Ps1_A_j1blkzytxk [ 2 ] ; real_T Ps1_C_a4z2vpeibb [ 2 ] ; real_T Processo1_X0
; real_T Ps4_A [ 2 ] ; real_T Ps4_C [ 2 ] ; real_T Constant2_Value ; real_T
Ps5_A [ 2 ] ; real_T Ps5_C [ 2 ] ; real_T Ps1_A_g40xqitwcm [ 2 ] ; real_T
Ps1_C_jxo2syryaa [ 2 ] ; real_T Constant1_Value ; real_T Ps1_A_cnuk0h4tar ;
real_T Ps1_C_gfroigzrf4 ; real_T Gain_Gain ; real_T HInfinity_A [ 604 ] ;
real_T HInfinity_B [ 112 ] ; real_T HInfinity_C [ 112 ] ; real_T HInfinity_X0
; real_T Ps1_A_p4fqdq0nyf [ 2 ] ; real_T Ps1_C_iunwv5jncy [ 2 ] ; real_T
Constant1_Value_onxqoxz4b2 ; real_T Constant2_Value_honrvu2u0e ; real_T
seno_Amp ; real_T seno_Bias ; real_T seno_Phase ; real_T Switch1_Threshold ;
real_T Switch2_Threshold ; real_T Constant_Value ; real_T
Constant2_Value_maeni3njql ; real_T SineOut_Bias ; real_T SineOut_Phase ;
real_T Switch_Threshold ; real_T Switch1_Threshold_a0fpu4ohya ; real_T
Constant_Value_gsg5wrsjtq ; real_T SineOut_Bias_amml0uhsh5 ; real_T
SineOut_Phase_ek4wf0nzh3 ; real_T Switch_Threshold_je2vilxnqj ; real_T
u2sen05t_Bias ; real_T u2sen05t_Phase ; real_T UniformRandomNumber_Seed ;
real_T FromWs_Time0 [ 6 ] ; real_T FromWs_Data0 [ 6 ] ; real_T
FromWs_Time0_mdh1vup2gy [ 6 ] ; real_T FromWs_Data0_amonalu2nw [ 6 ] ; real_T
FromWs_Time0_ni0stwjpd1 [ 6 ] ; real_T FromWs_Data0_bui33k5vfq [ 6 ] ; real_T
FromWs_Time0_ovshumjb0b [ 10 ] ; real_T FromWs_Data0_cjzjycivf4 [ 10 ] ;
real_T Saturation1_UpperSat ; real_T Saturation1_LowerSat ; uint8_T
ManualSwitch1_CurrentSetting ; } ; extern const char *
RT_MEMORY_ALLOCATION_ERROR ; extern B rtB ; extern X rtX ; extern DW rtDW ;
extern P rtP ; extern const rtwCAPI_ModelMappingStaticInfo *
HinfTenzo_GetCAPIStaticMap ( void ) ; extern SimStruct * const rtS ; extern
const int_T gblNumToFiles ; extern const int_T gblNumFrFiles ; extern const
int_T gblNumFrWksBlocks ; extern rtInportTUtable * gblInportTUtables ; extern
const char * gblInportFileName ; extern const int_T gblNumRootInportBlks ;
extern const int_T gblNumModelInputs ; extern const int_T
gblInportDataTypeIdx [ ] ; extern const int_T gblInportDims [ ] ; extern
const int_T gblInportComplex [ ] ; extern const int_T gblInportInterpoFlag [
] ; extern const int_T gblInportContinuous [ ] ; extern const int_T
gblParameterTuningTid ; extern size_t gblCurrentSFcnIdx ; extern size_t *
gblChildIdxToInfoIdx ; extern DataMapInfo * rt_dataMapInfoPtr ; extern
rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr ; void MdlOutputs ( int_T tid )
; void MdlOutputsParameterSampleTime ( int_T tid ) ; void MdlUpdate ( int_T
tid ) ; void MdlTerminate ( void ) ; void MdlInitializeSizes ( void ) ; void
MdlInitializeSampleTimes ( void ) ; SimStruct * raccel_register_model ( void
) ;
#endif
