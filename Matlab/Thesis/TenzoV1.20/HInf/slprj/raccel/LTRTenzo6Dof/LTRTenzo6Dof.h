#include "__cf_LTRTenzo6Dof.h"
#ifndef RTW_HEADER_LTRTenzo6Dof_h_
#define RTW_HEADER_LTRTenzo6Dof_h_
#include <stddef.h>
#include <string.h>
#include "rtw_modelmap.h"
#ifndef LTRTenzo6Dof_COMMON_INCLUDES_
#define LTRTenzo6Dof_COMMON_INCLUDES_
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
#include "LTRTenzo6Dof_types.h"
#include "multiword_types.h"
#include "mwmathutil.h"
#include "rt_defines.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#define MODEL_NAME LTRTenzo6Dof
#define NSAMPLE_TIMES (4) 
#define NINPUTS (0)       
#define NOUTPUTS (0)     
#define NBLOCKIO (23) 
#define NUM_ZC_EVENTS (0) 
#ifndef NCSTATES
#define NCSTATES (23)   
#elif NCSTATES != 23
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
typedef struct { real_T oaz5l4crsf ; real_T m4f4roaota ; real_T l3ya5q3dsy [
4 ] ; real_T oxa3bf04z4 ; real_T kk4pom5z0m ; real_T c1do0agcsm ; real_T
aolioyabae ; real_T mg0az2nzgj ; real_T dt0bxeukl1 ; real_T m5z5sj11wq ;
real_T px0r25lgtw ; real_T lk5cyp2tjd ; real_T exersch3pk [ 8 ] ; real_T
fxeigzy1kf ; real_T fwztmrgmqg ; real_T k3xhv4ixsk ; real_T ajlqbn3dtj [ 4 ]
; real_T obr0rt4zff [ 8 ] ; real_T odgn1acl0s [ 3 ] ; real_T gjftqhkwh2 [ 3 ]
; real_T fezrnkyxs4 [ 4 ] ; real_T anbxmjzqcz ; real_T klytvkrzsu ; } B ;
typedef struct { real_T dcnnnsw5q4 ; struct { void * LoggedData ; }
ci4od0mju3 ; struct { void * LoggedData ; } at54wibglc ; struct { void *
LoggedData ; } mwii1vbwa3 ; struct { void * LoggedData ; } obppf3fwyv ;
struct { void * LoggedData ; } nkasfhaegb ; struct { void * LoggedData ; }
d4slrxby1k ; struct { void * LoggedData ; } dcwsregl10 ; struct { void *
LoggedData ; } n42qfil22t ; struct { void * LoggedData ; } klvshyuq1n ;
struct { void * LoggedData ; } i4oe2vbkte ; struct { void * LoggedData ; }
kuqz0fzjc5 ; struct { void * TimePtr ; void * DataPtr ; void * RSimInfoPtr ;
} iqmfoh3acq ; struct { void * TimePtr ; void * DataPtr ; void * RSimInfoPtr
; } luaideq0uj ; struct { void * TimePtr ; void * DataPtr ; void *
RSimInfoPtr ; } hkxntuepnx ; struct { void * TimePtr ; void * DataPtr ; void
* RSimInfoPtr ; } anvo41nxpw ; struct { void * LoggedData ; } ed3up5fmtr ;
struct { void * LoggedData [ 4 ] ; } pi5lysgpgt ; struct { void * LoggedData
; } nozkwoectk ; struct { void * LoggedData ; } ftg1pj35ds ; struct { void *
LoggedData ; } gd0ec4ttnt ; struct { void * LoggedData ; } bxixgcgfkj ;
struct { void * LoggedData [ 4 ] ; } iob4th0cd0 ; struct { void * LoggedData
; } fk1piers5l ; uint32_T awp2bz51sp ; int_T iqdzkc4qec [ 2 ] ; int_T
brdcus5eot ; struct { int_T PrevIndex ; } axjurmlx3x ; int_T kv51vtwzgi [ 4 ]
; int_T oekfi2tozz ; struct { int_T PrevIndex ; } ch2s1cyhcx ; int_T
a4hbqd2cbd [ 2 ] ; int_T hjtypnuisf ; struct { int_T PrevIndex ; } oh5roumt0o
; int_T elawudclil [ 4 ] ; int_T iabwzwhwhy ; struct { int_T PrevIndex ; }
lqstcqepka ; boolean_T htlyqa0psc ; boolean_T baoant1mbx ; } DW ; typedef
struct { real_T bhid251wmy [ 8 ] ; real_T hhto124xqa [ 2 ] ; real_T
lsb1kf2p0a [ 2 ] ; real_T del2gd1uu5 [ 2 ] ; real_T owky1kts1u [ 8 ] ; real_T
mnq2gjnihw ; } X ; typedef struct { real_T bhid251wmy [ 8 ] ; real_T
hhto124xqa [ 2 ] ; real_T lsb1kf2p0a [ 2 ] ; real_T del2gd1uu5 [ 2 ] ; real_T
owky1kts1u [ 8 ] ; real_T mnq2gjnihw ; } XDot ; typedef struct { boolean_T
bhid251wmy [ 8 ] ; boolean_T hhto124xqa [ 2 ] ; boolean_T lsb1kf2p0a [ 2 ] ;
boolean_T del2gd1uu5 [ 2 ] ; boolean_T owky1kts1u [ 8 ] ; boolean_T
mnq2gjnihw ; } XDis ; typedef struct { real_T bhid251wmy [ 8 ] ; real_T
hhto124xqa [ 2 ] ; real_T lsb1kf2p0a [ 2 ] ; real_T del2gd1uu5 [ 2 ] ; real_T
owky1kts1u [ 8 ] ; real_T mnq2gjnihw ; } CStateAbsTol ; typedef struct {
real_T eog5tsvrmm ; real_T oeq2vseenb ; real_T lh4evpqhkw ; real_T kouc5v3spc
; real_T immtuxkjm4 ; real_T iduqvk4ja1 ; } ZCV ; typedef struct {
rtwCAPI_ModelMappingInfo mmi ; } DataMapInfo ; struct P_ { real_T A0 [ 64 ] ;
real_T B0 [ 32 ] ; real_T C0 [ 32 ] ; real_T KAoss [ 64 ] ; real_T KBossw [
64 ] ; real_T KCoss [ 64 ] ; real_T Kopt [ 32 ] ; real_T amplitudeNoise ;
real_T amplitudePertIn ; real_T amplitudePertOut ; real_T cstPertIn ; real_T
cstPertOut ; real_T omegaNoise ; real_T omegaPertIN ; real_T omegaPertOut ;
real_T randomAmpNoise ; real_T SinOut_Bias ; real_T SinOut_Phase ; real_T
Constant_Value ; real_T Switch_Threshold ; real_T Processo1_X0 ; real_T Ps4_A
[ 2 ] ; real_T Ps4_C [ 2 ] ; real_T Constant2_Value ; real_T Ps5_A [ 2 ] ;
real_T Ps5_C [ 2 ] ; real_T Ps1_A [ 2 ] ; real_T Ps1_C [ 2 ] ; real_T
kalman_X0 ; real_T SineIn_Bias ; real_T SineIn_Phase ; real_T
Constant_Value_gakmm5of0a ; real_T Switch_Threshold_b2p15hzzi4 ; real_T
Constant1_Value ; real_T Ps1_A_cnuk0h4tar ; real_T Ps1_C_gfroigzrf4 ; real_T
Gain_Gain ; real_T u2sen05t_Bias ; real_T u2sen05t_Phase ; real_T
UniformRandomNumber_Seed ; real_T FromWs_Time0 [ 6 ] ; real_T FromWs_Data0 [
6 ] ; real_T FromWs_Time0_nyx1zce205 [ 10 ] ; real_T FromWs_Data0_cogeshtz5t
[ 10 ] ; real_T FromWs_Time0_lk1gq232s2 [ 6 ] ; real_T
FromWs_Data0_igdugvbssl [ 6 ] ; real_T FromWs_Time0_jv04arnttw [ 10 ] ;
real_T FromWs_Data0_pirid4ox1s [ 10 ] ; real_T Gain_Gain_l2aa1kgqdh ; real_T
Gain_Gain_gpep4bzie5 ; uint8_T ManualSwitch1_CurrentSetting ; } ; extern
const char * RT_MEMORY_ALLOCATION_ERROR ; extern B rtB ; extern X rtX ;
extern DW rtDW ; extern P rtP ; extern const rtwCAPI_ModelMappingStaticInfo *
LTRTenzo6Dof_GetCAPIStaticMap ( void ) ; extern SimStruct * const rtS ;
extern const int_T gblNumToFiles ; extern const int_T gblNumFrFiles ; extern
const int_T gblNumFrWksBlocks ; extern rtInportTUtable * gblInportTUtables ;
extern const char * gblInportFileName ; extern const int_T
gblNumRootInportBlks ; extern const int_T gblNumModelInputs ; extern const
int_T gblInportDataTypeIdx [ ] ; extern const int_T gblInportDims [ ] ;
extern const int_T gblInportComplex [ ] ; extern const int_T
gblInportInterpoFlag [ ] ; extern const int_T gblInportContinuous [ ] ;
extern const int_T gblParameterTuningTid ; extern size_t gblCurrentSFcnIdx ;
extern size_t * gblChildIdxToInfoIdx ; extern DataMapInfo * rt_dataMapInfoPtr
; extern rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr ; void MdlOutputs (
int_T tid ) ; void MdlOutputsParameterSampleTime ( int_T tid ) ; void
MdlUpdate ( int_T tid ) ; void MdlTerminate ( void ) ; void
MdlInitializeSizes ( void ) ; void MdlInitializeSampleTimes ( void ) ;
SimStruct * raccel_register_model ( void ) ;
#endif
