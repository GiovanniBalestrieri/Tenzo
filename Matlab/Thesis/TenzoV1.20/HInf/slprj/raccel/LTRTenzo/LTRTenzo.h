#include "__cf_LTRTenzo.h"
#ifndef RTW_HEADER_LTRTenzo_h_
#define RTW_HEADER_LTRTenzo_h_
#include <stddef.h>
#include <string.h>
#include "rtw_modelmap.h"
#ifndef LTRTenzo_COMMON_INCLUDES_
#define LTRTenzo_COMMON_INCLUDES_
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
#include "LTRTenzo_types.h"
#include "multiword_types.h"
#include "mwmathutil.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#define MODEL_NAME LTRTenzo
#define NSAMPLE_TIMES (4) 
#define NINPUTS (0)       
#define NOUTPUTS (0)     
#define NBLOCKIO (23) 
#define NUM_ZC_EVENTS (0) 
#ifndef NCSTATES
#define NCSTATES (25)   
#elif NCSTATES != 25
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
typedef struct { real_T mbsguhsyce ; real_T gxh0zyu42a ; real_T j4tv1cqyj2 [
4 ] ; real_T gbemrwhpr5 ; real_T bdgzagdxra [ 8 ] ; real_T bxeh32cm4i ;
real_T ej3n31dyxa ; real_T mfm3tpnt2p [ 8 ] ; real_T hjr0spzzaf [ 4 ] ;
real_T csh4yoneye [ 4 ] ; real_T fgbq3jefrx ; real_T d4wqki33zw ; real_T
d1v04lphp2 ; real_T hzoqt0nc3o ; real_T otap4hfn5r ; real_T cfdpbmvjgv ;
real_T edtxh40dpx ; real_T gz1cngd0fd [ 8 ] ; real_T exb2j2haif [ 3 ] ;
real_T ajinddapxi [ 3 ] ; real_T oyi1nkeowt [ 4 ] ; real_T kwjrknjxrb ;
real_T fd3jf50j03 ; } B ; typedef struct { real_T cx2opkhkks ; real_T
pzg1f0zcsj ; real_T gs42o1safk ; real_T bthpfg5v51 ; real_T gzecfrgu4x ;
struct { void * TimePtr ; void * DataPtr ; void * RSimInfoPtr ; } pp4sameqxw
; struct { void * TimePtr ; void * DataPtr ; void * RSimInfoPtr ; }
pszu0ywji3 ; struct { void * TimePtr ; void * DataPtr ; void * RSimInfoPtr ;
} gmpueztb1y ; struct { void * LoggedData ; } p00itljrev ; struct { void *
LoggedData ; } nzgtrzui4f ; struct { void * LoggedData ; } gx3ie0fm1y ;
struct { void * LoggedData ; } ej3s3keedd ; struct { void * LoggedData ; }
djeg5hbiuo ; struct { void * LoggedData ; } i3tuhdmgb2 ; struct { void *
LoggedData ; } kcxqmgqjiz ; struct { void * LoggedData ; } a2kafglxv4 ;
struct { void * LoggedData ; } h5swgbqjqf ; struct { void * LoggedData ; }
p4kdljjvdf ; struct { void * LoggedData ; } nrvudg0031 ; struct { void *
LoggedData ; } if2ae1wsvj ; struct { void * TimePtr ; void * DataPtr ; void *
RSimInfoPtr ; } mq2efjljlp ; struct { void * TimePtr ; void * DataPtr ; void
* RSimInfoPtr ; } morfgxatez ; struct { void * TimePtr ; void * DataPtr ;
void * RSimInfoPtr ; } d2vfv2ffdo ; struct { void * TimePtr ; void * DataPtr
; void * RSimInfoPtr ; } ctabrgnsxy ; struct { void * TimePtr ; void *
DataPtr ; void * RSimInfoPtr ; } nl25ijg5ia ; struct { void * LoggedData ; }
eolcyfqkw1 ; struct { void * LoggedData [ 4 ] ; } mottvgtufs ; struct { void
* LoggedData ; } d3pymdz53u ; struct { void * LoggedData ; } aol5vuyb0a ;
struct { void * LoggedData ; } fd4jgacxhq ; struct { void * LoggedData ; }
dp4oobqveb ; struct { void * LoggedData [ 4 ] ; } bqfop01i2t ; struct { void
* LoggedData ; } jnlsbkpkpg ; uint32_T hjtfckrdib ; int_T mwqrktpz41 ; int_T
eknmvbtv5u ; struct { int_T PrevIndex ; } gooaqjnwdd ; int_T ndd51edmhg [ 2 ]
; int_T jobrukao3e ; struct { int_T PrevIndex ; } pxxh1knxpv ; int_T
fpdlolyb1y ; int_T ga0dnoquno ; struct { int_T PrevIndex ; } mgial2fbbs ;
int_T jisc0stmja ; int_T bhjarlltx0 ; struct { int_T PrevIndex ; } k2mrpavnr3
; int_T mmq01gbox5 [ 4 ] ; int_T gml05gaiwr ; struct { int_T PrevIndex ; }
p0ybzcrnsf ; int_T p1tupjnl2m [ 4 ] ; int_T pt0evqezqm ; struct { int_T
PrevIndex ; } dppeb2fcvo ; int_T jt4kc1ii0s [ 4 ] ; int_T hns1rud2yi ; struct
{ int_T PrevIndex ; } jdmacw4ijl ; int_T mek4mei4m5 [ 3 ] ; int_T ppttotus2n
; struct { int_T PrevIndex ; } i3b2ntrlap ; int_T ls41xbqrac [ 4 ] ;
boolean_T c0c55fi3jq ; boolean_T omgte3tg31 ; } DW ; typedef struct { real_T
irr1ohciq3 [ 8 ] ; real_T ptbb520ltm [ 2 ] ; real_T n0ofazoxbd [ 2 ] ; real_T
asbanfimve [ 2 ] ; real_T glltmukddh [ 2 ] ; real_T mcu5m2cl2g [ 8 ] ; real_T
odzwuprsis ; } X ; typedef struct { real_T irr1ohciq3 [ 8 ] ; real_T
ptbb520ltm [ 2 ] ; real_T n0ofazoxbd [ 2 ] ; real_T asbanfimve [ 2 ] ; real_T
glltmukddh [ 2 ] ; real_T mcu5m2cl2g [ 8 ] ; real_T odzwuprsis ; } XDot ;
typedef struct { boolean_T irr1ohciq3 [ 8 ] ; boolean_T ptbb520ltm [ 2 ] ;
boolean_T n0ofazoxbd [ 2 ] ; boolean_T asbanfimve [ 2 ] ; boolean_T
glltmukddh [ 2 ] ; boolean_T mcu5m2cl2g [ 8 ] ; boolean_T odzwuprsis ; } XDis
; typedef struct { real_T irr1ohciq3 [ 8 ] ; real_T ptbb520ltm [ 2 ] ; real_T
n0ofazoxbd [ 2 ] ; real_T asbanfimve [ 2 ] ; real_T glltmukddh [ 2 ] ; real_T
mcu5m2cl2g [ 8 ] ; real_T odzwuprsis ; } CStateAbsTol ; typedef struct {
real_T nyfkdeqvul ; real_T myassw5y3y ; real_T h2qpexisga ; real_T lggrvczrm0
; real_T pxstdrscuo ; real_T pc2sbokrzp [ 4 ] ; real_T gmud1ydvyf [ 4 ] ;
real_T jufr2dujbf ; real_T fv4bnk3lzr ; real_T i5zlkpfdg1 ; real_T pc0ofz0fa0
; real_T b2eqsgxxhs ; } ZCV ; typedef struct { rtwCAPI_ModelMappingInfo mmi ;
} DataMapInfo ; struct P_ { real_T A0 [ 64 ] ; real_T B0 [ 32 ] ; real_T C0 [
32 ] ; real_T KAoss [ 64 ] ; real_T KBossw [ 64 ] ; real_T KCoss [ 64 ] ;
real_T Kopt [ 32 ] ; real_T amplitudeNoise ; real_T amplitudePertIn ; real_T
amplitudePertOut ; real_T cstPertIn ; real_T cstPertOut ; real_T omegaNoise ;
real_T omegaPertIN ; real_T omegaPertOut ; real_T randomAmpNoise ; real_T
SinOut_Bias ; real_T SinOut_Phase ; real_T Constant_Value ; real_T
Switch_Threshold ; real_T Processo1_X0 ; real_T Ps1_A [ 2 ] ; real_T Ps1_C [
2 ] ; real_T Constant2_Value ; real_T Ps2_A [ 2 ] ; real_T Ps2_C [ 2 ] ;
real_T Ps3_A [ 2 ] ; real_T Ps3_C [ 2 ] ; real_T Ps4_A [ 2 ] ; real_T Ps4_C [
2 ] ; real_T FromWs_Time0 [ 5 ] ; real_T FromWs_Data0 [ 5 ] ; real_T
FromWs_Time0_h113reko55 [ 6 ] ; real_T FromWs_Data0_kupdvn51v2 [ 6 ] ; real_T
FromWs_Time0_acysjor4ip [ 4 ] ; real_T FromWs_Data0_ohxionynup [ 4 ] ; real_T
Constant1_Value ; real_T SineIn_Bias ; real_T SineIn_Phase ; real_T
Constant_Value_gakmm5of0a ; real_T Switch_Threshold_b2p15hzzi4 ; real_T
kalman_X0 ; real_T Saturation_UpperSat ; real_T Saturation_LowerSat ; real_T
Constant1_Value_ngreypf4ru ; real_T Ps1_A_cnuk0h4tar ; real_T
Ps1_C_gfroigzrf4 ; real_T Gain_Gain ; real_T u2sen05t_Bias ; real_T
u2sen05t_Phase ; real_T UniformRandomNumber_Seed ; real_T
FromWs_Time0_dnxa5yquln [ 4 ] ; real_T FromWs_Data0_pvombv1exi [ 4 ] ; real_T
FromWs_Time0_an32oq3ef4 [ 10 ] ; real_T FromWs_Data0_apflh1ms4l [ 10 ] ;
real_T FromWs_Time0_bxwfouleny [ 10 ] ; real_T FromWs_Data0_gglwjxomlu [ 10 ]
; real_T FromWs_Time0_dgntxkplur [ 10 ] ; real_T FromWs_Data0_n4fvl010iy [ 10
] ; real_T FromWs_Time0_jrdfiknsxo [ 8 ] ; real_T FromWs_Data0_i5qxt5w12n [ 8
] ; real_T Gain_Gain_l2aa1kgqdh ; real_T Gain_Gain_gpep4bzie5 ; uint8_T
ManualSwitch_CurrentSetting ; uint8_T ManualSwitch1_CurrentSetting ; } ;
extern const char * RT_MEMORY_ALLOCATION_ERROR ; extern B rtB ; extern X rtX
; extern DW rtDW ; extern P rtP ; extern const rtwCAPI_ModelMappingStaticInfo
* LTRTenzo_GetCAPIStaticMap ( void ) ; extern SimStruct * const rtS ; extern
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
