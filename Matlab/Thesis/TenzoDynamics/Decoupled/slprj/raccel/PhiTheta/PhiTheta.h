#include "__cf_PhiTheta.h"
#ifndef RTW_HEADER_PhiTheta_h_
#define RTW_HEADER_PhiTheta_h_
#include <stddef.h>
#include <string.h>
#include "rtw_modelmap.h"
#ifndef PhiTheta_COMMON_INCLUDES_
#define PhiTheta_COMMON_INCLUDES_
#include <stdlib.h>
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "raccel.h"
#include "rt_logging.h"
#include "dt_info.h"
#include "ext_work.h"
#endif
#include "PhiTheta_types.h"
#include "multiword_types.h"
#include "mwmathutil.h"
#include "rt_defines.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#define MODEL_NAME PhiTheta
#define NSAMPLE_TIMES (2) 
#define NINPUTS (0)       
#define NOUTPUTS (0)     
#define NBLOCKIO (11) 
#define NUM_ZC_EVENTS (0) 
#ifndef NCSTATES
#define NCSTATES (8)   
#elif NCSTATES != 8
#error Invalid specification of NCSTATES defined in compiler command
#endif
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm) (*rt_dataMapInfoPtr)
#endif
#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val) (rt_dataMapInfoPtr = &val)
#endif
typedef struct { real_T mk4dpnamwc ; real_T o3ul0nzcwe ; real_T dewu3oxpw1 ;
real_T opuqvtwzzt ; real_T awmfihve5b ; real_T ngxpbqhzzn ; real_T dgroez0siq
; real_T kqoy4n5gjo ; real_T pydoj1kzpi ; real_T blhrfoy5vb ; real_T
pk1lzuz22z ; } B ; typedef struct { real_T fmiwcm0sxa ; real_T mqn0il2cto ;
struct { void * TimePtr ; void * DataPtr ; void * RSimInfoPtr ; } bvmrb3pj3h
; struct { void * LoggedData ; } amidzcc4fy ; struct { void * LoggedData ; }
lrhqyru4a2 ; struct { void * LoggedData ; } hl4rq4szdl ; struct { void *
LoggedData ; } pnnqiesode ; struct { void * LoggedData ; } hognujx3kq ;
uint32_T hiarr4fcu2 ; uint32_T nf5mz1kdkf ; struct { int_T PrevIndex ; }
kuvxmwsxqi ; } DW ; typedef struct { real_T ip3stwqu5g [ 2 ] ; real_T
h4bztukumm ; real_T awo0llmjv3 ; real_T hu3bgmv23r [ 2 ] ; real_T icelwbnw0k
[ 2 ] ; } X ; typedef struct { real_T ip3stwqu5g [ 2 ] ; real_T h4bztukumm ;
real_T awo0llmjv3 ; real_T hu3bgmv23r [ 2 ] ; real_T icelwbnw0k [ 2 ] ; }
XDot ; typedef struct { boolean_T ip3stwqu5g [ 2 ] ; boolean_T h4bztukumm ;
boolean_T awo0llmjv3 ; boolean_T hu3bgmv23r [ 2 ] ; boolean_T icelwbnw0k [ 2
] ; } XDis ; typedef struct { rtwCAPI_ModelMappingInfo mmi ; } DataMapInfo ;
struct P_ { real_T SatThrust_UpperSat ; real_T SatThrust_LowerSat ; real_T
Ps_A [ 2 ] ; real_T Ps_C [ 2 ] ; real_T FromWs_Time0 [ 6 ] ; real_T
FromWs_Data0 [ 6 ] ; real_T u3sen80t_Amp ; real_T u3sen80t_Bias ; real_T
u3sen80t_Freq ; real_T u3sen80t_Phase ; real_T Ps1_A ; real_T Ps1_C ; real_T
Gain_Gain ; real_T usen05t_Amp ; real_T usen05t_Bias ; real_T usen05t_Freq ;
real_T usen05t_Phase ; real_T Ps1_A_e4tfyewuhv ; real_T Ps1_C_pry1qgt440 ;
real_T Gain_Gain_hvers1lhk2 ; real_T Ps_A_awgvmdvyjk [ 2 ] ; real_T
Ps_C_axb255v5cb [ 2 ] ; real_T usen05t_Amp_jzvjbf0x3j ; real_T
usen05t_Bias_o1wafowfst ; real_T usen05t_Freq_pv2suynl3g ; real_T
usen05t_Phase_eh1ecmzpx1 ; real_T UniformRandomNumber_Minimum ; real_T
UniformRandomNumber_Maximum ; real_T UniformRandomNumber_Seed ; real_T
usen05t_Amp_onejysspqe ; real_T usen05t_Bias_k0lwz1nqbi ; real_T
usen05t_Freq_lnfmendplc ; real_T usen05t_Phase_a1d0o3bxhg ; real_T
UniformRandomNumber_Minimum_dr3kabpyij ; real_T
UniformRandomNumber_Maximum_duhygprnbe ; real_T
UniformRandomNumber_Seed_mavckwa2ow ; real_T Cs1_A [ 2 ] ; real_T Cs1_C [ 2 ]
; real_T Cs1_D ; uint8_T ManualSwitch1_CurrentSetting ; uint8_T
ManualSwitch1_CurrentSetting_a1sxsoirxs ; uint8_T ManualSwitch_CurrentSetting
; } ; extern P rtP ; extern const char * RT_MEMORY_ALLOCATION_ERROR ; extern
B rtB ; extern X rtX ; extern DW rtDW ; extern const
rtwCAPI_ModelMappingStaticInfo * PhiTheta_GetCAPIStaticMap ( void ) ; extern
SimStruct * const rtS ; extern const int_T gblNumToFiles ; extern const int_T
gblNumFrFiles ; extern const int_T gblNumFrWksBlocks ; extern rtInportTUtable
* gblInportTUtables ; extern const char * gblInportFileName ; extern const
int_T gblNumRootInportBlks ; extern const int_T gblNumModelInputs ; extern
const int_T gblInportDataTypeIdx [ ] ; extern const int_T gblInportDims [ ] ;
extern const int_T gblInportComplex [ ] ; extern const int_T
gblInportInterpoFlag [ ] ; extern const int_T gblInportContinuous [ ] ;
extern DataMapInfo * rt_dataMapInfoPtr ; extern rtwCAPI_ModelMappingInfo *
rt_modelMapInfoPtr ;
#endif
