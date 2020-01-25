#include "__cf_LTRTenzo6Dof.h"
#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "LTRTenzo6Dof_capi_host.h"
#define sizeof(s) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)    
#else
#include "builtin_typeid_types.h"
#include "LTRTenzo6Dof.h"
#include "LTRTenzo6Dof_capi.h"
#include "LTRTenzo6Dof_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST                  
#define TARGET_STRING(s)               (NULL)                    
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static const rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 0 , TARGET_STRING (
"LTRTenzo6Dof/Sum1" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 1 , 0
, TARGET_STRING ( "LTRTenzo6Dof/Sum3" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 ,
0 , 0 } , { 2 , 0 , TARGET_STRING ( "LTRTenzo6Dof/Sum4" ) , TARGET_STRING (
"" ) , 0 , 0 , 1 , 0 , 0 } , { 3 , 0 , TARGET_STRING (
"LTRTenzo6Dof/Setpoints2/Constant2" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0
, 1 } , { 4 , 0 , TARGET_STRING ( "LTRTenzo6Dof/Setpoints2/Sum4" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 5 , 0 , TARGET_STRING (
"LTRTenzo6Dof/Setpoints2/P(s)1" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0
} , { 6 , 0 , TARGET_STRING ( "LTRTenzo6Dof/Setpoints2/P(s)4" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 7 , 0 , TARGET_STRING (
"LTRTenzo6Dof/Setpoints2/P(s)5" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0
} , { 8 , 3 , TARGET_STRING ( "LTRTenzo6Dof/XandY/C" ) , TARGET_STRING ( "" )
, 0 , 0 , 1 , 0 , 2 } , { 9 , 0 , TARGET_STRING (
"LTRTenzo6Dof/DisturboIn/ErrIn/Clock" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 ,
0 , 0 } , { 10 , 0 , TARGET_STRING ( "LTRTenzo6Dof/DisturboIn/ErrIn/Switch" )
, TARGET_STRING ( "DiturboStato" ) , 0 , 0 , 2 , 0 , 0 } , { 11 , 0 ,
TARGET_STRING ( "LTRTenzo6Dof/DisturboOut/ErrOut/Clock" ) , TARGET_STRING (
"" ) , 0 , 0 , 2 , 0 , 0 } , { 12 , 0 , TARGET_STRING (
"LTRTenzo6Dof/DisturboOut/ErrOut/Switch" ) , TARGET_STRING ( "DiturboStato" )
, 0 , 0 , 2 , 0 , 0 } , { 13 , 0 , TARGET_STRING (
"LTRTenzo6Dof/Err Mes/Filtered random noise + sin/Gain" ) , TARGET_STRING (
"" ) , 0 , 0 , 2 , 0 , 0 } , { 14 , 0 , TARGET_STRING (
"LTRTenzo6Dof/Err Mes/Filtered random noise + sin/Sum3" ) , TARGET_STRING (
"" ) , 0 , 0 , 2 , 0 , 0 } , { 15 , 0 , TARGET_STRING (
"LTRTenzo6Dof/Err Mes/Filtered random noise + sin/Uniform Random Number" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 3 } , { 16 , 0 , TARGET_STRING (
"LTRTenzo6Dof/Output3/R2D/Gain" ) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 1
} , { 17 , 0 , TARGET_STRING ( "LTRTenzo6Dof/Setpoints2/psi /FromWs" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 18 , 0 , TARGET_STRING (
"LTRTenzo6Dof/Setpoints2/zref(t)/FromWs" ) , TARGET_STRING ( "" ) , 0 , 0 , 2
, 0 , 0 } , { 19 , 1 , TARGET_STRING (
"LTRTenzo6Dof/DisturboIn/ErrIn/disturbo/Sum2" ) , TARGET_STRING ( "" ) , 0 ,
0 , 2 , 0 , 0 } , { 20 , 2 , TARGET_STRING (
"LTRTenzo6Dof/DisturboOut/ErrOut/disturbo/Sum2" ) , TARGET_STRING ( "" ) , 0
, 0 , 2 , 0 , 0 } , { 21 , 0 , TARGET_STRING (
"LTRTenzo6Dof/XandY/Output1/R2D/Gain" ) , TARGET_STRING ( "" ) , 0 , 0 , 3 ,
0 , 2 } , { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static
const rtwCAPI_BlockParameters rtBlockParameters [ ] = { { 22 , TARGET_STRING
( "LTRTenzo6Dof/  kalman  " ) , TARGET_STRING ( "X0" ) , 0 , 2 , 0 } , { 23 ,
TARGET_STRING ( "LTRTenzo6Dof/Setpoints2/Constant2" ) , TARGET_STRING (
"Value" ) , 0 , 2 , 0 } , { 24 , TARGET_STRING (
"LTRTenzo6Dof/Setpoints2/P(s)1" ) , TARGET_STRING ( "A" ) , 0 , 4 , 0 } , {
25 , TARGET_STRING ( "LTRTenzo6Dof/Setpoints2/P(s)1" ) , TARGET_STRING ( "C"
) , 0 , 5 , 0 } , { 26 , TARGET_STRING ( "LTRTenzo6Dof/Setpoints2/P(s)4" ) ,
TARGET_STRING ( "A" ) , 0 , 4 , 0 } , { 27 , TARGET_STRING (
"LTRTenzo6Dof/Setpoints2/P(s)4" ) , TARGET_STRING ( "C" ) , 0 , 5 , 0 } , {
28 , TARGET_STRING ( "LTRTenzo6Dof/Setpoints2/P(s)5" ) , TARGET_STRING ( "A"
) , 0 , 4 , 0 } , { 29 , TARGET_STRING ( "LTRTenzo6Dof/Setpoints2/P(s)5" ) ,
TARGET_STRING ( "C" ) , 0 , 5 , 0 } , { 30 , TARGET_STRING (
"LTRTenzo6Dof/processo/Processo1" ) , TARGET_STRING ( "X0" ) , 0 , 2 , 0 } ,
{ 31 , TARGET_STRING ( "LTRTenzo6Dof/DisturboIn/ErrIn/Constant" ) ,
TARGET_STRING ( "Value" ) , 0 , 2 , 0 } , { 32 , TARGET_STRING (
"LTRTenzo6Dof/DisturboIn/ErrIn/Switch" ) , TARGET_STRING ( "Threshold" ) , 0
, 2 , 0 } , { 33 , TARGET_STRING ( "LTRTenzo6Dof/DisturboOut/ErrOut/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 2 , 0 } , { 34 , TARGET_STRING (
"LTRTenzo6Dof/DisturboOut/ErrOut/Switch" ) , TARGET_STRING ( "Threshold" ) ,
0 , 2 , 0 } , { 35 , TARGET_STRING (
"LTRTenzo6Dof/Err Mes/Filtered random noise + sin/Constant1" ) ,
TARGET_STRING ( "Value" ) , 0 , 2 , 0 } , { 36 , TARGET_STRING (
"LTRTenzo6Dof/Err Mes/Filtered random noise + sin/Gain" ) , TARGET_STRING (
"Gain" ) , 0 , 2 , 0 } , { 37 , TARGET_STRING (
"LTRTenzo6Dof/Err Mes/Filtered random noise + sin/0.2 sen(0.5 t)" ) ,
TARGET_STRING ( "Bias" ) , 0 , 2 , 0 } , { 38 , TARGET_STRING (
"LTRTenzo6Dof/Err Mes/Filtered random noise + sin/0.2 sen(0.5 t)" ) ,
TARGET_STRING ( "Phase" ) , 0 , 2 , 0 } , { 39 , TARGET_STRING (
"LTRTenzo6Dof/Err Mes/Filtered random noise + sin/Manual Switch1" ) ,
TARGET_STRING ( "CurrentSetting" ) , 1 , 2 , 0 } , { 40 , TARGET_STRING (
"LTRTenzo6Dof/Err Mes/Filtered random noise + sin/P(s)1" ) , TARGET_STRING (
"A" ) , 0 , 2 , 0 } , { 41 , TARGET_STRING (
"LTRTenzo6Dof/Err Mes/Filtered random noise + sin/P(s)1" ) , TARGET_STRING (
"C" ) , 0 , 2 , 0 } , { 42 , TARGET_STRING (
"LTRTenzo6Dof/Err Mes/Filtered random noise + sin/Uniform Random Number" ) ,
TARGET_STRING ( "Seed" ) , 0 , 2 , 0 } , { 43 , TARGET_STRING (
"LTRTenzo6Dof/Output3/R2D/Gain" ) , TARGET_STRING ( "Gain" ) , 0 , 2 , 0 } ,
{ 44 , TARGET_STRING ( "LTRTenzo6Dof/Setpoints2/phy(t)/FromWs" ) ,
TARGET_STRING ( "Time0" ) , 0 , 6 , 0 } , { 45 , TARGET_STRING (
"LTRTenzo6Dof/Setpoints2/phy(t)/FromWs" ) , TARGET_STRING ( "Data0" ) , 0 , 6
, 0 } , { 46 , TARGET_STRING ( "LTRTenzo6Dof/Setpoints2/psi /FromWs" ) ,
TARGET_STRING ( "Time0" ) , 0 , 6 , 0 } , { 47 , TARGET_STRING (
"LTRTenzo6Dof/Setpoints2/psi /FromWs" ) , TARGET_STRING ( "Data0" ) , 0 , 6 ,
0 } , { 48 , TARGET_STRING ( "LTRTenzo6Dof/Setpoints2/theta(t)/FromWs" ) ,
TARGET_STRING ( "Time0" ) , 0 , 7 , 0 } , { 49 , TARGET_STRING (
"LTRTenzo6Dof/Setpoints2/theta(t)/FromWs" ) , TARGET_STRING ( "Data0" ) , 0 ,
7 , 0 } , { 50 , TARGET_STRING ( "LTRTenzo6Dof/Setpoints2/zref(t)/FromWs" ) ,
TARGET_STRING ( "Time0" ) , 0 , 7 , 0 } , { 51 , TARGET_STRING (
"LTRTenzo6Dof/Setpoints2/zref(t)/FromWs" ) , TARGET_STRING ( "Data0" ) , 0 ,
7 , 0 } , { 52 , TARGET_STRING (
"LTRTenzo6Dof/DisturboIn/ErrIn/disturbo/SineIn" ) , TARGET_STRING ( "Bias" )
, 0 , 2 , 0 } , { 53 , TARGET_STRING (
"LTRTenzo6Dof/DisturboIn/ErrIn/disturbo/SineIn" ) , TARGET_STRING ( "Phase" )
, 0 , 2 , 0 } , { 54 , TARGET_STRING (
"LTRTenzo6Dof/DisturboOut/ErrOut/disturbo/SinOut" ) , TARGET_STRING ( "Bias"
) , 0 , 2 , 0 } , { 55 , TARGET_STRING (
"LTRTenzo6Dof/DisturboOut/ErrOut/disturbo/SinOut" ) , TARGET_STRING ( "Phase"
) , 0 , 2 , 0 } , { 56 , TARGET_STRING (
"LTRTenzo6Dof/XandY/Output1/R2D/Gain" ) , TARGET_STRING ( "Gain" ) , 0 , 2 ,
0 } , { 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 } } ; static const
rtwCAPI_ModelParameters rtModelParameters [ ] = { { 57 , TARGET_STRING ( "A0"
) , 0 , 8 , 0 } , { 58 , TARGET_STRING ( "B0" ) , 0 , 9 , 0 } , { 59 ,
TARGET_STRING ( "C0" ) , 0 , 10 , 0 } , { 60 , TARGET_STRING ( "KAoss" ) , 0
, 8 , 0 } , { 61 , TARGET_STRING ( "KBossw" ) , 0 , 8 , 0 } , { 62 ,
TARGET_STRING ( "KCoss" ) , 0 , 8 , 0 } , { 63 , TARGET_STRING ( "Kopt" ) , 0
, 10 , 0 } , { 64 , TARGET_STRING ( "amplitudeNoise" ) , 0 , 2 , 0 } , { 65 ,
TARGET_STRING ( "amplitudePertIn" ) , 0 , 2 , 0 } , { 66 , TARGET_STRING (
"amplitudePertOut" ) , 0 , 2 , 0 } , { 67 , TARGET_STRING ( "cstPertIn" ) , 0
, 2 , 0 } , { 68 , TARGET_STRING ( "cstPertOut" ) , 0 , 2 , 0 } , { 69 ,
TARGET_STRING ( "omegaNoise" ) , 0 , 2 , 0 } , { 70 , TARGET_STRING (
"omegaPertIN" ) , 0 , 2 , 0 } , { 71 , TARGET_STRING ( "omegaPertOut" ) , 0 ,
2 , 0 } , { 72 , TARGET_STRING ( "randomAmpNoise" ) , 0 , 2 , 0 } , { 0 , (
NULL ) , 0 , 0 , 0 } } ;
#ifndef HOST_CAPI_BUILD
static void * rtDataAddrMap [ ] = { & rtB . exersch3pk [ 0 ] , & rtB .
l3ya5q3dsy [ 0 ] , & rtB . ajlqbn3dtj [ 0 ] , & rtB . kk4pom5z0m , & rtB .
fxeigzy1kf , & rtB . aolioyabae , & rtB . oxa3bf04z4 , & rtB . c1do0agcsm , &
rtB . fezrnkyxs4 [ 0 ] , & rtB . mg0az2nzgj , & rtB . dt0bxeukl1 , & rtB .
oaz5l4crsf , & rtB . m4f4roaota , & rtB . m5z5sj11wq , & rtB . lk5cyp2tjd , &
rtB . px0r25lgtw , & rtB . gjftqhkwh2 [ 0 ] , & rtB . fwztmrgmqg , & rtB .
k3xhv4ixsk , & rtB . klytvkrzsu , & rtB . anbxmjzqcz , & rtB . odgn1acl0s [ 0
] , & rtP . kalman_X0 , & rtP . Constant2_Value , & rtP . Ps1_A [ 0 ] , & rtP
. Ps1_C [ 0 ] , & rtP . Ps4_A [ 0 ] , & rtP . Ps4_C [ 0 ] , & rtP . Ps5_A [ 0
] , & rtP . Ps5_C [ 0 ] , & rtP . Processo1_X0 , & rtP .
Constant_Value_gakmm5of0a , & rtP . Switch_Threshold_b2p15hzzi4 , & rtP .
Constant_Value , & rtP . Switch_Threshold , & rtP . Constant1_Value , & rtP .
Gain_Gain , & rtP . u2sen05t_Bias , & rtP . u2sen05t_Phase , & rtP .
ManualSwitch1_CurrentSetting , & rtP . Ps1_A_cnuk0h4tar , & rtP .
Ps1_C_gfroigzrf4 , & rtP . UniformRandomNumber_Seed , & rtP .
Gain_Gain_gpep4bzie5 , & rtP . FromWs_Time0 [ 0 ] , & rtP . FromWs_Data0 [ 0
] , & rtP . FromWs_Time0_lk1gq232s2 [ 0 ] , & rtP . FromWs_Data0_igdugvbssl [
0 ] , & rtP . FromWs_Time0_nyx1zce205 [ 0 ] , & rtP . FromWs_Data0_cogeshtz5t
[ 0 ] , & rtP . FromWs_Time0_jv04arnttw [ 0 ] , & rtP .
FromWs_Data0_pirid4ox1s [ 0 ] , & rtP . SineIn_Bias , & rtP . SineIn_Phase ,
& rtP . SinOut_Bias , & rtP . SinOut_Phase , & rtP . Gain_Gain_l2aa1kgqdh , &
rtP . A0 [ 0 ] , & rtP . B0 [ 0 ] , & rtP . C0 [ 0 ] , & rtP . KAoss [ 0 ] ,
& rtP . KBossw [ 0 ] , & rtP . KCoss [ 0 ] , & rtP . Kopt [ 0 ] , & rtP .
amplitudeNoise , & rtP . amplitudePertIn , & rtP . amplitudePertOut , & rtP .
cstPertIn , & rtP . cstPertOut , & rtP . omegaNoise , & rtP . omegaPertIN , &
rtP . omegaPertOut , & rtP . randomAmpNoise , } ; static int32_T *
rtVarDimsAddrMap [ ] = { ( NULL ) } ;
#endif
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap [ ] = { { "double" ,
"real_T" , 0 , 0 , sizeof ( real_T ) , SS_DOUBLE , 0 , 0 } , {
"unsigned char" , "uint8_T" , 0 , 0 , sizeof ( uint8_T ) , SS_UINT8 , 0 , 0 }
} ;
#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif
static TARGET_CONST rtwCAPI_ElementMap rtElementMap [ ] = { { ( NULL ) , 0 ,
0 , 0 , 0 } , } ; static const rtwCAPI_DimensionMap rtDimensionMap [ ] = { {
rtwCAPI_VECTOR , 0 , 2 , 0 } , { rtwCAPI_VECTOR , 2 , 2 , 0 } , {
rtwCAPI_SCALAR , 4 , 2 , 0 } , { rtwCAPI_VECTOR , 6 , 2 , 0 } , {
rtwCAPI_VECTOR , 8 , 2 , 0 } , { rtwCAPI_VECTOR , 10 , 2 , 0 } , {
rtwCAPI_VECTOR , 12 , 2 , 0 } , { rtwCAPI_VECTOR , 14 , 2 , 0 } , {
rtwCAPI_MATRIX_COL_MAJOR , 16 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 18 , 2
, 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 20 , 2 , 0 } } ; static const uint_T
rtDimensionArray [ ] = { 8 , 1 , 4 , 1 , 1 , 1 , 3 , 1 , 2 , 1 , 1 , 2 , 6 ,
1 , 10 , 1 , 8 , 8 , 8 , 4 , 4 , 8 } ; static const real_T rtcapiStoredFloats
[ ] = { 0.0 , 1.0 , 0.1 } ; static const rtwCAPI_FixPtMap rtFixPtMap [ ] = {
{ ( NULL ) , ( NULL ) , rtwCAPI_FIX_RESERVED , 0 , 0 , 0 } , } ; static const
rtwCAPI_SampleTimeMap rtSampleTimeMap [ ] = { { ( const void * ) &
rtcapiStoredFloats [ 0 ] , ( const void * ) & rtcapiStoredFloats [ 0 ] , 0 ,
0 } , { ( NULL ) , ( NULL ) , 3 , 0 } , { ( const void * ) &
rtcapiStoredFloats [ 0 ] , ( const void * ) & rtcapiStoredFloats [ 1 ] , 1 ,
0 } , { ( const void * ) & rtcapiStoredFloats [ 2 ] , ( const void * ) &
rtcapiStoredFloats [ 0 ] , 2 , 0 } } ; static rtwCAPI_ModelMappingStaticInfo
mmiStatic = { { rtBlockSignals , 22 , ( NULL ) , 0 , ( NULL ) , 0 } , {
rtBlockParameters , 35 , rtModelParameters , 16 } , { ( NULL ) , 0 } , {
rtDataTypeMap , rtDimensionMap , rtFixPtMap , rtElementMap , rtSampleTimeMap
, rtDimensionArray } , "float" , { 3634810543U , 2521803458U , 2534976825U ,
85240381U } , ( NULL ) , 0 , 0 } ; const rtwCAPI_ModelMappingStaticInfo *
LTRTenzo6Dof_GetCAPIStaticMap ( ) { return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
void LTRTenzo6Dof_InitializeDataMapInfo ( ) { rtwCAPI_SetVersion ( ( *
rt_dataMapInfoPtr ) . mmi , 1 ) ; rtwCAPI_SetStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , & mmiStatic ) ; rtwCAPI_SetLoggingStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ; rtwCAPI_SetDataAddressMap ( ( *
rt_dataMapInfoPtr ) . mmi , rtDataAddrMap ) ; rtwCAPI_SetVarDimsAddressMap (
( * rt_dataMapInfoPtr ) . mmi , rtVarDimsAddrMap ) ;
rtwCAPI_SetInstanceLoggingInfo ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArray ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( ( * rt_dataMapInfoPtr ) . mmi , 0 ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void LTRTenzo6Dof_host_InitializeDataMapInfo (
LTRTenzo6Dof_host_DataMapInfo_T * dataMap , const char * path ) {
rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ; rtwCAPI_SetStaticMap ( dataMap ->
mmi , & mmiStatic ) ; rtwCAPI_SetDataAddressMap ( dataMap -> mmi , NULL ) ;
rtwCAPI_SetVarDimsAddressMap ( dataMap -> mmi , NULL ) ; rtwCAPI_SetPath (
dataMap -> mmi , path ) ; rtwCAPI_SetFullPath ( dataMap -> mmi , NULL ) ;
rtwCAPI_SetChildMMIArray ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( dataMap -> mmi , 0 ) ; }
#ifdef __cplusplus
}
#endif
#endif
