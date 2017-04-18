#include "__cf_LTRTenzo.h"
#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "LTRTenzo_capi_host.h"
#define sizeof(s) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)    
#else
#include "builtin_typeid_types.h"
#include "LTRTenzo.h"
#include "LTRTenzo_capi.h"
#include "LTRTenzo_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST                  
#define TARGET_STRING(s)               (NULL)                    
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static const rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 0 , TARGET_STRING (
"LTRTenzo/Saturation" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 1 ,
0 , TARGET_STRING ( "LTRTenzo/Sum1" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0
, 0 } , { 2 , 0 , TARGET_STRING ( "LTRTenzo/Sum3" ) , TARGET_STRING ( "" ) ,
0 , 0 , 0 , 0 , 0 } , { 3 , 0 , TARGET_STRING ( "LTRTenzo/Sum4" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 4 , 0 , TARGET_STRING (
"LTRTenzo/Setpoints1/Manual Switch" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0
, 0 } , { 5 , 3 , TARGET_STRING ( "LTRTenzo/XandY/C" ) , TARGET_STRING ( "" )
, 0 , 0 , 0 , 0 , 1 } , { 6 , 0 , TARGET_STRING (
"LTRTenzo/DisturboIn/ErrIn/Clock" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 ,
0 } , { 7 , 0 , TARGET_STRING ( "LTRTenzo/DisturboIn/ErrIn/Switch" ) ,
TARGET_STRING ( "DiturboStato" ) , 0 , 0 , 2 , 0 , 0 } , { 8 , 0 ,
TARGET_STRING ( "LTRTenzo/DisturboOut/ErrOut/Clock" ) , TARGET_STRING ( "" )
, 0 , 0 , 2 , 0 , 0 } , { 9 , 0 , TARGET_STRING (
"LTRTenzo/DisturboOut/ErrOut/Switch" ) , TARGET_STRING ( "DiturboStato" ) , 0
, 0 , 2 , 0 , 0 } , { 10 , 0 , TARGET_STRING (
"LTRTenzo/Err Mes/Filtered random noise + sin/Gain" ) , TARGET_STRING ( "" )
, 0 , 0 , 2 , 0 , 0 } , { 11 , 0 , TARGET_STRING (
"LTRTenzo/Err Mes/Filtered random noise + sin/Sum3" ) , TARGET_STRING ( "" )
, 0 , 0 , 2 , 0 , 0 } , { 12 , 0 , TARGET_STRING (
"LTRTenzo/Err Mes/Filtered random noise + sin/Uniform Random Number" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 2 } , { 13 , 0 , TARGET_STRING (
"LTRTenzo/Output3/R2D/Gain" ) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 3 } ,
{ 14 , 1 , TARGET_STRING ( "LTRTenzo/DisturboIn/ErrIn/disturbo/Sum2" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 15 , 2 , TARGET_STRING (
"LTRTenzo/DisturboOut/ErrOut/disturbo/Sum2" ) , TARGET_STRING ( "" ) , 0 , 0
, 2 , 0 , 0 } , { 16 , 0 , TARGET_STRING (
"LTRTenzo/Setpoints1/R step/zref(t)1/FromWs" ) , TARGET_STRING ( "" ) , 0 , 0
, 2 , 0 , 0 } , { 17 , 0 , TARGET_STRING (
"LTRTenzo/Setpoints1/Ref hard /phy(t)/FromWs" ) , TARGET_STRING ( "" ) , 0 ,
0 , 2 , 0 , 0 } , { 18 , 0 , TARGET_STRING (
"LTRTenzo/Setpoints1/Ref hard /psi /FromWs" ) , TARGET_STRING ( "" ) , 0 , 0
, 2 , 0 , 0 } , { 19 , 0 , TARGET_STRING (
"LTRTenzo/Setpoints1/Ref hard /theta(t)/FromWs" ) , TARGET_STRING ( "" ) , 0
, 0 , 2 , 0 , 0 } , { 20 , 0 , TARGET_STRING (
"LTRTenzo/Setpoints1/Ref hard /zref(t)/FromWs" ) , TARGET_STRING ( "" ) , 0 ,
0 , 2 , 0 , 0 } , { 21 , 0 , TARGET_STRING (
"LTRTenzo/XandY/Output1/R2D/Gain" ) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 ,
1 } , { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static const
rtwCAPI_BlockParameters rtBlockParameters [ ] = { { 22 , TARGET_STRING (
"LTRTenzo/Saturation" ) , TARGET_STRING ( "UpperLimit" ) , 0 , 2 , 0 } , { 23
, TARGET_STRING ( "LTRTenzo/Saturation" ) , TARGET_STRING ( "LowerLimit" ) ,
0 , 2 , 0 } , { 24 , TARGET_STRING ( "LTRTenzo/  kalman  " ) , TARGET_STRING
( "X0" ) , 0 , 2 , 0 } , { 25 , TARGET_STRING (
"LTRTenzo/Setpoints1/Manual Switch" ) , TARGET_STRING ( "CurrentSetting" ) ,
1 , 2 , 0 } , { 26 , TARGET_STRING ( "LTRTenzo/processo/Processo1" ) ,
TARGET_STRING ( "X0" ) , 0 , 2 , 0 } , { 27 , TARGET_STRING (
"LTRTenzo/DisturboIn/ErrIn/Constant" ) , TARGET_STRING ( "Value" ) , 0 , 2 ,
0 } , { 28 , TARGET_STRING ( "LTRTenzo/DisturboIn/ErrIn/Switch" ) ,
TARGET_STRING ( "Threshold" ) , 0 , 2 , 0 } , { 29 , TARGET_STRING (
"LTRTenzo/DisturboOut/ErrOut/Constant" ) , TARGET_STRING ( "Value" ) , 0 , 2
, 0 } , { 30 , TARGET_STRING ( "LTRTenzo/DisturboOut/ErrOut/Switch" ) ,
TARGET_STRING ( "Threshold" ) , 0 , 2 , 0 } , { 31 , TARGET_STRING (
"LTRTenzo/Err Mes/Filtered random noise + sin/Constant1" ) , TARGET_STRING (
"Value" ) , 0 , 2 , 0 } , { 32 , TARGET_STRING (
"LTRTenzo/Err Mes/Filtered random noise + sin/Gain" ) , TARGET_STRING (
"Gain" ) , 0 , 2 , 0 } , { 33 , TARGET_STRING (
"LTRTenzo/Err Mes/Filtered random noise + sin/0.2 sen(0.5 t)" ) ,
TARGET_STRING ( "Bias" ) , 0 , 2 , 0 } , { 34 , TARGET_STRING (
"LTRTenzo/Err Mes/Filtered random noise + sin/0.2 sen(0.5 t)" ) ,
TARGET_STRING ( "Phase" ) , 0 , 2 , 0 } , { 35 , TARGET_STRING (
"LTRTenzo/Err Mes/Filtered random noise + sin/Manual Switch1" ) ,
TARGET_STRING ( "CurrentSetting" ) , 1 , 2 , 0 } , { 36 , TARGET_STRING (
"LTRTenzo/Err Mes/Filtered random noise + sin/P(s)1" ) , TARGET_STRING ( "A"
) , 0 , 2 , 0 } , { 37 , TARGET_STRING (
"LTRTenzo/Err Mes/Filtered random noise + sin/P(s)1" ) , TARGET_STRING ( "C"
) , 0 , 2 , 0 } , { 38 , TARGET_STRING (
"LTRTenzo/Err Mes/Filtered random noise + sin/Uniform Random Number" ) ,
TARGET_STRING ( "Seed" ) , 0 , 2 , 0 } , { 39 , TARGET_STRING (
"LTRTenzo/Output3/R2D/Gain" ) , TARGET_STRING ( "Gain" ) , 0 , 2 , 0 } , { 40
, TARGET_STRING ( "LTRTenzo/Setpoints1/R step/Constant1" ) , TARGET_STRING (
"Value" ) , 0 , 2 , 0 } , { 41 , TARGET_STRING (
"LTRTenzo/Setpoints1/Ref hard /Constant2" ) , TARGET_STRING ( "Value" ) , 0 ,
2 , 0 } , { 42 , TARGET_STRING ( "LTRTenzo/Setpoints1/Ref hard /P(s)1" ) ,
TARGET_STRING ( "A" ) , 0 , 4 , 0 } , { 43 , TARGET_STRING (
"LTRTenzo/Setpoints1/Ref hard /P(s)1" ) , TARGET_STRING ( "C" ) , 0 , 5 , 0 }
, { 44 , TARGET_STRING ( "LTRTenzo/Setpoints1/Ref hard /P(s)2" ) ,
TARGET_STRING ( "A" ) , 0 , 4 , 0 } , { 45 , TARGET_STRING (
"LTRTenzo/Setpoints1/Ref hard /P(s)2" ) , TARGET_STRING ( "C" ) , 0 , 5 , 0 }
, { 46 , TARGET_STRING ( "LTRTenzo/Setpoints1/Ref hard /P(s)3" ) ,
TARGET_STRING ( "A" ) , 0 , 4 , 0 } , { 47 , TARGET_STRING (
"LTRTenzo/Setpoints1/Ref hard /P(s)3" ) , TARGET_STRING ( "C" ) , 0 , 5 , 0 }
, { 48 , TARGET_STRING ( "LTRTenzo/Setpoints1/Ref hard /P(s)4" ) ,
TARGET_STRING ( "A" ) , 0 , 4 , 0 } , { 49 , TARGET_STRING (
"LTRTenzo/Setpoints1/Ref hard /P(s)4" ) , TARGET_STRING ( "C" ) , 0 , 5 , 0 }
, { 50 , TARGET_STRING ( "LTRTenzo/DisturboIn/ErrIn/disturbo/SineIn" ) ,
TARGET_STRING ( "Bias" ) , 0 , 2 , 0 } , { 51 , TARGET_STRING (
"LTRTenzo/DisturboIn/ErrIn/disturbo/SineIn" ) , TARGET_STRING ( "Phase" ) , 0
, 2 , 0 } , { 52 , TARGET_STRING (
"LTRTenzo/DisturboOut/ErrOut/disturbo/SinOut" ) , TARGET_STRING ( "Bias" ) ,
0 , 2 , 0 } , { 53 , TARGET_STRING (
"LTRTenzo/DisturboOut/ErrOut/disturbo/SinOut" ) , TARGET_STRING ( "Phase" ) ,
0 , 2 , 0 } , { 54 , TARGET_STRING (
"LTRTenzo/Setpoints1/R step/phy(t)1/FromWs" ) , TARGET_STRING ( "Time0" ) , 0
, 6 , 0 } , { 55 , TARGET_STRING (
"LTRTenzo/Setpoints1/R step/phy(t)1/FromWs" ) , TARGET_STRING ( "Data0" ) , 0
, 6 , 0 } , { 56 , TARGET_STRING ( "LTRTenzo/Setpoints1/R step/psi 1/FromWs"
) , TARGET_STRING ( "Time0" ) , 0 , 0 , 0 } , { 57 , TARGET_STRING (
"LTRTenzo/Setpoints1/R step/psi 1/FromWs" ) , TARGET_STRING ( "Data0" ) , 0 ,
0 , 0 } , { 58 , TARGET_STRING (
"LTRTenzo/Setpoints1/R step/theta(t)1/FromWs" ) , TARGET_STRING ( "Time0" ) ,
0 , 0 , 0 } , { 59 , TARGET_STRING (
"LTRTenzo/Setpoints1/R step/theta(t)1/FromWs" ) , TARGET_STRING ( "Data0" ) ,
0 , 0 , 0 } , { 60 , TARGET_STRING (
"LTRTenzo/Setpoints1/R step/zref(t)1/FromWs" ) , TARGET_STRING ( "Time0" ) ,
0 , 7 , 0 } , { 61 , TARGET_STRING (
"LTRTenzo/Setpoints1/R step/zref(t)1/FromWs" ) , TARGET_STRING ( "Data0" ) ,
0 , 7 , 0 } , { 62 , TARGET_STRING (
"LTRTenzo/Setpoints1/Ref hard /phy(t)/FromWs" ) , TARGET_STRING ( "Time0" ) ,
0 , 8 , 0 } , { 63 , TARGET_STRING (
"LTRTenzo/Setpoints1/Ref hard /phy(t)/FromWs" ) , TARGET_STRING ( "Data0" ) ,
0 , 8 , 0 } , { 64 , TARGET_STRING (
"LTRTenzo/Setpoints1/Ref hard /psi /FromWs" ) , TARGET_STRING ( "Time0" ) , 0
, 8 , 0 } , { 65 , TARGET_STRING (
"LTRTenzo/Setpoints1/Ref hard /psi /FromWs" ) , TARGET_STRING ( "Data0" ) , 0
, 8 , 0 } , { 66 , TARGET_STRING (
"LTRTenzo/Setpoints1/Ref hard /theta(t)/FromWs" ) , TARGET_STRING ( "Time0" )
, 0 , 8 , 0 } , { 67 , TARGET_STRING (
"LTRTenzo/Setpoints1/Ref hard /theta(t)/FromWs" ) , TARGET_STRING ( "Data0" )
, 0 , 8 , 0 } , { 68 , TARGET_STRING (
"LTRTenzo/Setpoints1/Ref hard /zref(t)/FromWs" ) , TARGET_STRING ( "Time0" )
, 0 , 1 , 0 } , { 69 , TARGET_STRING (
"LTRTenzo/Setpoints1/Ref hard /zref(t)/FromWs" ) , TARGET_STRING ( "Data0" )
, 0 , 1 , 0 } , { 70 , TARGET_STRING ( "LTRTenzo/XandY/Output1/R2D/Gain" ) ,
TARGET_STRING ( "Gain" ) , 0 , 2 , 0 } , { 0 , ( NULL ) , ( NULL ) , 0 , 0 ,
0 } } ; static const rtwCAPI_ModelParameters rtModelParameters [ ] = { { 71 ,
TARGET_STRING ( "A0" ) , 0 , 9 , 0 } , { 72 , TARGET_STRING ( "B0" ) , 0 , 10
, 0 } , { 73 , TARGET_STRING ( "C0" ) , 0 , 11 , 0 } , { 74 , TARGET_STRING (
"KAoss" ) , 0 , 9 , 0 } , { 75 , TARGET_STRING ( "KBossw" ) , 0 , 9 , 0 } , {
76 , TARGET_STRING ( "KCoss" ) , 0 , 9 , 0 } , { 77 , TARGET_STRING ( "Kopt"
) , 0 , 11 , 0 } , { 78 , TARGET_STRING ( "amplitudeNoise" ) , 0 , 2 , 0 } ,
{ 79 , TARGET_STRING ( "amplitudePertIn" ) , 0 , 2 , 0 } , { 80 ,
TARGET_STRING ( "amplitudePertOut" ) , 0 , 2 , 0 } , { 81 , TARGET_STRING (
"cstPertIn" ) , 0 , 2 , 0 } , { 82 , TARGET_STRING ( "cstPertOut" ) , 0 , 2 ,
0 } , { 83 , TARGET_STRING ( "omegaNoise" ) , 0 , 2 , 0 } , { 84 ,
TARGET_STRING ( "omegaPertIN" ) , 0 , 2 , 0 } , { 85 , TARGET_STRING (
"omegaPertOut" ) , 0 , 2 , 0 } , { 86 , TARGET_STRING ( "randomAmpNoise" ) ,
0 , 2 , 0 } , { 0 , ( NULL ) , 0 , 0 , 0 } } ;
#ifndef HOST_CAPI_BUILD
static void * rtDataAddrMap [ ] = { & rtB . csh4yoneye [ 0 ] , & rtB .
mfm3tpnt2p [ 0 ] , & rtB . j4tv1cqyj2 [ 0 ] , & rtB . hjr0spzzaf [ 0 ] , &
rtB . bdgzagdxra [ 0 ] , & rtB . oyi1nkeowt [ 0 ] , & rtB . bxeh32cm4i , &
rtB . ej3n31dyxa , & rtB . mbsguhsyce , & rtB . gxh0zyu42a , & rtB .
fgbq3jefrx , & rtB . d1v04lphp2 , & rtB . d4wqki33zw , & rtB . ajinddapxi [ 0
] , & rtB . fd3jf50j03 , & rtB . kwjrknjxrb , & rtB . gbemrwhpr5 , & rtB .
hzoqt0nc3o , & rtB . otap4hfn5r , & rtB . cfdpbmvjgv , & rtB . edtxh40dpx , &
rtB . exb2j2haif [ 0 ] , & rtP . Saturation_UpperSat , & rtP .
Saturation_LowerSat , & rtP . kalman_X0 , & rtP . ManualSwitch_CurrentSetting
, & rtP . Processo1_X0 , & rtP . Constant_Value_gakmm5of0a , & rtP .
Switch_Threshold_b2p15hzzi4 , & rtP . Constant_Value , & rtP .
Switch_Threshold , & rtP . Constant1_Value_ngreypf4ru , & rtP . Gain_Gain , &
rtP . u2sen05t_Bias , & rtP . u2sen05t_Phase , & rtP .
ManualSwitch1_CurrentSetting , & rtP . Ps1_A_cnuk0h4tar , & rtP .
Ps1_C_gfroigzrf4 , & rtP . UniformRandomNumber_Seed , & rtP .
Gain_Gain_gpep4bzie5 , & rtP . Constant1_Value , & rtP . Constant2_Value , &
rtP . Ps1_A [ 0 ] , & rtP . Ps1_C [ 0 ] , & rtP . Ps2_A [ 0 ] , & rtP . Ps2_C
[ 0 ] , & rtP . Ps3_A [ 0 ] , & rtP . Ps3_C [ 0 ] , & rtP . Ps4_A [ 0 ] , &
rtP . Ps4_C [ 0 ] , & rtP . SineIn_Bias , & rtP . SineIn_Phase , & rtP .
SinOut_Bias , & rtP . SinOut_Phase , & rtP . FromWs_Time0_h113reko55 [ 0 ] ,
& rtP . FromWs_Data0_kupdvn51v2 [ 0 ] , & rtP . FromWs_Time0_dnxa5yquln [ 0 ]
, & rtP . FromWs_Data0_pvombv1exi [ 0 ] , & rtP . FromWs_Time0_acysjor4ip [ 0
] , & rtP . FromWs_Data0_ohxionynup [ 0 ] , & rtP . FromWs_Time0 [ 0 ] , &
rtP . FromWs_Data0 [ 0 ] , & rtP . FromWs_Time0_an32oq3ef4 [ 0 ] , & rtP .
FromWs_Data0_apflh1ms4l [ 0 ] , & rtP . FromWs_Time0_bxwfouleny [ 0 ] , & rtP
. FromWs_Data0_gglwjxomlu [ 0 ] , & rtP . FromWs_Time0_dgntxkplur [ 0 ] , &
rtP . FromWs_Data0_n4fvl010iy [ 0 ] , & rtP . FromWs_Time0_jrdfiknsxo [ 0 ] ,
& rtP . FromWs_Data0_i5qxt5w12n [ 0 ] , & rtP . Gain_Gain_l2aa1kgqdh , & rtP
. A0 [ 0 ] , & rtP . B0 [ 0 ] , & rtP . C0 [ 0 ] , & rtP . KAoss [ 0 ] , &
rtP . KBossw [ 0 ] , & rtP . KCoss [ 0 ] , & rtP . Kopt [ 0 ] , & rtP .
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
rtwCAPI_VECTOR , 16 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 18 , 2 , 0 } , {
rtwCAPI_MATRIX_COL_MAJOR , 20 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 22 , 2
, 0 } } ; static const uint_T rtDimensionArray [ ] = { 4 , 1 , 8 , 1 , 1 , 1
, 3 , 1 , 2 , 1 , 1 , 2 , 6 , 1 , 5 , 1 , 10 , 1 , 8 , 8 , 8 , 4 , 4 , 8 } ;
static const real_T rtcapiStoredFloats [ ] = { 0.0 , 1.0 , 0.1 } ; static
const rtwCAPI_FixPtMap rtFixPtMap [ ] = { { ( NULL ) , ( NULL ) ,
rtwCAPI_FIX_RESERVED , 0 , 0 , 0 } , } ; static const rtwCAPI_SampleTimeMap
rtSampleTimeMap [ ] = { { ( const void * ) & rtcapiStoredFloats [ 0 ] , (
const void * ) & rtcapiStoredFloats [ 0 ] , 0 , 0 } , { ( const void * ) &
rtcapiStoredFloats [ 0 ] , ( const void * ) & rtcapiStoredFloats [ 1 ] , 1 ,
0 } , { ( const void * ) & rtcapiStoredFloats [ 2 ] , ( const void * ) &
rtcapiStoredFloats [ 0 ] , 2 , 0 } , { ( NULL ) , ( NULL ) , 3 , 0 } } ;
static rtwCAPI_ModelMappingStaticInfo mmiStatic = { { rtBlockSignals , 22 , (
NULL ) , 0 , ( NULL ) , 0 } , { rtBlockParameters , 49 , rtModelParameters ,
16 } , { ( NULL ) , 0 } , { rtDataTypeMap , rtDimensionMap , rtFixPtMap ,
rtElementMap , rtSampleTimeMap , rtDimensionArray } , "float" , { 4144187791U
, 48234202U , 2997501980U , 3888407977U } , ( NULL ) , 0 , 0 } ; const
rtwCAPI_ModelMappingStaticInfo * LTRTenzo_GetCAPIStaticMap ( ) { return &
mmiStatic ; }
#ifndef HOST_CAPI_BUILD
void LTRTenzo_InitializeDataMapInfo ( ) { rtwCAPI_SetVersion ( ( *
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
void LTRTenzo_host_InitializeDataMapInfo ( LTRTenzo_host_DataMapInfo_T *
dataMap , const char * path ) { rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ;
rtwCAPI_SetStaticMap ( dataMap -> mmi , & mmiStatic ) ;
rtwCAPI_SetDataAddressMap ( dataMap -> mmi , NULL ) ;
rtwCAPI_SetVarDimsAddressMap ( dataMap -> mmi , NULL ) ; rtwCAPI_SetPath (
dataMap -> mmi , path ) ; rtwCAPI_SetFullPath ( dataMap -> mmi , NULL ) ;
rtwCAPI_SetChildMMIArray ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( dataMap -> mmi , 0 ) ; }
#ifdef __cplusplus
}
#endif
#endif
