#include "__cf_HinfTenzo.h"
#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "HinfTenzo_capi_host.h"
#define sizeof(s) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)    
#else
#include "builtin_typeid_types.h"
#include "HinfTenzo.h"
#include "HinfTenzo_capi.h"
#include "HinfTenzo_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST                  
#define TARGET_STRING(s)               (NULL)                    
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static const rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 0 , TARGET_STRING (
"HinfTenzo/Sum1" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 1 , 0 ,
TARGET_STRING ( "HinfTenzo/Sum3" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0
} , { 2 , 0 , TARGET_STRING ( "HinfTenzo/Sum4" ) , TARGET_STRING ( "" ) , 0 ,
0 , 0 , 0 , 0 } , { 3 , 0 , TARGET_STRING ( "HinfTenzo/Setpoints/Constant2" )
, TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 1 } , { 4 , 0 , TARGET_STRING (
"HinfTenzo/Setpoints/Sum4" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , {
5 , 0 , TARGET_STRING ( "HinfTenzo/Setpoints/P(s)1" ) , TARGET_STRING ( "" )
, 0 , 0 , 1 , 0 , 0 } , { 6 , 0 , TARGET_STRING ( "HinfTenzo/Setpoints/P(s)4"
) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , { 7 , 0 , TARGET_STRING (
"HinfTenzo/Setpoints/P(s)5" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } ,
{ 8 , 0 , TARGET_STRING ( "HinfTenzo/processo/Saturation1" ) , TARGET_STRING
( "" ) , 0 , 0 , 0 , 0 , 0 } , { 9 , 0 , TARGET_STRING (
"HinfTenzo/DisturboIn/ErrIn/Clock1" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0
, 0 } , { 10 , 0 , TARGET_STRING ( "HinfTenzo/DisturboIn/ErrIn/Clock2" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , { 11 , 0 , TARGET_STRING (
"HinfTenzo/DisturboIn/ErrIn/Switch1" ) , TARGET_STRING ( "DiturboStato" ) , 0
, 0 , 1 , 0 , 0 } , { 12 , 0 , TARGET_STRING (
"HinfTenzo/DisturboIn/ErrIn/Switch2" ) , TARGET_STRING ( "DiturboStato" ) , 0
, 0 , 1 , 0 , 0 } , { 13 , 0 , TARGET_STRING (
"HinfTenzo/DisturboIn/ErrIn/P(s)1" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 ,
0 } , { 14 , 0 , TARGET_STRING ( "HinfTenzo/DisturboOut/ErrOutAtt/Clock" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , { 15 , 0 , TARGET_STRING (
"HinfTenzo/DisturboOut/ErrOutAtt/Clock1" ) , TARGET_STRING ( "" ) , 0 , 0 , 1
, 0 , 0 } , { 16 , 0 , TARGET_STRING (
"HinfTenzo/DisturboOut/ErrOutAtt/Switch" ) , TARGET_STRING ( "DiturboStato" )
, 0 , 0 , 1 , 0 , 0 } , { 17 , 0 , TARGET_STRING (
"HinfTenzo/DisturboOut/ErrOutAtt/Switch1" ) , TARGET_STRING ( "DiturboStato"
) , 0 , 0 , 1 , 0 , 0 } , { 18 , 0 , TARGET_STRING (
"HinfTenzo/DisturboOut/ErrOutAtt/P(s)1" ) , TARGET_STRING ( "" ) , 0 , 0 , 1
, 0 , 0 } , { 19 , 0 , TARGET_STRING ( "HinfTenzo/DisturboOut/ErrOutZ/Clock"
) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , { 20 , 0 , TARGET_STRING (
"HinfTenzo/DisturboOut/ErrOutZ/Switch" ) , TARGET_STRING ( "DiturboStato" ) ,
0 , 0 , 1 , 0 , 0 } , { 21 , 0 , TARGET_STRING (
"HinfTenzo/DisturboOut/ErrOutZ/P(s)1" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 ,
0 , 0 } , { 22 , 0 , TARGET_STRING (
"HinfTenzo/Err Mes/Filtered random noise + sin/Gain" ) , TARGET_STRING ( "" )
, 0 , 0 , 1 , 0 , 0 } , { 23 , 0 , TARGET_STRING (
"HinfTenzo/Err Mes/Filtered random noise + sin/Sum3" ) , TARGET_STRING ( "" )
, 0 , 0 , 1 , 0 , 0 } , { 24 , 0 , TARGET_STRING (
"HinfTenzo/Err Mes/Filtered random noise + sin/Uniform Random Number" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 2 } , { 25 , 0 , TARGET_STRING (
"HinfTenzo/Setpoints/psi /FromWs" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 ,
0 } , { 26 , 0 , TARGET_STRING ( "HinfTenzo/Setpoints/zref(t)/FromWs" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , { 27 , 1 , TARGET_STRING (
"HinfTenzo/DisturboIn/ErrIn/distOut /Sum2" ) , TARGET_STRING ( "" ) , 0 , 0 ,
1 , 0 , 0 } , { 28 , 2 , TARGET_STRING (
"HinfTenzo/DisturboOut/ErrOutAtt/disturbo/Sum2" ) , TARGET_STRING ( "" ) , 0
, 0 , 1 , 0 , 0 } , { 29 , 3 , TARGET_STRING (
"HinfTenzo/DisturboOut/ErrOutZ/disturbo/Sum2" ) , TARGET_STRING ( "" ) , 0 ,
0 , 1 , 0 , 0 } , { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ;
static const rtwCAPI_BlockParameters rtBlockParameters [ ] = { { 30 ,
TARGET_STRING ( "HinfTenzo/Controller/H-Infinity" ) , TARGET_STRING ( "A" ) ,
0 , 2 , 0 } , { 31 , TARGET_STRING ( "HinfTenzo/Controller/H-Infinity" ) ,
TARGET_STRING ( "B" ) , 0 , 3 , 0 } , { 32 , TARGET_STRING (
"HinfTenzo/Controller/H-Infinity" ) , TARGET_STRING ( "C" ) , 0 , 3 , 0 } , {
33 , TARGET_STRING ( "HinfTenzo/Controller/H-Infinity" ) , TARGET_STRING (
"X0" ) , 0 , 1 , 0 } , { 34 , TARGET_STRING ( "HinfTenzo/Setpoints/Constant2"
) , TARGET_STRING ( "Value" ) , 0 , 1 , 0 } , { 35 , TARGET_STRING (
"HinfTenzo/Setpoints/P(s)1" ) , TARGET_STRING ( "A" ) , 0 , 4 , 0 } , { 36 ,
TARGET_STRING ( "HinfTenzo/Setpoints/P(s)1" ) , TARGET_STRING ( "C" ) , 0 , 5
, 0 } , { 37 , TARGET_STRING ( "HinfTenzo/Setpoints/P(s)4" ) , TARGET_STRING
( "A" ) , 0 , 4 , 0 } , { 38 , TARGET_STRING ( "HinfTenzo/Setpoints/P(s)4" )
, TARGET_STRING ( "C" ) , 0 , 5 , 0 } , { 39 , TARGET_STRING (
"HinfTenzo/Setpoints/P(s)5" ) , TARGET_STRING ( "A" ) , 0 , 4 , 0 } , { 40 ,
TARGET_STRING ( "HinfTenzo/Setpoints/P(s)5" ) , TARGET_STRING ( "C" ) , 0 , 5
, 0 } , { 41 , TARGET_STRING ( "HinfTenzo/processo/Saturation1" ) ,
TARGET_STRING ( "UpperLimit" ) , 0 , 1 , 0 } , { 42 , TARGET_STRING (
"HinfTenzo/processo/Saturation1" ) , TARGET_STRING ( "LowerLimit" ) , 0 , 1 ,
0 } , { 43 , TARGET_STRING ( "HinfTenzo/processo/Processo1" ) , TARGET_STRING
( "X0" ) , 0 , 1 , 0 } , { 44 , TARGET_STRING (
"HinfTenzo/DisturboIn/ErrIn/Constant1" ) , TARGET_STRING ( "Value" ) , 0 , 1
, 0 } , { 45 , TARGET_STRING ( "HinfTenzo/DisturboIn/ErrIn/Constant2" ) ,
TARGET_STRING ( "Value" ) , 0 , 1 , 0 } , { 46 , TARGET_STRING (
"HinfTenzo/DisturboIn/ErrIn/Switch1" ) , TARGET_STRING ( "Threshold" ) , 0 ,
1 , 0 } , { 47 , TARGET_STRING ( "HinfTenzo/DisturboIn/ErrIn/Switch2" ) ,
TARGET_STRING ( "Threshold" ) , 0 , 1 , 0 } , { 48 , TARGET_STRING (
"HinfTenzo/DisturboIn/ErrIn/P(s)1" ) , TARGET_STRING ( "A" ) , 0 , 4 , 0 } ,
{ 49 , TARGET_STRING ( "HinfTenzo/DisturboIn/ErrIn/P(s)1" ) , TARGET_STRING (
"C" ) , 0 , 5 , 0 } , { 50 , TARGET_STRING (
"HinfTenzo/DisturboOut/ErrOutAtt/Constant" ) , TARGET_STRING ( "Value" ) , 0
, 1 , 0 } , { 51 , TARGET_STRING (
"HinfTenzo/DisturboOut/ErrOutAtt/Constant2" ) , TARGET_STRING ( "Value" ) , 0
, 1 , 0 } , { 52 , TARGET_STRING ( "HinfTenzo/DisturboOut/ErrOutAtt/Switch" )
, TARGET_STRING ( "Threshold" ) , 0 , 1 , 0 } , { 53 , TARGET_STRING (
"HinfTenzo/DisturboOut/ErrOutAtt/Switch1" ) , TARGET_STRING ( "Threshold" ) ,
0 , 1 , 0 } , { 54 , TARGET_STRING ( "HinfTenzo/DisturboOut/ErrOutAtt/P(s)1"
) , TARGET_STRING ( "A" ) , 0 , 4 , 0 } , { 55 , TARGET_STRING (
"HinfTenzo/DisturboOut/ErrOutAtt/P(s)1" ) , TARGET_STRING ( "C" ) , 0 , 5 , 0
} , { 56 , TARGET_STRING ( "HinfTenzo/DisturboOut/ErrOutZ/Constant" ) ,
TARGET_STRING ( "Value" ) , 0 , 1 , 0 } , { 57 , TARGET_STRING (
"HinfTenzo/DisturboOut/ErrOutZ/Switch" ) , TARGET_STRING ( "Threshold" ) , 0
, 1 , 0 } , { 58 , TARGET_STRING ( "HinfTenzo/DisturboOut/ErrOutZ/P(s)1" ) ,
TARGET_STRING ( "A" ) , 0 , 4 , 0 } , { 59 , TARGET_STRING (
"HinfTenzo/DisturboOut/ErrOutZ/P(s)1" ) , TARGET_STRING ( "C" ) , 0 , 5 , 0 }
, { 60 , TARGET_STRING (
"HinfTenzo/Err Mes/Filtered random noise + sin/Constant1" ) , TARGET_STRING (
"Value" ) , 0 , 1 , 0 } , { 61 , TARGET_STRING (
"HinfTenzo/Err Mes/Filtered random noise + sin/Gain" ) , TARGET_STRING (
"Gain" ) , 0 , 1 , 0 } , { 62 , TARGET_STRING (
"HinfTenzo/Err Mes/Filtered random noise + sin/0.2 sen(0.5 t)" ) ,
TARGET_STRING ( "Bias" ) , 0 , 1 , 0 } , { 63 , TARGET_STRING (
"HinfTenzo/Err Mes/Filtered random noise + sin/0.2 sen(0.5 t)" ) ,
TARGET_STRING ( "Phase" ) , 0 , 1 , 0 } , { 64 , TARGET_STRING (
"HinfTenzo/Err Mes/Filtered random noise + sin/Manual Switch1" ) ,
TARGET_STRING ( "CurrentSetting" ) , 1 , 1 , 0 } , { 65 , TARGET_STRING (
"HinfTenzo/Err Mes/Filtered random noise + sin/P(s)1" ) , TARGET_STRING ( "A"
) , 0 , 1 , 0 } , { 66 , TARGET_STRING (
"HinfTenzo/Err Mes/Filtered random noise + sin/P(s)1" ) , TARGET_STRING ( "C"
) , 0 , 1 , 0 } , { 67 , TARGET_STRING (
"HinfTenzo/Err Mes/Filtered random noise + sin/Uniform Random Number" ) ,
TARGET_STRING ( "Seed" ) , 0 , 1 , 0 } , { 68 , TARGET_STRING (
"HinfTenzo/Setpoints/phy(t)/FromWs" ) , TARGET_STRING ( "Time0" ) , 0 , 6 , 0
} , { 69 , TARGET_STRING ( "HinfTenzo/Setpoints/phy(t)/FromWs" ) ,
TARGET_STRING ( "Data0" ) , 0 , 6 , 0 } , { 70 , TARGET_STRING (
"HinfTenzo/Setpoints/psi /FromWs" ) , TARGET_STRING ( "Time0" ) , 0 , 6 , 0 }
, { 71 , TARGET_STRING ( "HinfTenzo/Setpoints/psi /FromWs" ) , TARGET_STRING
( "Data0" ) , 0 , 6 , 0 } , { 72 , TARGET_STRING (
"HinfTenzo/Setpoints/theta(t)/FromWs" ) , TARGET_STRING ( "Time0" ) , 0 , 6 ,
0 } , { 73 , TARGET_STRING ( "HinfTenzo/Setpoints/theta(t)/FromWs" ) ,
TARGET_STRING ( "Data0" ) , 0 , 6 , 0 } , { 74 , TARGET_STRING (
"HinfTenzo/Setpoints/zref(t)/FromWs" ) , TARGET_STRING ( "Time0" ) , 0 , 7 ,
0 } , { 75 , TARGET_STRING ( "HinfTenzo/Setpoints/zref(t)/FromWs" ) ,
TARGET_STRING ( "Data0" ) , 0 , 7 , 0 } , { 76 , TARGET_STRING (
"HinfTenzo/DisturboIn/ErrIn/distOut /seno" ) , TARGET_STRING ( "Amplitude" )
, 0 , 1 , 0 } , { 77 , TARGET_STRING (
"HinfTenzo/DisturboIn/ErrIn/distOut /seno" ) , TARGET_STRING ( "Bias" ) , 0 ,
1 , 0 } , { 78 , TARGET_STRING ( "HinfTenzo/DisturboIn/ErrIn/distOut /seno" )
, TARGET_STRING ( "Phase" ) , 0 , 1 , 0 } , { 79 , TARGET_STRING (
"HinfTenzo/DisturboOut/ErrOutAtt/disturbo/SineOut" ) , TARGET_STRING ( "Bias"
) , 0 , 1 , 0 } , { 80 , TARGET_STRING (
"HinfTenzo/DisturboOut/ErrOutAtt/disturbo/SineOut" ) , TARGET_STRING (
"Phase" ) , 0 , 1 , 0 } , { 81 , TARGET_STRING (
"HinfTenzo/DisturboOut/ErrOutZ/disturbo/SineOut" ) , TARGET_STRING ( "Bias" )
, 0 , 1 , 0 } , { 82 , TARGET_STRING (
"HinfTenzo/DisturboOut/ErrOutZ/disturbo/SineOut" ) , TARGET_STRING ( "Phase"
) , 0 , 1 , 0 } , { 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 } } ; static const
rtwCAPI_ModelParameters rtModelParameters [ ] = { { 83 , TARGET_STRING ( "A0"
) , 0 , 8 , 0 } , { 84 , TARGET_STRING ( "B0" ) , 0 , 9 , 0 } , { 85 ,
TARGET_STRING ( "C0" ) , 0 , 10 , 0 } , { 86 , TARGET_STRING (
"amplitudeNoise" ) , 0 , 1 , 0 } , { 87 , TARGET_STRING ( "amplitudePertOutZ"
) , 0 , 1 , 0 } , { 88 , TARGET_STRING ( "amplitudePertOutptp" ) , 0 , 1 , 0
} , { 89 , TARGET_STRING ( "cstPertOut" ) , 0 , 1 , 0 } , { 90 ,
TARGET_STRING ( "omegaNoise" ) , 0 , 1 , 0 } , { 91 , TARGET_STRING (
"omegaPertOut" ) , 0 , 1 , 0 } , { 92 , TARGET_STRING ( "randomAmpNoise" ) ,
0 , 1 , 0 } , { 0 , ( NULL ) , 0 , 0 , 0 } } ;
#ifndef HOST_CAPI_BUILD
static void * rtDataAddrMap [ ] = { & rtB . g3cxppgcad [ 0 ] , & rtB .
cuhucxnwbl [ 0 ] , & rtB . jf1eral2ve [ 0 ] , & rtB . kohnzyllz4 , & rtB .
eivqzdcosm , & rtB . cgqon1bp5k , & rtB . ndvxwhqhzh , & rtB . lywwonuzni , &
rtB . ngbxrb2p33 [ 0 ] , & rtB . ffcuwotdjl , & rtB . h0k2fzanuq , & rtB .
l3ibipirjv , & rtB . o4bvr14r1c , & rtB . nq0m4x3haw , & rtB . iklusr35kc , &
rtB . elhmsr12bw , & rtB . epfm0vmgr0 , & rtB . dx0ibnxabm , & rtB .
iglrdpeo0u , & rtB . ahq3gaiq3g , & rtB . jfilnvyqtk , & rtB . mikmmv5pjy , &
rtB . o2ra02ecjo , & rtB . afiufrufpa , & rtB . bnmysexqkh , & rtB .
bnmtoyr0ou , & rtB . ji3tio223s , & rtB . nue240oiht , & rtB . l5s1awmxqx , &
rtB . c33unhaobk , & rtP . HInfinity_A [ 0 ] , & rtP . HInfinity_B [ 0 ] , &
rtP . HInfinity_C [ 0 ] , & rtP . HInfinity_X0 , & rtP . Constant2_Value , &
rtP . Ps1_A_g40xqitwcm [ 0 ] , & rtP . Ps1_C_jxo2syryaa [ 0 ] , & rtP . Ps4_A
[ 0 ] , & rtP . Ps4_C [ 0 ] , & rtP . Ps5_A [ 0 ] , & rtP . Ps5_C [ 0 ] , &
rtP . Saturation1_UpperSat , & rtP . Saturation1_LowerSat , & rtP .
Processo1_X0 , & rtP . Constant1_Value_onxqoxz4b2 , & rtP .
Constant2_Value_honrvu2u0e , & rtP . Switch1_Threshold , & rtP .
Switch2_Threshold , & rtP . Ps1_A_p4fqdq0nyf [ 0 ] , & rtP . Ps1_C_iunwv5jncy
[ 0 ] , & rtP . Constant_Value , & rtP . Constant2_Value_maeni3njql , & rtP .
Switch_Threshold , & rtP . Switch1_Threshold_a0fpu4ohya , & rtP .
Ps1_A_j1blkzytxk [ 0 ] , & rtP . Ps1_C_a4z2vpeibb [ 0 ] , & rtP .
Constant_Value_gsg5wrsjtq , & rtP . Switch_Threshold_je2vilxnqj , & rtP .
Ps1_A [ 0 ] , & rtP . Ps1_C [ 0 ] , & rtP . Constant1_Value , & rtP .
Gain_Gain , & rtP . u2sen05t_Bias , & rtP . u2sen05t_Phase , & rtP .
ManualSwitch1_CurrentSetting , & rtP . Ps1_A_cnuk0h4tar , & rtP .
Ps1_C_gfroigzrf4 , & rtP . UniformRandomNumber_Seed , & rtP . FromWs_Time0 [
0 ] , & rtP . FromWs_Data0 [ 0 ] , & rtP . FromWs_Time0_ni0stwjpd1 [ 0 ] , &
rtP . FromWs_Data0_bui33k5vfq [ 0 ] , & rtP . FromWs_Time0_mdh1vup2gy [ 0 ] ,
& rtP . FromWs_Data0_amonalu2nw [ 0 ] , & rtP . FromWs_Time0_ovshumjb0b [ 0 ]
, & rtP . FromWs_Data0_cjzjycivf4 [ 0 ] , & rtP . seno_Amp , & rtP .
seno_Bias , & rtP . seno_Phase , & rtP . SineOut_Bias , & rtP . SineOut_Phase
, & rtP . SineOut_Bias_amml0uhsh5 , & rtP . SineOut_Phase_ek4wf0nzh3 , & rtP
. A0 [ 0 ] , & rtP . B0 [ 0 ] , & rtP . C0 [ 0 ] , & rtP . amplitudeNoise , &
rtP . amplitudePertOutZ , & rtP . amplitudePertOutptp , & rtP . cstPertOut ,
& rtP . omegaNoise , & rtP . omegaPertOut , & rtP . randomAmpNoise , } ;
static int32_T * rtVarDimsAddrMap [ ] = { ( NULL ) } ;
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
rtwCAPI_VECTOR , 0 , 2 , 0 } , { rtwCAPI_SCALAR , 2 , 2 , 0 } , {
rtwCAPI_VECTOR , 4 , 2 , 0 } , { rtwCAPI_VECTOR , 6 , 2 , 0 } , {
rtwCAPI_VECTOR , 8 , 2 , 0 } , { rtwCAPI_VECTOR , 10 , 2 , 0 } , {
rtwCAPI_VECTOR , 12 , 2 , 0 } , { rtwCAPI_VECTOR , 14 , 2 , 0 } , {
rtwCAPI_MATRIX_COL_MAJOR , 16 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 18 , 2
, 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 20 , 2 , 0 } } ; static const uint_T
rtDimensionArray [ ] = { 4 , 1 , 1 , 1 , 604 , 1 , 112 , 1 , 2 , 1 , 1 , 2 ,
6 , 1 , 10 , 1 , 8 , 8 , 8 , 4 , 4 , 8 } ; static const real_T
rtcapiStoredFloats [ ] = { 0.0 , 0.1 } ; static const rtwCAPI_FixPtMap
rtFixPtMap [ ] = { { ( NULL ) , ( NULL ) , rtwCAPI_FIX_RESERVED , 0 , 0 , 0 }
, } ; static const rtwCAPI_SampleTimeMap rtSampleTimeMap [ ] = { { ( const
void * ) & rtcapiStoredFloats [ 0 ] , ( const void * ) & rtcapiStoredFloats [
0 ] , 0 , 0 } , { ( NULL ) , ( NULL ) , 2 , 0 } , { ( const void * ) &
rtcapiStoredFloats [ 1 ] , ( const void * ) & rtcapiStoredFloats [ 0 ] , 1 ,
0 } } ; static rtwCAPI_ModelMappingStaticInfo mmiStatic = { { rtBlockSignals
, 30 , ( NULL ) , 0 , ( NULL ) , 0 } , { rtBlockParameters , 53 ,
rtModelParameters , 10 } , { ( NULL ) , 0 } , { rtDataTypeMap ,
rtDimensionMap , rtFixPtMap , rtElementMap , rtSampleTimeMap ,
rtDimensionArray } , "float" , { 2359241291U , 787542146U , 808962859U ,
974030338U } , ( NULL ) , 0 , 0 } ; const rtwCAPI_ModelMappingStaticInfo *
HinfTenzo_GetCAPIStaticMap ( ) { return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
void HinfTenzo_InitializeDataMapInfo ( ) { rtwCAPI_SetVersion ( ( *
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
void HinfTenzo_host_InitializeDataMapInfo ( HinfTenzo_host_DataMapInfo_T *
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
