#include "__cf_PhiTheta.h"
#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "PhiTheta_capi_host.h"
#define sizeof(s) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)    
#else
#include "builtin_typeid_types.h"
#include "PhiTheta.h"
#include "PhiTheta_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST                  
#define TARGET_STRING(s)               (NULL)                    
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static const rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 0 , TARGET_STRING (
"PhiTheta/Plant/P(s)" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 1 ,
0 , TARGET_STRING ( "PhiTheta/Roll Pitch  Dynamic1/Sum" ) , TARGET_STRING (
"" ) , 0 , 0 , 0 , 0 , 0 } , { 2 , 0 , TARGET_STRING (
"PhiTheta/Roll Pitch  Dynamic1/Sum4" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0
, 0 } , { 3 , 0 , TARGET_STRING ( "PhiTheta/Signal Builder/FromWs" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 4 , 0 , TARGET_STRING (
"PhiTheta/Roll Pitch  Dynamic1/Disturbi/Gain" ) , TARGET_STRING ( "" ) , 0 ,
0 , 0 , 0 , 0 } , { 5 , 0 , TARGET_STRING (
"PhiTheta/Roll Pitch  Dynamic1/Disturbi/Sum3" ) , TARGET_STRING ( "" ) , 0 ,
0 , 0 , 0 , 0 } , { 6 , 0 , TARGET_STRING (
"PhiTheta/Roll Pitch  Dynamic1/Disturbi/Uniform Random Number" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 7 , 0 , TARGET_STRING (
"PhiTheta/Roll Pitch  Dynamic1/Err Mes/Gain" ) , TARGET_STRING ( "" ) , 0 , 0
, 0 , 0 , 0 } , { 8 , 0 , TARGET_STRING (
"PhiTheta/Roll Pitch  Dynamic1/Err Mes/Sum3" ) , TARGET_STRING ( "" ) , 0 , 0
, 0 , 0 , 0 } , { 9 , 0 , TARGET_STRING (
"PhiTheta/Roll Pitch  Dynamic1/Err Mes/Uniform Random Number" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 10 , 0 , TARGET_STRING (
"PhiTheta/Roll Pitch  Dynamic1/NonLin1/Manual Switch" ) , TARGET_STRING ( ""
) , 0 , 0 , 0 , 0 , 1 } , { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 }
} ;
#ifndef HOST_CAPI_BUILD
static void * rtDataAddrMap [ ] = { & rtB . mk4dpnamwc , & rtB . awmfihve5b ,
& rtB . blhrfoy5vb , & rtB . o3ul0nzcwe , & rtB . opuqvtwzzt , & rtB .
dgroez0siq , & rtB . ngxpbqhzzn , & rtB . dewu3oxpw1 , & rtB . pydoj1kzpi , &
rtB . kqoy4n5gjo , & rtB . pk1lzuz22z , } ; static int32_T * rtVarDimsAddrMap
[ ] = { ( NULL ) } ;
#endif
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap [ ] = { { "double" ,
"real_T" , 0 , 0 , sizeof ( real_T ) , SS_DOUBLE , 0 , 0 } } ;
#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif
static TARGET_CONST rtwCAPI_ElementMap rtElementMap [ ] = { { ( NULL ) , 0 ,
0 , 0 , 0 } , } ; static const rtwCAPI_DimensionMap rtDimensionMap [ ] = { {
rtwCAPI_SCALAR , 0 , 2 , 0 } } ; static const uint_T rtDimensionArray [ ] = {
1 , 1 } ; static const real_T rtcapiStoredFloats [ ] = { 0.0 , 0.1 } ; static
const rtwCAPI_FixPtMap rtFixPtMap [ ] = { { ( NULL ) , ( NULL ) ,
rtwCAPI_FIX_RESERVED , 0 , 0 , 0 } , } ; static const rtwCAPI_SampleTimeMap
rtSampleTimeMap [ ] = { { ( const void * ) & rtcapiStoredFloats [ 0 ] , (
const void * ) & rtcapiStoredFloats [ 0 ] , 0 , 0 } , { ( const void * ) &
rtcapiStoredFloats [ 1 ] , ( const void * ) & rtcapiStoredFloats [ 0 ] , 1 ,
0 } } ; static rtwCAPI_ModelMappingStaticInfo mmiStatic = { { rtBlockSignals
, 11 , ( NULL ) , 0 , ( NULL ) , 0 } , { ( NULL ) , 0 , ( NULL ) , 0 } , { (
NULL ) , 0 } , { rtDataTypeMap , rtDimensionMap , rtFixPtMap , rtElementMap ,
rtSampleTimeMap , rtDimensionArray } , "float" , { 2450082240U , 2327675151U
, 2800790287U , 2590810103U } , ( NULL ) , 0 } ; const
rtwCAPI_ModelMappingStaticInfo * PhiTheta_GetCAPIStaticMap ( ) { return &
mmiStatic ; }
#ifndef HOST_CAPI_BUILD
void PhiTheta_InitializeDataMapInfo ( SimStruct * const rtS ) {
rtwCAPI_SetVersion ( ( * rt_dataMapInfoPtr ) . mmi , 1 ) ;
rtwCAPI_SetStaticMap ( ( * rt_dataMapInfoPtr ) . mmi , & mmiStatic ) ;
rtwCAPI_SetLoggingStaticMap ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetDataAddressMap ( ( * rt_dataMapInfoPtr ) . mmi , rtDataAddrMap ) ;
rtwCAPI_SetVarDimsAddressMap ( ( * rt_dataMapInfoPtr ) . mmi ,
rtVarDimsAddrMap ) ; rtwCAPI_SetInstanceLoggingInfo ( ( * rt_dataMapInfoPtr )
. mmi , ( NULL ) ) ; rtwCAPI_SetChildMMIArray ( ( * rt_dataMapInfoPtr ) . mmi
, ( NULL ) ) ; rtwCAPI_SetChildMMIArrayLen ( ( * rt_dataMapInfoPtr ) . mmi ,
0 ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void PhiTheta_host_InitializeDataMapInfo ( PhiTheta_host_DataMapInfo_T *
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
