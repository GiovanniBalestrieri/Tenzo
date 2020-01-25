#include "__cf_HinfTenzo.h"
#include "rt_logging_mmi.h"
#include "HinfTenzo_capi.h"
#include <math.h>
#include "HinfTenzo.h"
#include "HinfTenzo_private.h"
#include "HinfTenzo_dt.h"
extern void * CreateDiagnosticAsVoidPtr_wrapper ( const char * id , int nargs
, ... ) ; RTWExtModeInfo * gblRTWExtModeInfo = NULL ; extern boolean_T
gblExtModeStartPktReceived ; void raccelForceExtModeShutdown ( ) { if ( !
gblExtModeStartPktReceived ) { boolean_T stopRequested = false ;
rtExtModeWaitForStartPkt ( gblRTWExtModeInfo , 2 , & stopRequested ) ; }
rtExtModeShutdown ( 2 ) ; } const int_T gblNumToFiles = 0 ; const int_T
gblNumFrFiles = 0 ; const int_T gblNumFrWksBlocks = 4 ;
#ifdef RSIM_WITH_SOLVER_MULTITASKING
boolean_T gbl_raccel_isMultitasking = 1 ;
#else
boolean_T gbl_raccel_isMultitasking = 0 ;
#endif
boolean_T gbl_raccel_tid01eq = 0 ; int_T gbl_raccel_NumST = 3 ; const char_T
* gbl_raccel_Version = "8.12 (R2017a) 16-Feb-2017" ; void
raccel_setup_MMIStateLog ( SimStruct * S ) {
#ifdef UseMMIDataLogging
rt_FillStateSigInfoFromMMI ( ssGetRTWLogInfo ( S ) , & ssGetErrorStatus ( S )
) ;
#else
UNUSED_PARAMETER ( S ) ;
#endif
} static DataMapInfo rt_dataMapInfo ; DataMapInfo * rt_dataMapInfoPtr = &
rt_dataMapInfo ; rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr = & (
rt_dataMapInfo . mmi ) ; const char * gblSlvrJacPatternFileName =
"slprj//raccel//HinfTenzo//HinfTenzo_Jpattern.mat" ; const int_T
gblNumRootInportBlks = 0 ; const int_T gblNumModelInputs = 0 ; extern
rtInportTUtable * gblInportTUtables ; extern const char * gblInportFileName ;
const int_T gblInportDataTypeIdx [ ] = { - 1 } ; const int_T gblInportDims [
] = { - 1 } ; const int_T gblInportComplex [ ] = { - 1 } ; const int_T
gblInportInterpoFlag [ ] = { - 1 } ; const int_T gblInportContinuous [ ] = {
- 1 } ;
#include "simstruc.h"
#include "fixedpoint.h"
B rtB ; X rtX ; DW rtDW ; static SimStruct model_S ; SimStruct * const rtS =
& model_S ; real_T rt_urand_Upu32_Yd_f_pw_snf ( uint32_T * u ) { uint32_T lo
; uint32_T hi ; lo = * u % 127773U * 16807U ; hi = * u / 127773U * 2836U ; if
( lo < hi ) { * u = 2147483647U - ( hi - lo ) ; } else { * u = lo - hi ; }
return ( real_T ) * u * 4.6566128752457969E-10 ; } void MdlInitialize ( void
) { int_T is ; uint32_T tseed ; int32_T t ; real_T tmp ; rtX . ltfask1ptj [ 0
] = 0.0 ; rtX . k5wabiqsuk [ 0 ] = 0.0 ; rtX . ltfask1ptj [ 1 ] = 0.0 ; rtX .
k5wabiqsuk [ 1 ] = 0.0 ; for ( is = 0 ; is < 8 ; is ++ ) { rtX . b11juaaqeh [
is ] = rtP . Processo1_X0 ; } rtX . oacfijplwi [ 0 ] = 0.0 ; rtX . njhzavtx5o
[ 0 ] = 0.0 ; rtX . hudxhgdxi2 [ 0 ] = 0.0 ; rtX . oacfijplwi [ 1 ] = 0.0 ;
rtX . njhzavtx5o [ 1 ] = 0.0 ; rtX . hudxhgdxi2 [ 1 ] = 0.0 ; rtX .
l2dleorvkm = 0.0 ; for ( is = 0 ; is < 28 ; is ++ ) { rtX . phhupngpjs [ is ]
= rtP . HInfinity_X0 ; } rtX . inruainrus [ 0 ] = 0.0 ; rtX . inruainrus [ 1
] = 0.0 ; tmp = muDoubleScalarFloor ( rtP . UniformRandomNumber_Seed ) ; if (
muDoubleScalarIsNaN ( tmp ) || muDoubleScalarIsInf ( tmp ) ) { tmp = 0.0 ; }
else { tmp = muDoubleScalarRem ( tmp , 4.294967296E+9 ) ; } tseed = tmp < 0.0
? ( uint32_T ) - ( int32_T ) ( uint32_T ) - tmp : ( uint32_T ) tmp ; is = (
int32_T ) ( tseed >> 16U ) ; t = ( int32_T ) ( tseed & 32768U ) ; tseed = ( (
( ( tseed - ( ( uint32_T ) is << 16U ) ) + t ) << 16U ) + t ) + is ; if (
tseed < 1U ) { tseed = 1144108930U ; } else { if ( tseed > 2147483646U ) {
tseed = 2147483646U ; } } rtDW . ic2yhbr32w = tseed ; rtDW . ox0fllt2el = (
rtP . randomAmpNoise - ( - rtP . randomAmpNoise ) ) *
rt_urand_Upu32_Yd_f_pw_snf ( & rtDW . ic2yhbr32w ) + - rtP . randomAmpNoise ;
} void MdlStart ( void ) { { void * * slioCatalogueAddr =
rt_slioCatalogueAddr ( ) ; void * r2 = ( NULL ) ; void * *
pOSigstreamManagerAddr = ( NULL ) ; const char *
errorCreatingOSigstreamManager = ( NULL ) ; const char *
errorAddingR2SharedResource = ( NULL ) ; * slioCatalogueAddr =
rtwGetNewSlioCatalogue ( rt_GetMatSigLogSelectorFileName ( ) ) ;
errorAddingR2SharedResource = rtwAddR2SharedResource (
rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( ) ) , 1 ) ; if (
errorAddingR2SharedResource != ( NULL ) ) { rtwTerminateSlioCatalogue (
slioCatalogueAddr ) ; * slioCatalogueAddr = ( NULL ) ; ssSetErrorStatus ( rtS
, errorAddingR2SharedResource ) ; return ; } r2 = rtwGetR2SharedResource (
rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( ) ) ) ;
pOSigstreamManagerAddr = rt_GetOSigstreamManagerAddr ( ) ;
errorCreatingOSigstreamManager = rtwOSigstreamManagerCreateInstance (
rt_GetMatSigLogSelectorFileName ( ) , r2 , pOSigstreamManagerAddr ) ; if (
errorCreatingOSigstreamManager != ( NULL ) ) { * pOSigstreamManagerAddr = (
NULL ) ; ssSetErrorStatus ( rtS , errorCreatingOSigstreamManager ) ; return ;
} } { static int_T rt_ToWksWidths [ ] = { 2 } ; static int_T
rt_ToWksNumDimensions [ ] = { 1 } ; static int_T rt_ToWksDimensions [ ] = { 2
} ; static boolean_T rt_ToWksIsVarDims [ ] = { 0 } ; static void *
rt_ToWksCurrSigDims [ ] = { ( NULL ) } ; static int_T rt_ToWksCurrSigDimsSize
[ ] = { 4 } ; static BuiltInDTypeId rt_ToWksDataTypeIds [ ] = { SS_DOUBLE } ;
static int_T rt_ToWksComplexSignals [ ] = { 0 } ; static int_T
rt_ToWksFrameData [ ] = { 0 } ; static const char_T * rt_ToWksLabels [ ] = {
"" } ; static RTWLogSignalInfo rt_ToWksSignalInfo = { 1 , rt_ToWksWidths ,
rt_ToWksNumDimensions , rt_ToWksDimensions , rt_ToWksIsVarDims ,
rt_ToWksCurrSigDims , rt_ToWksCurrSigDimsSize , rt_ToWksDataTypeIds ,
rt_ToWksComplexSignals , rt_ToWksFrameData , { rt_ToWksLabels } , ( NULL ) ,
( NULL ) , ( NULL ) , { ( NULL ) } , { ( NULL ) } , ( NULL ) , ( NULL ) } ;
static const char_T rt_ToWksBlockName [ ] = "HinfTenzo/Output/zW(t)" ; rtDW .
hwrxgbpfob . LoggedData = rt_CreateStructLogVar ( ssGetRTWLogInfo ( rtS ) ,
ssGetTStart ( rtS ) , ssGetTFinal ( rtS ) , 0.0 , ( & ssGetErrorStatus ( rtS
) ) , "ztHinf" , 1 , 0 , 1 , 0.0 , & rt_ToWksSignalInfo , rt_ToWksBlockName )
; if ( rtDW . hwrxgbpfob . LoggedData == ( NULL ) ) return ; } { static int_T
rt_ToWksWidths [ ] = { 4 } ; static int_T rt_ToWksNumDimensions [ ] = { 1 } ;
static int_T rt_ToWksDimensions [ ] = { 4 } ; static boolean_T
rt_ToWksIsVarDims [ ] = { 0 } ; static void * rt_ToWksCurrSigDims [ ] = { (
NULL ) } ; static int_T rt_ToWksCurrSigDimsSize [ ] = { 4 } ; static
BuiltInDTypeId rt_ToWksDataTypeIds [ ] = { SS_DOUBLE } ; static int_T
rt_ToWksComplexSignals [ ] = { 0 } ; static int_T rt_ToWksFrameData [ ] = { 0
} ; static const char_T * rt_ToWksLabels [ ] = { "" } ; static
RTWLogSignalInfo rt_ToWksSignalInfo = { 1 , rt_ToWksWidths ,
rt_ToWksNumDimensions , rt_ToWksDimensions , rt_ToWksIsVarDims ,
rt_ToWksCurrSigDims , rt_ToWksCurrSigDimsSize , rt_ToWksDataTypeIds ,
rt_ToWksComplexSignals , rt_ToWksFrameData , { rt_ToWksLabels } , ( NULL ) ,
( NULL ) , ( NULL ) , { ( NULL ) } , { ( NULL ) } , ( NULL ) , ( NULL ) } ;
static const char_T rt_ToWksBlockName [ ] =
"HinfTenzo/Controller/To Workspace" ; rtDW . nnbpbghk5r . LoggedData =
rt_CreateStructLogVar ( ssGetRTWLogInfo ( rtS ) , ssGetTStart ( rtS ) ,
ssGetTFinal ( rtS ) , 0.0 , ( & ssGetErrorStatus ( rtS ) ) , "errSim" , 0 , 0
, 1 , 0.0 , & rt_ToWksSignalInfo , rt_ToWksBlockName ) ; if ( rtDW .
nnbpbghk5r . LoggedData == ( NULL ) ) return ; } { FWksInfo * fromwksInfo ;
if ( ( fromwksInfo = ( FWksInfo * ) calloc ( 1 , sizeof ( FWksInfo ) ) ) == (
NULL ) ) { ssSetErrorStatus ( rtS ,
"from workspace STRING(Name) memory allocation error" ) ; } else {
fromwksInfo -> origWorkspaceVarName = "tuvar" ; fromwksInfo -> origDataTypeId
= 0 ; fromwksInfo -> origIsComplex = 0 ; fromwksInfo -> origWidth = 1 ;
fromwksInfo -> origElSize = sizeof ( real_T ) ; fromwksInfo -> data = ( void
* ) rtP . FromWs_Data0 ; fromwksInfo -> nDataPoints = 6 ; fromwksInfo -> time
= ( double * ) rtP . FromWs_Time0 ; rtDW . lmen2irgnh . TimePtr = fromwksInfo
-> time ; rtDW . lmen2irgnh . DataPtr = fromwksInfo -> data ; rtDW .
lmen2irgnh . RSimInfoPtr = fromwksInfo ; } rtDW . oosdo3rh02 . PrevIndex = 0
; { int_T * zcTimeIndices = & rtDW . ihm14vuvou [ 0 ] ; const double *
timePoints = ( double * ) rtDW . lmen2irgnh . TimePtr ; boolean_T
justHadZcTime = false ; int_T zcIdx = 0 ; int_T i ; for ( i = 0 ; i < 6 - 1 ;
i ++ ) { if ( ! justHadZcTime && timePoints [ i ] == timePoints [ i + 1 ] ) {
zcTimeIndices [ zcIdx ++ ] = i ; justHadZcTime = true ; } else {
justHadZcTime = false ; } } rtDW . mizp5pcuu3 = 0 ; } } { FWksInfo *
fromwksInfo ; if ( ( fromwksInfo = ( FWksInfo * ) calloc ( 1 , sizeof (
FWksInfo ) ) ) == ( NULL ) ) { ssSetErrorStatus ( rtS ,
"from workspace STRING(Name) memory allocation error" ) ; } else {
fromwksInfo -> origWorkspaceVarName = "tuvar" ; fromwksInfo -> origDataTypeId
= 0 ; fromwksInfo -> origIsComplex = 0 ; fromwksInfo -> origWidth = 1 ;
fromwksInfo -> origElSize = sizeof ( real_T ) ; fromwksInfo -> data = ( void
* ) rtP . FromWs_Data0_amonalu2nw ; fromwksInfo -> nDataPoints = 6 ;
fromwksInfo -> time = ( double * ) rtP . FromWs_Time0_mdh1vup2gy ; rtDW .
kxuk4bilk3 . TimePtr = fromwksInfo -> time ; rtDW . kxuk4bilk3 . DataPtr =
fromwksInfo -> data ; rtDW . kxuk4bilk3 . RSimInfoPtr = fromwksInfo ; } rtDW
. l2pbteie2e . PrevIndex = 0 ; { int_T * zcTimeIndices = & rtDW . fth52qijjp
[ 0 ] ; const double * timePoints = ( double * ) rtDW . kxuk4bilk3 . TimePtr
; boolean_T justHadZcTime = false ; int_T zcIdx = 0 ; int_T i ; for ( i = 0 ;
i < 6 - 1 ; i ++ ) { if ( ! justHadZcTime && timePoints [ i ] == timePoints [
i + 1 ] ) { zcTimeIndices [ zcIdx ++ ] = i ; justHadZcTime = true ; } else {
justHadZcTime = false ; } } rtDW . j5kfgeryef = 0 ; } } { FWksInfo *
fromwksInfo ; if ( ( fromwksInfo = ( FWksInfo * ) calloc ( 1 , sizeof (
FWksInfo ) ) ) == ( NULL ) ) { ssSetErrorStatus ( rtS ,
"from workspace STRING(Name) memory allocation error" ) ; } else {
fromwksInfo -> origWorkspaceVarName = "tuvar" ; fromwksInfo -> origDataTypeId
= 0 ; fromwksInfo -> origIsComplex = 0 ; fromwksInfo -> origWidth = 1 ;
fromwksInfo -> origElSize = sizeof ( real_T ) ; fromwksInfo -> data = ( void
* ) rtP . FromWs_Data0_bui33k5vfq ; fromwksInfo -> nDataPoints = 6 ;
fromwksInfo -> time = ( double * ) rtP . FromWs_Time0_ni0stwjpd1 ; rtDW .
jmmcgd4r4a . TimePtr = fromwksInfo -> time ; rtDW . jmmcgd4r4a . DataPtr =
fromwksInfo -> data ; rtDW . jmmcgd4r4a . RSimInfoPtr = fromwksInfo ; } rtDW
. agvnfrpxdp . PrevIndex = 0 ; { int_T * zcTimeIndices = & rtDW . pshgckdv2q
[ 0 ] ; const double * timePoints = ( double * ) rtDW . jmmcgd4r4a . TimePtr
; boolean_T justHadZcTime = false ; int_T zcIdx = 0 ; int_T i ; for ( i = 0 ;
i < 6 - 1 ; i ++ ) { if ( ! justHadZcTime && timePoints [ i ] == timePoints [
i + 1 ] ) { zcTimeIndices [ zcIdx ++ ] = i ; justHadZcTime = true ; } else {
justHadZcTime = false ; } } rtDW . nm2quutkdt = 0 ; } } { FWksInfo *
fromwksInfo ; if ( ( fromwksInfo = ( FWksInfo * ) calloc ( 1 , sizeof (
FWksInfo ) ) ) == ( NULL ) ) { ssSetErrorStatus ( rtS ,
"from workspace STRING(Name) memory allocation error" ) ; } else {
fromwksInfo -> origWorkspaceVarName = "tuvar" ; fromwksInfo -> origDataTypeId
= 0 ; fromwksInfo -> origIsComplex = 0 ; fromwksInfo -> origWidth = 1 ;
fromwksInfo -> origElSize = sizeof ( real_T ) ; fromwksInfo -> data = ( void
* ) rtP . FromWs_Data0_cjzjycivf4 ; fromwksInfo -> nDataPoints = 10 ;
fromwksInfo -> time = ( double * ) rtP . FromWs_Time0_ovshumjb0b ; rtDW .
mzmo2dtgp1 . TimePtr = fromwksInfo -> time ; rtDW . mzmo2dtgp1 . DataPtr =
fromwksInfo -> data ; rtDW . mzmo2dtgp1 . RSimInfoPtr = fromwksInfo ; } rtDW
. n0unfq1ojc . PrevIndex = 0 ; { int_T * zcTimeIndices = & rtDW . mx32usueux
[ 0 ] ; const double * timePoints = ( double * ) rtDW . mzmo2dtgp1 . TimePtr
; boolean_T justHadZcTime = false ; int_T zcIdx = 0 ; int_T i ; for ( i = 0 ;
i < 10 - 1 ; i ++ ) { if ( ! justHadZcTime && timePoints [ i ] == timePoints
[ i + 1 ] ) { zcTimeIndices [ zcIdx ++ ] = i ; justHadZcTime = true ; } else
{ justHadZcTime = false ; } } rtDW . i3sszjlhmx = 0 ; } } MdlInitialize ( ) ;
{ bool externalInputIsInDatasetFormat = false ; void * pISigstreamManager =
rt_GetISigstreamManager ( ) ; rtwISigstreamManagerGetInputIsInDatasetFormat (
pISigstreamManager , & externalInputIsInDatasetFormat ) ; if (
externalInputIsInDatasetFormat ) { } } } void MdlOutputs ( int_T tid ) {
real_T aeaum0g5tb ; real_T axvel1edh0 ; int_T iy ; int_T ci ; real_T
amxh1muxiy [ 4 ] ; static const int8_T jc [ 112 ] = { 0 , 1 , 2 , 3 , 4 , 5 ,
6 , 7 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 21 ,
22 , 23 , 24 , 25 , 26 , 27 , 0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 9 , 10 , 11
, 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 21 , 22 , 23 , 24 , 25 , 26 ,
27 , 0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 , 16
, 17 , 18 , 19 , 20 , 21 , 22 , 23 , 24 , 25 , 26 , 27 , 0 , 1 , 2 , 3 , 4 ,
5 , 6 , 7 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 21
, 22 , 23 , 24 , 25 , 26 , 27 } ; real_T tmp ; rtB . mikmmv5pjy = 0.0 ; rtB .
mikmmv5pjy += rtP . Ps1_C [ 0 ] * rtX . ltfask1ptj [ 0 ] ; rtB . mikmmv5pjy
+= rtP . Ps1_C [ 1 ] * rtX . ltfask1ptj [ 1 ] ; rtB . iglrdpeo0u = 0.0 ; rtB
. iglrdpeo0u += rtP . Ps1_C_a4z2vpeibb [ 0 ] * rtX . k5wabiqsuk [ 0 ] ; rtB .
iglrdpeo0u += rtP . Ps1_C_a4z2vpeibb [ 1 ] * rtX . k5wabiqsuk [ 1 ] ; for (
iy = 0 ; iy < 4 ; iy ++ ) { amxh1muxiy [ iy ] = 0.0 ; for ( ci = 0 ; ci < 8 ;
ci ++ ) { amxh1muxiy [ iy ] += rtP . C0 [ ( ci << 2 ) + iy ] * rtX .
b11juaaqeh [ ci ] ; } } rtB . cuhucxnwbl [ 0 ] = rtB . mikmmv5pjy +
amxh1muxiy [ 0 ] ; rtB . cuhucxnwbl [ 1 ] = rtB . iglrdpeo0u + amxh1muxiy [ 1
] ; rtB . cuhucxnwbl [ 2 ] = rtB . iglrdpeo0u + amxh1muxiy [ 2 ] ; rtB .
cuhucxnwbl [ 3 ] = rtB . iglrdpeo0u + amxh1muxiy [ 3 ] ; rtB . ndvxwhqhzh =
0.0 ; rtB . ndvxwhqhzh += rtP . Ps4_C [ 0 ] * rtX . oacfijplwi [ 0 ] ; rtB .
ndvxwhqhzh += rtP . Ps4_C [ 1 ] * rtX . oacfijplwi [ 1 ] ; rtB . lywwonuzni =
0.0 ; rtB . lywwonuzni += rtP . Ps5_C [ 0 ] * rtX . njhzavtx5o [ 0 ] ; rtB .
lywwonuzni += rtP . Ps5_C [ 1 ] * rtX . njhzavtx5o [ 1 ] ; rtB . cgqon1bp5k =
0.0 ; rtB . cgqon1bp5k += rtP . Ps1_C_jxo2syryaa [ 0 ] * rtX . hudxhgdxi2 [ 0
] ; rtB . cgqon1bp5k += rtP . Ps1_C_jxo2syryaa [ 1 ] * rtX . hudxhgdxi2 [ 1 ]
; rtB . ccnrqukp0p [ 0 ] = rtB . cuhucxnwbl [ 0 ] ; rtB . ccnrqukp0p [ 1 ] =
rtB . cgqon1bp5k ; { double locTime = ssGetTaskTime ( rtS , 0 ) ; ; if (
ssGetLogOutput ( rtS ) ) { { double locTime = ssGetTaskTime ( rtS , 0 ) ; ;
if ( rtwTimeInLoggingInterval ( rtliGetLoggingInterval ( ssGetRootSS ( rtS )
-> mdlInfo -> rtwLogInfo ) , locTime ) ) { rt_UpdateStructLogVar ( (
StructLogVar * ) rtDW . hwrxgbpfob . LoggedData , & locTime , & rtB .
ccnrqukp0p [ 0 ] ) ; } } } } if ( rtP . ManualSwitch1_CurrentSetting == 1 ) {
tmp = rtP . Constant1_Value ; } else { tmp = rtP . Ps1_C_gfroigzrf4 * rtX .
l2dleorvkm ; } rtB . o2ra02ecjo = rtP . Gain_Gain * tmp ; rtB . g3cxppgcad [
0 ] = rtB . cgqon1bp5k - ( rtB . cuhucxnwbl [ 0 ] + rtB . o2ra02ecjo ) ; rtB
. g3cxppgcad [ 1 ] = rtB . ndvxwhqhzh - ( rtB . cuhucxnwbl [ 1 ] + rtB .
o2ra02ecjo ) ; rtB . g3cxppgcad [ 2 ] = rtB . lywwonuzni - ( rtB . cuhucxnwbl
[ 2 ] + rtB . o2ra02ecjo ) ; rtB . g3cxppgcad [ 3 ] = rtB . kohnzyllz4 - (
rtB . cuhucxnwbl [ 3 ] + rtB . o2ra02ecjo ) ; if ( ssGetLogOutput ( rtS ) ) {
{ double locTime = ssGetTaskTime ( rtS , 0 ) ; ; if (
rtwTimeInLoggingInterval ( rtliGetLoggingInterval ( ssGetRootSS ( rtS ) ->
mdlInfo -> rtwLogInfo ) , locTime ) ) { rt_UpdateStructLogVar ( (
StructLogVar * ) rtDW . nnbpbghk5r . LoggedData , ( NULL ) , & rtB .
g3cxppgcad [ 0 ] ) ; } } } amxh1muxiy [ 0 ] = 0.0 ; for ( ci = 0 ; ci < 28 ;
ci ++ ) { amxh1muxiy [ 0 ] += rtP . HInfinity_C [ ci ] * rtX . phhupngpjs [
jc [ ci ] ] ; } amxh1muxiy [ 1 ] = 0.0 ; while ( ci < 56 ) { amxh1muxiy [ 1 ]
+= rtP . HInfinity_C [ ci ] * rtX . phhupngpjs [ jc [ ci ] ] ; ci ++ ; }
amxh1muxiy [ 2 ] = 0.0 ; while ( ci < 84 ) { amxh1muxiy [ 2 ] += rtP .
HInfinity_C [ ci ] * rtX . phhupngpjs [ jc [ ci ] ] ; ci ++ ; } amxh1muxiy [
3 ] = 0.0 ; while ( ci < 112 ) { amxh1muxiy [ 3 ] += rtP . HInfinity_C [ ci ]
* rtX . phhupngpjs [ jc [ ci ] ] ; ci ++ ; } rtB . nq0m4x3haw = 0.0 ; rtB .
nq0m4x3haw += rtP . Ps1_C_iunwv5jncy [ 0 ] * rtX . inruainrus [ 0 ] ; rtB .
nq0m4x3haw += rtP . Ps1_C_iunwv5jncy [ 1 ] * rtX . inruainrus [ 1 ] ; rtB .
ffcuwotdjl = ssGetT ( rtS ) ; rtB . h0k2fzanuq = ssGetT ( rtS ) ; if (
ssIsMajorTimeStep ( rtS ) ) { rtDW . nx03g0oqvx = ( rtB . ffcuwotdjl >= rtP .
Switch1_Threshold ) ; rtDW . k2y005jslo = ( rtB . h0k2fzanuq >= rtP .
Switch2_Threshold ) ; } if ( rtDW . nx03g0oqvx ) { rtB . nue240oiht = (
muDoubleScalarSin ( rtP . omegaPertOut * ssGetTaskTime ( rtS , 0 ) + rtP .
seno_Phase ) * rtP . seno_Amp + rtP . seno_Bias ) + rtP . cstPertOut ; rtB .
l3ibipirjv = rtB . nue240oiht ; } else { rtB . l3ibipirjv = rtP .
Constant1_Value_onxqoxz4b2 ; } if ( rtDW . k2y005jslo ) { rtB . o4bvr14r1c =
rtP . Constant2_Value_honrvu2u0e ; } else { rtB . o4bvr14r1c = rtB .
l3ibipirjv ; } rtB . iklusr35kc = ssGetT ( rtS ) ; rtB . elhmsr12bw = ssGetT
( rtS ) ; if ( ssIsMajorTimeStep ( rtS ) ) { rtDW . lsikoznc1f = ( rtB .
iklusr35kc >= rtP . Switch_Threshold ) ; rtDW . pvf1npkacw = ( rtB .
elhmsr12bw >= rtP . Switch1_Threshold_a0fpu4ohya ) ; } if ( rtDW . lsikoznc1f
) { rtB . l5s1awmxqx = ( muDoubleScalarSin ( rtP . omegaPertOut *
ssGetTaskTime ( rtS , 0 ) + rtP . SineOut_Phase ) * rtP . amplitudePertOutptp
+ rtP . SineOut_Bias ) + rtP . cstPertOut ; rtB . epfm0vmgr0 = rtB .
l5s1awmxqx ; } else { rtB . epfm0vmgr0 = rtP . Constant_Value ; } if ( rtDW .
pvf1npkacw ) { rtB . dx0ibnxabm = rtP . Constant2_Value_maeni3njql ; } else {
rtB . dx0ibnxabm = rtB . epfm0vmgr0 ; } rtB . ahq3gaiq3g = ssGetT ( rtS ) ;
if ( ssIsMajorTimeStep ( rtS ) ) { rtDW . p424nko4vw = ( rtB . ahq3gaiq3g >=
rtP . Switch_Threshold_je2vilxnqj ) ; } if ( rtDW . p424nko4vw ) { rtB .
c33unhaobk = ( muDoubleScalarSin ( rtP . omegaPertOut * ssGetTaskTime ( rtS ,
0 ) + rtP . SineOut_Phase_ek4wf0nzh3 ) * rtP . amplitudePertOutZ + rtP .
SineOut_Bias_amml0uhsh5 ) + rtP . cstPertOut ; rtB . jfilnvyqtk = rtB .
c33unhaobk ; } else { rtB . jfilnvyqtk = rtP . Constant_Value_gsg5wrsjtq ; }
if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { rtB . bnmysexqkh = rtDW . ox0fllt2el ;
} rtB . afiufrufpa = ( muDoubleScalarSin ( rtP . omegaNoise * ssGetTaskTime (
rtS , 0 ) + rtP . u2sen05t_Phase ) * rtP . amplitudeNoise + rtP .
u2sen05t_Bias ) + rtB . bnmysexqkh ; { real_T * pDataValues = ( real_T * )
rtDW . lmen2irgnh . DataPtr ; real_T * pTimeValues = ( real_T * ) rtDW .
lmen2irgnh . TimePtr ; int_T currTimeIndex = rtDW . oosdo3rh02 . PrevIndex ;
real_T t = ssGetTaskTime ( rtS , 0 ) ; int numPoints , lastPoint ; FWksInfo *
fromwksInfo = ( FWksInfo * ) rtDW . lmen2irgnh . RSimInfoPtr ; numPoints =
fromwksInfo -> nDataPoints ; lastPoint = numPoints - 1 ; if ( t <=
pTimeValues [ 0 ] ) { currTimeIndex = 0 ; } else if ( t >= pTimeValues [
lastPoint ] ) { currTimeIndex = lastPoint - 1 ; } else { if ( t < pTimeValues
[ currTimeIndex ] ) { while ( t < pTimeValues [ currTimeIndex ] ) {
currTimeIndex -- ; } } else { while ( t >= pTimeValues [ currTimeIndex + 1 ]
) { currTimeIndex ++ ; } } } rtDW . oosdo3rh02 . PrevIndex = currTimeIndex ;
{ real_T t1 = pTimeValues [ currTimeIndex ] ; real_T t2 = pTimeValues [
currTimeIndex + 1 ] ; if ( t1 == t2 ) { if ( t < t1 ) { aeaum0g5tb =
pDataValues [ currTimeIndex ] ; } else { aeaum0g5tb = pDataValues [
currTimeIndex + 1 ] ; } } else { real_T f1 = ( t2 - t ) / ( t2 - t1 ) ;
real_T f2 = 1.0 - f1 ; real_T d1 ; real_T d2 ; int_T TimeIndex =
currTimeIndex ; int_T * zcTimeIndices = & rtDW . ihm14vuvou [ 0 ] ; int_T *
zcTimeIndicesIdx = & rtDW . mizp5pcuu3 ; int_T zcIdx = zcTimeIndices [ *
zcTimeIndicesIdx ] ; int_T numZcTimes = 2 ; if ( * zcTimeIndicesIdx <
numZcTimes ) { if ( ssIsMajorTimeStep ( rtS ) ) { if ( t > pTimeValues [
zcIdx ] ) { while ( * zcTimeIndicesIdx < ( numZcTimes - 1 ) && ( t >
pTimeValues [ zcIdx ] ) ) { ( * zcTimeIndicesIdx ) ++ ; zcIdx = zcTimeIndices
[ * zcTimeIndicesIdx ] ; } } } else { if ( t >= pTimeValues [ zcIdx ] && (
ssGetTimeOfLastOutput ( rtS ) < pTimeValues [ zcIdx ] ) ) { t2 = pTimeValues
[ zcIdx ] ; if ( zcIdx == 0 ) { TimeIndex = 0 ; t1 = t2 - 1 ; } else { t1 =
pTimeValues [ zcIdx - 1 ] ; TimeIndex = zcIdx - 1 ; } f1 = ( t2 - t ) / ( t2
- t1 ) ; f2 = 1.0 - f1 ; } } } d1 = pDataValues [ TimeIndex ] ; d2 =
pDataValues [ TimeIndex + 1 ] ; if ( zcIdx == 0 ) { d2 = d1 ; } aeaum0g5tb =
( real_T ) rtInterpolate ( d1 , d2 , f1 , f2 ) ; pDataValues += numPoints ; }
} } { real_T * pDataValues = ( real_T * ) rtDW . kxuk4bilk3 . DataPtr ;
real_T * pTimeValues = ( real_T * ) rtDW . kxuk4bilk3 . TimePtr ; int_T
currTimeIndex = rtDW . l2pbteie2e . PrevIndex ; real_T t = ssGetTaskTime (
rtS , 0 ) ; int numPoints , lastPoint ; FWksInfo * fromwksInfo = ( FWksInfo *
) rtDW . kxuk4bilk3 . RSimInfoPtr ; numPoints = fromwksInfo -> nDataPoints ;
lastPoint = numPoints - 1 ; if ( t <= pTimeValues [ 0 ] ) { currTimeIndex = 0
; } else if ( t >= pTimeValues [ lastPoint ] ) { currTimeIndex = lastPoint -
1 ; } else { if ( t < pTimeValues [ currTimeIndex ] ) { while ( t <
pTimeValues [ currTimeIndex ] ) { currTimeIndex -- ; } } else { while ( t >=
pTimeValues [ currTimeIndex + 1 ] ) { currTimeIndex ++ ; } } } rtDW .
l2pbteie2e . PrevIndex = currTimeIndex ; { real_T t1 = pTimeValues [
currTimeIndex ] ; real_T t2 = pTimeValues [ currTimeIndex + 1 ] ; if ( t1 ==
t2 ) { if ( t < t1 ) { axvel1edh0 = pDataValues [ currTimeIndex ] ; } else {
axvel1edh0 = pDataValues [ currTimeIndex + 1 ] ; } } else { real_T f1 = ( t2
- t ) / ( t2 - t1 ) ; real_T f2 = 1.0 - f1 ; real_T d1 ; real_T d2 ; int_T
TimeIndex = currTimeIndex ; int_T * zcTimeIndices = & rtDW . fth52qijjp [ 0 ]
; int_T * zcTimeIndicesIdx = & rtDW . j5kfgeryef ; int_T zcIdx =
zcTimeIndices [ * zcTimeIndicesIdx ] ; int_T numZcTimes = 2 ; if ( *
zcTimeIndicesIdx < numZcTimes ) { if ( ssIsMajorTimeStep ( rtS ) ) { if ( t >
pTimeValues [ zcIdx ] ) { while ( * zcTimeIndicesIdx < ( numZcTimes - 1 ) &&
( t > pTimeValues [ zcIdx ] ) ) { ( * zcTimeIndicesIdx ) ++ ; zcIdx =
zcTimeIndices [ * zcTimeIndicesIdx ] ; } } } else { if ( t >= pTimeValues [
zcIdx ] && ( ssGetTimeOfLastOutput ( rtS ) < pTimeValues [ zcIdx ] ) ) { t2 =
pTimeValues [ zcIdx ] ; if ( zcIdx == 0 ) { TimeIndex = 0 ; t1 = t2 - 1 ; }
else { t1 = pTimeValues [ zcIdx - 1 ] ; TimeIndex = zcIdx - 1 ; } f1 = ( t2 -
t ) / ( t2 - t1 ) ; f2 = 1.0 - f1 ; } } } d1 = pDataValues [ TimeIndex ] ; d2
= pDataValues [ TimeIndex + 1 ] ; if ( zcIdx == 0 ) { d2 = d1 ; } axvel1edh0
= ( real_T ) rtInterpolate ( d1 , d2 , f1 , f2 ) ; pDataValues += numPoints ;
} } } rtB . eivqzdcosm = aeaum0g5tb + axvel1edh0 ; { real_T * pDataValues = (
real_T * ) rtDW . jmmcgd4r4a . DataPtr ; real_T * pTimeValues = ( real_T * )
rtDW . jmmcgd4r4a . TimePtr ; int_T currTimeIndex = rtDW . agvnfrpxdp .
PrevIndex ; real_T t = ssGetTaskTime ( rtS , 0 ) ; int numPoints , lastPoint
; FWksInfo * fromwksInfo = ( FWksInfo * ) rtDW . jmmcgd4r4a . RSimInfoPtr ;
numPoints = fromwksInfo -> nDataPoints ; lastPoint = numPoints - 1 ; if ( t
<= pTimeValues [ 0 ] ) { currTimeIndex = 0 ; } else if ( t >= pTimeValues [
lastPoint ] ) { currTimeIndex = lastPoint - 1 ; } else { if ( t < pTimeValues
[ currTimeIndex ] ) { while ( t < pTimeValues [ currTimeIndex ] ) {
currTimeIndex -- ; } } else { while ( t >= pTimeValues [ currTimeIndex + 1 ]
) { currTimeIndex ++ ; } } } rtDW . agvnfrpxdp . PrevIndex = currTimeIndex ;
{ real_T t1 = pTimeValues [ currTimeIndex ] ; real_T t2 = pTimeValues [
currTimeIndex + 1 ] ; if ( t1 == t2 ) { if ( t < t1 ) { rtB . bnmtoyr0ou =
pDataValues [ currTimeIndex ] ; } else { rtB . bnmtoyr0ou = pDataValues [
currTimeIndex + 1 ] ; } } else { real_T f1 = ( t2 - t ) / ( t2 - t1 ) ;
real_T f2 = 1.0 - f1 ; real_T d1 ; real_T d2 ; int_T TimeIndex =
currTimeIndex ; int_T * zcTimeIndices = & rtDW . pshgckdv2q [ 0 ] ; int_T *
zcTimeIndicesIdx = & rtDW . nm2quutkdt ; int_T zcIdx = zcTimeIndices [ *
zcTimeIndicesIdx ] ; int_T numZcTimes = 2 ; if ( * zcTimeIndicesIdx <
numZcTimes ) { if ( ssIsMajorTimeStep ( rtS ) ) { if ( t > pTimeValues [
zcIdx ] ) { while ( * zcTimeIndicesIdx < ( numZcTimes - 1 ) && ( t >
pTimeValues [ zcIdx ] ) ) { ( * zcTimeIndicesIdx ) ++ ; zcIdx = zcTimeIndices
[ * zcTimeIndicesIdx ] ; } } } else { if ( t >= pTimeValues [ zcIdx ] && (
ssGetTimeOfLastOutput ( rtS ) < pTimeValues [ zcIdx ] ) ) { t2 = pTimeValues
[ zcIdx ] ; if ( zcIdx == 0 ) { TimeIndex = 0 ; t1 = t2 - 1 ; } else { t1 =
pTimeValues [ zcIdx - 1 ] ; TimeIndex = zcIdx - 1 ; } f1 = ( t2 - t ) / ( t2
- t1 ) ; f2 = 1.0 - f1 ; } } } d1 = pDataValues [ TimeIndex ] ; d2 =
pDataValues [ TimeIndex + 1 ] ; if ( zcIdx == 0 ) { d2 = d1 ; } rtB .
bnmtoyr0ou = ( real_T ) rtInterpolate ( d1 , d2 , f1 , f2 ) ; pDataValues +=
numPoints ; } } } { real_T * pDataValues = ( real_T * ) rtDW . mzmo2dtgp1 .
DataPtr ; real_T * pTimeValues = ( real_T * ) rtDW . mzmo2dtgp1 . TimePtr ;
int_T currTimeIndex = rtDW . n0unfq1ojc . PrevIndex ; real_T t =
ssGetTaskTime ( rtS , 0 ) ; int numPoints , lastPoint ; FWksInfo *
fromwksInfo = ( FWksInfo * ) rtDW . mzmo2dtgp1 . RSimInfoPtr ; numPoints =
fromwksInfo -> nDataPoints ; lastPoint = numPoints - 1 ; if ( t <=
pTimeValues [ 0 ] ) { currTimeIndex = 0 ; } else if ( t >= pTimeValues [
lastPoint ] ) { currTimeIndex = lastPoint - 1 ; } else { if ( t < pTimeValues
[ currTimeIndex ] ) { while ( t < pTimeValues [ currTimeIndex ] ) {
currTimeIndex -- ; } } else { while ( t >= pTimeValues [ currTimeIndex + 1 ]
) { currTimeIndex ++ ; } } } rtDW . n0unfq1ojc . PrevIndex = currTimeIndex ;
{ real_T t1 = pTimeValues [ currTimeIndex ] ; real_T t2 = pTimeValues [
currTimeIndex + 1 ] ; if ( t1 == t2 ) { if ( t < t1 ) { rtB . ji3tio223s =
pDataValues [ currTimeIndex ] ; } else { rtB . ji3tio223s = pDataValues [
currTimeIndex + 1 ] ; } } else { real_T f1 = ( t2 - t ) / ( t2 - t1 ) ;
real_T f2 = 1.0 - f1 ; real_T d1 ; real_T d2 ; int_T TimeIndex =
currTimeIndex ; int_T * zcTimeIndices = & rtDW . mx32usueux [ 0 ] ; int_T *
zcTimeIndicesIdx = & rtDW . i3sszjlhmx ; int_T zcIdx = zcTimeIndices [ *
zcTimeIndicesIdx ] ; int_T numZcTimes = 4 ; if ( * zcTimeIndicesIdx <
numZcTimes ) { if ( ssIsMajorTimeStep ( rtS ) ) { if ( t > pTimeValues [
zcIdx ] ) { while ( * zcTimeIndicesIdx < ( numZcTimes - 1 ) && ( t >
pTimeValues [ zcIdx ] ) ) { ( * zcTimeIndicesIdx ) ++ ; zcIdx = zcTimeIndices
[ * zcTimeIndicesIdx ] ; } } } else { if ( t >= pTimeValues [ zcIdx ] && (
ssGetTimeOfLastOutput ( rtS ) < pTimeValues [ zcIdx ] ) ) { t2 = pTimeValues
[ zcIdx ] ; if ( zcIdx == 0 ) { TimeIndex = 0 ; t1 = t2 - 1 ; } else { t1 =
pTimeValues [ zcIdx - 1 ] ; TimeIndex = zcIdx - 1 ; } f1 = ( t2 - t ) / ( t2
- t1 ) ; f2 = 1.0 - f1 ; } } } d1 = pDataValues [ TimeIndex ] ; d2 =
pDataValues [ TimeIndex + 1 ] ; if ( zcIdx == 0 ) { d2 = d1 ; } rtB .
ji3tio223s = ( real_T ) rtInterpolate ( d1 , d2 , f1 , f2 ) ; pDataValues +=
numPoints ; } } } rtB . jf1eral2ve [ 0 ] = rtB . nq0m4x3haw + amxh1muxiy [ 0
] ; rtB . jf1eral2ve [ 1 ] = rtB . nq0m4x3haw + amxh1muxiy [ 1 ] ; rtB .
jf1eral2ve [ 2 ] = rtB . nq0m4x3haw + amxh1muxiy [ 2 ] ; rtB . jf1eral2ve [ 3
] = rtB . nq0m4x3haw + amxh1muxiy [ 3 ] ; if ( ssIsMajorTimeStep ( rtS ) ) {
rtDW . dbgdqimhbs [ 0 ] = rtB . jf1eral2ve [ 0 ] >= rtP .
Saturation1_UpperSat ? 1 : rtB . jf1eral2ve [ 0 ] > rtP .
Saturation1_LowerSat ? 0 : - 1 ; rtDW . dbgdqimhbs [ 1 ] = rtB . jf1eral2ve [
1 ] >= rtP . Saturation1_UpperSat ? 1 : rtB . jf1eral2ve [ 1 ] > rtP .
Saturation1_LowerSat ? 0 : - 1 ; rtDW . dbgdqimhbs [ 2 ] = rtB . jf1eral2ve [
2 ] >= rtP . Saturation1_UpperSat ? 1 : rtB . jf1eral2ve [ 2 ] > rtP .
Saturation1_LowerSat ? 0 : - 1 ; rtDW . dbgdqimhbs [ 3 ] = rtB . jf1eral2ve [
3 ] >= rtP . Saturation1_UpperSat ? 1 : rtB . jf1eral2ve [ 3 ] > rtP .
Saturation1_LowerSat ? 0 : - 1 ; } rtB . ngbxrb2p33 [ 0 ] = rtDW . dbgdqimhbs
[ 0 ] == 1 ? rtP . Saturation1_UpperSat : rtDW . dbgdqimhbs [ 0 ] == - 1 ?
rtP . Saturation1_LowerSat : rtB . jf1eral2ve [ 0 ] ; rtB . ngbxrb2p33 [ 1 ]
= rtDW . dbgdqimhbs [ 1 ] == 1 ? rtP . Saturation1_UpperSat : rtDW .
dbgdqimhbs [ 1 ] == - 1 ? rtP . Saturation1_LowerSat : rtB . jf1eral2ve [ 1 ]
; rtB . ngbxrb2p33 [ 2 ] = rtDW . dbgdqimhbs [ 2 ] == 1 ? rtP .
Saturation1_UpperSat : rtDW . dbgdqimhbs [ 2 ] == - 1 ? rtP .
Saturation1_LowerSat : rtB . jf1eral2ve [ 2 ] ; rtB . ngbxrb2p33 [ 3 ] = rtDW
. dbgdqimhbs [ 3 ] == 1 ? rtP . Saturation1_UpperSat : rtDW . dbgdqimhbs [ 3
] == - 1 ? rtP . Saturation1_LowerSat : rtB . jf1eral2ve [ 3 ] ;
UNUSED_PARAMETER ( tid ) ; } void MdlOutputsTID2 ( int_T tid ) { rtB .
kohnzyllz4 = rtP . Constant2_Value ; UNUSED_PARAMETER ( tid ) ; } void
MdlUpdate ( int_T tid ) { if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { rtDW .
ox0fllt2el = ( rtP . randomAmpNoise - ( - rtP . randomAmpNoise ) ) *
rt_urand_Upu32_Yd_f_pw_snf ( & rtDW . ic2yhbr32w ) + - rtP . randomAmpNoise ;
} UNUSED_PARAMETER ( tid ) ; } void MdlUpdateTID2 ( int_T tid ) {
UNUSED_PARAMETER ( tid ) ; } void MdlDerivatives ( void ) { int_T is ; int_T
ri ; static const int16_T ir [ 29 ] = { 0 , 21 , 42 , 63 , 84 , 105 , 126 ,
147 , 168 , 188 , 208 , 228 , 248 , 268 , 288 , 308 , 328 , 348 , 368 , 388 ,
408 , 429 , 457 , 478 , 499 , 520 , 548 , 576 , 604 } ; static const int8_T
jc [ 604 ] = { 0 , 1 , 2 , 4 , 6 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 , 16 ,
17 , 18 , 19 , 20 , 22 , 23 , 24 , 0 , 1 , 2 , 4 , 6 , 8 , 9 , 10 , 11 , 12 ,
13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 22 , 23 , 24 , 0 , 2 , 3 , 4 , 6 , 8
, 9 , 10 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 22 , 23 , 24 , 0
, 2 , 3 , 4 , 6 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 ,
20 , 22 , 23 , 24 , 0 , 2 , 4 , 5 , 6 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 ,
16 , 17 , 18 , 19 , 20 , 22 , 23 , 24 , 0 , 2 , 4 , 5 , 6 , 8 , 9 , 10 , 11 ,
12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 22 , 23 , 24 , 0 , 2 , 4 , 6 , 7
, 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 22 , 23 , 24
, 0 , 2 , 4 , 6 , 7 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19
, 20 , 22 , 23 , 24 , 0 , 2 , 4 , 6 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 ,
16 , 17 , 18 , 19 , 20 , 22 , 23 , 24 , 0 , 2 , 4 , 6 , 8 , 9 , 10 , 11 , 12
, 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 22 , 23 , 24 , 0 , 2 , 4 , 6 , 8 ,
9 , 10 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 22 , 23 , 24 , 0 ,
2 , 4 , 6 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 22
, 23 , 24 , 0 , 2 , 4 , 6 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 , 16 , 17 ,
18 , 19 , 20 , 22 , 23 , 24 , 0 , 2 , 4 , 6 , 8 , 9 , 10 , 11 , 12 , 13 , 14
, 15 , 16 , 17 , 18 , 19 , 20 , 22 , 23 , 24 , 0 , 2 , 4 , 6 , 8 , 9 , 10 ,
11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 22 , 23 , 24 , 0 , 2 , 4 ,
6 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 22 , 23 ,
24 , 0 , 2 , 4 , 6 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19
, 20 , 22 , 23 , 24 , 0 , 2 , 4 , 6 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 ,
16 , 17 , 18 , 19 , 20 , 22 , 23 , 24 , 0 , 2 , 4 , 6 , 8 , 9 , 10 , 11 , 12
, 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 22 , 23 , 24 , 0 , 2 , 4 , 6 , 8 ,
9 , 10 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 22 , 23 , 24 , 0 ,
2 , 4 , 6 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 21
, 22 , 23 , 24 , 0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 9 , 10 , 11 , 12 , 13 ,
14 , 15 , 16 , 17 , 18 , 19 , 20 , 21 , 22 , 23 , 24 , 25 , 26 , 27 , 0 , 2 ,
4 , 6 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 22 ,
23 , 24 , 25 , 0 , 2 , 4 , 6 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 , 16 , 17
, 18 , 19 , 20 , 22 , 23 , 24 , 26 , 0 , 2 , 4 , 6 , 8 , 9 , 10 , 11 , 12 ,
13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 22 , 23 , 24 , 27 , 0 , 1 , 2 , 3 , 4
, 5 , 6 , 7 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 ,
21 , 22 , 23 , 24 , 25 , 26 , 27 , 0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 9 , 10
, 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 21 , 22 , 23 , 24 , 25 ,
26 , 27 , 0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15
, 16 , 17 , 18 , 19 , 20 , 21 , 22 , 23 , 24 , 25 , 26 , 27 } ; static const
int8_T jc_p [ 112 ] = { 0 , 1 , 2 , 3 , 0 , 1 , 2 , 3 , 0 , 1 , 2 , 3 , 0 , 1
, 2 , 3 , 0 , 1 , 2 , 3 , 0 , 1 , 2 , 3 , 0 , 1 , 2 , 3 , 0 , 1 , 2 , 3 , 0 ,
1 , 2 , 3 , 0 , 1 , 2 , 3 , 0 , 1 , 2 , 3 , 0 , 1 , 2 , 3 , 0 , 1 , 2 , 3 , 0
, 1 , 2 , 3 , 0 , 1 , 2 , 3 , 0 , 1 , 2 , 3 , 0 , 1 , 2 , 3 , 0 , 1 , 2 , 3 ,
0 , 1 , 2 , 3 , 0 , 1 , 2 , 3 , 0 , 1 , 2 , 3 , 0 , 1 , 2 , 3 , 0 , 1 , 2 , 3
, 0 , 1 , 2 , 3 , 0 , 1 , 2 , 3 , 0 , 1 , 2 , 3 , 0 , 1 , 2 , 3 , 0 , 1 , 2 ,
3 } ; XDot * _rtXdot ; _rtXdot = ( ( XDot * ) ssGetdX ( rtS ) ) ; _rtXdot ->
ltfask1ptj [ 0 ] = 0.0 ; _rtXdot -> ltfask1ptj [ 0 ] += rtP . Ps1_A [ 0 ] *
rtX . ltfask1ptj [ 0 ] ; _rtXdot -> k5wabiqsuk [ 0 ] = 0.0 ; _rtXdot ->
k5wabiqsuk [ 0 ] += rtP . Ps1_A_j1blkzytxk [ 0 ] * rtX . k5wabiqsuk [ 0 ] ;
_rtXdot -> ltfask1ptj [ 1 ] = 0.0 ; _rtXdot -> ltfask1ptj [ 0 ] += rtP .
Ps1_A [ 1 ] * rtX . ltfask1ptj [ 1 ] ; _rtXdot -> k5wabiqsuk [ 1 ] = 0.0 ;
_rtXdot -> k5wabiqsuk [ 0 ] += rtP . Ps1_A_j1blkzytxk [ 1 ] * rtX .
k5wabiqsuk [ 1 ] ; _rtXdot -> ltfask1ptj [ 1 ] += rtX . ltfask1ptj [ 0 ] ;
_rtXdot -> ltfask1ptj [ 0 ] += rtB . jfilnvyqtk ; _rtXdot -> k5wabiqsuk [ 1 ]
+= rtX . k5wabiqsuk [ 0 ] ; _rtXdot -> k5wabiqsuk [ 0 ] += rtB . dx0ibnxabm ;
memset ( & _rtXdot -> b11juaaqeh [ 0 ] , 0 , sizeof ( real_T ) << 3U ) ; for
( ri = 0 ; ri < 8 ; ri ++ ) { for ( is = 0 ; is < 8 ; is ++ ) { _rtXdot ->
b11juaaqeh [ ri ] += rtP . A0 [ ( is << 3 ) + ri ] * rtX . b11juaaqeh [ is ]
; } _rtXdot -> b11juaaqeh [ ri ] += rtP . B0 [ ri ] * rtB . ngbxrb2p33 [ 0 ]
; _rtXdot -> b11juaaqeh [ ri ] += rtP . B0 [ 8 + ri ] * rtB . ngbxrb2p33 [ 1
] ; _rtXdot -> b11juaaqeh [ ri ] += rtP . B0 [ 16 + ri ] * rtB . ngbxrb2p33 [
2 ] ; _rtXdot -> b11juaaqeh [ ri ] += rtP . B0 [ 24 + ri ] * rtB . ngbxrb2p33
[ 3 ] ; } _rtXdot -> oacfijplwi [ 0 ] = 0.0 ; _rtXdot -> oacfijplwi [ 0 ] +=
rtP . Ps4_A [ 0 ] * rtX . oacfijplwi [ 0 ] ; _rtXdot -> njhzavtx5o [ 0 ] =
0.0 ; _rtXdot -> njhzavtx5o [ 0 ] += rtP . Ps5_A [ 0 ] * rtX . njhzavtx5o [ 0
] ; _rtXdot -> hudxhgdxi2 [ 0 ] = 0.0 ; _rtXdot -> hudxhgdxi2 [ 0 ] += rtP .
Ps1_A_g40xqitwcm [ 0 ] * rtX . hudxhgdxi2 [ 0 ] ; _rtXdot -> oacfijplwi [ 1 ]
= 0.0 ; _rtXdot -> oacfijplwi [ 0 ] += rtP . Ps4_A [ 1 ] * rtX . oacfijplwi [
1 ] ; _rtXdot -> njhzavtx5o [ 1 ] = 0.0 ; _rtXdot -> njhzavtx5o [ 0 ] += rtP
. Ps5_A [ 1 ] * rtX . njhzavtx5o [ 1 ] ; _rtXdot -> hudxhgdxi2 [ 1 ] = 0.0 ;
_rtXdot -> hudxhgdxi2 [ 0 ] += rtP . Ps1_A_g40xqitwcm [ 1 ] * rtX .
hudxhgdxi2 [ 1 ] ; _rtXdot -> oacfijplwi [ 1 ] += rtX . oacfijplwi [ 0 ] ;
_rtXdot -> oacfijplwi [ 0 ] += rtB . eivqzdcosm ; _rtXdot -> njhzavtx5o [ 1 ]
+= rtX . njhzavtx5o [ 0 ] ; _rtXdot -> njhzavtx5o [ 0 ] += rtB . bnmtoyr0ou ;
_rtXdot -> hudxhgdxi2 [ 1 ] += rtX . hudxhgdxi2 [ 0 ] ; _rtXdot -> hudxhgdxi2
[ 0 ] += rtB . ji3tio223s ; _rtXdot -> l2dleorvkm = 0.0 ; _rtXdot ->
l2dleorvkm += rtP . Ps1_A_cnuk0h4tar * rtX . l2dleorvkm ; _rtXdot ->
l2dleorvkm += rtB . afiufrufpa ; memset ( & _rtXdot -> phhupngpjs [ 0 ] , 0 ,
28U * sizeof ( real_T ) ) ; for ( ri = 0 ; ri < 28 ; ri ++ ) { for ( is = ir
[ ri ] ; is < ir [ ri + 1 ] ; is ++ ) { _rtXdot -> phhupngpjs [ ri ] += rtP .
HInfinity_A [ is ] * rtX . phhupngpjs [ jc [ is ] ] ; } } for ( ri = 0 ; ri <
28 ; ri ++ ) { for ( is = ri << 2 ; is < ( ( ri + 1 ) << 2 ) ; is ++ ) {
_rtXdot -> phhupngpjs [ ri ] += rtP . HInfinity_B [ is ] * rtB . g3cxppgcad [
jc_p [ is ] ] ; } } _rtXdot -> inruainrus [ 0 ] = 0.0 ; _rtXdot -> inruainrus
[ 0 ] += rtP . Ps1_A_p4fqdq0nyf [ 0 ] * rtX . inruainrus [ 0 ] ; _rtXdot ->
inruainrus [ 1 ] = 0.0 ; _rtXdot -> inruainrus [ 0 ] += rtP .
Ps1_A_p4fqdq0nyf [ 1 ] * rtX . inruainrus [ 1 ] ; _rtXdot -> inruainrus [ 1 ]
+= rtX . inruainrus [ 0 ] ; _rtXdot -> inruainrus [ 0 ] += rtB . o4bvr14r1c ;
} void MdlProjection ( void ) { } void MdlZeroCrossings ( void ) { ZCV *
_rtZCSV ; _rtZCSV = ( ( ZCV * ) ssGetSolverZcSignalVector ( rtS ) ) ; _rtZCSV
-> l3ghldah3m = rtB . ffcuwotdjl - rtP . Switch1_Threshold ; _rtZCSV ->
g4rwffgs43 = rtB . h0k2fzanuq - rtP . Switch2_Threshold ; _rtZCSV ->
fw4rhgfac4 = rtB . iklusr35kc - rtP . Switch_Threshold ; _rtZCSV ->
lefgzjm3hy = rtB . elhmsr12bw - rtP . Switch1_Threshold_a0fpu4ohya ; _rtZCSV
-> hfjunh22va = rtB . ahq3gaiq3g - rtP . Switch_Threshold_je2vilxnqj ; {
const double * timePtr = ( double * ) rtDW . lmen2irgnh . TimePtr ; int_T *
zcTimeIndices = & rtDW . ihm14vuvou [ 0 ] ; int_T zcTimeIndicesIdx = rtDW .
mizp5pcuu3 ; ( ( ZCV * ) ssGetSolverZcSignalVector ( rtS ) ) -> kc2m3kgcvp =
ssGetT ( rtS ) - timePtr [ zcTimeIndices [ zcTimeIndicesIdx ] ] ; } { const
double * timePtr = ( double * ) rtDW . kxuk4bilk3 . TimePtr ; int_T *
zcTimeIndices = & rtDW . fth52qijjp [ 0 ] ; int_T zcTimeIndicesIdx = rtDW .
j5kfgeryef ; ( ( ZCV * ) ssGetSolverZcSignalVector ( rtS ) ) -> m1bxttvtml =
ssGetT ( rtS ) - timePtr [ zcTimeIndices [ zcTimeIndicesIdx ] ] ; } { const
double * timePtr = ( double * ) rtDW . jmmcgd4r4a . TimePtr ; int_T *
zcTimeIndices = & rtDW . pshgckdv2q [ 0 ] ; int_T zcTimeIndicesIdx = rtDW .
nm2quutkdt ; ( ( ZCV * ) ssGetSolverZcSignalVector ( rtS ) ) -> msymzkqbvl =
ssGetT ( rtS ) - timePtr [ zcTimeIndices [ zcTimeIndicesIdx ] ] ; } { const
double * timePtr = ( double * ) rtDW . mzmo2dtgp1 . TimePtr ; int_T *
zcTimeIndices = & rtDW . mx32usueux [ 0 ] ; int_T zcTimeIndicesIdx = rtDW .
i3sszjlhmx ; ( ( ZCV * ) ssGetSolverZcSignalVector ( rtS ) ) -> donjsqodtq =
ssGetT ( rtS ) - timePtr [ zcTimeIndices [ zcTimeIndicesIdx ] ] ; } _rtZCSV
-> huw2sg4ptv [ 0 ] = rtB . jf1eral2ve [ 0 ] - rtP . Saturation1_UpperSat ;
_rtZCSV -> mykym5a1ll [ 0 ] = rtB . jf1eral2ve [ 0 ] - rtP .
Saturation1_LowerSat ; _rtZCSV -> huw2sg4ptv [ 1 ] = rtB . jf1eral2ve [ 1 ] -
rtP . Saturation1_UpperSat ; _rtZCSV -> mykym5a1ll [ 1 ] = rtB . jf1eral2ve [
1 ] - rtP . Saturation1_LowerSat ; _rtZCSV -> huw2sg4ptv [ 2 ] = rtB .
jf1eral2ve [ 2 ] - rtP . Saturation1_UpperSat ; _rtZCSV -> mykym5a1ll [ 2 ] =
rtB . jf1eral2ve [ 2 ] - rtP . Saturation1_LowerSat ; _rtZCSV -> huw2sg4ptv [
3 ] = rtB . jf1eral2ve [ 3 ] - rtP . Saturation1_UpperSat ; _rtZCSV ->
mykym5a1ll [ 3 ] = rtB . jf1eral2ve [ 3 ] - rtP . Saturation1_LowerSat ; }
void MdlTerminate ( void ) { rt_FREE ( rtDW . lmen2irgnh . RSimInfoPtr ) ;
rt_FREE ( rtDW . kxuk4bilk3 . RSimInfoPtr ) ; rt_FREE ( rtDW . jmmcgd4r4a .
RSimInfoPtr ) ; rt_FREE ( rtDW . mzmo2dtgp1 . RSimInfoPtr ) ; { if (
rt_slioCatalogue ( ) != ( NULL ) ) { void * * slioCatalogueAddr =
rt_slioCatalogueAddr ( ) ; rtwCreateSigstreamSlioClient (
rt_GetOSigstreamManager ( ) , rtwGetPointerFromUniquePtr ( rt_slioCatalogue (
) ) ) ; rtwSaveDatasetsToMatFile ( rtwGetPointerFromUniquePtr (
rt_slioCatalogue ( ) ) , rt_GetMatSigstreamLoggingFileName ( ) ) ;
rtwOSigstreamManagerDestroyInstance ( rt_GetOSigstreamManager ( ) ) ;
rtwTerminateSlioCatalogue ( slioCatalogueAddr ) ; * slioCatalogueAddr = (
NULL ) ; } } } void MdlInitializeSizes ( void ) { ssSetNumContStates ( rtS ,
49 ) ; ssSetNumPeriodicContStates ( rtS , 0 ) ; ssSetNumY ( rtS , 0 ) ;
ssSetNumU ( rtS , 0 ) ; ssSetDirectFeedThrough ( rtS , 0 ) ;
ssSetNumSampleTimes ( rtS , 2 ) ; ssSetNumBlocks ( rtS , 67 ) ;
ssSetNumBlockIO ( rtS , 31 ) ; ssSetNumBlockParams ( rtS , 1073 ) ; } void
MdlInitializeSampleTimes ( void ) { ssSetSampleTime ( rtS , 0 , 0.0 ) ;
ssSetSampleTime ( rtS , 1 , 0.1 ) ; ssSetOffsetTime ( rtS , 0 , 0.0 ) ;
ssSetOffsetTime ( rtS , 1 , 0.0 ) ; } void raccel_set_checksum ( SimStruct *
rtS ) { ssSetChecksumVal ( rtS , 0 , 2359241291U ) ; ssSetChecksumVal ( rtS ,
1 , 787542146U ) ; ssSetChecksumVal ( rtS , 2 , 808962859U ) ;
ssSetChecksumVal ( rtS , 3 , 974030338U ) ; } SimStruct *
raccel_register_model ( void ) { static struct _ssMdlInfo mdlInfo ; ( void )
memset ( ( char * ) rtS , 0 , sizeof ( SimStruct ) ) ; ( void ) memset ( (
char * ) & mdlInfo , 0 , sizeof ( struct _ssMdlInfo ) ) ; ssSetMdlInfoPtr (
rtS , & mdlInfo ) ; { static time_T mdlPeriod [ NSAMPLE_TIMES ] ; static
time_T mdlOffset [ NSAMPLE_TIMES ] ; static time_T mdlTaskTimes [
NSAMPLE_TIMES ] ; static int_T mdlTsMap [ NSAMPLE_TIMES ] ; static int_T
mdlSampleHits [ NSAMPLE_TIMES ] ; static boolean_T mdlTNextWasAdjustedPtr [
NSAMPLE_TIMES ] ; static int_T mdlPerTaskSampleHits [ NSAMPLE_TIMES *
NSAMPLE_TIMES ] ; static time_T mdlTimeOfNextSampleHit [ NSAMPLE_TIMES ] ; {
int_T i ; for ( i = 0 ; i < NSAMPLE_TIMES ; i ++ ) { mdlPeriod [ i ] = 0.0 ;
mdlOffset [ i ] = 0.0 ; mdlTaskTimes [ i ] = 0.0 ; mdlTsMap [ i ] = i ;
mdlSampleHits [ i ] = 1 ; } } ssSetSampleTimePtr ( rtS , & mdlPeriod [ 0 ] )
; ssSetOffsetTimePtr ( rtS , & mdlOffset [ 0 ] ) ; ssSetSampleTimeTaskIDPtr (
rtS , & mdlTsMap [ 0 ] ) ; ssSetTPtr ( rtS , & mdlTaskTimes [ 0 ] ) ;
ssSetSampleHitPtr ( rtS , & mdlSampleHits [ 0 ] ) ; ssSetTNextWasAdjustedPtr
( rtS , & mdlTNextWasAdjustedPtr [ 0 ] ) ; ssSetPerTaskSampleHitsPtr ( rtS ,
& mdlPerTaskSampleHits [ 0 ] ) ; ssSetTimeOfNextSampleHitPtr ( rtS , &
mdlTimeOfNextSampleHit [ 0 ] ) ; } ssSetSolverMode ( rtS ,
SOLVER_MODE_SINGLETASKING ) ; { ssSetBlockIO ( rtS , ( ( void * ) & rtB ) ) ;
( void ) memset ( ( ( void * ) & rtB ) , 0 , sizeof ( B ) ) ; }
ssSetDefaultParam ( rtS , ( real_T * ) & rtP ) ; { real_T * x = ( real_T * )
& rtX ; ssSetContStates ( rtS , x ) ; ( void ) memset ( ( void * ) x , 0 ,
sizeof ( X ) ) ; } { void * dwork = ( void * ) & rtDW ; ssSetRootDWork ( rtS
, dwork ) ; ( void ) memset ( dwork , 0 , sizeof ( DW ) ) ; } { static
DataTypeTransInfo dtInfo ; ( void ) memset ( ( char_T * ) & dtInfo , 0 ,
sizeof ( dtInfo ) ) ; ssSetModelMappingInfo ( rtS , & dtInfo ) ; dtInfo .
numDataTypes = 14 ; dtInfo . dataTypeSizes = & rtDataTypeSizes [ 0 ] ; dtInfo
. dataTypeNames = & rtDataTypeNames [ 0 ] ; dtInfo . BTransTable = &
rtBTransTable ; dtInfo . PTransTable = & rtPTransTable ; }
HinfTenzo_InitializeDataMapInfo ( ) ; ssSetIsRapidAcceleratorActive ( rtS ,
true ) ; ssSetRootSS ( rtS , rtS ) ; ssSetVersion ( rtS ,
SIMSTRUCT_VERSION_LEVEL2 ) ; ssSetModelName ( rtS , "HinfTenzo" ) ; ssSetPath
( rtS , "HinfTenzo" ) ; ssSetTStart ( rtS , 0.0 ) ; ssSetTFinal ( rtS , 15.0
) ; { static RTWLogInfo rt_DataLoggingInfo ; rt_DataLoggingInfo .
loggingInterval = NULL ; ssSetRTWLogInfo ( rtS , & rt_DataLoggingInfo ) ; } {
{ static int_T rt_LoggedStateWidths [ ] = { 2 , 2 , 8 , 2 , 2 , 2 , 1 , 28 ,
2 } ; static int_T rt_LoggedStateNumDimensions [ ] = { 1 , 1 , 1 , 1 , 1 , 1
, 1 , 1 , 1 } ; static int_T rt_LoggedStateDimensions [ ] = { 2 , 2 , 8 , 2 ,
2 , 2 , 1 , 28 , 2 } ; static boolean_T rt_LoggedStateIsVarDims [ ] = { 0 , 0
, 0 , 0 , 0 , 0 , 0 , 0 , 0 } ; static BuiltInDTypeId
rt_LoggedStateDataTypeIds [ ] = { SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE } ;
static int_T rt_LoggedStateComplexSignals [ ] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
0 , 0 } ; static const char_T * rt_LoggedStateLabels [ ] = { "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" } ; static const char_T * rt_LoggedStateBlockNames [ ] = {
"HinfTenzo/DisturboOut/ErrOutZ/P(s)1" ,
"HinfTenzo/DisturboOut/ErrOutAtt/P(s)1" , "HinfTenzo/processo/Processo1" ,
"HinfTenzo/Setpoints/P(s)4" , "HinfTenzo/Setpoints/P(s)5" ,
"HinfTenzo/Setpoints/P(s)1" ,
"HinfTenzo/Err Mes/Filtered random noise + sin/P(s)1" ,
"HinfTenzo/Controller/H-Infinity" , "HinfTenzo/DisturboIn/ErrIn/P(s)1" } ;
static const char_T * rt_LoggedStateNames [ ] = { "" , "" , "" , "" , "" , ""
, "" , "" , "" } ; static boolean_T rt_LoggedStateCrossMdlRef [ ] = { 0 , 0 ,
0 , 0 , 0 , 0 , 0 , 0 , 0 } ; static RTWLogDataTypeConvert
rt_RTWLogDataTypeConvert [ ] = { { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 ,
1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } ,
{ 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } } ; static RTWLogSignalInfo
rt_LoggedStateSignalInfo = { 9 , rt_LoggedStateWidths ,
rt_LoggedStateNumDimensions , rt_LoggedStateDimensions ,
rt_LoggedStateIsVarDims , ( NULL ) , ( NULL ) , rt_LoggedStateDataTypeIds ,
rt_LoggedStateComplexSignals , ( NULL ) , { rt_LoggedStateLabels } , ( NULL )
, ( NULL ) , ( NULL ) , { rt_LoggedStateBlockNames } , { rt_LoggedStateNames
} , rt_LoggedStateCrossMdlRef , rt_RTWLogDataTypeConvert } ; static void *
rt_LoggedStateSignalPtrs [ 9 ] ; rtliSetLogXSignalPtrs ( ssGetRTWLogInfo (
rtS ) , ( LogSignalPtrsType ) rt_LoggedStateSignalPtrs ) ;
rtliSetLogXSignalInfo ( ssGetRTWLogInfo ( rtS ) , & rt_LoggedStateSignalInfo
) ; rt_LoggedStateSignalPtrs [ 0 ] = ( void * ) & rtX . ltfask1ptj [ 0 ] ;
rt_LoggedStateSignalPtrs [ 1 ] = ( void * ) & rtX . k5wabiqsuk [ 0 ] ;
rt_LoggedStateSignalPtrs [ 2 ] = ( void * ) & rtX . b11juaaqeh [ 0 ] ;
rt_LoggedStateSignalPtrs [ 3 ] = ( void * ) & rtX . oacfijplwi [ 0 ] ;
rt_LoggedStateSignalPtrs [ 4 ] = ( void * ) & rtX . njhzavtx5o [ 0 ] ;
rt_LoggedStateSignalPtrs [ 5 ] = ( void * ) & rtX . hudxhgdxi2 [ 0 ] ;
rt_LoggedStateSignalPtrs [ 6 ] = ( void * ) & rtX . l2dleorvkm ;
rt_LoggedStateSignalPtrs [ 7 ] = ( void * ) & rtX . phhupngpjs [ 0 ] ;
rt_LoggedStateSignalPtrs [ 8 ] = ( void * ) & rtX . inruainrus [ 0 ] ; }
rtliSetLogT ( ssGetRTWLogInfo ( rtS ) , "tout" ) ; rtliSetLogX (
ssGetRTWLogInfo ( rtS ) , "tmp_raccel_xout" ) ; rtliSetLogXFinal (
ssGetRTWLogInfo ( rtS ) , "xFinal" ) ; rtliSetLogVarNameModifier (
ssGetRTWLogInfo ( rtS ) , "none" ) ; rtliSetLogFormat ( ssGetRTWLogInfo ( rtS
) , 2 ) ; rtliSetLogMaxRows ( ssGetRTWLogInfo ( rtS ) , 1000 ) ;
rtliSetLogDecimation ( ssGetRTWLogInfo ( rtS ) , 1 ) ; rtliSetLogY (
ssGetRTWLogInfo ( rtS ) , "" ) ; rtliSetLogYSignalInfo ( ssGetRTWLogInfo (
rtS ) , ( NULL ) ) ; rtliSetLogYSignalPtrs ( ssGetRTWLogInfo ( rtS ) , ( NULL
) ) ; } { static struct _ssStatesInfo2 statesInfo2 ; ssSetStatesInfo2 ( rtS ,
& statesInfo2 ) ; } { static ssPeriodicStatesInfo periodicStatesInfo ;
ssSetPeriodicStatesInfo ( rtS , & periodicStatesInfo ) ; } { static
ssSolverInfo slvrInfo ; static boolean_T contStatesDisabled [ 49 ] ; static
real_T absTol [ 49 ] = { 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6
, 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 ,
1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 ,
1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 ,
1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 ,
1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 ,
1.0E-6 , 1.0E-6 , 1.0E-6 } ; static uint8_T absTolControl [ 49 ] = { 0U , 0U
, 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U ,
0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U
, 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U ,
0U } ; static uint8_T zcAttributes [ 17 ] = { ( ZC_EVENT_ALL ) , (
ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , (
ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP ) , (
ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) ,
( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , (
ZC_EVENT_ALL ) } ; static ssNonContDerivSigInfo nonContDerivSigInfo [ 1 ] = {
{ 1 * sizeof ( real_T ) , ( char * ) ( & rtB . bnmysexqkh ) , ( NULL ) } } ;
ssSetSolverRelTol ( rtS , 0.001 ) ; ssSetStepSize ( rtS , 0.0 ) ;
ssSetMinStepSize ( rtS , 0.0 ) ; ssSetMaxNumMinSteps ( rtS , - 1 ) ;
ssSetMinStepViolatedError ( rtS , 0 ) ; ssSetMaxStepSize ( rtS ,
0.0056625141562853904 ) ; ssSetSolverMaxOrder ( rtS , - 1 ) ;
ssSetSolverRefineFactor ( rtS , 1 ) ; ssSetOutputTimes ( rtS , ( NULL ) ) ;
ssSetNumOutputTimes ( rtS , 0 ) ; ssSetOutputTimesOnly ( rtS , 0 ) ;
ssSetOutputTimesIndex ( rtS , 0 ) ; ssSetZCCacheNeedsReset ( rtS , 0 ) ;
ssSetDerivCacheNeedsReset ( rtS , 0 ) ; ssSetNumNonContDerivSigInfos ( rtS ,
1 ) ; ssSetNonContDerivSigInfos ( rtS , nonContDerivSigInfo ) ;
ssSetSolverInfo ( rtS , & slvrInfo ) ; ssSetSolverName ( rtS , "ode23" ) ;
ssSetVariableStepSolver ( rtS , 1 ) ; ssSetSolverConsistencyChecking ( rtS ,
0 ) ; ssSetSolverAdaptiveZcDetection ( rtS , 0 ) ;
ssSetSolverRobustResetMethod ( rtS , 0 ) ; ssSetAbsTolVector ( rtS , absTol )
; ssSetAbsTolControlVector ( rtS , absTolControl ) ;
ssSetSolverAbsTol_Obsolete ( rtS , absTol ) ;
ssSetSolverAbsTolControl_Obsolete ( rtS , absTolControl ) ;
ssSetSolverStateProjection ( rtS , 0 ) ; ssSetSolverMassMatrixType ( rtS , (
ssMatrixType ) 0 ) ; ssSetSolverMassMatrixNzMax ( rtS , 0 ) ;
ssSetModelOutputs ( rtS , MdlOutputs ) ; ssSetModelLogData ( rtS ,
rt_UpdateTXYLogVars ) ; ssSetModelLogDataIfInInterval ( rtS ,
rt_UpdateTXXFYLogVars ) ; ssSetModelUpdate ( rtS , MdlUpdate ) ;
ssSetModelDerivatives ( rtS , MdlDerivatives ) ; ssSetSolverZcSignalAttrib (
rtS , zcAttributes ) ; ssSetSolverNumZcSignals ( rtS , 17 ) ;
ssSetModelZeroCrossings ( rtS , MdlZeroCrossings ) ;
ssSetSolverConsecutiveZCsStepRelTol ( rtS , 2.8421709430404007E-13 ) ;
ssSetSolverMaxConsecutiveZCs ( rtS , 1000 ) ; ssSetSolverConsecutiveZCsError
( rtS , 2 ) ; ssSetSolverMaskedZcDiagnostic ( rtS , 1 ) ;
ssSetSolverIgnoredZcDiagnostic ( rtS , 1 ) ; ssSetSolverMaxConsecutiveMinStep
( rtS , 1 ) ; ssSetSolverShapePreserveControl ( rtS , 2 ) ; ssSetTNextTid (
rtS , INT_MIN ) ; ssSetTNext ( rtS , rtMinusInf ) ; ssSetSolverNeedsReset (
rtS ) ; ssSetNumNonsampledZCs ( rtS , 17 ) ; ssSetContStateDisabled ( rtS ,
contStatesDisabled ) ; ssSetSolverMaxConsecutiveMinStep ( rtS , 1 ) ; }
ssSetChecksumVal ( rtS , 0 , 2359241291U ) ; ssSetChecksumVal ( rtS , 1 ,
787542146U ) ; ssSetChecksumVal ( rtS , 2 , 808962859U ) ; ssSetChecksumVal (
rtS , 3 , 974030338U ) ; { static const sysRanDType rtAlwaysEnabled =
SUBSYS_RAN_BC_ENABLE ; static RTWExtModeInfo rt_ExtModeInfo ; static const
sysRanDType * systemRan [ 4 ] ; gblRTWExtModeInfo = & rt_ExtModeInfo ;
ssSetRTWExtModeInfo ( rtS , & rt_ExtModeInfo ) ;
rteiSetSubSystemActiveVectorAddresses ( & rt_ExtModeInfo , systemRan ) ;
systemRan [ 0 ] = & rtAlwaysEnabled ; systemRan [ 1 ] = & rtAlwaysEnabled ;
systemRan [ 2 ] = & rtAlwaysEnabled ; systemRan [ 3 ] = & rtAlwaysEnabled ;
rteiSetModelMappingInfoPtr ( ssGetRTWExtModeInfo ( rtS ) , &
ssGetModelMappingInfo ( rtS ) ) ; rteiSetChecksumsPtr ( ssGetRTWExtModeInfo (
rtS ) , ssGetChecksums ( rtS ) ) ; rteiSetTPtr ( ssGetRTWExtModeInfo ( rtS )
, ssGetTPtr ( rtS ) ) ; } return rtS ; } const int_T gblParameterTuningTid =
2 ; void MdlOutputsParameterSampleTime ( int_T tid ) { MdlOutputsTID2 ( tid )
; }
