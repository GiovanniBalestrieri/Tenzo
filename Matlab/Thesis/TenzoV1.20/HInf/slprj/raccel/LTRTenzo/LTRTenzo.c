#include "__cf_LTRTenzo.h"
#include "rt_logging_mmi.h"
#include "LTRTenzo_capi.h"
#include <math.h>
#include "LTRTenzo.h"
#include "LTRTenzo_private.h"
#include "LTRTenzo_dt.h"
extern void * CreateDiagnosticAsVoidPtr_wrapper ( const char * id , int nargs
, ... ) ; RTWExtModeInfo * gblRTWExtModeInfo = NULL ; extern boolean_T
gblExtModeStartPktReceived ; void raccelForceExtModeShutdown ( ) { if ( !
gblExtModeStartPktReceived ) { boolean_T stopRequested = false ;
rtExtModeWaitForStartPkt ( gblRTWExtModeInfo , 3 , & stopRequested ) ; }
rtExtModeShutdown ( 3 ) ; } const int_T gblNumToFiles = 0 ; const int_T
gblNumFrFiles = 0 ; const int_T gblNumFrWksBlocks = 8 ;
#ifdef RSIM_WITH_SOLVER_MULTITASKING
boolean_T gbl_raccel_isMultitasking = 1 ;
#else
boolean_T gbl_raccel_isMultitasking = 0 ;
#endif
boolean_T gbl_raccel_tid01eq = 0 ; int_T gbl_raccel_NumST = 4 ; const char_T
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
"slprj//raccel//LTRTenzo//LTRTenzo_Jpattern.mat" ; const int_T
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
) { int_T is ; uint32_T tseed ; int32_T t ; real_T tmp ; rtX . ptbb520ltm [ 0
] = 0.0 ; rtX . n0ofazoxbd [ 0 ] = 0.0 ; rtX . asbanfimve [ 0 ] = 0.0 ; rtX .
glltmukddh [ 0 ] = 0.0 ; rtX . ptbb520ltm [ 1 ] = 0.0 ; rtX . n0ofazoxbd [ 1
] = 0.0 ; rtX . asbanfimve [ 1 ] = 0.0 ; rtX . glltmukddh [ 1 ] = 0.0 ; rtDW
. cx2opkhkks = ( rtInf ) ; rtDW . gs42o1safk = ( rtInf ) ; for ( is = 0 ; is
< 8 ; is ++ ) { rtX . irr1ohciq3 [ is ] = rtP . Processo1_X0 ; rtX .
mcu5m2cl2g [ is ] = rtP . kalman_X0 ; } rtX . odzwuprsis = 0.0 ; tmp =
muDoubleScalarFloor ( rtP . UniformRandomNumber_Seed ) ; if (
muDoubleScalarIsNaN ( tmp ) || muDoubleScalarIsInf ( tmp ) ) { tmp = 0.0 ; }
else { tmp = muDoubleScalarRem ( tmp , 4.294967296E+9 ) ; } tseed = tmp < 0.0
? ( uint32_T ) - ( int32_T ) ( uint32_T ) - tmp : ( uint32_T ) tmp ; is = (
int32_T ) ( tseed >> 16U ) ; t = ( int32_T ) ( tseed & 32768U ) ; tseed = ( (
( ( tseed - ( ( uint32_T ) is << 16U ) ) + t ) << 16U ) + t ) + is ; if (
tseed < 1U ) { tseed = 1144108930U ; } else { if ( tseed > 2147483646U ) {
tseed = 2147483646U ; } } rtDW . hjtfckrdib = tseed ; rtDW . gzecfrgu4x = (
rtP . randomAmpNoise - ( - rtP . randomAmpNoise ) ) *
rt_urand_Upu32_Yd_f_pw_snf ( & rtDW . hjtfckrdib ) + - rtP . randomAmpNoise ;
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
} } { FWksInfo * fromwksInfo ; if ( ( fromwksInfo = ( FWksInfo * ) calloc ( 1
, sizeof ( FWksInfo ) ) ) == ( NULL ) ) { ssSetErrorStatus ( rtS ,
"from workspace STRING(Name) memory allocation error" ) ; } else {
fromwksInfo -> origWorkspaceVarName = "tuvar" ; fromwksInfo -> origDataTypeId
= 0 ; fromwksInfo -> origIsComplex = 0 ; fromwksInfo -> origWidth = 1 ;
fromwksInfo -> origElSize = sizeof ( real_T ) ; fromwksInfo -> data = ( void
* ) rtP . FromWs_Data0 ; fromwksInfo -> nDataPoints = 5 ; fromwksInfo -> time
= ( double * ) rtP . FromWs_Time0 ; rtDW . pp4sameqxw . TimePtr = fromwksInfo
-> time ; rtDW . pp4sameqxw . DataPtr = fromwksInfo -> data ; rtDW .
pp4sameqxw . RSimInfoPtr = fromwksInfo ; } rtDW . gooaqjnwdd . PrevIndex = 0
; { int_T * zcTimeIndices = & rtDW . mwqrktpz41 ; const double * timePoints =
( double * ) rtDW . pp4sameqxw . TimePtr ; boolean_T justHadZcTime = false ;
int_T zcIdx = 0 ; int_T i ; for ( i = 0 ; i < 5 - 1 ; i ++ ) { if ( !
justHadZcTime && timePoints [ i ] == timePoints [ i + 1 ] ) { zcTimeIndices [
zcIdx ++ ] = i ; justHadZcTime = true ; } else { justHadZcTime = false ; } }
rtDW . eknmvbtv5u = 0 ; } } { FWksInfo * fromwksInfo ; if ( ( fromwksInfo = (
FWksInfo * ) calloc ( 1 , sizeof ( FWksInfo ) ) ) == ( NULL ) ) {
ssSetErrorStatus ( rtS ,
"from workspace STRING(Name) memory allocation error" ) ; } else {
fromwksInfo -> origWorkspaceVarName = "tuvar" ; fromwksInfo -> origDataTypeId
= 0 ; fromwksInfo -> origIsComplex = 0 ; fromwksInfo -> origWidth = 1 ;
fromwksInfo -> origElSize = sizeof ( real_T ) ; fromwksInfo -> data = ( void
* ) rtP . FromWs_Data0_kupdvn51v2 ; fromwksInfo -> nDataPoints = 6 ;
fromwksInfo -> time = ( double * ) rtP . FromWs_Time0_h113reko55 ; rtDW .
pszu0ywji3 . TimePtr = fromwksInfo -> time ; rtDW . pszu0ywji3 . DataPtr =
fromwksInfo -> data ; rtDW . pszu0ywji3 . RSimInfoPtr = fromwksInfo ; } rtDW
. pxxh1knxpv . PrevIndex = 0 ; { int_T * zcTimeIndices = & rtDW . ndd51edmhg
[ 0 ] ; const double * timePoints = ( double * ) rtDW . pszu0ywji3 . TimePtr
; boolean_T justHadZcTime = false ; int_T zcIdx = 0 ; int_T i ; for ( i = 0 ;
i < 6 - 1 ; i ++ ) { if ( ! justHadZcTime && timePoints [ i ] == timePoints [
i + 1 ] ) { zcTimeIndices [ zcIdx ++ ] = i ; justHadZcTime = true ; } else {
justHadZcTime = false ; } } rtDW . jobrukao3e = 0 ; } } { FWksInfo *
fromwksInfo ; if ( ( fromwksInfo = ( FWksInfo * ) calloc ( 1 , sizeof (
FWksInfo ) ) ) == ( NULL ) ) { ssSetErrorStatus ( rtS ,
"from workspace STRING(Name) memory allocation error" ) ; } else {
fromwksInfo -> origWorkspaceVarName = "tuvar" ; fromwksInfo -> origDataTypeId
= 0 ; fromwksInfo -> origIsComplex = 0 ; fromwksInfo -> origWidth = 1 ;
fromwksInfo -> origElSize = sizeof ( real_T ) ; fromwksInfo -> data = ( void
* ) rtP . FromWs_Data0_ohxionynup ; fromwksInfo -> nDataPoints = 4 ;
fromwksInfo -> time = ( double * ) rtP . FromWs_Time0_acysjor4ip ; rtDW .
gmpueztb1y . TimePtr = fromwksInfo -> time ; rtDW . gmpueztb1y . DataPtr =
fromwksInfo -> data ; rtDW . gmpueztb1y . RSimInfoPtr = fromwksInfo ; } rtDW
. mgial2fbbs . PrevIndex = 0 ; { int_T * zcTimeIndices = & rtDW . fpdlolyb1y
; const double * timePoints = ( double * ) rtDW . gmpueztb1y . TimePtr ;
boolean_T justHadZcTime = false ; int_T zcIdx = 0 ; int_T i ; for ( i = 0 ; i
< 4 - 1 ; i ++ ) { if ( ! justHadZcTime && timePoints [ i ] == timePoints [ i
+ 1 ] ) { zcTimeIndices [ zcIdx ++ ] = i ; justHadZcTime = true ; } else {
justHadZcTime = false ; } } rtDW . ga0dnoquno = 0 ; } } { static int_T
rt_ToWksWidths [ ] = { 8 } ; static int_T rt_ToWksNumDimensions [ ] = { 1 } ;
static int_T rt_ToWksDimensions [ ] = { 8 } ; static boolean_T
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
"LTRTenzo/Optima Controller/To Workspace" ; rtDW . nrvudg0031 . LoggedData =
rt_CreateStructLogVar ( ssGetRTWLogInfo ( rtS ) , ssGetTStart ( rtS ) ,
ssGetTFinal ( rtS ) , 0.0 , ( & ssGetErrorStatus ( rtS ) ) , "errSimLTR" , 0
, 0 , 1 , 0.0 , & rt_ToWksSignalInfo , rt_ToWksBlockName ) ; if ( rtDW .
nrvudg0031 . LoggedData == ( NULL ) ) return ; } { FWksInfo * fromwksInfo ;
if ( ( fromwksInfo = ( FWksInfo * ) calloc ( 1 , sizeof ( FWksInfo ) ) ) == (
NULL ) ) { ssSetErrorStatus ( rtS ,
"from workspace STRING(Name) memory allocation error" ) ; } else {
fromwksInfo -> origWorkspaceVarName = "tuvar" ; fromwksInfo -> origDataTypeId
= 0 ; fromwksInfo -> origIsComplex = 0 ; fromwksInfo -> origWidth = 1 ;
fromwksInfo -> origElSize = sizeof ( real_T ) ; fromwksInfo -> data = ( void
* ) rtP . FromWs_Data0_pvombv1exi ; fromwksInfo -> nDataPoints = 4 ;
fromwksInfo -> time = ( double * ) rtP . FromWs_Time0_dnxa5yquln ; rtDW .
mq2efjljlp . TimePtr = fromwksInfo -> time ; rtDW . mq2efjljlp . DataPtr =
fromwksInfo -> data ; rtDW . mq2efjljlp . RSimInfoPtr = fromwksInfo ; } rtDW
. k2mrpavnr3 . PrevIndex = 0 ; { int_T * zcTimeIndices = & rtDW . jisc0stmja
; const double * timePoints = ( double * ) rtDW . mq2efjljlp . TimePtr ;
boolean_T justHadZcTime = false ; int_T zcIdx = 0 ; int_T i ; for ( i = 0 ; i
< 4 - 1 ; i ++ ) { if ( ! justHadZcTime && timePoints [ i ] == timePoints [ i
+ 1 ] ) { zcTimeIndices [ zcIdx ++ ] = i ; justHadZcTime = true ; } else {
justHadZcTime = false ; } } rtDW . bhjarlltx0 = 0 ; } } { FWksInfo *
fromwksInfo ; if ( ( fromwksInfo = ( FWksInfo * ) calloc ( 1 , sizeof (
FWksInfo ) ) ) == ( NULL ) ) { ssSetErrorStatus ( rtS ,
"from workspace STRING(Name) memory allocation error" ) ; } else {
fromwksInfo -> origWorkspaceVarName = "tuvar" ; fromwksInfo -> origDataTypeId
= 0 ; fromwksInfo -> origIsComplex = 0 ; fromwksInfo -> origWidth = 1 ;
fromwksInfo -> origElSize = sizeof ( real_T ) ; fromwksInfo -> data = ( void
* ) rtP . FromWs_Data0_apflh1ms4l ; fromwksInfo -> nDataPoints = 10 ;
fromwksInfo -> time = ( double * ) rtP . FromWs_Time0_an32oq3ef4 ; rtDW .
morfgxatez . TimePtr = fromwksInfo -> time ; rtDW . morfgxatez . DataPtr =
fromwksInfo -> data ; rtDW . morfgxatez . RSimInfoPtr = fromwksInfo ; } rtDW
. p0ybzcrnsf . PrevIndex = 0 ; { int_T * zcTimeIndices = & rtDW . mmq01gbox5
[ 0 ] ; const double * timePoints = ( double * ) rtDW . morfgxatez . TimePtr
; boolean_T justHadZcTime = false ; int_T zcIdx = 0 ; int_T i ; for ( i = 0 ;
i < 10 - 1 ; i ++ ) { if ( ! justHadZcTime && timePoints [ i ] == timePoints
[ i + 1 ] ) { zcTimeIndices [ zcIdx ++ ] = i ; justHadZcTime = true ; } else
{ justHadZcTime = false ; } } rtDW . gml05gaiwr = 0 ; } } { FWksInfo *
fromwksInfo ; if ( ( fromwksInfo = ( FWksInfo * ) calloc ( 1 , sizeof (
FWksInfo ) ) ) == ( NULL ) ) { ssSetErrorStatus ( rtS ,
"from workspace STRING(Name) memory allocation error" ) ; } else {
fromwksInfo -> origWorkspaceVarName = "tuvar" ; fromwksInfo -> origDataTypeId
= 0 ; fromwksInfo -> origIsComplex = 0 ; fromwksInfo -> origWidth = 1 ;
fromwksInfo -> origElSize = sizeof ( real_T ) ; fromwksInfo -> data = ( void
* ) rtP . FromWs_Data0_gglwjxomlu ; fromwksInfo -> nDataPoints = 10 ;
fromwksInfo -> time = ( double * ) rtP . FromWs_Time0_bxwfouleny ; rtDW .
d2vfv2ffdo . TimePtr = fromwksInfo -> time ; rtDW . d2vfv2ffdo . DataPtr =
fromwksInfo -> data ; rtDW . d2vfv2ffdo . RSimInfoPtr = fromwksInfo ; } rtDW
. dppeb2fcvo . PrevIndex = 0 ; { int_T * zcTimeIndices = & rtDW . p1tupjnl2m
[ 0 ] ; const double * timePoints = ( double * ) rtDW . d2vfv2ffdo . TimePtr
; boolean_T justHadZcTime = false ; int_T zcIdx = 0 ; int_T i ; for ( i = 0 ;
i < 10 - 1 ; i ++ ) { if ( ! justHadZcTime && timePoints [ i ] == timePoints
[ i + 1 ] ) { zcTimeIndices [ zcIdx ++ ] = i ; justHadZcTime = true ; } else
{ justHadZcTime = false ; } } rtDW . pt0evqezqm = 0 ; } } { FWksInfo *
fromwksInfo ; if ( ( fromwksInfo = ( FWksInfo * ) calloc ( 1 , sizeof (
FWksInfo ) ) ) == ( NULL ) ) { ssSetErrorStatus ( rtS ,
"from workspace STRING(Name) memory allocation error" ) ; } else {
fromwksInfo -> origWorkspaceVarName = "tuvar" ; fromwksInfo -> origDataTypeId
= 0 ; fromwksInfo -> origIsComplex = 0 ; fromwksInfo -> origWidth = 1 ;
fromwksInfo -> origElSize = sizeof ( real_T ) ; fromwksInfo -> data = ( void
* ) rtP . FromWs_Data0_n4fvl010iy ; fromwksInfo -> nDataPoints = 10 ;
fromwksInfo -> time = ( double * ) rtP . FromWs_Time0_dgntxkplur ; rtDW .
ctabrgnsxy . TimePtr = fromwksInfo -> time ; rtDW . ctabrgnsxy . DataPtr =
fromwksInfo -> data ; rtDW . ctabrgnsxy . RSimInfoPtr = fromwksInfo ; } rtDW
. jdmacw4ijl . PrevIndex = 0 ; { int_T * zcTimeIndices = & rtDW . jt4kc1ii0s
[ 0 ] ; const double * timePoints = ( double * ) rtDW . ctabrgnsxy . TimePtr
; boolean_T justHadZcTime = false ; int_T zcIdx = 0 ; int_T i ; for ( i = 0 ;
i < 10 - 1 ; i ++ ) { if ( ! justHadZcTime && timePoints [ i ] == timePoints
[ i + 1 ] ) { zcTimeIndices [ zcIdx ++ ] = i ; justHadZcTime = true ; } else
{ justHadZcTime = false ; } } rtDW . hns1rud2yi = 0 ; } } { FWksInfo *
fromwksInfo ; if ( ( fromwksInfo = ( FWksInfo * ) calloc ( 1 , sizeof (
FWksInfo ) ) ) == ( NULL ) ) { ssSetErrorStatus ( rtS ,
"from workspace STRING(Name) memory allocation error" ) ; } else {
fromwksInfo -> origWorkspaceVarName = "tuvar" ; fromwksInfo -> origDataTypeId
= 0 ; fromwksInfo -> origIsComplex = 0 ; fromwksInfo -> origWidth = 1 ;
fromwksInfo -> origElSize = sizeof ( real_T ) ; fromwksInfo -> data = ( void
* ) rtP . FromWs_Data0_i5qxt5w12n ; fromwksInfo -> nDataPoints = 8 ;
fromwksInfo -> time = ( double * ) rtP . FromWs_Time0_jrdfiknsxo ; rtDW .
nl25ijg5ia . TimePtr = fromwksInfo -> time ; rtDW . nl25ijg5ia . DataPtr =
fromwksInfo -> data ; rtDW . nl25ijg5ia . RSimInfoPtr = fromwksInfo ; } rtDW
. i3b2ntrlap . PrevIndex = 0 ; { int_T * zcTimeIndices = & rtDW . mek4mei4m5
[ 0 ] ; const double * timePoints = ( double * ) rtDW . nl25ijg5ia . TimePtr
; boolean_T justHadZcTime = false ; int_T zcIdx = 0 ; int_T i ; for ( i = 0 ;
i < 8 - 1 ; i ++ ) { if ( ! justHadZcTime && timePoints [ i ] == timePoints [
i + 1 ] ) { zcTimeIndices [ zcIdx ++ ] = i ; justHadZcTime = true ; } else {
justHadZcTime = false ; } } rtDW . ppttotus2n = 0 ; } } MdlInitialize ( ) ; {
bool externalInputIsInDatasetFormat = false ; void * pISigstreamManager =
rt_GetISigstreamManager ( ) ; rtwISigstreamManagerGetInputIsInDatasetFormat (
pISigstreamManager , & externalInputIsInDatasetFormat ) ; if (
externalInputIsInDatasetFormat ) { } } } void MdlOutputs ( int_T tid ) {
real_T o21ke3zjbt ; real_T n1k5e2cope ; real_T mlm3lu03dt ; int_T iy ; real_T
* lastU ; int_T ci ; real_T grvb3sdz54 [ 4 ] ; real_T n4m0dezwe5 [ 8 ] ;
real_T hjn5i4m4s0 ; real_T akilpw5pwp ; real_T ck23yc4dbv ; real_T ikvvurqrwh
; real_T jtth5yy2zo ; real_T tmp [ 4 ] ; rtB . mbsguhsyce = ssGetT ( rtS ) ;
if ( ssIsMajorTimeStep ( rtS ) ) { rtDW . c0c55fi3jq = ( rtB . mbsguhsyce >=
rtP . Switch_Threshold ) ; } if ( rtDW . c0c55fi3jq ) { rtB . kwjrknjxrb = (
muDoubleScalarSin ( rtP . omegaPertOut * ssGetTaskTime ( rtS , 0 ) + rtP .
SinOut_Phase ) * rtP . amplitudePertOut + rtP . SinOut_Bias ) + rtP .
cstPertOut ; rtB . gxh0zyu42a = rtB . kwjrknjxrb ; } else { rtB . gxh0zyu42a
= rtP . Constant_Value ; } for ( iy = 0 ; iy < 4 ; iy ++ ) { grvb3sdz54 [ iy
] = 0.0 ; for ( ci = 0 ; ci < 8 ; ci ++ ) { grvb3sdz54 [ iy ] += rtP . C0 [ (
ci << 2 ) + iy ] * rtX . irr1ohciq3 [ ci ] ; } } rtB . j4tv1cqyj2 [ 0 ] = rtB
. gxh0zyu42a + grvb3sdz54 [ 0 ] ; rtB . j4tv1cqyj2 [ 1 ] = rtB . gxh0zyu42a +
grvb3sdz54 [ 1 ] ; rtB . j4tv1cqyj2 [ 2 ] = rtB . gxh0zyu42a + grvb3sdz54 [ 2
] ; rtB . j4tv1cqyj2 [ 3 ] = rtB . gxh0zyu42a + grvb3sdz54 [ 3 ] ; hjn5i4m4s0
= rtP . Ps1_C [ 0 ] * rtX . ptbb520ltm [ 0 ] + rtP . Ps1_C [ 1 ] * rtX .
ptbb520ltm [ 1 ] ; akilpw5pwp = rtP . Ps2_C [ 0 ] * rtX . n0ofazoxbd [ 0 ] +
rtP . Ps2_C [ 1 ] * rtX . n0ofazoxbd [ 1 ] ; ck23yc4dbv = rtP . Ps3_C [ 0 ] *
rtX . asbanfimve [ 0 ] + rtP . Ps3_C [ 1 ] * rtX . asbanfimve [ 1 ] ;
ikvvurqrwh = rtP . Ps4_C [ 0 ] * rtX . glltmukddh [ 0 ] + rtP . Ps4_C [ 1 ] *
rtX . glltmukddh [ 1 ] ; { real_T * pDataValues = ( real_T * ) rtDW .
pp4sameqxw . DataPtr ; real_T * pTimeValues = ( real_T * ) rtDW . pp4sameqxw
. TimePtr ; int_T currTimeIndex = rtDW . gooaqjnwdd . PrevIndex ; real_T t =
ssGetTaskTime ( rtS , 0 ) ; int numPoints , lastPoint ; FWksInfo *
fromwksInfo = ( FWksInfo * ) rtDW . pp4sameqxw . RSimInfoPtr ; numPoints =
fromwksInfo -> nDataPoints ; lastPoint = numPoints - 1 ; if ( t <=
pTimeValues [ 0 ] ) { currTimeIndex = 0 ; } else if ( t >= pTimeValues [
lastPoint ] ) { currTimeIndex = lastPoint - 1 ; } else { if ( t < pTimeValues
[ currTimeIndex ] ) { while ( t < pTimeValues [ currTimeIndex ] ) {
currTimeIndex -- ; } } else { while ( t >= pTimeValues [ currTimeIndex + 1 ]
) { currTimeIndex ++ ; } } } rtDW . gooaqjnwdd . PrevIndex = currTimeIndex ;
{ real_T t1 = pTimeValues [ currTimeIndex ] ; real_T t2 = pTimeValues [
currTimeIndex + 1 ] ; if ( t1 == t2 ) { if ( t < t1 ) { rtB . gbemrwhpr5 =
pDataValues [ currTimeIndex ] ; } else { rtB . gbemrwhpr5 = pDataValues [
currTimeIndex + 1 ] ; } } else { real_T f1 = ( t2 - t ) / ( t2 - t1 ) ;
real_T f2 = 1.0 - f1 ; real_T d1 ; real_T d2 ; int_T TimeIndex =
currTimeIndex ; int_T * zcTimeIndices = & rtDW . mwqrktpz41 ; int_T *
zcTimeIndicesIdx = & rtDW . eknmvbtv5u ; int_T zcIdx = zcTimeIndices [ *
zcTimeIndicesIdx ] ; int_T numZcTimes = 1 ; if ( * zcTimeIndicesIdx <
numZcTimes ) { if ( ssIsMajorTimeStep ( rtS ) ) { if ( t > pTimeValues [
zcIdx ] ) { while ( * zcTimeIndicesIdx < ( numZcTimes - 1 ) && ( t >
pTimeValues [ zcIdx ] ) ) { ( * zcTimeIndicesIdx ) ++ ; zcIdx = zcTimeIndices
[ * zcTimeIndicesIdx ] ; } } } else { if ( t >= pTimeValues [ zcIdx ] && (
ssGetTimeOfLastOutput ( rtS ) < pTimeValues [ zcIdx ] ) ) { t2 = pTimeValues
[ zcIdx ] ; if ( zcIdx == 0 ) { TimeIndex = 0 ; t1 = t2 - 1 ; } else { t1 =
pTimeValues [ zcIdx - 1 ] ; TimeIndex = zcIdx - 1 ; } f1 = ( t2 - t ) / ( t2
- t1 ) ; f2 = 1.0 - f1 ; } } } d1 = pDataValues [ TimeIndex ] ; d2 =
pDataValues [ TimeIndex + 1 ] ; if ( zcIdx == 0 ) { d2 = d1 ; } rtB .
gbemrwhpr5 = ( real_T ) rtInterpolate ( d1 , d2 , f1 , f2 ) ; pDataValues +=
numPoints ; } } } if ( ( rtDW . cx2opkhkks >= ssGetT ( rtS ) ) && ( rtDW .
gs42o1safk >= ssGetT ( rtS ) ) ) { jtth5yy2zo = 0.0 ; } else { jtth5yy2zo =
rtDW . cx2opkhkks ; lastU = & rtDW . pzg1f0zcsj ; if ( rtDW . cx2opkhkks <
rtDW . gs42o1safk ) { if ( rtDW . gs42o1safk < ssGetT ( rtS ) ) { jtth5yy2zo
= rtDW . gs42o1safk ; lastU = & rtDW . bthpfg5v51 ; } } else { if ( rtDW .
cx2opkhkks >= ssGetT ( rtS ) ) { jtth5yy2zo = rtDW . gs42o1safk ; lastU = &
rtDW . bthpfg5v51 ; } } jtth5yy2zo = ( rtB . gbemrwhpr5 - * lastU ) / (
ssGetT ( rtS ) - jtth5yy2zo ) ; } { real_T * pDataValues = ( real_T * ) rtDW
. pszu0ywji3 . DataPtr ; real_T * pTimeValues = ( real_T * ) rtDW .
pszu0ywji3 . TimePtr ; int_T currTimeIndex = rtDW . pxxh1knxpv . PrevIndex ;
real_T t = ssGetTaskTime ( rtS , 0 ) ; int numPoints , lastPoint ; FWksInfo *
fromwksInfo = ( FWksInfo * ) rtDW . pszu0ywji3 . RSimInfoPtr ; numPoints =
fromwksInfo -> nDataPoints ; lastPoint = numPoints - 1 ; if ( t <=
pTimeValues [ 0 ] ) { currTimeIndex = 0 ; } else if ( t >= pTimeValues [
lastPoint ] ) { currTimeIndex = lastPoint - 1 ; } else { if ( t < pTimeValues
[ currTimeIndex ] ) { while ( t < pTimeValues [ currTimeIndex ] ) {
currTimeIndex -- ; } } else { while ( t >= pTimeValues [ currTimeIndex + 1 ]
) { currTimeIndex ++ ; } } } rtDW . pxxh1knxpv . PrevIndex = currTimeIndex ;
{ real_T t1 = pTimeValues [ currTimeIndex ] ; real_T t2 = pTimeValues [
currTimeIndex + 1 ] ; if ( t1 == t2 ) { if ( t < t1 ) { o21ke3zjbt =
pDataValues [ currTimeIndex ] ; } else { o21ke3zjbt = pDataValues [
currTimeIndex + 1 ] ; } } else { real_T f1 = ( t2 - t ) / ( t2 - t1 ) ;
real_T f2 = 1.0 - f1 ; real_T d1 ; real_T d2 ; int_T TimeIndex =
currTimeIndex ; int_T * zcTimeIndices = & rtDW . ndd51edmhg [ 0 ] ; int_T *
zcTimeIndicesIdx = & rtDW . jobrukao3e ; int_T zcIdx = zcTimeIndices [ *
zcTimeIndicesIdx ] ; int_T numZcTimes = 2 ; if ( * zcTimeIndicesIdx <
numZcTimes ) { if ( ssIsMajorTimeStep ( rtS ) ) { if ( t > pTimeValues [
zcIdx ] ) { while ( * zcTimeIndicesIdx < ( numZcTimes - 1 ) && ( t >
pTimeValues [ zcIdx ] ) ) { ( * zcTimeIndicesIdx ) ++ ; zcIdx = zcTimeIndices
[ * zcTimeIndicesIdx ] ; } } } else { if ( t >= pTimeValues [ zcIdx ] && (
ssGetTimeOfLastOutput ( rtS ) < pTimeValues [ zcIdx ] ) ) { t2 = pTimeValues
[ zcIdx ] ; if ( zcIdx == 0 ) { TimeIndex = 0 ; t1 = t2 - 1 ; } else { t1 =
pTimeValues [ zcIdx - 1 ] ; TimeIndex = zcIdx - 1 ; } f1 = ( t2 - t ) / ( t2
- t1 ) ; f2 = 1.0 - f1 ; } } } d1 = pDataValues [ TimeIndex ] ; d2 =
pDataValues [ TimeIndex + 1 ] ; if ( zcIdx == 0 ) { d2 = d1 ; } o21ke3zjbt =
( real_T ) rtInterpolate ( d1 , d2 , f1 , f2 ) ; pDataValues += numPoints ; }
} } { real_T * pDataValues = ( real_T * ) rtDW . gmpueztb1y . DataPtr ;
real_T * pTimeValues = ( real_T * ) rtDW . gmpueztb1y . TimePtr ; int_T
currTimeIndex = rtDW . mgial2fbbs . PrevIndex ; real_T t = ssGetTaskTime (
rtS , 0 ) ; int numPoints , lastPoint ; FWksInfo * fromwksInfo = ( FWksInfo *
) rtDW . gmpueztb1y . RSimInfoPtr ; numPoints = fromwksInfo -> nDataPoints ;
lastPoint = numPoints - 1 ; if ( t <= pTimeValues [ 0 ] ) { currTimeIndex = 0
; } else if ( t >= pTimeValues [ lastPoint ] ) { currTimeIndex = lastPoint -
1 ; } else { if ( t < pTimeValues [ currTimeIndex ] ) { while ( t <
pTimeValues [ currTimeIndex ] ) { currTimeIndex -- ; } } else { while ( t >=
pTimeValues [ currTimeIndex + 1 ] ) { currTimeIndex ++ ; } } } rtDW .
mgial2fbbs . PrevIndex = currTimeIndex ; { real_T t1 = pTimeValues [
currTimeIndex ] ; real_T t2 = pTimeValues [ currTimeIndex + 1 ] ; if ( t1 ==
t2 ) { if ( t < t1 ) { n1k5e2cope = pDataValues [ currTimeIndex ] ; } else {
n1k5e2cope = pDataValues [ currTimeIndex + 1 ] ; } } else { real_T f1 = ( t2
- t ) / ( t2 - t1 ) ; real_T f2 = 1.0 - f1 ; real_T d1 ; real_T d2 ; int_T
TimeIndex = currTimeIndex ; int_T * zcTimeIndices = & rtDW . fpdlolyb1y ;
int_T * zcTimeIndicesIdx = & rtDW . ga0dnoquno ; int_T zcIdx = zcTimeIndices
[ * zcTimeIndicesIdx ] ; int_T numZcTimes = 1 ; if ( * zcTimeIndicesIdx <
numZcTimes ) { if ( ssIsMajorTimeStep ( rtS ) ) { if ( t > pTimeValues [
zcIdx ] ) { while ( * zcTimeIndicesIdx < ( numZcTimes - 1 ) && ( t >
pTimeValues [ zcIdx ] ) ) { ( * zcTimeIndicesIdx ) ++ ; zcIdx = zcTimeIndices
[ * zcTimeIndicesIdx ] ; } } } else { if ( t >= pTimeValues [ zcIdx ] && (
ssGetTimeOfLastOutput ( rtS ) < pTimeValues [ zcIdx ] ) ) { t2 = pTimeValues
[ zcIdx ] ; if ( zcIdx == 0 ) { TimeIndex = 0 ; t1 = t2 - 1 ; } else { t1 =
pTimeValues [ zcIdx - 1 ] ; TimeIndex = zcIdx - 1 ; } f1 = ( t2 - t ) / ( t2
- t1 ) ; f2 = 1.0 - f1 ; } } } d1 = pDataValues [ TimeIndex ] ; d2 =
pDataValues [ TimeIndex + 1 ] ; if ( zcIdx == 0 ) { d2 = d1 ; } n1k5e2cope =
( real_T ) rtInterpolate ( d1 , d2 , f1 , f2 ) ; pDataValues += numPoints ; }
} } if ( rtP . ManualSwitch_CurrentSetting == 1 ) { rtB . bdgzagdxra [ 0 ] =
hjn5i4m4s0 ; rtB . bdgzagdxra [ 1 ] = rtP . Constant2_Value ; rtB .
bdgzagdxra [ 2 ] = akilpw5pwp ; rtB . bdgzagdxra [ 3 ] = ck23yc4dbv ; rtB .
bdgzagdxra [ 4 ] = ikvvurqrwh ; rtB . bdgzagdxra [ 5 ] = rtP .
Constant2_Value ; rtB . bdgzagdxra [ 6 ] = rtP . Constant2_Value ; rtB .
bdgzagdxra [ 7 ] = rtP . Constant2_Value ; } else { rtB . bdgzagdxra [ 0 ] =
rtB . gbemrwhpr5 ; rtB . bdgzagdxra [ 1 ] = jtth5yy2zo ; rtB . bdgzagdxra [ 2
] = o21ke3zjbt ; rtB . bdgzagdxra [ 3 ] = n1k5e2cope ; rtB . bdgzagdxra [ 4 ]
= rtP . Constant1_Value ; rtB . bdgzagdxra [ 5 ] = rtP . Constant1_Value ;
rtB . bdgzagdxra [ 6 ] = rtP . Constant1_Value ; rtB . bdgzagdxra [ 7 ] = rtP
. Constant1_Value ; } rtB . bxeh32cm4i = ssGetT ( rtS ) ; if (
ssIsMajorTimeStep ( rtS ) ) { rtDW . omgte3tg31 = ( rtB . bxeh32cm4i >= rtP .
Switch_Threshold_b2p15hzzi4 ) ; } if ( rtDW . omgte3tg31 ) { rtB . fd3jf50j03
= ( muDoubleScalarSin ( rtP . omegaPertIN * ssGetTaskTime ( rtS , 0 ) + rtP .
SineIn_Phase ) * rtP . amplitudePertIn + rtP . SineIn_Bias ) + rtP .
cstPertIn ; rtB . ej3n31dyxa = rtB . fd3jf50j03 ; } else { rtB . ej3n31dyxa =
rtP . Constant_Value_gakmm5of0a ; } for ( iy = 0 ; iy < 8 ; iy ++ ) {
n4m0dezwe5 [ iy ] = 0.0 ; for ( ci = 0 ; ci < 8 ; ci ++ ) { n4m0dezwe5 [ iy ]
+= rtP . KCoss [ ( ci << 3 ) + iy ] * rtX . mcu5m2cl2g [ ci ] ; } rtB .
mfm3tpnt2p [ iy ] = rtB . bdgzagdxra [ iy ] - n4m0dezwe5 [ iy ] ; }
grvb3sdz54 [ 0 ] = rtB . ej3n31dyxa ; grvb3sdz54 [ 1 ] = rtB . ej3n31dyxa ;
grvb3sdz54 [ 2 ] = rtB . ej3n31dyxa ; grvb3sdz54 [ 3 ] = rtB . ej3n31dyxa ;
for ( iy = 0 ; iy < 4 ; iy ++ ) { tmp [ iy ] = 0.0 ; for ( ci = 0 ; ci < 8 ;
ci ++ ) { tmp [ iy ] += rtP . Kopt [ ( ci << 2 ) + iy ] * rtB . mfm3tpnt2p [
ci ] ; } rtB . hjr0spzzaf [ iy ] = grvb3sdz54 [ iy ] + tmp [ iy ] ; } if (
ssIsMajorTimeStep ( rtS ) ) { rtDW . ls41xbqrac [ 0 ] = rtB . hjr0spzzaf [ 0
] >= rtP . Saturation_UpperSat ? 1 : rtB . hjr0spzzaf [ 0 ] > rtP .
Saturation_LowerSat ? 0 : - 1 ; rtDW . ls41xbqrac [ 1 ] = rtB . hjr0spzzaf [
1 ] >= rtP . Saturation_UpperSat ? 1 : rtB . hjr0spzzaf [ 1 ] > rtP .
Saturation_LowerSat ? 0 : - 1 ; rtDW . ls41xbqrac [ 2 ] = rtB . hjr0spzzaf [
2 ] >= rtP . Saturation_UpperSat ? 1 : rtB . hjr0spzzaf [ 2 ] > rtP .
Saturation_LowerSat ? 0 : - 1 ; rtDW . ls41xbqrac [ 3 ] = rtB . hjr0spzzaf [
3 ] >= rtP . Saturation_UpperSat ? 1 : rtB . hjr0spzzaf [ 3 ] > rtP .
Saturation_LowerSat ? 0 : - 1 ; } rtB . csh4yoneye [ 0 ] = rtDW . ls41xbqrac
[ 0 ] == 1 ? rtP . Saturation_UpperSat : rtDW . ls41xbqrac [ 0 ] == - 1 ? rtP
. Saturation_LowerSat : rtB . hjr0spzzaf [ 0 ] ; rtB . csh4yoneye [ 1 ] =
rtDW . ls41xbqrac [ 1 ] == 1 ? rtP . Saturation_UpperSat : rtDW . ls41xbqrac
[ 1 ] == - 1 ? rtP . Saturation_LowerSat : rtB . hjr0spzzaf [ 1 ] ; rtB .
csh4yoneye [ 2 ] = rtDW . ls41xbqrac [ 2 ] == 1 ? rtP . Saturation_UpperSat :
rtDW . ls41xbqrac [ 2 ] == - 1 ? rtP . Saturation_LowerSat : rtB . hjr0spzzaf
[ 2 ] ; rtB . csh4yoneye [ 3 ] = rtDW . ls41xbqrac [ 3 ] == 1 ? rtP .
Saturation_UpperSat : rtDW . ls41xbqrac [ 3 ] == - 1 ? rtP .
Saturation_LowerSat : rtB . hjr0spzzaf [ 3 ] ; if ( rtP .
ManualSwitch1_CurrentSetting == 1 ) { hjn5i4m4s0 = rtP .
Constant1_Value_ngreypf4ru ; } else { hjn5i4m4s0 = rtP . Ps1_C_gfroigzrf4 *
rtX . odzwuprsis ; } rtB . fgbq3jefrx = rtP . Gain_Gain * hjn5i4m4s0 ; if (
ssIsSampleHit ( rtS , 2 , 0 ) ) { rtB . d4wqki33zw = rtDW . gzecfrgu4x ; }
rtB . d1v04lphp2 = ( muDoubleScalarSin ( rtP . omegaNoise * ssGetTaskTime (
rtS , 0 ) + rtP . u2sen05t_Phase ) * rtP . amplitudeNoise + rtP .
u2sen05t_Bias ) + rtB . d4wqki33zw ; if ( ssGetLogOutput ( rtS ) ) { { double
locTime = ssGetTaskTime ( rtS , 0 ) ; ; if ( rtwTimeInLoggingInterval (
rtliGetLoggingInterval ( ssGetRootSS ( rtS ) -> mdlInfo -> rtwLogInfo ) ,
locTime ) ) { rt_UpdateStructLogVar ( ( StructLogVar * ) rtDW . nrvudg0031 .
LoggedData , ( NULL ) , & rtB . mfm3tpnt2p [ 0 ] ) ; } } } { real_T *
pDataValues = ( real_T * ) rtDW . mq2efjljlp . DataPtr ; real_T * pTimeValues
= ( real_T * ) rtDW . mq2efjljlp . TimePtr ; int_T currTimeIndex = rtDW .
k2mrpavnr3 . PrevIndex ; real_T t = ssGetTaskTime ( rtS , 0 ) ; int numPoints
, lastPoint ; FWksInfo * fromwksInfo = ( FWksInfo * ) rtDW . mq2efjljlp .
RSimInfoPtr ; numPoints = fromwksInfo -> nDataPoints ; lastPoint = numPoints
- 1 ; if ( t <= pTimeValues [ 0 ] ) { currTimeIndex = 0 ; } else if ( t >=
pTimeValues [ lastPoint ] ) { currTimeIndex = lastPoint - 1 ; } else { if ( t
< pTimeValues [ currTimeIndex ] ) { while ( t < pTimeValues [ currTimeIndex ]
) { currTimeIndex -- ; } } else { while ( t >= pTimeValues [ currTimeIndex +
1 ] ) { currTimeIndex ++ ; } } } rtDW . k2mrpavnr3 . PrevIndex =
currTimeIndex ; { real_T t1 = pTimeValues [ currTimeIndex ] ; real_T t2 =
pTimeValues [ currTimeIndex + 1 ] ; if ( t1 == t2 ) { if ( t < t1 ) {
mlm3lu03dt = pDataValues [ currTimeIndex ] ; } else { mlm3lu03dt =
pDataValues [ currTimeIndex + 1 ] ; } } else { real_T f1 = ( t2 - t ) / ( t2
- t1 ) ; real_T f2 = 1.0 - f1 ; real_T d1 ; real_T d2 ; int_T TimeIndex =
currTimeIndex ; int_T * zcTimeIndices = & rtDW . jisc0stmja ; int_T *
zcTimeIndicesIdx = & rtDW . bhjarlltx0 ; int_T zcIdx = zcTimeIndices [ *
zcTimeIndicesIdx ] ; int_T numZcTimes = 1 ; if ( * zcTimeIndicesIdx <
numZcTimes ) { if ( ssIsMajorTimeStep ( rtS ) ) { if ( t > pTimeValues [
zcIdx ] ) { while ( * zcTimeIndicesIdx < ( numZcTimes - 1 ) && ( t >
pTimeValues [ zcIdx ] ) ) { ( * zcTimeIndicesIdx ) ++ ; zcIdx = zcTimeIndices
[ * zcTimeIndicesIdx ] ; } } } else { if ( t >= pTimeValues [ zcIdx ] && (
ssGetTimeOfLastOutput ( rtS ) < pTimeValues [ zcIdx ] ) ) { t2 = pTimeValues
[ zcIdx ] ; if ( zcIdx == 0 ) { TimeIndex = 0 ; t1 = t2 - 1 ; } else { t1 =
pTimeValues [ zcIdx - 1 ] ; TimeIndex = zcIdx - 1 ; } f1 = ( t2 - t ) / ( t2
- t1 ) ; f2 = 1.0 - f1 ; } } } d1 = pDataValues [ TimeIndex ] ; d2 =
pDataValues [ TimeIndex + 1 ] ; if ( zcIdx == 0 ) { d2 = d1 ; } mlm3lu03dt =
( real_T ) rtInterpolate ( d1 , d2 , f1 , f2 ) ; pDataValues += numPoints ; }
} } { real_T * pDataValues = ( real_T * ) rtDW . morfgxatez . DataPtr ;
real_T * pTimeValues = ( real_T * ) rtDW . morfgxatez . TimePtr ; int_T
currTimeIndex = rtDW . p0ybzcrnsf . PrevIndex ; real_T t = ssGetTaskTime (
rtS , 0 ) ; int numPoints , lastPoint ; FWksInfo * fromwksInfo = ( FWksInfo *
) rtDW . morfgxatez . RSimInfoPtr ; numPoints = fromwksInfo -> nDataPoints ;
lastPoint = numPoints - 1 ; if ( t <= pTimeValues [ 0 ] ) { currTimeIndex = 0
; } else if ( t >= pTimeValues [ lastPoint ] ) { currTimeIndex = lastPoint -
1 ; } else { if ( t < pTimeValues [ currTimeIndex ] ) { while ( t <
pTimeValues [ currTimeIndex ] ) { currTimeIndex -- ; } } else { while ( t >=
pTimeValues [ currTimeIndex + 1 ] ) { currTimeIndex ++ ; } } } rtDW .
p0ybzcrnsf . PrevIndex = currTimeIndex ; { real_T t1 = pTimeValues [
currTimeIndex ] ; real_T t2 = pTimeValues [ currTimeIndex + 1 ] ; if ( t1 ==
t2 ) { if ( t < t1 ) { rtB . hzoqt0nc3o = pDataValues [ currTimeIndex ] ; }
else { rtB . hzoqt0nc3o = pDataValues [ currTimeIndex + 1 ] ; } } else {
real_T f1 = ( t2 - t ) / ( t2 - t1 ) ; real_T f2 = 1.0 - f1 ; real_T d1 ;
real_T d2 ; int_T TimeIndex = currTimeIndex ; int_T * zcTimeIndices = & rtDW
. mmq01gbox5 [ 0 ] ; int_T * zcTimeIndicesIdx = & rtDW . gml05gaiwr ; int_T
zcIdx = zcTimeIndices [ * zcTimeIndicesIdx ] ; int_T numZcTimes = 4 ; if ( *
zcTimeIndicesIdx < numZcTimes ) { if ( ssIsMajorTimeStep ( rtS ) ) { if ( t >
pTimeValues [ zcIdx ] ) { while ( * zcTimeIndicesIdx < ( numZcTimes - 1 ) &&
( t > pTimeValues [ zcIdx ] ) ) { ( * zcTimeIndicesIdx ) ++ ; zcIdx =
zcTimeIndices [ * zcTimeIndicesIdx ] ; } } } else { if ( t >= pTimeValues [
zcIdx ] && ( ssGetTimeOfLastOutput ( rtS ) < pTimeValues [ zcIdx ] ) ) { t2 =
pTimeValues [ zcIdx ] ; if ( zcIdx == 0 ) { TimeIndex = 0 ; t1 = t2 - 1 ; }
else { t1 = pTimeValues [ zcIdx - 1 ] ; TimeIndex = zcIdx - 1 ; } f1 = ( t2 -
t ) / ( t2 - t1 ) ; f2 = 1.0 - f1 ; } } } d1 = pDataValues [ TimeIndex ] ; d2
= pDataValues [ TimeIndex + 1 ] ; if ( zcIdx == 0 ) { d2 = d1 ; } rtB .
hzoqt0nc3o = ( real_T ) rtInterpolate ( d1 , d2 , f1 , f2 ) ; pDataValues +=
numPoints ; } } } { real_T * pDataValues = ( real_T * ) rtDW . d2vfv2ffdo .
DataPtr ; real_T * pTimeValues = ( real_T * ) rtDW . d2vfv2ffdo . TimePtr ;
int_T currTimeIndex = rtDW . dppeb2fcvo . PrevIndex ; real_T t =
ssGetTaskTime ( rtS , 0 ) ; int numPoints , lastPoint ; FWksInfo *
fromwksInfo = ( FWksInfo * ) rtDW . d2vfv2ffdo . RSimInfoPtr ; numPoints =
fromwksInfo -> nDataPoints ; lastPoint = numPoints - 1 ; if ( t <=
pTimeValues [ 0 ] ) { currTimeIndex = 0 ; } else if ( t >= pTimeValues [
lastPoint ] ) { currTimeIndex = lastPoint - 1 ; } else { if ( t < pTimeValues
[ currTimeIndex ] ) { while ( t < pTimeValues [ currTimeIndex ] ) {
currTimeIndex -- ; } } else { while ( t >= pTimeValues [ currTimeIndex + 1 ]
) { currTimeIndex ++ ; } } } rtDW . dppeb2fcvo . PrevIndex = currTimeIndex ;
{ real_T t1 = pTimeValues [ currTimeIndex ] ; real_T t2 = pTimeValues [
currTimeIndex + 1 ] ; if ( t1 == t2 ) { if ( t < t1 ) { rtB . otap4hfn5r =
pDataValues [ currTimeIndex ] ; } else { rtB . otap4hfn5r = pDataValues [
currTimeIndex + 1 ] ; } } else { real_T f1 = ( t2 - t ) / ( t2 - t1 ) ;
real_T f2 = 1.0 - f1 ; real_T d1 ; real_T d2 ; int_T TimeIndex =
currTimeIndex ; int_T * zcTimeIndices = & rtDW . p1tupjnl2m [ 0 ] ; int_T *
zcTimeIndicesIdx = & rtDW . pt0evqezqm ; int_T zcIdx = zcTimeIndices [ *
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
otap4hfn5r = ( real_T ) rtInterpolate ( d1 , d2 , f1 , f2 ) ; pDataValues +=
numPoints ; } } } { real_T * pDataValues = ( real_T * ) rtDW . ctabrgnsxy .
DataPtr ; real_T * pTimeValues = ( real_T * ) rtDW . ctabrgnsxy . TimePtr ;
int_T currTimeIndex = rtDW . jdmacw4ijl . PrevIndex ; real_T t =
ssGetTaskTime ( rtS , 0 ) ; int numPoints , lastPoint ; FWksInfo *
fromwksInfo = ( FWksInfo * ) rtDW . ctabrgnsxy . RSimInfoPtr ; numPoints =
fromwksInfo -> nDataPoints ; lastPoint = numPoints - 1 ; if ( t <=
pTimeValues [ 0 ] ) { currTimeIndex = 0 ; } else if ( t >= pTimeValues [
lastPoint ] ) { currTimeIndex = lastPoint - 1 ; } else { if ( t < pTimeValues
[ currTimeIndex ] ) { while ( t < pTimeValues [ currTimeIndex ] ) {
currTimeIndex -- ; } } else { while ( t >= pTimeValues [ currTimeIndex + 1 ]
) { currTimeIndex ++ ; } } } rtDW . jdmacw4ijl . PrevIndex = currTimeIndex ;
{ real_T t1 = pTimeValues [ currTimeIndex ] ; real_T t2 = pTimeValues [
currTimeIndex + 1 ] ; if ( t1 == t2 ) { if ( t < t1 ) { rtB . cfdpbmvjgv =
pDataValues [ currTimeIndex ] ; } else { rtB . cfdpbmvjgv = pDataValues [
currTimeIndex + 1 ] ; } } else { real_T f1 = ( t2 - t ) / ( t2 - t1 ) ;
real_T f2 = 1.0 - f1 ; real_T d1 ; real_T d2 ; int_T TimeIndex =
currTimeIndex ; int_T * zcTimeIndices = & rtDW . jt4kc1ii0s [ 0 ] ; int_T *
zcTimeIndicesIdx = & rtDW . hns1rud2yi ; int_T zcIdx = zcTimeIndices [ *
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
cfdpbmvjgv = ( real_T ) rtInterpolate ( d1 , d2 , f1 , f2 ) ; pDataValues +=
numPoints ; } } } { real_T * pDataValues = ( real_T * ) rtDW . nl25ijg5ia .
DataPtr ; real_T * pTimeValues = ( real_T * ) rtDW . nl25ijg5ia . TimePtr ;
int_T currTimeIndex = rtDW . i3b2ntrlap . PrevIndex ; real_T t =
ssGetTaskTime ( rtS , 0 ) ; int numPoints , lastPoint ; FWksInfo *
fromwksInfo = ( FWksInfo * ) rtDW . nl25ijg5ia . RSimInfoPtr ; numPoints =
fromwksInfo -> nDataPoints ; lastPoint = numPoints - 1 ; if ( t <=
pTimeValues [ 0 ] ) { currTimeIndex = 0 ; } else if ( t >= pTimeValues [
lastPoint ] ) { currTimeIndex = lastPoint - 1 ; } else { if ( t < pTimeValues
[ currTimeIndex ] ) { while ( t < pTimeValues [ currTimeIndex ] ) {
currTimeIndex -- ; } } else { while ( t >= pTimeValues [ currTimeIndex + 1 ]
) { currTimeIndex ++ ; } } } rtDW . i3b2ntrlap . PrevIndex = currTimeIndex ;
{ real_T t1 = pTimeValues [ currTimeIndex ] ; real_T t2 = pTimeValues [
currTimeIndex + 1 ] ; if ( t1 == t2 ) { if ( t < t1 ) { rtB . edtxh40dpx =
pDataValues [ currTimeIndex ] ; } else { rtB . edtxh40dpx = pDataValues [
currTimeIndex + 1 ] ; } } else { real_T f1 = ( t2 - t ) / ( t2 - t1 ) ;
real_T f2 = 1.0 - f1 ; real_T d1 ; real_T d2 ; int_T TimeIndex =
currTimeIndex ; int_T * zcTimeIndices = & rtDW . mek4mei4m5 [ 0 ] ; int_T *
zcTimeIndicesIdx = & rtDW . ppttotus2n ; int_T zcIdx = zcTimeIndices [ *
zcTimeIndicesIdx ] ; int_T numZcTimes = 3 ; if ( * zcTimeIndicesIdx <
numZcTimes ) { if ( ssIsMajorTimeStep ( rtS ) ) { if ( t > pTimeValues [
zcIdx ] ) { while ( * zcTimeIndicesIdx < ( numZcTimes - 1 ) && ( t >
pTimeValues [ zcIdx ] ) ) { ( * zcTimeIndicesIdx ) ++ ; zcIdx = zcTimeIndices
[ * zcTimeIndicesIdx ] ; } } } else { if ( t >= pTimeValues [ zcIdx ] && (
ssGetTimeOfLastOutput ( rtS ) < pTimeValues [ zcIdx ] ) ) { t2 = pTimeValues
[ zcIdx ] ; if ( zcIdx == 0 ) { TimeIndex = 0 ; t1 = t2 - 1 ; } else { t1 =
pTimeValues [ zcIdx - 1 ] ; TimeIndex = zcIdx - 1 ; } f1 = ( t2 - t ) / ( t2
- t1 ) ; f2 = 1.0 - f1 ; } } } d1 = pDataValues [ TimeIndex ] ; d2 =
pDataValues [ TimeIndex + 1 ] ; if ( zcIdx == 0 ) { d2 = d1 ; } rtB .
edtxh40dpx = ( real_T ) rtInterpolate ( d1 , d2 , f1 , f2 ) ; pDataValues +=
numPoints ; } } } rtB . gz1cngd0fd [ 0 ] = rtB . csh4yoneye [ 0 ] ; rtB .
gz1cngd0fd [ 1 ] = rtB . csh4yoneye [ 1 ] ; rtB . gz1cngd0fd [ 2 ] = rtB .
csh4yoneye [ 2 ] ; rtB . gz1cngd0fd [ 3 ] = rtB . csh4yoneye [ 3 ] ; rtB .
gz1cngd0fd [ 4 ] = rtB . j4tv1cqyj2 [ 0 ] + rtB . fgbq3jefrx ; rtB .
gz1cngd0fd [ 5 ] = rtB . j4tv1cqyj2 [ 1 ] + rtB . fgbq3jefrx ; rtB .
gz1cngd0fd [ 6 ] = rtB . j4tv1cqyj2 [ 2 ] + rtB . fgbq3jefrx ; rtB .
gz1cngd0fd [ 7 ] = rtB . j4tv1cqyj2 [ 3 ] + rtB . fgbq3jefrx ; if (
ssIsSampleHit ( rtS , 1 , 0 ) ) { for ( iy = 0 ; iy < 4 ; iy ++ ) { rtB .
oyi1nkeowt [ iy ] = 0.0 ; } rtB . exb2j2haif [ 0 ] = rtP .
Gain_Gain_l2aa1kgqdh * rtB . oyi1nkeowt [ 0 ] ; rtB . exb2j2haif [ 1 ] = rtP
. Gain_Gain_l2aa1kgqdh * rtB . oyi1nkeowt [ 1 ] ; rtB . exb2j2haif [ 2 ] =
rtP . Gain_Gain_l2aa1kgqdh * rtB . oyi1nkeowt [ 2 ] ; } if ( ssIsSampleHit (
rtS , 1 , 0 ) ) { } UNUSED_PARAMETER ( tid ) ; } void MdlOutputsTID3 ( int_T
tid ) { rtB . ajinddapxi [ 0 ] = rtP . Gain_Gain_gpep4bzie5 * 0.0 ; rtB .
ajinddapxi [ 1 ] = rtP . Gain_Gain_gpep4bzie5 * 0.0 ; rtB . ajinddapxi [ 2 ]
= rtP . Gain_Gain_gpep4bzie5 * 0.0 ; UNUSED_PARAMETER ( tid ) ; } void
MdlUpdate ( int_T tid ) { real_T * lastU ; if ( rtDW . cx2opkhkks == ( rtInf
) ) { rtDW . cx2opkhkks = ssGetT ( rtS ) ; lastU = & rtDW . pzg1f0zcsj ; }
else if ( rtDW . gs42o1safk == ( rtInf ) ) { rtDW . gs42o1safk = ssGetT ( rtS
) ; lastU = & rtDW . bthpfg5v51 ; } else if ( rtDW . cx2opkhkks < rtDW .
gs42o1safk ) { rtDW . cx2opkhkks = ssGetT ( rtS ) ; lastU = & rtDW .
pzg1f0zcsj ; } else { rtDW . gs42o1safk = ssGetT ( rtS ) ; lastU = & rtDW .
bthpfg5v51 ; } * lastU = rtB . gbemrwhpr5 ; if ( ssIsSampleHit ( rtS , 2 , 0
) ) { rtDW . gzecfrgu4x = ( rtP . randomAmpNoise - ( - rtP . randomAmpNoise )
) * rt_urand_Upu32_Yd_f_pw_snf ( & rtDW . hjtfckrdib ) + - rtP .
randomAmpNoise ; } UNUSED_PARAMETER ( tid ) ; } void MdlUpdateTID3 ( int_T
tid ) { UNUSED_PARAMETER ( tid ) ; } void MdlDerivatives ( void ) { int_T is
; int_T ci ; XDot * _rtXdot ; _rtXdot = ( ( XDot * ) ssGetdX ( rtS ) ) ;
_rtXdot -> ptbb520ltm [ 0 ] = 0.0 ; _rtXdot -> ptbb520ltm [ 0 ] += rtP .
Ps1_A [ 0 ] * rtX . ptbb520ltm [ 0 ] ; _rtXdot -> ptbb520ltm [ 1 ] = 0.0 ;
_rtXdot -> ptbb520ltm [ 0 ] += rtP . Ps1_A [ 1 ] * rtX . ptbb520ltm [ 1 ] ;
_rtXdot -> ptbb520ltm [ 1 ] += rtX . ptbb520ltm [ 0 ] ; _rtXdot -> ptbb520ltm
[ 0 ] += rtB . edtxh40dpx ; _rtXdot -> n0ofazoxbd [ 0 ] = 0.0 ; _rtXdot ->
n0ofazoxbd [ 0 ] += rtP . Ps2_A [ 0 ] * rtX . n0ofazoxbd [ 0 ] ; _rtXdot ->
n0ofazoxbd [ 1 ] = 0.0 ; _rtXdot -> n0ofazoxbd [ 0 ] += rtP . Ps2_A [ 1 ] *
rtX . n0ofazoxbd [ 1 ] ; _rtXdot -> n0ofazoxbd [ 1 ] += rtX . n0ofazoxbd [ 0
] ; _rtXdot -> n0ofazoxbd [ 0 ] += rtB . hzoqt0nc3o ; _rtXdot -> asbanfimve [
0 ] = 0.0 ; _rtXdot -> asbanfimve [ 0 ] += rtP . Ps3_A [ 0 ] * rtX .
asbanfimve [ 0 ] ; _rtXdot -> asbanfimve [ 1 ] = 0.0 ; _rtXdot -> asbanfimve
[ 0 ] += rtP . Ps3_A [ 1 ] * rtX . asbanfimve [ 1 ] ; _rtXdot -> asbanfimve [
1 ] += rtX . asbanfimve [ 0 ] ; _rtXdot -> asbanfimve [ 0 ] += rtB .
cfdpbmvjgv ; _rtXdot -> glltmukddh [ 0 ] = 0.0 ; _rtXdot -> glltmukddh [ 0 ]
+= rtP . Ps4_A [ 0 ] * rtX . glltmukddh [ 0 ] ; _rtXdot -> glltmukddh [ 1 ] =
0.0 ; _rtXdot -> glltmukddh [ 0 ] += rtP . Ps4_A [ 1 ] * rtX . glltmukddh [ 1
] ; _rtXdot -> glltmukddh [ 1 ] += rtX . glltmukddh [ 0 ] ; _rtXdot ->
glltmukddh [ 0 ] += rtB . otap4hfn5r ; for ( is = 0 ; is < 8 ; is ++ ) {
_rtXdot -> irr1ohciq3 [ is ] = 0.0 ; for ( ci = 0 ; ci < 8 ; ci ++ ) {
_rtXdot -> irr1ohciq3 [ is ] += rtP . A0 [ ( ci << 3 ) + is ] * rtX .
irr1ohciq3 [ ci ] ; } _rtXdot -> irr1ohciq3 [ is ] += rtP . B0 [ is ] * rtB .
csh4yoneye [ 0 ] ; _rtXdot -> irr1ohciq3 [ is ] += rtP . B0 [ 8 + is ] * rtB
. csh4yoneye [ 1 ] ; _rtXdot -> irr1ohciq3 [ is ] += rtP . B0 [ 16 + is ] *
rtB . csh4yoneye [ 2 ] ; _rtXdot -> irr1ohciq3 [ is ] += rtP . B0 [ 24 + is ]
* rtB . csh4yoneye [ 3 ] ; _rtXdot -> mcu5m2cl2g [ is ] = 0.0 ; for ( ci = 0
; ci < 8 ; ci ++ ) { _rtXdot -> mcu5m2cl2g [ is ] += rtP . KAoss [ ( ci << 3
) + is ] * rtX . mcu5m2cl2g [ ci ] ; } for ( ci = 0 ; ci < 8 ; ci ++ ) {
_rtXdot -> mcu5m2cl2g [ is ] += rtP . KBossw [ ( ci << 3 ) + is ] * rtB .
gz1cngd0fd [ ci ] ; } } _rtXdot -> odzwuprsis = 0.0 ; _rtXdot -> odzwuprsis
+= rtP . Ps1_A_cnuk0h4tar * rtX . odzwuprsis ; _rtXdot -> odzwuprsis += rtB .
d1v04lphp2 ; } void MdlProjection ( void ) { } void MdlZeroCrossings ( void )
{ ZCV * _rtZCSV ; _rtZCSV = ( ( ZCV * ) ssGetSolverZcSignalVector ( rtS ) ) ;
_rtZCSV -> nyfkdeqvul = rtB . mbsguhsyce - rtP . Switch_Threshold ; { const
double * timePtr = ( double * ) rtDW . pp4sameqxw . TimePtr ; int_T *
zcTimeIndices = & rtDW . mwqrktpz41 ; int_T zcTimeIndicesIdx = rtDW .
eknmvbtv5u ; ( ( ZCV * ) ssGetSolverZcSignalVector ( rtS ) ) -> myassw5y3y =
ssGetT ( rtS ) - timePtr [ zcTimeIndices [ zcTimeIndicesIdx ] ] ; } { const
double * timePtr = ( double * ) rtDW . pszu0ywji3 . TimePtr ; int_T *
zcTimeIndices = & rtDW . ndd51edmhg [ 0 ] ; int_T zcTimeIndicesIdx = rtDW .
jobrukao3e ; ( ( ZCV * ) ssGetSolverZcSignalVector ( rtS ) ) -> h2qpexisga =
ssGetT ( rtS ) - timePtr [ zcTimeIndices [ zcTimeIndicesIdx ] ] ; } { const
double * timePtr = ( double * ) rtDW . gmpueztb1y . TimePtr ; int_T *
zcTimeIndices = & rtDW . fpdlolyb1y ; int_T zcTimeIndicesIdx = rtDW .
ga0dnoquno ; ( ( ZCV * ) ssGetSolverZcSignalVector ( rtS ) ) -> lggrvczrm0 =
ssGetT ( rtS ) - timePtr [ zcTimeIndices [ zcTimeIndicesIdx ] ] ; } _rtZCSV
-> pxstdrscuo = rtB . bxeh32cm4i - rtP . Switch_Threshold_b2p15hzzi4 ;
_rtZCSV -> pc2sbokrzp [ 0 ] = rtB . hjr0spzzaf [ 0 ] - rtP .
Saturation_UpperSat ; _rtZCSV -> gmud1ydvyf [ 0 ] = rtB . hjr0spzzaf [ 0 ] -
rtP . Saturation_LowerSat ; _rtZCSV -> pc2sbokrzp [ 1 ] = rtB . hjr0spzzaf [
1 ] - rtP . Saturation_UpperSat ; _rtZCSV -> gmud1ydvyf [ 1 ] = rtB .
hjr0spzzaf [ 1 ] - rtP . Saturation_LowerSat ; _rtZCSV -> pc2sbokrzp [ 2 ] =
rtB . hjr0spzzaf [ 2 ] - rtP . Saturation_UpperSat ; _rtZCSV -> gmud1ydvyf [
2 ] = rtB . hjr0spzzaf [ 2 ] - rtP . Saturation_LowerSat ; _rtZCSV ->
pc2sbokrzp [ 3 ] = rtB . hjr0spzzaf [ 3 ] - rtP . Saturation_UpperSat ;
_rtZCSV -> gmud1ydvyf [ 3 ] = rtB . hjr0spzzaf [ 3 ] - rtP .
Saturation_LowerSat ; { const double * timePtr = ( double * ) rtDW .
mq2efjljlp . TimePtr ; int_T * zcTimeIndices = & rtDW . jisc0stmja ; int_T
zcTimeIndicesIdx = rtDW . bhjarlltx0 ; ( ( ZCV * ) ssGetSolverZcSignalVector
( rtS ) ) -> jufr2dujbf = ssGetT ( rtS ) - timePtr [ zcTimeIndices [
zcTimeIndicesIdx ] ] ; } { const double * timePtr = ( double * ) rtDW .
morfgxatez . TimePtr ; int_T * zcTimeIndices = & rtDW . mmq01gbox5 [ 0 ] ;
int_T zcTimeIndicesIdx = rtDW . gml05gaiwr ; ( ( ZCV * )
ssGetSolverZcSignalVector ( rtS ) ) -> fv4bnk3lzr = ssGetT ( rtS ) - timePtr
[ zcTimeIndices [ zcTimeIndicesIdx ] ] ; } { const double * timePtr = (
double * ) rtDW . d2vfv2ffdo . TimePtr ; int_T * zcTimeIndices = & rtDW .
p1tupjnl2m [ 0 ] ; int_T zcTimeIndicesIdx = rtDW . pt0evqezqm ; ( ( ZCV * )
ssGetSolverZcSignalVector ( rtS ) ) -> i5zlkpfdg1 = ssGetT ( rtS ) - timePtr
[ zcTimeIndices [ zcTimeIndicesIdx ] ] ; } { const double * timePtr = (
double * ) rtDW . ctabrgnsxy . TimePtr ; int_T * zcTimeIndices = & rtDW .
jt4kc1ii0s [ 0 ] ; int_T zcTimeIndicesIdx = rtDW . hns1rud2yi ; ( ( ZCV * )
ssGetSolverZcSignalVector ( rtS ) ) -> pc0ofz0fa0 = ssGetT ( rtS ) - timePtr
[ zcTimeIndices [ zcTimeIndicesIdx ] ] ; } { const double * timePtr = (
double * ) rtDW . nl25ijg5ia . TimePtr ; int_T * zcTimeIndices = & rtDW .
mek4mei4m5 [ 0 ] ; int_T zcTimeIndicesIdx = rtDW . ppttotus2n ; ( ( ZCV * )
ssGetSolverZcSignalVector ( rtS ) ) -> b2eqsgxxhs = ssGetT ( rtS ) - timePtr
[ zcTimeIndices [ zcTimeIndicesIdx ] ] ; } } void MdlTerminate ( void ) {
rt_FREE ( rtDW . pp4sameqxw . RSimInfoPtr ) ; rt_FREE ( rtDW . pszu0ywji3 .
RSimInfoPtr ) ; rt_FREE ( rtDW . gmpueztb1y . RSimInfoPtr ) ; rt_FREE ( rtDW
. mq2efjljlp . RSimInfoPtr ) ; rt_FREE ( rtDW . morfgxatez . RSimInfoPtr ) ;
rt_FREE ( rtDW . d2vfv2ffdo . RSimInfoPtr ) ; rt_FREE ( rtDW . ctabrgnsxy .
RSimInfoPtr ) ; rt_FREE ( rtDW . nl25ijg5ia . RSimInfoPtr ) ; { if (
rt_slioCatalogue ( ) != ( NULL ) ) { void * * slioCatalogueAddr =
rt_slioCatalogueAddr ( ) ; rtwCreateSigstreamSlioClient (
rt_GetOSigstreamManager ( ) , rtwGetPointerFromUniquePtr ( rt_slioCatalogue (
) ) ) ; rtwSaveDatasetsToMatFile ( rtwGetPointerFromUniquePtr (
rt_slioCatalogue ( ) ) , rt_GetMatSigstreamLoggingFileName ( ) ) ;
rtwOSigstreamManagerDestroyInstance ( rt_GetOSigstreamManager ( ) ) ;
rtwTerminateSlioCatalogue ( slioCatalogueAddr ) ; * slioCatalogueAddr = (
NULL ) ; } } } void MdlInitializeSizes ( void ) { ssSetNumContStates ( rtS ,
25 ) ; ssSetNumPeriodicContStates ( rtS , 0 ) ; ssSetNumY ( rtS , 0 ) ;
ssSetNumU ( rtS , 0 ) ; ssSetDirectFeedThrough ( rtS , 0 ) ;
ssSetNumSampleTimes ( rtS , 3 ) ; ssSetNumBlocks ( rtS , 71 ) ;
ssSetNumBlockIO ( rtS , 23 ) ; ssSetNumBlockParams ( rtS , 516 ) ; } void
MdlInitializeSampleTimes ( void ) { ssSetSampleTime ( rtS , 0 , 0.0 ) ;
ssSetSampleTime ( rtS , 1 , 0.0 ) ; ssSetSampleTime ( rtS , 2 , 0.1 ) ;
ssSetOffsetTime ( rtS , 0 , 0.0 ) ; ssSetOffsetTime ( rtS , 1 , 1.0 ) ;
ssSetOffsetTime ( rtS , 2 , 0.0 ) ; } void raccel_set_checksum ( SimStruct *
rtS ) { ssSetChecksumVal ( rtS , 0 , 4144187791U ) ; ssSetChecksumVal ( rtS ,
1 , 48234202U ) ; ssSetChecksumVal ( rtS , 2 , 2997501980U ) ;
ssSetChecksumVal ( rtS , 3 , 3888407977U ) ; } SimStruct *
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
LTRTenzo_InitializeDataMapInfo ( ) ; ssSetIsRapidAcceleratorActive ( rtS ,
true ) ; ssSetRootSS ( rtS , rtS ) ; ssSetVersion ( rtS ,
SIMSTRUCT_VERSION_LEVEL2 ) ; ssSetModelName ( rtS , "LTRTenzo" ) ; ssSetPath
( rtS , "LTRTenzo" ) ; ssSetTStart ( rtS , 0.0 ) ; ssSetTFinal ( rtS , 20.0 )
; { static RTWLogInfo rt_DataLoggingInfo ; rt_DataLoggingInfo .
loggingInterval = NULL ; ssSetRTWLogInfo ( rtS , & rt_DataLoggingInfo ) ; } {
{ static int_T rt_LoggedStateWidths [ ] = { 8 , 2 , 2 , 2 , 2 , 8 , 1 } ;
static int_T rt_LoggedStateNumDimensions [ ] = { 1 , 1 , 1 , 1 , 1 , 1 , 1 }
; static int_T rt_LoggedStateDimensions [ ] = { 8 , 2 , 2 , 2 , 2 , 8 , 1 } ;
static boolean_T rt_LoggedStateIsVarDims [ ] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 }
; static BuiltInDTypeId rt_LoggedStateDataTypeIds [ ] = { SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE } ;
static int_T rt_LoggedStateComplexSignals [ ] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 }
; static const char_T * rt_LoggedStateLabels [ ] = { "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" } ; static const char_T
* rt_LoggedStateBlockNames [ ] = { "LTRTenzo/processo/Processo1" ,
"LTRTenzo/Setpoints1/Ref hard\n/P(s)1" ,
"LTRTenzo/Setpoints1/Ref hard\n/P(s)2" ,
"LTRTenzo/Setpoints1/Ref hard\n/P(s)3" ,
"LTRTenzo/Setpoints1/Ref hard\n/P(s)4" , "LTRTenzo/\n\nkalman\n\n" ,
"LTRTenzo/Err Mes/Filtered random noise + sin/P(s)1" } ; static const char_T
* rt_LoggedStateNames [ ] = { "" , "" , "" , "" , "" , "" , "" } ; static
boolean_T rt_LoggedStateCrossMdlRef [ ] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 } ;
static RTWLogDataTypeConvert rt_RTWLogDataTypeConvert [ ] = { { 0 , SS_DOUBLE
, SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 ,
0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } } ; static RTWLogSignalInfo rt_LoggedStateSignalInfo =
{ 7 , rt_LoggedStateWidths , rt_LoggedStateNumDimensions ,
rt_LoggedStateDimensions , rt_LoggedStateIsVarDims , ( NULL ) , ( NULL ) ,
rt_LoggedStateDataTypeIds , rt_LoggedStateComplexSignals , ( NULL ) , {
rt_LoggedStateLabels } , ( NULL ) , ( NULL ) , ( NULL ) , {
rt_LoggedStateBlockNames } , { rt_LoggedStateNames } ,
rt_LoggedStateCrossMdlRef , rt_RTWLogDataTypeConvert } ; static void *
rt_LoggedStateSignalPtrs [ 7 ] ; rtliSetLogXSignalPtrs ( ssGetRTWLogInfo (
rtS ) , ( LogSignalPtrsType ) rt_LoggedStateSignalPtrs ) ;
rtliSetLogXSignalInfo ( ssGetRTWLogInfo ( rtS ) , & rt_LoggedStateSignalInfo
) ; rt_LoggedStateSignalPtrs [ 0 ] = ( void * ) & rtX . irr1ohciq3 [ 0 ] ;
rt_LoggedStateSignalPtrs [ 1 ] = ( void * ) & rtX . ptbb520ltm [ 0 ] ;
rt_LoggedStateSignalPtrs [ 2 ] = ( void * ) & rtX . n0ofazoxbd [ 0 ] ;
rt_LoggedStateSignalPtrs [ 3 ] = ( void * ) & rtX . asbanfimve [ 0 ] ;
rt_LoggedStateSignalPtrs [ 4 ] = ( void * ) & rtX . glltmukddh [ 0 ] ;
rt_LoggedStateSignalPtrs [ 5 ] = ( void * ) & rtX . mcu5m2cl2g [ 0 ] ;
rt_LoggedStateSignalPtrs [ 6 ] = ( void * ) & rtX . odzwuprsis ; }
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
ssSolverInfo slvrInfo ; static boolean_T contStatesDisabled [ 25 ] ; static
real_T absTol [ 25 ] = { 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6
, 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 ,
1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 ,
1.0E-6 , 1.0E-6 , 1.0E-6 } ; static uint8_T absTolControl [ 25 ] = { 0U , 0U
, 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U ,
0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U } ; static uint8_T zcAttributes [ 18 ]
= { ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP ) , (
ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) ,
( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , (
ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP )
, ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP ) } ; static
ssNonContDerivSigInfo nonContDerivSigInfo [ 1 ] = { { 1 * sizeof ( real_T ) ,
( char * ) ( & rtB . d4wqki33zw ) , ( NULL ) } } ; ssSetSolverRelTol ( rtS ,
0.001 ) ; ssSetStepSize ( rtS , 0.0 ) ; ssSetMinStepSize ( rtS , 0.0 ) ;
ssSetMaxNumMinSteps ( rtS , - 1 ) ; ssSetMinStepViolatedError ( rtS , 0 ) ;
ssSetMaxStepSize ( rtS , 0.0056609114067364841 ) ; ssSetSolverMaxOrder ( rtS
, - 1 ) ; ssSetSolverRefineFactor ( rtS , 1 ) ; ssSetOutputTimes ( rtS , (
NULL ) ) ; ssSetNumOutputTimes ( rtS , 0 ) ; ssSetOutputTimesOnly ( rtS , 0 )
; ssSetOutputTimesIndex ( rtS , 0 ) ; ssSetZCCacheNeedsReset ( rtS , 0 ) ;
ssSetDerivCacheNeedsReset ( rtS , 0 ) ; ssSetNumNonContDerivSigInfos ( rtS ,
1 ) ; ssSetNonContDerivSigInfos ( rtS , nonContDerivSigInfo ) ;
ssSetSolverInfo ( rtS , & slvrInfo ) ; ssSetSolverName ( rtS , "ode45" ) ;
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
rtS , zcAttributes ) ; ssSetSolverNumZcSignals ( rtS , 18 ) ;
ssSetModelZeroCrossings ( rtS , MdlZeroCrossings ) ;
ssSetSolverConsecutiveZCsStepRelTol ( rtS , 2.8421709430404007E-13 ) ;
ssSetSolverMaxConsecutiveZCs ( rtS , 1000 ) ; ssSetSolverConsecutiveZCsError
( rtS , 2 ) ; ssSetSolverMaskedZcDiagnostic ( rtS , 1 ) ;
ssSetSolverIgnoredZcDiagnostic ( rtS , 1 ) ; ssSetSolverMaxConsecutiveMinStep
( rtS , 1 ) ; ssSetSolverShapePreserveControl ( rtS , 2 ) ; ssSetTNextTid (
rtS , INT_MIN ) ; ssSetTNext ( rtS , rtMinusInf ) ; ssSetSolverNeedsReset (
rtS ) ; ssSetNumNonsampledZCs ( rtS , 18 ) ; ssSetContStateDisabled ( rtS ,
contStatesDisabled ) ; ssSetSolverMaxConsecutiveMinStep ( rtS , 1 ) ; }
ssSetChecksumVal ( rtS , 0 , 4144187791U ) ; ssSetChecksumVal ( rtS , 1 ,
48234202U ) ; ssSetChecksumVal ( rtS , 2 , 2997501980U ) ; ssSetChecksumVal (
rtS , 3 , 3888407977U ) ; { static const sysRanDType rtAlwaysEnabled =
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
3 ; void MdlOutputsParameterSampleTime ( int_T tid ) { MdlOutputsTID3 ( tid )
; }
