#include "__cf_LTRTenzo6Dof.h"
#include "rt_logging_mmi.h"
#include "LTRTenzo6Dof_capi.h"
#include <math.h>
#include "LTRTenzo6Dof.h"
#include "LTRTenzo6Dof_private.h"
#include "LTRTenzo6Dof_dt.h"
extern void * CreateDiagnosticAsVoidPtr_wrapper ( const char * id , int nargs
, ... ) ; RTWExtModeInfo * gblRTWExtModeInfo = NULL ; extern boolean_T
gblExtModeStartPktReceived ; void raccelForceExtModeShutdown ( ) { if ( !
gblExtModeStartPktReceived ) { boolean_T stopRequested = false ;
rtExtModeWaitForStartPkt ( gblRTWExtModeInfo , 3 , & stopRequested ) ; }
rtExtModeShutdown ( 3 ) ; } const int_T gblNumToFiles = 0 ; const int_T
gblNumFrFiles = 0 ; const int_T gblNumFrWksBlocks = 4 ;
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
"slprj//raccel//LTRTenzo6Dof//LTRTenzo6Dof_Jpattern.mat" ; const int_T
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
) { int_T is ; uint32_T tseed ; int32_T t ; real_T tmp ; rtX . hhto124xqa [ 0
] = 0.0 ; rtX . lsb1kf2p0a [ 0 ] = 0.0 ; rtX . del2gd1uu5 [ 0 ] = 0.0 ; rtX .
hhto124xqa [ 1 ] = 0.0 ; rtX . lsb1kf2p0a [ 1 ] = 0.0 ; rtX . del2gd1uu5 [ 1
] = 0.0 ; for ( is = 0 ; is < 8 ; is ++ ) { rtX . bhid251wmy [ is ] = rtP .
Processo1_X0 ; rtX . owky1kts1u [ is ] = rtP . kalman_X0 ; } rtX . mnq2gjnihw
= 0.0 ; tmp = muDoubleScalarFloor ( rtP . UniformRandomNumber_Seed ) ; if (
muDoubleScalarIsNaN ( tmp ) || muDoubleScalarIsInf ( tmp ) ) { tmp = 0.0 ; }
else { tmp = muDoubleScalarRem ( tmp , 4.294967296E+9 ) ; } tseed = tmp < 0.0
? ( uint32_T ) - ( int32_T ) ( uint32_T ) - tmp : ( uint32_T ) tmp ; is = (
int32_T ) ( tseed >> 16U ) ; t = ( int32_T ) ( tseed & 32768U ) ; tseed = ( (
( ( tseed - ( ( uint32_T ) is << 16U ) ) + t ) << 16U ) + t ) + is ; if (
tseed < 1U ) { tseed = 1144108930U ; } else { if ( tseed > 2147483646U ) {
tseed = 2147483646U ; } } rtDW . awp2bz51sp = tseed ; rtDW . dcnnnsw5q4 = (
rtP . randomAmpNoise - ( - rtP . randomAmpNoise ) ) *
rt_urand_Upu32_Yd_f_pw_snf ( & rtDW . awp2bz51sp ) + - rtP . randomAmpNoise ;
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
} } { static int_T rt_ToWksWidths [ ] = { 8 } ; static int_T
rt_ToWksNumDimensions [ ] = { 1 } ; static int_T rt_ToWksDimensions [ ] = { 8
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
static const char_T rt_ToWksBlockName [ ] =
"LTRTenzo6Dof/Optima Controller/To Workspace" ; rtDW . i4oe2vbkte .
LoggedData = rt_CreateStructLogVar ( ssGetRTWLogInfo ( rtS ) , ssGetTStart (
rtS ) , ssGetTFinal ( rtS ) , 0.0 , ( & ssGetErrorStatus ( rtS ) ) ,
"errSimLTR" , 0 , 0 , 1 , 0.0 , & rt_ToWksSignalInfo , rt_ToWksBlockName ) ;
if ( rtDW . i4oe2vbkte . LoggedData == ( NULL ) ) return ; } { FWksInfo *
fromwksInfo ; if ( ( fromwksInfo = ( FWksInfo * ) calloc ( 1 , sizeof (
FWksInfo ) ) ) == ( NULL ) ) { ssSetErrorStatus ( rtS ,
"from workspace STRING(Name) memory allocation error" ) ; } else {
fromwksInfo -> origWorkspaceVarName = "tuvar" ; fromwksInfo -> origDataTypeId
= 0 ; fromwksInfo -> origIsComplex = 0 ; fromwksInfo -> origWidth = 1 ;
fromwksInfo -> origElSize = sizeof ( real_T ) ; fromwksInfo -> data = ( void
* ) rtP . FromWs_Data0 ; fromwksInfo -> nDataPoints = 6 ; fromwksInfo -> time
= ( double * ) rtP . FromWs_Time0 ; rtDW . iqmfoh3acq . TimePtr = fromwksInfo
-> time ; rtDW . iqmfoh3acq . DataPtr = fromwksInfo -> data ; rtDW .
iqmfoh3acq . RSimInfoPtr = fromwksInfo ; } rtDW . axjurmlx3x . PrevIndex = 0
; { int_T * zcTimeIndices = & rtDW . iqdzkc4qec [ 0 ] ; const double *
timePoints = ( double * ) rtDW . iqmfoh3acq . TimePtr ; boolean_T
justHadZcTime = false ; int_T zcIdx = 0 ; int_T i ; for ( i = 0 ; i < 6 - 1 ;
i ++ ) { if ( ! justHadZcTime && timePoints [ i ] == timePoints [ i + 1 ] ) {
zcTimeIndices [ zcIdx ++ ] = i ; justHadZcTime = true ; } else {
justHadZcTime = false ; } } rtDW . brdcus5eot = 0 ; } } { FWksInfo *
fromwksInfo ; if ( ( fromwksInfo = ( FWksInfo * ) calloc ( 1 , sizeof (
FWksInfo ) ) ) == ( NULL ) ) { ssSetErrorStatus ( rtS ,
"from workspace STRING(Name) memory allocation error" ) ; } else {
fromwksInfo -> origWorkspaceVarName = "tuvar" ; fromwksInfo -> origDataTypeId
= 0 ; fromwksInfo -> origIsComplex = 0 ; fromwksInfo -> origWidth = 1 ;
fromwksInfo -> origElSize = sizeof ( real_T ) ; fromwksInfo -> data = ( void
* ) rtP . FromWs_Data0_cogeshtz5t ; fromwksInfo -> nDataPoints = 10 ;
fromwksInfo -> time = ( double * ) rtP . FromWs_Time0_nyx1zce205 ; rtDW .
luaideq0uj . TimePtr = fromwksInfo -> time ; rtDW . luaideq0uj . DataPtr =
fromwksInfo -> data ; rtDW . luaideq0uj . RSimInfoPtr = fromwksInfo ; } rtDW
. ch2s1cyhcx . PrevIndex = 0 ; { int_T * zcTimeIndices = & rtDW . kv51vtwzgi
[ 0 ] ; const double * timePoints = ( double * ) rtDW . luaideq0uj . TimePtr
; boolean_T justHadZcTime = false ; int_T zcIdx = 0 ; int_T i ; for ( i = 0 ;
i < 10 - 1 ; i ++ ) { if ( ! justHadZcTime && timePoints [ i ] == timePoints
[ i + 1 ] ) { zcTimeIndices [ zcIdx ++ ] = i ; justHadZcTime = true ; } else
{ justHadZcTime = false ; } } rtDW . oekfi2tozz = 0 ; } } { FWksInfo *
fromwksInfo ; if ( ( fromwksInfo = ( FWksInfo * ) calloc ( 1 , sizeof (
FWksInfo ) ) ) == ( NULL ) ) { ssSetErrorStatus ( rtS ,
"from workspace STRING(Name) memory allocation error" ) ; } else {
fromwksInfo -> origWorkspaceVarName = "tuvar" ; fromwksInfo -> origDataTypeId
= 0 ; fromwksInfo -> origIsComplex = 0 ; fromwksInfo -> origWidth = 1 ;
fromwksInfo -> origElSize = sizeof ( real_T ) ; fromwksInfo -> data = ( void
* ) rtP . FromWs_Data0_igdugvbssl ; fromwksInfo -> nDataPoints = 6 ;
fromwksInfo -> time = ( double * ) rtP . FromWs_Time0_lk1gq232s2 ; rtDW .
hkxntuepnx . TimePtr = fromwksInfo -> time ; rtDW . hkxntuepnx . DataPtr =
fromwksInfo -> data ; rtDW . hkxntuepnx . RSimInfoPtr = fromwksInfo ; } rtDW
. oh5roumt0o . PrevIndex = 0 ; { int_T * zcTimeIndices = & rtDW . a4hbqd2cbd
[ 0 ] ; const double * timePoints = ( double * ) rtDW . hkxntuepnx . TimePtr
; boolean_T justHadZcTime = false ; int_T zcIdx = 0 ; int_T i ; for ( i = 0 ;
i < 6 - 1 ; i ++ ) { if ( ! justHadZcTime && timePoints [ i ] == timePoints [
i + 1 ] ) { zcTimeIndices [ zcIdx ++ ] = i ; justHadZcTime = true ; } else {
justHadZcTime = false ; } } rtDW . hjtypnuisf = 0 ; } } { FWksInfo *
fromwksInfo ; if ( ( fromwksInfo = ( FWksInfo * ) calloc ( 1 , sizeof (
FWksInfo ) ) ) == ( NULL ) ) { ssSetErrorStatus ( rtS ,
"from workspace STRING(Name) memory allocation error" ) ; } else {
fromwksInfo -> origWorkspaceVarName = "tuvar" ; fromwksInfo -> origDataTypeId
= 0 ; fromwksInfo -> origIsComplex = 0 ; fromwksInfo -> origWidth = 1 ;
fromwksInfo -> origElSize = sizeof ( real_T ) ; fromwksInfo -> data = ( void
* ) rtP . FromWs_Data0_pirid4ox1s ; fromwksInfo -> nDataPoints = 10 ;
fromwksInfo -> time = ( double * ) rtP . FromWs_Time0_jv04arnttw ; rtDW .
anvo41nxpw . TimePtr = fromwksInfo -> time ; rtDW . anvo41nxpw . DataPtr =
fromwksInfo -> data ; rtDW . anvo41nxpw . RSimInfoPtr = fromwksInfo ; } rtDW
. lqstcqepka . PrevIndex = 0 ; { int_T * zcTimeIndices = & rtDW . elawudclil
[ 0 ] ; const double * timePoints = ( double * ) rtDW . anvo41nxpw . TimePtr
; boolean_T justHadZcTime = false ; int_T zcIdx = 0 ; int_T i ; for ( i = 0 ;
i < 10 - 1 ; i ++ ) { if ( ! justHadZcTime && timePoints [ i ] == timePoints
[ i + 1 ] ) { zcTimeIndices [ zcIdx ++ ] = i ; justHadZcTime = true ; } else
{ justHadZcTime = false ; } } rtDW . iabwzwhwhy = 0 ; } } MdlInitialize ( ) ;
{ bool externalInputIsInDatasetFormat = false ; void * pISigstreamManager =
rt_GetISigstreamManager ( ) ; rtwISigstreamManagerGetInputIsInDatasetFormat (
pISigstreamManager , & externalInputIsInDatasetFormat ) ; if (
externalInputIsInDatasetFormat ) { } } } void MdlOutputs ( int_T tid ) {
real_T iamvpolmsi ; real_T euubazox5n ; int_T iy ; int_T ci ; real_T
nt2my3gsda [ 4 ] ; real_T n4m0dezwe5 [ 8 ] ; real_T tmp ; rtB . oaz5l4crsf =
ssGetT ( rtS ) ; if ( ssIsMajorTimeStep ( rtS ) ) { rtDW . htlyqa0psc = ( rtB
. oaz5l4crsf >= rtP . Switch_Threshold ) ; } if ( rtDW . htlyqa0psc ) { rtB .
anbxmjzqcz = ( muDoubleScalarSin ( rtP . omegaPertOut * ssGetTaskTime ( rtS ,
0 ) + rtP . SinOut_Phase ) * rtP . amplitudePertOut + rtP . SinOut_Bias ) +
rtP . cstPertOut ; rtB . m4f4roaota = rtB . anbxmjzqcz ; } else { rtB .
m4f4roaota = rtP . Constant_Value ; } for ( iy = 0 ; iy < 4 ; iy ++ ) {
nt2my3gsda [ iy ] = 0.0 ; for ( ci = 0 ; ci < 8 ; ci ++ ) { nt2my3gsda [ iy ]
+= rtP . C0 [ ( ci << 2 ) + iy ] * rtX . bhid251wmy [ ci ] ; } } rtB .
l3ya5q3dsy [ 0 ] = rtB . m4f4roaota + nt2my3gsda [ 0 ] ; rtB . l3ya5q3dsy [ 1
] = rtB . m4f4roaota + nt2my3gsda [ 1 ] ; rtB . l3ya5q3dsy [ 2 ] = rtB .
m4f4roaota + nt2my3gsda [ 2 ] ; rtB . l3ya5q3dsy [ 3 ] = rtB . m4f4roaota +
nt2my3gsda [ 3 ] ; rtB . oxa3bf04z4 = 0.0 ; rtB . oxa3bf04z4 += rtP . Ps4_C [
0 ] * rtX . hhto124xqa [ 0 ] ; rtB . oxa3bf04z4 += rtP . Ps4_C [ 1 ] * rtX .
hhto124xqa [ 1 ] ; rtB . c1do0agcsm = 0.0 ; rtB . c1do0agcsm += rtP . Ps5_C [
0 ] * rtX . lsb1kf2p0a [ 0 ] ; rtB . c1do0agcsm += rtP . Ps5_C [ 1 ] * rtX .
lsb1kf2p0a [ 1 ] ; rtB . aolioyabae = 0.0 ; rtB . aolioyabae += rtP . Ps1_C [
0 ] * rtX . del2gd1uu5 [ 0 ] ; rtB . aolioyabae += rtP . Ps1_C [ 1 ] * rtX .
del2gd1uu5 [ 1 ] ; for ( iy = 0 ; iy < 8 ; iy ++ ) { n4m0dezwe5 [ iy ] = 0.0
; for ( ci = 0 ; ci < 8 ; ci ++ ) { n4m0dezwe5 [ iy ] += rtP . KCoss [ ( ci
<< 3 ) + iy ] * rtX . owky1kts1u [ ci ] ; } } rtB . mg0az2nzgj = ssGetT ( rtS
) ; if ( ssIsMajorTimeStep ( rtS ) ) { rtDW . baoant1mbx = ( rtB . mg0az2nzgj
>= rtP . Switch_Threshold_b2p15hzzi4 ) ; } if ( rtDW . baoant1mbx ) { rtB .
klytvkrzsu = ( muDoubleScalarSin ( rtP . omegaPertIN * ssGetTaskTime ( rtS ,
0 ) + rtP . SineIn_Phase ) * rtP . amplitudePertIn + rtP . SineIn_Bias ) +
rtP . cstPertIn ; rtB . dt0bxeukl1 = rtB . klytvkrzsu ; } else { rtB .
dt0bxeukl1 = rtP . Constant_Value_gakmm5of0a ; } if ( rtP .
ManualSwitch1_CurrentSetting == 1 ) { tmp = rtP . Constant1_Value ; } else {
tmp = rtP . Ps1_C_gfroigzrf4 * rtX . mnq2gjnihw ; } rtB . m5z5sj11wq = rtP .
Gain_Gain * tmp ; if ( ssIsSampleHit ( rtS , 2 , 0 ) ) { rtB . px0r25lgtw =
rtDW . dcnnnsw5q4 ; } rtB . lk5cyp2tjd = ( muDoubleScalarSin ( rtP .
omegaNoise * ssGetTaskTime ( rtS , 0 ) + rtP . u2sen05t_Phase ) * rtP .
amplitudeNoise + rtP . u2sen05t_Bias ) + rtB . px0r25lgtw ; rtB . exersch3pk
[ 0 ] = rtB . aolioyabae - n4m0dezwe5 [ 0 ] ; rtB . exersch3pk [ 1 ] = rtB .
kk4pom5z0m - n4m0dezwe5 [ 1 ] ; rtB . exersch3pk [ 2 ] = rtB . oxa3bf04z4 -
n4m0dezwe5 [ 2 ] ; rtB . exersch3pk [ 3 ] = rtB . c1do0agcsm - n4m0dezwe5 [ 3
] ; rtB . exersch3pk [ 4 ] = rtB . kk4pom5z0m - n4m0dezwe5 [ 4 ] ; rtB .
exersch3pk [ 5 ] = rtB . kk4pom5z0m - n4m0dezwe5 [ 5 ] ; rtB . exersch3pk [ 6
] = rtB . kk4pom5z0m - n4m0dezwe5 [ 6 ] ; rtB . exersch3pk [ 7 ] = rtB .
kk4pom5z0m - n4m0dezwe5 [ 7 ] ; if ( ssGetLogOutput ( rtS ) ) { { double
locTime = ssGetTaskTime ( rtS , 0 ) ; ; if ( rtwTimeInLoggingInterval (
rtliGetLoggingInterval ( ssGetRootSS ( rtS ) -> mdlInfo -> rtwLogInfo ) ,
locTime ) ) { rt_UpdateStructLogVar ( ( StructLogVar * ) rtDW . i4oe2vbkte .
LoggedData , ( NULL ) , & rtB . exersch3pk [ 0 ] ) ; } } } for ( iy = 0 ; iy
< 4 ; iy ++ ) { nt2my3gsda [ iy ] = 0.0 ; for ( ci = 0 ; ci < 8 ; ci ++ ) {
nt2my3gsda [ iy ] += rtP . Kopt [ ( ci << 2 ) + iy ] * rtB . exersch3pk [ ci
] ; } } { real_T * pDataValues = ( real_T * ) rtDW . iqmfoh3acq . DataPtr ;
real_T * pTimeValues = ( real_T * ) rtDW . iqmfoh3acq . TimePtr ; int_T
currTimeIndex = rtDW . axjurmlx3x . PrevIndex ; real_T t = ssGetTaskTime (
rtS , 0 ) ; int numPoints , lastPoint ; FWksInfo * fromwksInfo = ( FWksInfo *
) rtDW . iqmfoh3acq . RSimInfoPtr ; numPoints = fromwksInfo -> nDataPoints ;
lastPoint = numPoints - 1 ; if ( t <= pTimeValues [ 0 ] ) { currTimeIndex = 0
; } else if ( t >= pTimeValues [ lastPoint ] ) { currTimeIndex = lastPoint -
1 ; } else { if ( t < pTimeValues [ currTimeIndex ] ) { while ( t <
pTimeValues [ currTimeIndex ] ) { currTimeIndex -- ; } } else { while ( t >=
pTimeValues [ currTimeIndex + 1 ] ) { currTimeIndex ++ ; } } } rtDW .
axjurmlx3x . PrevIndex = currTimeIndex ; { real_T t1 = pTimeValues [
currTimeIndex ] ; real_T t2 = pTimeValues [ currTimeIndex + 1 ] ; if ( t1 ==
t2 ) { if ( t < t1 ) { iamvpolmsi = pDataValues [ currTimeIndex ] ; } else {
iamvpolmsi = pDataValues [ currTimeIndex + 1 ] ; } } else { real_T f1 = ( t2
- t ) / ( t2 - t1 ) ; real_T f2 = 1.0 - f1 ; real_T d1 ; real_T d2 ; int_T
TimeIndex = currTimeIndex ; int_T * zcTimeIndices = & rtDW . iqdzkc4qec [ 0 ]
; int_T * zcTimeIndicesIdx = & rtDW . brdcus5eot ; int_T zcIdx =
zcTimeIndices [ * zcTimeIndicesIdx ] ; int_T numZcTimes = 2 ; if ( *
zcTimeIndicesIdx < numZcTimes ) { if ( ssIsMajorTimeStep ( rtS ) ) { if ( t >
pTimeValues [ zcIdx ] ) { while ( * zcTimeIndicesIdx < ( numZcTimes - 1 ) &&
( t > pTimeValues [ zcIdx ] ) ) { ( * zcTimeIndicesIdx ) ++ ; zcIdx =
zcTimeIndices [ * zcTimeIndicesIdx ] ; } } } else { if ( t >= pTimeValues [
zcIdx ] && ( ssGetTimeOfLastOutput ( rtS ) < pTimeValues [ zcIdx ] ) ) { t2 =
pTimeValues [ zcIdx ] ; if ( zcIdx == 0 ) { TimeIndex = 0 ; t1 = t2 - 1 ; }
else { t1 = pTimeValues [ zcIdx - 1 ] ; TimeIndex = zcIdx - 1 ; } f1 = ( t2 -
t ) / ( t2 - t1 ) ; f2 = 1.0 - f1 ; } } } d1 = pDataValues [ TimeIndex ] ; d2
= pDataValues [ TimeIndex + 1 ] ; if ( zcIdx == 0 ) { d2 = d1 ; } iamvpolmsi
= ( real_T ) rtInterpolate ( d1 , d2 , f1 , f2 ) ; pDataValues += numPoints ;
} } } { real_T * pDataValues = ( real_T * ) rtDW . luaideq0uj . DataPtr ;
real_T * pTimeValues = ( real_T * ) rtDW . luaideq0uj . TimePtr ; int_T
currTimeIndex = rtDW . ch2s1cyhcx . PrevIndex ; real_T t = ssGetTaskTime (
rtS , 0 ) ; int numPoints , lastPoint ; FWksInfo * fromwksInfo = ( FWksInfo *
) rtDW . luaideq0uj . RSimInfoPtr ; numPoints = fromwksInfo -> nDataPoints ;
lastPoint = numPoints - 1 ; if ( t <= pTimeValues [ 0 ] ) { currTimeIndex = 0
; } else if ( t >= pTimeValues [ lastPoint ] ) { currTimeIndex = lastPoint -
1 ; } else { if ( t < pTimeValues [ currTimeIndex ] ) { while ( t <
pTimeValues [ currTimeIndex ] ) { currTimeIndex -- ; } } else { while ( t >=
pTimeValues [ currTimeIndex + 1 ] ) { currTimeIndex ++ ; } } } rtDW .
ch2s1cyhcx . PrevIndex = currTimeIndex ; { real_T t1 = pTimeValues [
currTimeIndex ] ; real_T t2 = pTimeValues [ currTimeIndex + 1 ] ; if ( t1 ==
t2 ) { if ( t < t1 ) { euubazox5n = pDataValues [ currTimeIndex ] ; } else {
euubazox5n = pDataValues [ currTimeIndex + 1 ] ; } } else { real_T f1 = ( t2
- t ) / ( t2 - t1 ) ; real_T f2 = 1.0 - f1 ; real_T d1 ; real_T d2 ; int_T
TimeIndex = currTimeIndex ; int_T * zcTimeIndices = & rtDW . kv51vtwzgi [ 0 ]
; int_T * zcTimeIndicesIdx = & rtDW . oekfi2tozz ; int_T zcIdx =
zcTimeIndices [ * zcTimeIndicesIdx ] ; int_T numZcTimes = 4 ; if ( *
zcTimeIndicesIdx < numZcTimes ) { if ( ssIsMajorTimeStep ( rtS ) ) { if ( t >
pTimeValues [ zcIdx ] ) { while ( * zcTimeIndicesIdx < ( numZcTimes - 1 ) &&
( t > pTimeValues [ zcIdx ] ) ) { ( * zcTimeIndicesIdx ) ++ ; zcIdx =
zcTimeIndices [ * zcTimeIndicesIdx ] ; } } } else { if ( t >= pTimeValues [
zcIdx ] && ( ssGetTimeOfLastOutput ( rtS ) < pTimeValues [ zcIdx ] ) ) { t2 =
pTimeValues [ zcIdx ] ; if ( zcIdx == 0 ) { TimeIndex = 0 ; t1 = t2 - 1 ; }
else { t1 = pTimeValues [ zcIdx - 1 ] ; TimeIndex = zcIdx - 1 ; } f1 = ( t2 -
t ) / ( t2 - t1 ) ; f2 = 1.0 - f1 ; } } } d1 = pDataValues [ TimeIndex ] ; d2
= pDataValues [ TimeIndex + 1 ] ; if ( zcIdx == 0 ) { d2 = d1 ; } euubazox5n
= ( real_T ) rtInterpolate ( d1 , d2 , f1 , f2 ) ; pDataValues += numPoints ;
} } } rtB . fxeigzy1kf = iamvpolmsi + euubazox5n ; { real_T * pDataValues = (
real_T * ) rtDW . hkxntuepnx . DataPtr ; real_T * pTimeValues = ( real_T * )
rtDW . hkxntuepnx . TimePtr ; int_T currTimeIndex = rtDW . oh5roumt0o .
PrevIndex ; real_T t = ssGetTaskTime ( rtS , 0 ) ; int numPoints , lastPoint
; FWksInfo * fromwksInfo = ( FWksInfo * ) rtDW . hkxntuepnx . RSimInfoPtr ;
numPoints = fromwksInfo -> nDataPoints ; lastPoint = numPoints - 1 ; if ( t
<= pTimeValues [ 0 ] ) { currTimeIndex = 0 ; } else if ( t >= pTimeValues [
lastPoint ] ) { currTimeIndex = lastPoint - 1 ; } else { if ( t < pTimeValues
[ currTimeIndex ] ) { while ( t < pTimeValues [ currTimeIndex ] ) {
currTimeIndex -- ; } } else { while ( t >= pTimeValues [ currTimeIndex + 1 ]
) { currTimeIndex ++ ; } } } rtDW . oh5roumt0o . PrevIndex = currTimeIndex ;
{ real_T t1 = pTimeValues [ currTimeIndex ] ; real_T t2 = pTimeValues [
currTimeIndex + 1 ] ; if ( t1 == t2 ) { if ( t < t1 ) { rtB . fwztmrgmqg =
pDataValues [ currTimeIndex ] ; } else { rtB . fwztmrgmqg = pDataValues [
currTimeIndex + 1 ] ; } } else { real_T f1 = ( t2 - t ) / ( t2 - t1 ) ;
real_T f2 = 1.0 - f1 ; real_T d1 ; real_T d2 ; int_T TimeIndex =
currTimeIndex ; int_T * zcTimeIndices = & rtDW . a4hbqd2cbd [ 0 ] ; int_T *
zcTimeIndicesIdx = & rtDW . hjtypnuisf ; int_T zcIdx = zcTimeIndices [ *
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
fwztmrgmqg = ( real_T ) rtInterpolate ( d1 , d2 , f1 , f2 ) ; pDataValues +=
numPoints ; } } } { real_T * pDataValues = ( real_T * ) rtDW . anvo41nxpw .
DataPtr ; real_T * pTimeValues = ( real_T * ) rtDW . anvo41nxpw . TimePtr ;
int_T currTimeIndex = rtDW . lqstcqepka . PrevIndex ; real_T t =
ssGetTaskTime ( rtS , 0 ) ; int numPoints , lastPoint ; FWksInfo *
fromwksInfo = ( FWksInfo * ) rtDW . anvo41nxpw . RSimInfoPtr ; numPoints =
fromwksInfo -> nDataPoints ; lastPoint = numPoints - 1 ; if ( t <=
pTimeValues [ 0 ] ) { currTimeIndex = 0 ; } else if ( t >= pTimeValues [
lastPoint ] ) { currTimeIndex = lastPoint - 1 ; } else { if ( t < pTimeValues
[ currTimeIndex ] ) { while ( t < pTimeValues [ currTimeIndex ] ) {
currTimeIndex -- ; } } else { while ( t >= pTimeValues [ currTimeIndex + 1 ]
) { currTimeIndex ++ ; } } } rtDW . lqstcqepka . PrevIndex = currTimeIndex ;
{ real_T t1 = pTimeValues [ currTimeIndex ] ; real_T t2 = pTimeValues [
currTimeIndex + 1 ] ; if ( t1 == t2 ) { if ( t < t1 ) { rtB . k3xhv4ixsk =
pDataValues [ currTimeIndex ] ; } else { rtB . k3xhv4ixsk = pDataValues [
currTimeIndex + 1 ] ; } } else { real_T f1 = ( t2 - t ) / ( t2 - t1 ) ;
real_T f2 = 1.0 - f1 ; real_T d1 ; real_T d2 ; int_T TimeIndex =
currTimeIndex ; int_T * zcTimeIndices = & rtDW . elawudclil [ 0 ] ; int_T *
zcTimeIndicesIdx = & rtDW . iabwzwhwhy ; int_T zcIdx = zcTimeIndices [ *
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
k3xhv4ixsk = ( real_T ) rtInterpolate ( d1 , d2 , f1 , f2 ) ; pDataValues +=
numPoints ; } } } rtB . ajlqbn3dtj [ 0 ] = rtB . dt0bxeukl1 + nt2my3gsda [ 0
] ; rtB . ajlqbn3dtj [ 1 ] = rtB . dt0bxeukl1 + nt2my3gsda [ 1 ] ; rtB .
ajlqbn3dtj [ 2 ] = rtB . dt0bxeukl1 + nt2my3gsda [ 2 ] ; rtB . ajlqbn3dtj [ 3
] = rtB . dt0bxeukl1 + nt2my3gsda [ 3 ] ; rtB . obr0rt4zff [ 0 ] = rtB .
ajlqbn3dtj [ 0 ] ; rtB . obr0rt4zff [ 1 ] = rtB . ajlqbn3dtj [ 1 ] ; rtB .
obr0rt4zff [ 2 ] = rtB . ajlqbn3dtj [ 2 ] ; rtB . obr0rt4zff [ 3 ] = rtB .
ajlqbn3dtj [ 3 ] ; rtB . obr0rt4zff [ 4 ] = rtB . l3ya5q3dsy [ 0 ] + rtB .
m5z5sj11wq ; rtB . obr0rt4zff [ 5 ] = rtB . l3ya5q3dsy [ 1 ] + rtB .
m5z5sj11wq ; rtB . obr0rt4zff [ 6 ] = rtB . l3ya5q3dsy [ 2 ] + rtB .
m5z5sj11wq ; rtB . obr0rt4zff [ 7 ] = rtB . l3ya5q3dsy [ 3 ] + rtB .
m5z5sj11wq ; if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { for ( iy = 0 ; iy < 4 ;
iy ++ ) { rtB . fezrnkyxs4 [ iy ] = 0.0 ; } rtB . odgn1acl0s [ 0 ] = rtP .
Gain_Gain_l2aa1kgqdh * rtB . fezrnkyxs4 [ 0 ] ; rtB . odgn1acl0s [ 1 ] = rtP
. Gain_Gain_l2aa1kgqdh * rtB . fezrnkyxs4 [ 1 ] ; rtB . odgn1acl0s [ 2 ] =
rtP . Gain_Gain_l2aa1kgqdh * rtB . fezrnkyxs4 [ 2 ] ; } if ( ssIsSampleHit (
rtS , 1 , 0 ) ) { } if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { } UNUSED_PARAMETER
( tid ) ; } void MdlOutputsTID3 ( int_T tid ) { rtB . kk4pom5z0m = rtP .
Constant2_Value ; rtB . gjftqhkwh2 [ 0 ] = rtP . Gain_Gain_gpep4bzie5 * 0.0 ;
rtB . gjftqhkwh2 [ 1 ] = rtP . Gain_Gain_gpep4bzie5 * 0.0 ; rtB . gjftqhkwh2
[ 2 ] = rtP . Gain_Gain_gpep4bzie5 * 0.0 ; UNUSED_PARAMETER ( tid ) ; } void
MdlUpdate ( int_T tid ) { if ( ssIsSampleHit ( rtS , 2 , 0 ) ) { rtDW .
dcnnnsw5q4 = ( rtP . randomAmpNoise - ( - rtP . randomAmpNoise ) ) *
rt_urand_Upu32_Yd_f_pw_snf ( & rtDW . awp2bz51sp ) + - rtP . randomAmpNoise ;
} UNUSED_PARAMETER ( tid ) ; } void MdlUpdateTID3 ( int_T tid ) {
UNUSED_PARAMETER ( tid ) ; } void MdlDerivatives ( void ) { int_T is ; int_T
ci ; XDot * _rtXdot ; _rtXdot = ( ( XDot * ) ssGetdX ( rtS ) ) ; _rtXdot ->
hhto124xqa [ 0 ] = 0.0 ; _rtXdot -> hhto124xqa [ 0 ] += rtP . Ps4_A [ 0 ] *
rtX . hhto124xqa [ 0 ] ; _rtXdot -> hhto124xqa [ 1 ] = 0.0 ; _rtXdot ->
hhto124xqa [ 0 ] += rtP . Ps4_A [ 1 ] * rtX . hhto124xqa [ 1 ] ; _rtXdot ->
hhto124xqa [ 1 ] += rtX . hhto124xqa [ 0 ] ; _rtXdot -> hhto124xqa [ 0 ] +=
rtB . fxeigzy1kf ; _rtXdot -> lsb1kf2p0a [ 0 ] = 0.0 ; _rtXdot -> lsb1kf2p0a
[ 0 ] += rtP . Ps5_A [ 0 ] * rtX . lsb1kf2p0a [ 0 ] ; _rtXdot -> lsb1kf2p0a [
1 ] = 0.0 ; _rtXdot -> lsb1kf2p0a [ 0 ] += rtP . Ps5_A [ 1 ] * rtX .
lsb1kf2p0a [ 1 ] ; _rtXdot -> lsb1kf2p0a [ 1 ] += rtX . lsb1kf2p0a [ 0 ] ;
_rtXdot -> lsb1kf2p0a [ 0 ] += rtB . fwztmrgmqg ; _rtXdot -> del2gd1uu5 [ 0 ]
= 0.0 ; _rtXdot -> del2gd1uu5 [ 0 ] += rtP . Ps1_A [ 0 ] * rtX . del2gd1uu5 [
0 ] ; _rtXdot -> del2gd1uu5 [ 1 ] = 0.0 ; _rtXdot -> del2gd1uu5 [ 0 ] += rtP
. Ps1_A [ 1 ] * rtX . del2gd1uu5 [ 1 ] ; _rtXdot -> del2gd1uu5 [ 1 ] += rtX .
del2gd1uu5 [ 0 ] ; _rtXdot -> del2gd1uu5 [ 0 ] += rtB . k3xhv4ixsk ; for ( is
= 0 ; is < 8 ; is ++ ) { _rtXdot -> bhid251wmy [ is ] = 0.0 ; for ( ci = 0 ;
ci < 8 ; ci ++ ) { _rtXdot -> bhid251wmy [ is ] += rtP . A0 [ ( ci << 3 ) +
is ] * rtX . bhid251wmy [ ci ] ; } _rtXdot -> bhid251wmy [ is ] += rtP . B0 [
is ] * rtB . ajlqbn3dtj [ 0 ] ; _rtXdot -> bhid251wmy [ is ] += rtP . B0 [ 8
+ is ] * rtB . ajlqbn3dtj [ 1 ] ; _rtXdot -> bhid251wmy [ is ] += rtP . B0 [
16 + is ] * rtB . ajlqbn3dtj [ 2 ] ; _rtXdot -> bhid251wmy [ is ] += rtP . B0
[ 24 + is ] * rtB . ajlqbn3dtj [ 3 ] ; _rtXdot -> owky1kts1u [ is ] = 0.0 ;
for ( ci = 0 ; ci < 8 ; ci ++ ) { _rtXdot -> owky1kts1u [ is ] += rtP . KAoss
[ ( ci << 3 ) + is ] * rtX . owky1kts1u [ ci ] ; } for ( ci = 0 ; ci < 8 ; ci
++ ) { _rtXdot -> owky1kts1u [ is ] += rtP . KBossw [ ( ci << 3 ) + is ] *
rtB . obr0rt4zff [ ci ] ; } } _rtXdot -> mnq2gjnihw = 0.0 ; _rtXdot ->
mnq2gjnihw += rtP . Ps1_A_cnuk0h4tar * rtX . mnq2gjnihw ; _rtXdot ->
mnq2gjnihw += rtB . lk5cyp2tjd ; } void MdlProjection ( void ) { } void
MdlZeroCrossings ( void ) { ZCV * _rtZCSV ; _rtZCSV = ( ( ZCV * )
ssGetSolverZcSignalVector ( rtS ) ) ; _rtZCSV -> eog5tsvrmm = rtB .
oaz5l4crsf - rtP . Switch_Threshold ; _rtZCSV -> oeq2vseenb = rtB .
mg0az2nzgj - rtP . Switch_Threshold_b2p15hzzi4 ; { const double * timePtr = (
double * ) rtDW . iqmfoh3acq . TimePtr ; int_T * zcTimeIndices = & rtDW .
iqdzkc4qec [ 0 ] ; int_T zcTimeIndicesIdx = rtDW . brdcus5eot ; ( ( ZCV * )
ssGetSolverZcSignalVector ( rtS ) ) -> lh4evpqhkw = ssGetT ( rtS ) - timePtr
[ zcTimeIndices [ zcTimeIndicesIdx ] ] ; } { const double * timePtr = (
double * ) rtDW . luaideq0uj . TimePtr ; int_T * zcTimeIndices = & rtDW .
kv51vtwzgi [ 0 ] ; int_T zcTimeIndicesIdx = rtDW . oekfi2tozz ; ( ( ZCV * )
ssGetSolverZcSignalVector ( rtS ) ) -> kouc5v3spc = ssGetT ( rtS ) - timePtr
[ zcTimeIndices [ zcTimeIndicesIdx ] ] ; } { const double * timePtr = (
double * ) rtDW . hkxntuepnx . TimePtr ; int_T * zcTimeIndices = & rtDW .
a4hbqd2cbd [ 0 ] ; int_T zcTimeIndicesIdx = rtDW . hjtypnuisf ; ( ( ZCV * )
ssGetSolverZcSignalVector ( rtS ) ) -> immtuxkjm4 = ssGetT ( rtS ) - timePtr
[ zcTimeIndices [ zcTimeIndicesIdx ] ] ; } { const double * timePtr = (
double * ) rtDW . anvo41nxpw . TimePtr ; int_T * zcTimeIndices = & rtDW .
elawudclil [ 0 ] ; int_T zcTimeIndicesIdx = rtDW . iabwzwhwhy ; ( ( ZCV * )
ssGetSolverZcSignalVector ( rtS ) ) -> iduqvk4ja1 = ssGetT ( rtS ) - timePtr
[ zcTimeIndices [ zcTimeIndicesIdx ] ] ; } } void MdlTerminate ( void ) {
rt_FREE ( rtDW . iqmfoh3acq . RSimInfoPtr ) ; rt_FREE ( rtDW . luaideq0uj .
RSimInfoPtr ) ; rt_FREE ( rtDW . hkxntuepnx . RSimInfoPtr ) ; rt_FREE ( rtDW
. anvo41nxpw . RSimInfoPtr ) ; { if ( rt_slioCatalogue ( ) != ( NULL ) ) {
void * * slioCatalogueAddr = rt_slioCatalogueAddr ( ) ;
rtwCreateSigstreamSlioClient ( rt_GetOSigstreamManager ( ) ,
rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( ) ) ) ;
rtwSaveDatasetsToMatFile ( rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( )
) , rt_GetMatSigstreamLoggingFileName ( ) ) ;
rtwOSigstreamManagerDestroyInstance ( rt_GetOSigstreamManager ( ) ) ;
rtwTerminateSlioCatalogue ( slioCatalogueAddr ) ; * slioCatalogueAddr = (
NULL ) ; } } } void MdlInitializeSizes ( void ) { ssSetNumContStates ( rtS ,
23 ) ; ssSetNumPeriodicContStates ( rtS , 0 ) ; ssSetNumY ( rtS , 0 ) ;
ssSetNumU ( rtS , 0 ) ; ssSetDirectFeedThrough ( rtS , 0 ) ;
ssSetNumSampleTimes ( rtS , 3 ) ; ssSetNumBlocks ( rtS , 62 ) ;
ssSetNumBlockIO ( rtS , 23 ) ; ssSetNumBlockParams ( rtS , 458 ) ; } void
MdlInitializeSampleTimes ( void ) { ssSetSampleTime ( rtS , 0 , 0.0 ) ;
ssSetSampleTime ( rtS , 1 , 0.0 ) ; ssSetSampleTime ( rtS , 2 , 0.1 ) ;
ssSetOffsetTime ( rtS , 0 , 0.0 ) ; ssSetOffsetTime ( rtS , 1 , 1.0 ) ;
ssSetOffsetTime ( rtS , 2 , 0.0 ) ; } void raccel_set_checksum ( SimStruct *
rtS ) { ssSetChecksumVal ( rtS , 0 , 3634810543U ) ; ssSetChecksumVal ( rtS ,
1 , 2521803458U ) ; ssSetChecksumVal ( rtS , 2 , 2534976825U ) ;
ssSetChecksumVal ( rtS , 3 , 85240381U ) ; } SimStruct *
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
LTRTenzo6Dof_InitializeDataMapInfo ( ) ; ssSetIsRapidAcceleratorActive ( rtS
, true ) ; ssSetRootSS ( rtS , rtS ) ; ssSetVersion ( rtS ,
SIMSTRUCT_VERSION_LEVEL2 ) ; ssSetModelName ( rtS , "LTRTenzo6Dof" ) ;
ssSetPath ( rtS , "LTRTenzo6Dof" ) ; ssSetTStart ( rtS , 0.0 ) ; ssSetTFinal
( rtS , 10.0 ) ; { static RTWLogInfo rt_DataLoggingInfo ; rt_DataLoggingInfo
. loggingInterval = NULL ; ssSetRTWLogInfo ( rtS , & rt_DataLoggingInfo ) ; }
{ { static int_T rt_LoggedStateWidths [ ] = { 8 , 2 , 2 , 2 , 8 , 1 } ;
static int_T rt_LoggedStateNumDimensions [ ] = { 1 , 1 , 1 , 1 , 1 , 1 } ;
static int_T rt_LoggedStateDimensions [ ] = { 8 , 2 , 2 , 2 , 8 , 1 } ;
static boolean_T rt_LoggedStateIsVarDims [ ] = { 0 , 0 , 0 , 0 , 0 , 0 } ;
static BuiltInDTypeId rt_LoggedStateDataTypeIds [ ] = { SS_DOUBLE , SS_DOUBLE
, SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE } ; static int_T
rt_LoggedStateComplexSignals [ ] = { 0 , 0 , 0 , 0 , 0 , 0 } ; static const
char_T * rt_LoggedStateLabels [ ] = { "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" } ; static const char_T *
rt_LoggedStateBlockNames [ ] = { "LTRTenzo6Dof/processo/Processo1" ,
"LTRTenzo6Dof/Setpoints2/P(s)4" , "LTRTenzo6Dof/Setpoints2/P(s)5" ,
"LTRTenzo6Dof/Setpoints2/P(s)1" , "LTRTenzo6Dof/\n\nkalman\n\n" ,
"LTRTenzo6Dof/Err Mes/Filtered random noise + sin/P(s)1" } ; static const
char_T * rt_LoggedStateNames [ ] = { "" , "" , "" , "" , "" , "" } ; static
boolean_T rt_LoggedStateCrossMdlRef [ ] = { 0 , 0 , 0 , 0 , 0 , 0 } ; static
RTWLogDataTypeConvert rt_RTWLogDataTypeConvert [ ] = { { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } } ; static RTWLogSignalInfo
rt_LoggedStateSignalInfo = { 6 , rt_LoggedStateWidths ,
rt_LoggedStateNumDimensions , rt_LoggedStateDimensions ,
rt_LoggedStateIsVarDims , ( NULL ) , ( NULL ) , rt_LoggedStateDataTypeIds ,
rt_LoggedStateComplexSignals , ( NULL ) , { rt_LoggedStateLabels } , ( NULL )
, ( NULL ) , ( NULL ) , { rt_LoggedStateBlockNames } , { rt_LoggedStateNames
} , rt_LoggedStateCrossMdlRef , rt_RTWLogDataTypeConvert } ; static void *
rt_LoggedStateSignalPtrs [ 6 ] ; rtliSetLogXSignalPtrs ( ssGetRTWLogInfo (
rtS ) , ( LogSignalPtrsType ) rt_LoggedStateSignalPtrs ) ;
rtliSetLogXSignalInfo ( ssGetRTWLogInfo ( rtS ) , & rt_LoggedStateSignalInfo
) ; rt_LoggedStateSignalPtrs [ 0 ] = ( void * ) & rtX . bhid251wmy [ 0 ] ;
rt_LoggedStateSignalPtrs [ 1 ] = ( void * ) & rtX . hhto124xqa [ 0 ] ;
rt_LoggedStateSignalPtrs [ 2 ] = ( void * ) & rtX . lsb1kf2p0a [ 0 ] ;
rt_LoggedStateSignalPtrs [ 3 ] = ( void * ) & rtX . del2gd1uu5 [ 0 ] ;
rt_LoggedStateSignalPtrs [ 4 ] = ( void * ) & rtX . owky1kts1u [ 0 ] ;
rt_LoggedStateSignalPtrs [ 5 ] = ( void * ) & rtX . mnq2gjnihw ; }
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
ssSolverInfo slvrInfo ; static boolean_T contStatesDisabled [ 23 ] ; static
real_T absTol [ 23 ] = { 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6
, 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 ,
1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 ,
1.0E-6 } ; static uint8_T absTolControl [ 23 ] = { 0U , 0U , 0U , 0U , 0U ,
0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U
, 0U , 0U } ; static uint8_T zcAttributes [ 6 ] = { ( ZC_EVENT_ALL ) , (
ZC_EVENT_ALL ) , ( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP ) , (
ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP ) } ; static ssNonContDerivSigInfo
nonContDerivSigInfo [ 1 ] = { { 1 * sizeof ( real_T ) , ( char * ) ( & rtB .
px0r25lgtw ) , ( NULL ) } } ; ssSetSolverRelTol ( rtS , 0.001 ) ;
ssSetStepSize ( rtS , 0.0 ) ; ssSetMinStepSize ( rtS , 0.0 ) ;
ssSetMaxNumMinSteps ( rtS , - 1 ) ; ssSetMinStepViolatedError ( rtS , 0 ) ;
ssSetMaxStepSize ( rtS , 0.0056625141562853904 ) ; ssSetSolverMaxOrder ( rtS
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
rtS , zcAttributes ) ; ssSetSolverNumZcSignals ( rtS , 6 ) ;
ssSetModelZeroCrossings ( rtS , MdlZeroCrossings ) ;
ssSetSolverConsecutiveZCsStepRelTol ( rtS , 2.8421709430404007E-13 ) ;
ssSetSolverMaxConsecutiveZCs ( rtS , 1000 ) ; ssSetSolverConsecutiveZCsError
( rtS , 2 ) ; ssSetSolverMaskedZcDiagnostic ( rtS , 1 ) ;
ssSetSolverIgnoredZcDiagnostic ( rtS , 1 ) ; ssSetSolverMaxConsecutiveMinStep
( rtS , 1 ) ; ssSetSolverShapePreserveControl ( rtS , 2 ) ; ssSetTNextTid (
rtS , INT_MIN ) ; ssSetTNext ( rtS , rtMinusInf ) ; ssSetSolverNeedsReset (
rtS ) ; ssSetNumNonsampledZCs ( rtS , 6 ) ; ssSetContStateDisabled ( rtS ,
contStatesDisabled ) ; ssSetSolverMaxConsecutiveMinStep ( rtS , 1 ) ; }
ssSetChecksumVal ( rtS , 0 , 3634810543U ) ; ssSetChecksumVal ( rtS , 1 ,
2521803458U ) ; ssSetChecksumVal ( rtS , 2 , 2534976825U ) ; ssSetChecksumVal
( rtS , 3 , 85240381U ) ; { static const sysRanDType rtAlwaysEnabled =
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
