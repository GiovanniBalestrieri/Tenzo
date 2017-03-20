#include "__cf_PhiTheta.h"
#include "rt_logging_mmi.h"
#include "PhiTheta_capi.h"
#include <math.h>
#include "PhiTheta.h"
#include "PhiTheta_private.h"
#include "PhiTheta_dt.h"
const int_T gblNumToFiles = 0 ; const int_T gblNumFrFiles = 0 ; const int_T
gblNumFrWksBlocks = 1 ;
#ifdef RSIM_WITH_SOLVER_MULTITASKING
const boolean_T gbl_raccel_isMultitasking = 1 ;
#else
const boolean_T gbl_raccel_isMultitasking = 0 ;
#endif
const boolean_T gbl_raccel_tid01eq = 1 ; const int_T gbl_raccel_NumST = 2 ;
const char_T * gbl_raccel_Version = "8.6 (R2014a) 27-Dec-2013" ; void
raccel_setup_MMIStateLog ( SimStruct * S ) {
#ifdef UseMMIDataLogging
rt_FillStateSigInfoFromMMI ( ssGetRTWLogInfo ( S ) , & ssGetErrorStatus ( S )
) ;
#endif
} static DataMapInfo rt_dataMapInfo ; DataMapInfo * rt_dataMapInfoPtr = &
rt_dataMapInfo ; rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr = & (
rt_dataMapInfo . mmi ) ; const char * gblSlvrJacPatternFileName =
"slprj//raccel//PhiTheta//PhiTheta_Jpattern.mat" ; const int_T
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
) { rtX . ip3stwqu5g [ 0 ] = 0.0 ; rtX . ip3stwqu5g [ 1 ] = 0.0 ; rtX .
h4bztukumm = 0.0 ; rtX . awo0llmjv3 = 0.0 ; rtX . hu3bgmv23r [ 0 ] = 0.0 ;
rtX . hu3bgmv23r [ 1 ] = 0.0 ; rtX . icelwbnw0k [ 0 ] = 0.0 ; rtX .
icelwbnw0k [ 1 ] = 0.0 ; } void MdlStart ( void ) { uint32_T tseed ; int32_T
r ; int32_T t ; real_T tmp ; { FWksInfo * fromwksInfo ; if ( ( fromwksInfo =
( FWksInfo * ) calloc ( 1 , sizeof ( FWksInfo ) ) ) == ( NULL ) ) {
ssSetErrorStatus ( rtS ,
"from workspace STRING(Name) memory allocation error" ) ; } else {
fromwksInfo -> origWorkspaceVarName = "tuvar" ; fromwksInfo -> origDataTypeId
= 0 ; fromwksInfo -> origIsComplex = 0 ; fromwksInfo -> origWidth = 1 ;
fromwksInfo -> origElSize = sizeof ( real_T ) ; fromwksInfo -> data = ( void
* ) rtP . FromWs_Data0 ; fromwksInfo -> nDataPoints = 6 ; fromwksInfo -> time
= ( double * ) rtP . FromWs_Time0 ; rtDW . bvmrb3pj3h . TimePtr = fromwksInfo
-> time ; rtDW . bvmrb3pj3h . DataPtr = fromwksInfo -> data ; rtDW .
bvmrb3pj3h . RSimInfoPtr = fromwksInfo ; } rtDW . kuvxmwsxqi . PrevIndex = 0
; } tmp = muDoubleScalarFloor ( rtP . UniformRandomNumber_Seed ) ; if (
muDoubleScalarIsNaN ( tmp ) || muDoubleScalarIsInf ( tmp ) ) { tmp = 0.0 ; }
else { tmp = muDoubleScalarRem ( tmp , 4.294967296E+9 ) ; } tseed = tmp < 0.0
? ( uint32_T ) - ( int32_T ) ( uint32_T ) - tmp : ( uint32_T ) tmp ; r = (
int32_T ) ( tseed >> 16U ) ; t = ( int32_T ) ( tseed & 32768U ) ; tseed = ( (
( ( tseed - ( ( uint32_T ) r << 16U ) ) + t ) << 16U ) + t ) + r ; if ( tseed
< 1U ) { tseed = 1144108930U ; } else { if ( tseed > 2147483646U ) { tseed =
2147483646U ; } } rtDW . hiarr4fcu2 = tseed ; rtDW . fmiwcm0sxa = ( rtP .
UniformRandomNumber_Maximum - rtP . UniformRandomNumber_Minimum ) *
rt_urand_Upu32_Yd_f_pw_snf ( & rtDW . hiarr4fcu2 ) + rtP .
UniformRandomNumber_Minimum ; tmp = muDoubleScalarFloor ( rtP .
UniformRandomNumber_Seed_mavckwa2ow ) ; if ( muDoubleScalarIsNaN ( tmp ) ||
muDoubleScalarIsInf ( tmp ) ) { tmp = 0.0 ; } else { tmp = muDoubleScalarRem
( tmp , 4.294967296E+9 ) ; } tseed = tmp < 0.0 ? ( uint32_T ) - ( int32_T ) (
uint32_T ) - tmp : ( uint32_T ) tmp ; r = ( int32_T ) ( tseed >> 16U ) ; t =
( int32_T ) ( tseed & 32768U ) ; tseed = ( ( ( ( tseed - ( ( uint32_T ) r <<
16U ) ) + t ) << 16U ) + t ) + r ; if ( tseed < 1U ) { tseed = 1144108930U ;
} else { if ( tseed > 2147483646U ) { tseed = 2147483646U ; } } rtDW .
nf5mz1kdkf = tseed ; rtDW . mqn0il2cto = ( rtP .
UniformRandomNumber_Maximum_duhygprnbe - rtP .
UniformRandomNumber_Minimum_dr3kabpyij ) * rt_urand_Upu32_Yd_f_pw_snf ( &
rtDW . nf5mz1kdkf ) + rtP . UniformRandomNumber_Minimum_dr3kabpyij ;
MdlInitialize ( ) ; } void MdlOutputs ( int_T tid ) { real_T doii4njpv3 ; rtB
. mk4dpnamwc = 0.0 ; rtB . mk4dpnamwc += rtP . Ps_C [ 0 ] * rtX . ip3stwqu5g
[ 0 ] ; rtB . mk4dpnamwc += rtP . Ps_C [ 1 ] * rtX . ip3stwqu5g [ 1 ] ; {
real_T * pDataValues = ( real_T * ) rtDW . bvmrb3pj3h . DataPtr ; real_T *
pTimeValues = ( real_T * ) rtDW . bvmrb3pj3h . TimePtr ; int_T currTimeIndex
= rtDW . kuvxmwsxqi . PrevIndex ; real_T t = ssGetTaskTime ( rtS , 0 ) ; int
numPoints , lastPoint ; FWksInfo * fromwksInfo = ( FWksInfo * ) rtDW .
bvmrb3pj3h . RSimInfoPtr ; numPoints = fromwksInfo -> nDataPoints ; lastPoint
= numPoints - 1 ; if ( t <= pTimeValues [ 0 ] ) { currTimeIndex = 0 ; } else
if ( t >= pTimeValues [ lastPoint ] ) { currTimeIndex = lastPoint - 1 ; }
else { if ( t < pTimeValues [ currTimeIndex ] ) { while ( t < pTimeValues [
currTimeIndex ] ) { currTimeIndex -- ; } } else { while ( t >= pTimeValues [
currTimeIndex + 1 ] ) { currTimeIndex ++ ; } } } rtDW . kuvxmwsxqi .
PrevIndex = currTimeIndex ; { real_T t1 = pTimeValues [ currTimeIndex ] ;
real_T t2 = pTimeValues [ currTimeIndex + 1 ] ; if ( t1 == t2 ) { if ( t < t1
) { rtB . o3ul0nzcwe = pDataValues [ currTimeIndex ] ; } else { rtB .
o3ul0nzcwe = pDataValues [ currTimeIndex + 1 ] ; } } else { real_T f1 = ( t2
- t ) / ( t2 - t1 ) ; real_T f2 = 1.0 - f1 ; real_T d1 ; real_T d2 ; int_T
TimeIndex = currTimeIndex ; d1 = pDataValues [ TimeIndex ] ; d2 = pDataValues
[ TimeIndex + 1 ] ; rtB . o3ul0nzcwe = ( real_T ) rtInterpolate ( d1 , d2 ,
f1 , f2 ) ; pDataValues += numPoints ; } } } if ( ssIsSampleHit ( rtS , 1 , 0
) ) { } doii4njpv3 = rtP . Ps1_C * rtX . h4bztukumm ; if ( rtP .
ManualSwitch1_CurrentSetting == 1 ) { doii4njpv3 = muDoubleScalarSin ( rtP .
u3sen80t_Freq * ssGetTaskTime ( rtS , 0 ) + rtP . u3sen80t_Phase ) * rtP .
u3sen80t_Amp + rtP . u3sen80t_Bias ; } rtB . dewu3oxpw1 = rtP . Gain_Gain *
doii4njpv3 ; if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { } doii4njpv3 = rtP .
Ps1_C_pry1qgt440 * rtX . awo0llmjv3 ; if ( rtP .
ManualSwitch1_CurrentSetting_a1sxsoirxs == 1 ) { doii4njpv3 =
muDoubleScalarSin ( rtP . usen05t_Freq * ssGetTaskTime ( rtS , 0 ) + rtP .
usen05t_Phase ) * rtP . usen05t_Amp + rtP . usen05t_Bias ; } rtB . opuqvtwzzt
= rtP . Gain_Gain_hvers1lhk2 * doii4njpv3 ; if ( ssIsSampleHit ( rtS , 1 , 0
) ) { } rtB . awmfihve5b = 0.0 - ( ( ( rtP . Ps_C_axb255v5cb [ 0 ] * rtX .
hu3bgmv23r [ 0 ] + rtP . Ps_C_axb255v5cb [ 1 ] * rtX . hu3bgmv23r [ 1 ] ) +
rtB . opuqvtwzzt ) + rtB . dewu3oxpw1 ) ; if ( ssIsSampleHit ( rtS , 1 , 0 )
) { rtB . ngxpbqhzzn = rtDW . fmiwcm0sxa ; } rtB . dgroez0siq = (
muDoubleScalarSin ( rtP . usen05t_Freq_pv2suynl3g * ssGetTaskTime ( rtS , 0 )
+ rtP . usen05t_Phase_eh1ecmzpx1 ) * rtP . usen05t_Amp_jzvjbf0x3j + rtP .
usen05t_Bias_o1wafowfst ) + rtB . ngxpbqhzzn ; if ( ssIsSampleHit ( rtS , 1 ,
0 ) ) { rtB . kqoy4n5gjo = rtDW . mqn0il2cto ; } rtB . pydoj1kzpi = (
muDoubleScalarSin ( rtP . usen05t_Freq_lnfmendplc * ssGetTaskTime ( rtS , 0 )
+ rtP . usen05t_Phase_a1d0o3bxhg ) * rtP . usen05t_Amp_onejysspqe + rtP .
usen05t_Bias_k0lwz1nqbi ) + rtB . kqoy4n5gjo ; rtB . blhrfoy5vb = rtB .
opuqvtwzzt + rtB . awmfihve5b ; if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { if (
rtP . ManualSwitch_CurrentSetting == 1 ) { rtB . pk1lzuz22z = 0.0 ; } else if
( 0.0 > rtP . SatThrust_UpperSat ) { rtB . pk1lzuz22z = rtP .
SatThrust_UpperSat ; } else if ( 0.0 < rtP . SatThrust_LowerSat ) { rtB .
pk1lzuz22z = rtP . SatThrust_LowerSat ; } else { rtB . pk1lzuz22z = 0.0 ; } }
UNUSED_PARAMETER ( tid ) ; } void MdlUpdate ( int_T tid ) { if (
ssIsSampleHit ( rtS , 1 , 0 ) ) { rtDW . fmiwcm0sxa = ( rtP .
UniformRandomNumber_Maximum - rtP . UniformRandomNumber_Minimum ) *
rt_urand_Upu32_Yd_f_pw_snf ( & rtDW . hiarr4fcu2 ) + rtP .
UniformRandomNumber_Minimum ; rtDW . mqn0il2cto = ( rtP .
UniformRandomNumber_Maximum_duhygprnbe - rtP .
UniformRandomNumber_Minimum_dr3kabpyij ) * rt_urand_Upu32_Yd_f_pw_snf ( &
rtDW . nf5mz1kdkf ) + rtP . UniformRandomNumber_Minimum_dr3kabpyij ; }
UNUSED_PARAMETER ( tid ) ; } void MdlDerivatives ( void ) { XDot * _rtXdot ;
_rtXdot = ( ( XDot * ) ssGetdX ( rtS ) ) ; _rtXdot -> ip3stwqu5g [ 0 ] = 0.0
; _rtXdot -> ip3stwqu5g [ 1 ] = 0.0 ; _rtXdot -> ip3stwqu5g [ 0U ] += rtP .
Ps_A [ 0 ] * rtX . ip3stwqu5g [ 0 ] ; _rtXdot -> ip3stwqu5g [ 0U ] += rtP .
Ps_A [ 1 ] * rtX . ip3stwqu5g [ 1 ] ; _rtXdot -> ip3stwqu5g [ 1 ] += rtX .
ip3stwqu5g [ 0 ] ; _rtXdot -> ip3stwqu5g [ 0U ] += rtB . o3ul0nzcwe ; _rtXdot
-> h4bztukumm = 0.0 ; _rtXdot -> h4bztukumm += rtP . Ps1_A * rtX . h4bztukumm
; _rtXdot -> h4bztukumm += rtB . pydoj1kzpi ; _rtXdot -> awo0llmjv3 = 0.0 ;
_rtXdot -> awo0llmjv3 += rtP . Ps1_A_e4tfyewuhv * rtX . awo0llmjv3 ; _rtXdot
-> awo0llmjv3 += rtB . dgroez0siq ; _rtXdot -> hu3bgmv23r [ 0 ] = 0.0 ;
_rtXdot -> hu3bgmv23r [ 1 ] = 0.0 ; _rtXdot -> hu3bgmv23r [ 0U ] += rtP .
Ps_A_awgvmdvyjk [ 0 ] * rtX . hu3bgmv23r [ 0 ] ; _rtXdot -> hu3bgmv23r [ 0U ]
+= rtP . Ps_A_awgvmdvyjk [ 1 ] * rtX . hu3bgmv23r [ 1 ] ; _rtXdot ->
hu3bgmv23r [ 1 ] += rtX . hu3bgmv23r [ 0 ] ; _rtXdot -> hu3bgmv23r [ 0U ] +=
rtB . blhrfoy5vb ; _rtXdot -> icelwbnw0k [ 0 ] = 0.0 ; _rtXdot -> icelwbnw0k
[ 1 ] = 0.0 ; _rtXdot -> icelwbnw0k [ 0U ] += rtP . Cs1_A [ 0 ] * rtX .
icelwbnw0k [ 0 ] ; _rtXdot -> icelwbnw0k [ 0U ] += rtP . Cs1_A [ 1 ] * rtX .
icelwbnw0k [ 1 ] ; _rtXdot -> icelwbnw0k [ 1 ] += rtX . icelwbnw0k [ 0 ] ;
_rtXdot -> icelwbnw0k [ 0U ] += 0.0 ; } void MdlProjection ( void ) { } void
MdlTerminate ( void ) { rt_FREE ( rtDW . bvmrb3pj3h . RSimInfoPtr ) ; } void
MdlInitializeSizes ( void ) { ssSetNumContStates ( rtS , 8 ) ; ssSetNumY (
rtS , 0 ) ; ssSetNumU ( rtS , 0 ) ; ssSetDirectFeedThrough ( rtS , 0 ) ;
ssSetNumSampleTimes ( rtS , 2 ) ; ssSetNumBlocks ( rtS , 30 ) ;
ssSetNumBlockIO ( rtS , 11 ) ; ssSetNumBlockParams ( rtS , 58 ) ; } void
MdlInitializeSampleTimes ( void ) { ssSetSampleTime ( rtS , 0 , 0.0 ) ;
ssSetSampleTime ( rtS , 1 , 0.1 ) ; ssSetOffsetTime ( rtS , 0 , 0.0 ) ;
ssSetOffsetTime ( rtS , 1 , 0.0 ) ; } void raccel_set_checksum ( SimStruct *
rtS ) { ssSetChecksumVal ( rtS , 0 , 2450082240U ) ; ssSetChecksumVal ( rtS ,
1 , 2327675151U ) ; ssSetChecksumVal ( rtS , 2 , 2800790287U ) ;
ssSetChecksumVal ( rtS , 3 , 2590810103U ) ; } SimStruct *
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
. dataTypeNames = & rtDataTypeNames [ 0 ] ; dtInfo . B = & rtBTransTable ;
dtInfo . P = & rtPTransTable ; } PhiTheta_InitializeDataMapInfo ( rtS ) ;
ssSetIsRapidAcceleratorActive ( rtS , true ) ; ssSetRootSS ( rtS , rtS ) ;
ssSetVersion ( rtS , SIMSTRUCT_VERSION_LEVEL2 ) ; ssSetModelName ( rtS ,
"PhiTheta" ) ; ssSetPath ( rtS , "PhiTheta" ) ; ssSetTStart ( rtS , 0.0 ) ;
ssSetTFinal ( rtS , 15.0 ) ; ssSetStepSize ( rtS , 0.1 ) ; ssSetFixedStepSize
( rtS , 0.1 ) ; { static RTWLogInfo rt_DataLoggingInfo ; ssSetRTWLogInfo (
rtS , & rt_DataLoggingInfo ) ; } { { static int_T rt_LoggedStateWidths [ ] =
{ 2 , 1 , 1 , 2 , 2 } ; static int_T rt_LoggedStateNumDimensions [ ] = { 1 ,
1 , 1 , 1 , 1 } ; static int_T rt_LoggedStateDimensions [ ] = { 2 , 1 , 1 , 2
, 2 } ; static boolean_T rt_LoggedStateIsVarDims [ ] = { 0 , 0 , 0 , 0 , 0 }
; static BuiltInDTypeId rt_LoggedStateDataTypeIds [ ] = { SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE } ; static int_T
rt_LoggedStateComplexSignals [ ] = { 0 , 0 , 0 , 0 , 0 } ; static const
char_T * rt_LoggedStateLabels [ ] = { "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" } ; static const char_T * rt_LoggedStateBlockNames [ ] =
{ "PhiTheta/Plant/P(s)" , "PhiTheta/Roll Pitch\n Dynamic1/Err Mes/P(s)1" ,
"PhiTheta/Roll Pitch\n Dynamic1/Disturbi/P(s)1" ,
"PhiTheta/Roll Pitch\n Dynamic1/Plant/P(s)" ,
"PhiTheta/Roll Pitch\n Dynamic1/Control1/C(s)1" } ; static const char_T *
rt_LoggedStateNames [ ] = { "phi" , "" , "" , "phi" , "" } ; static boolean_T
rt_LoggedStateCrossMdlRef [ ] = { 0 , 0 , 0 , 0 , 0 } ; static
RTWLogDataTypeConvert rt_RTWLogDataTypeConvert [ ] = { { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } } ; static
RTWLogSignalInfo rt_LoggedStateSignalInfo = { 5 , rt_LoggedStateWidths ,
rt_LoggedStateNumDimensions , rt_LoggedStateDimensions ,
rt_LoggedStateIsVarDims , ( NULL ) , ( NULL ) , rt_LoggedStateDataTypeIds ,
rt_LoggedStateComplexSignals , ( NULL ) , { rt_LoggedStateLabels } , ( NULL )
, ( NULL ) , ( NULL ) , { rt_LoggedStateBlockNames } , { rt_LoggedStateNames
} , rt_LoggedStateCrossMdlRef , rt_RTWLogDataTypeConvert } ; static void *
rt_LoggedStateSignalPtrs [ 5 ] ; rtliSetLogXSignalPtrs ( ssGetRTWLogInfo (
rtS ) , ( LogSignalPtrsType ) rt_LoggedStateSignalPtrs ) ;
rtliSetLogXSignalInfo ( ssGetRTWLogInfo ( rtS ) , & rt_LoggedStateSignalInfo
) ; rt_LoggedStateSignalPtrs [ 0 ] = ( void * ) & rtX . ip3stwqu5g [ 0 ] ;
rt_LoggedStateSignalPtrs [ 1 ] = ( void * ) & rtX . h4bztukumm ;
rt_LoggedStateSignalPtrs [ 2 ] = ( void * ) & rtX . awo0llmjv3 ;
rt_LoggedStateSignalPtrs [ 3 ] = ( void * ) & rtX . hu3bgmv23r [ 0 ] ;
rt_LoggedStateSignalPtrs [ 4 ] = ( void * ) & rtX . icelwbnw0k [ 0 ] ; }
rtliSetLogT ( ssGetRTWLogInfo ( rtS ) , "tout" ) ; rtliSetLogX (
ssGetRTWLogInfo ( rtS ) , "tmp_raccel_xout" ) ; rtliSetLogXFinal (
ssGetRTWLogInfo ( rtS ) , "xFinal" ) ; rtliSetLogVarNameModifier (
ssGetRTWLogInfo ( rtS ) , "none" ) ; rtliSetLogFormat ( ssGetRTWLogInfo ( rtS
) , 2 ) ; rtliSetLogMaxRows ( ssGetRTWLogInfo ( rtS ) , 1000 ) ;
rtliSetLogDecimation ( ssGetRTWLogInfo ( rtS ) , 1 ) ; rtliSetLogY (
ssGetRTWLogInfo ( rtS ) , "" ) ; rtliSetLogYSignalInfo ( ssGetRTWLogInfo (
rtS ) , ( NULL ) ) ; rtliSetLogYSignalPtrs ( ssGetRTWLogInfo ( rtS ) , ( NULL
) ) ; } { static struct _ssStatesInfo2 statesInfo2 ; ssSetStatesInfo2 ( rtS ,
& statesInfo2 ) ; } { static ssSolverInfo slvrInfo ; static boolean_T
contStatesDisabled [ 8 ] ; ssSetSolverInfo ( rtS , & slvrInfo ) ;
ssSetSolverName ( rtS , "ode3" ) ; ssSetVariableStepSolver ( rtS , 0 ) ;
ssSetSolverConsistencyChecking ( rtS , 0 ) ; ssSetSolverAdaptiveZcDetection (
rtS , 0 ) ; ssSetSolverRobustResetMethod ( rtS , 0 ) ;
ssSetSolverStateProjection ( rtS , 0 ) ; ssSetSolverMassMatrixType ( rtS , (
ssMatrixType ) 0 ) ; ssSetSolverMassMatrixNzMax ( rtS , 0 ) ;
ssSetModelOutputs ( rtS , MdlOutputs ) ; ssSetModelLogData ( rtS ,
rt_UpdateTXYLogVars ) ; ssSetModelUpdate ( rtS , MdlUpdate ) ;
ssSetModelDerivatives ( rtS , MdlDerivatives ) ; ssSetTNextTid ( rtS ,
INT_MIN ) ; ssSetTNext ( rtS , rtMinusInf ) ; ssSetSolverNeedsReset ( rtS ) ;
ssSetNumNonsampledZCs ( rtS , 0 ) ; ssSetContStateDisabled ( rtS ,
contStatesDisabled ) ; } ssSetChecksumVal ( rtS , 0 , 2450082240U ) ;
ssSetChecksumVal ( rtS , 1 , 2327675151U ) ; ssSetChecksumVal ( rtS , 2 ,
2800790287U ) ; ssSetChecksumVal ( rtS , 3 , 2590810103U ) ; { static const
sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE ; static RTWExtModeInfo
rt_ExtModeInfo ; static const sysRanDType * systemRan [ 2 ] ;
ssSetRTWExtModeInfo ( rtS , & rt_ExtModeInfo ) ;
rteiSetSubSystemActiveVectorAddresses ( & rt_ExtModeInfo , systemRan ) ;
systemRan [ 0 ] = & rtAlwaysEnabled ; systemRan [ 1 ] = & rtAlwaysEnabled ;
rteiSetModelMappingInfoPtr ( ssGetRTWExtModeInfo ( rtS ) , &
ssGetModelMappingInfo ( rtS ) ) ; rteiSetChecksumsPtr ( ssGetRTWExtModeInfo (
rtS ) , ssGetChecksums ( rtS ) ) ; rteiSetTPtr ( ssGetRTWExtModeInfo ( rtS )
, ssGetTPtr ( rtS ) ) ; } return rtS ; }
