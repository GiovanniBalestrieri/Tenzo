#include "__cf_PhiTheta.h"
#include "ext_types.h"
static uint_T rtDataTypeSizes [ ] = { sizeof ( real_T ) , sizeof ( real32_T )
, sizeof ( int8_T ) , sizeof ( uint8_T ) , sizeof ( int16_T ) , sizeof (
uint16_T ) , sizeof ( int32_T ) , sizeof ( uint32_T ) , sizeof ( boolean_T )
, sizeof ( fcn_call_T ) , sizeof ( int_T ) , sizeof ( pointer_T ) , sizeof (
action_T ) , 2 * sizeof ( uint32_T ) } ; static const char_T *
rtDataTypeNames [ ] = { "real_T" , "real32_T" , "int8_T" , "uint8_T" ,
"int16_T" , "uint16_T" , "int32_T" , "uint32_T" , "boolean_T" , "fcn_call_T"
, "int_T" , "pointer_T" , "action_T" , "timer_uint32_pair_T" } ; static
DataTypeTransition rtBTransitions [ ] = { { ( char_T * ) ( & rtB . mk4dpnamwc
) , 0 , 0 , 11 } , { ( char_T * ) ( & rtDW . fmiwcm0sxa ) , 0 , 0 , 2 } , { (
char_T * ) ( & rtDW . bvmrb3pj3h . TimePtr ) , 11 , 0 , 6 } , { ( char_T * )
( & rtDW . hiarr4fcu2 ) , 7 , 0 , 2 } , { ( char_T * ) ( & rtDW . kuvxmwsxqi
. PrevIndex ) , 10 , 0 , 1 } } ; static DataTypeTransitionTable rtBTransTable
= { 5U , rtBTransitions } ; static DataTypeTransition rtPTransitions [ ] = {
{ ( char_T * ) ( & rtP . SatThrust_UpperSat ) , 0 , 0 , 55 } , { ( char_T * )
( & rtP . ManualSwitch1_CurrentSetting ) , 3 , 0 , 3 } } ; static
DataTypeTransitionTable rtPTransTable = { 2U , rtPTransitions } ;
