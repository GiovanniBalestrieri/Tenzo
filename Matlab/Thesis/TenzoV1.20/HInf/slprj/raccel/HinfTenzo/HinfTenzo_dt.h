#include "__cf_HinfTenzo.h"
#include "ext_types.h"
static uint_T rtDataTypeSizes [ ] = { sizeof ( real_T ) , sizeof ( real32_T )
, sizeof ( int8_T ) , sizeof ( uint8_T ) , sizeof ( int16_T ) , sizeof (
uint16_T ) , sizeof ( int32_T ) , sizeof ( uint32_T ) , sizeof ( boolean_T )
, sizeof ( fcn_call_T ) , sizeof ( int_T ) , sizeof ( pointer_T ) , sizeof (
action_T ) , 2 * sizeof ( uint32_T ) } ; static const char_T *
rtDataTypeNames [ ] = { "real_T" , "real32_T" , "int8_T" , "uint8_T" ,
"int16_T" , "uint16_T" , "int32_T" , "uint32_T" , "boolean_T" , "fcn_call_T"
, "int_T" , "pointer_T" , "action_T" , "timer_uint32_pair_T" } ; static
DataTypeTransition rtBTransitions [ ] = { { ( char_T * ) ( & rtB . mikmmv5pjy
) , 0 , 0 , 44 } , { ( char_T * ) ( & rtDW . ox0fllt2el ) , 0 , 0 , 1 } , { (
char_T * ) ( & rtDW . dbuidsfmth . LoggedData ) , 11 , 0 , 17 } , { ( char_T
* ) ( & rtDW . ic2yhbr32w ) , 7 , 0 , 1 } , { ( char_T * ) ( & rtDW .
ihm14vuvou [ 0 ] ) , 10 , 0 , 22 } , { ( char_T * ) ( & rtDW . nx03g0oqvx ) ,
8 , 0 , 5 } } ; static DataTypeTransitionTable rtBTransTable = { 6U ,
rtBTransitions } ; static DataTypeTransition rtPTransitions [ ] = { { (
char_T * ) ( & rtP . A0 [ 0 ] ) , 0 , 0 , 1072 } , { ( char_T * ) ( & rtP .
ManualSwitch1_CurrentSetting ) , 3 , 0 , 1 } } ; static
DataTypeTransitionTable rtPTransTable = { 2U , rtPTransitions } ;
