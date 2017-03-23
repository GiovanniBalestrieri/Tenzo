/* Include files */

#include "testRollContSolo_sfun.h"
#include "testRollContSolo_sfun_debug_macros.h"
#include "c2_testRollContSolo.h"
#include "c3_testRollContSolo.h"
#include "c4_testRollContSolo.h"
#include "c5_testRollContSolo.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _testRollContSoloMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void testRollContSolo_initializer(void)
{
}

void testRollContSolo_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_testRollContSolo_method_dispatcher(SimStruct *simstructPtr,
  unsigned int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==2) {
    c2_testRollContSolo_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_testRollContSolo_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_testRollContSolo_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==5) {
    c5_testRollContSolo_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_testRollContSolo_process_check_sum_call( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2935437581U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1535312467U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2013322323U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(174843478U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1096209918U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(364392683U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1878553168U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3146084449U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 2:
        {
          extern void sf_c2_testRollContSolo_get_check_sum(mxArray *plhs[]);
          sf_c2_testRollContSolo_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_testRollContSolo_get_check_sum(mxArray *plhs[]);
          sf_c3_testRollContSolo_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_c4_testRollContSolo_get_check_sum(mxArray *plhs[]);
          sf_c4_testRollContSolo_get_check_sum(plhs);
          break;
        }

       case 5:
        {
          extern void sf_c5_testRollContSolo_get_check_sum(mxArray *plhs[]);
          sf_c5_testRollContSolo_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2083502392U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1110276785U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3258378658U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3926592909U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1622749405U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(422897959U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(195682652U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(487394460U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_testRollContSolo_autoinheritance_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(aiChksum, "zgNYGJobDJAn1AkNFFC3DB") == 0) {
          extern mxArray *sf_c2_testRollContSolo_get_autoinheritance_info(void);
          plhs[0] = sf_c2_testRollContSolo_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "8JuZqudrypSeqryZ9k8I3D") == 0) {
          extern mxArray *sf_c3_testRollContSolo_get_autoinheritance_info(void);
          plhs[0] = sf_c3_testRollContSolo_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 4:
      {
        if (strcmp(aiChksum, "UlTw2jkKgWrZT1luN0zeeG") == 0) {
          extern mxArray *sf_c4_testRollContSolo_get_autoinheritance_info(void);
          plhs[0] = sf_c4_testRollContSolo_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 5:
      {
        if (strcmp(aiChksum, "ixRniWqFXu6oexb7rzkopF") == 0) {
          extern mxArray *sf_c5_testRollContSolo_get_autoinheritance_info(void);
          plhs[0] = sf_c5_testRollContSolo_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_testRollContSolo_get_eml_resolved_functions_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        extern const mxArray
          *sf_c2_testRollContSolo_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_testRollContSolo_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray
          *sf_c3_testRollContSolo_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_testRollContSolo_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 4:
      {
        extern const mxArray
          *sf_c4_testRollContSolo_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_testRollContSolo_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 5:
      {
        extern const mxArray
          *sf_c5_testRollContSolo_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c5_testRollContSolo_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_testRollContSolo_third_party_uses_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(tpChksum, "EleXj0yLKIHARuYufkFgAE") == 0) {
          extern mxArray *sf_c2_testRollContSolo_third_party_uses_info(void);
          plhs[0] = sf_c2_testRollContSolo_third_party_uses_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "6n9fLz13JOYdOokQ7WCOPF") == 0) {
          extern mxArray *sf_c3_testRollContSolo_third_party_uses_info(void);
          plhs[0] = sf_c3_testRollContSolo_third_party_uses_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "YX6ZESPQKQG5jItZ1KN5Y") == 0) {
          extern mxArray *sf_c4_testRollContSolo_third_party_uses_info(void);
          plhs[0] = sf_c4_testRollContSolo_third_party_uses_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "au6BStrrIpFTBoLGrsvfxC") == 0) {
          extern mxArray *sf_c5_testRollContSolo_third_party_uses_info(void);
          plhs[0] = sf_c5_testRollContSolo_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_testRollContSolo_updateBuildInfo_args_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the updateBuildInfo_args_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(tpChksum, "EleXj0yLKIHARuYufkFgAE") == 0) {
          extern mxArray *sf_c2_testRollContSolo_updateBuildInfo_args_info(void);
          plhs[0] = sf_c2_testRollContSolo_updateBuildInfo_args_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "6n9fLz13JOYdOokQ7WCOPF") == 0) {
          extern mxArray *sf_c3_testRollContSolo_updateBuildInfo_args_info(void);
          plhs[0] = sf_c3_testRollContSolo_updateBuildInfo_args_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "YX6ZESPQKQG5jItZ1KN5Y") == 0) {
          extern mxArray *sf_c4_testRollContSolo_updateBuildInfo_args_info(void);
          plhs[0] = sf_c4_testRollContSolo_updateBuildInfo_args_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "au6BStrrIpFTBoLGrsvfxC") == 0) {
          extern mxArray *sf_c5_testRollContSolo_updateBuildInfo_args_info(void);
          plhs[0] = sf_c5_testRollContSolo_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void testRollContSolo_debug_initialize(struct SfDebugInstanceStruct*
  debugInstance)
{
  _testRollContSoloMachineNumber_ = sf_debug_initialize_machine(debugInstance,
    "testRollContSolo","sfun",0,4,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _testRollContSoloMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _testRollContSoloMachineNumber_,0);
}

void testRollContSolo_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_testRollContSolo_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info(
      "testRollContSolo", "testRollContSolo");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_testRollContSolo_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
