/*
 * simpleModel.c
 *
 * Code generation for model "simpleModel".
 *
 * Model version              : 1.11
 * Simulink Coder version : 8.2 (R2012a) 29-Dec-2011
 * C source code generated on : Sat Jan 11 02:55:28 2014
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */
#include "simpleModel.h"
#include "simpleModel_private.h"
#include "simpleModel_dt.h"

/* Block signals (auto storage) */
BlockIO_simpleModel simpleModel_B;

/* Block states (auto storage) */
D_Work_simpleModel simpleModel_DWork;

/* Real-time model */
RT_MODEL_simpleModel simpleModel_M_;
RT_MODEL_simpleModel *const simpleModel_M = &simpleModel_M_;

/* Model output function */
void simpleModel_output(void)
{
  /* Sin: '<Root>/Sine Wave' */
  simpleModel_B.SineWave = sin(simpleModel_P.SineWave_Freq *
    simpleModel_M->Timing.t[0] + simpleModel_P.SineWave_Phase) *
    simpleModel_P.SineWave_Amp + simpleModel_P.SineWave_Bias;

  /* Gain: '<Root>/Gain' */
  simpleModel_B.Gain = simpleModel_P.Gain_Gain * simpleModel_B.SineWave;
}

/* Model update function */
void simpleModel_update(void)
{
  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++simpleModel_M->Timing.clockTick0)) {
    ++simpleModel_M->Timing.clockTickH0;
  }

  simpleModel_M->Timing.t[0] = simpleModel_M->Timing.clockTick0 *
    simpleModel_M->Timing.stepSize0 + simpleModel_M->Timing.clockTickH0 *
    simpleModel_M->Timing.stepSize0 * 4294967296.0;

  {
    /* Update absolute timer for sample time: [0.01s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick1"
     * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++simpleModel_M->Timing.clockTick1)) {
      ++simpleModel_M->Timing.clockTickH1;
    }

    simpleModel_M->Timing.t[1] = simpleModel_M->Timing.clockTick1 *
      simpleModel_M->Timing.stepSize1 + simpleModel_M->Timing.clockTickH1 *
      simpleModel_M->Timing.stepSize1 * 4294967296.0;
  }
}

/* Model initialize function */
void simpleModel_initialize(void)
{
}

/* Model terminate function */
void simpleModel_terminate(void)
{
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/
void MdlOutputs(int_T tid)
{
  simpleModel_output();

  /* tid is required for a uniform function interface.
   * Argument tid is not used in the function. */
  UNUSED_PARAMETER(tid);
}

void MdlUpdate(int_T tid)
{
  simpleModel_update();

  /* tid is required for a uniform function interface.
   * Argument tid is not used in the function. */
  UNUSED_PARAMETER(tid);
}

void MdlInitializeSizes(void)
{
}

void MdlInitializeSampleTimes(void)
{
}

void MdlInitialize(void)
{
}

void MdlStart(void)
{
  simpleModel_initialize();
}

void MdlTerminate(void)
{
  simpleModel_terminate();
}

RT_MODEL_simpleModel *simpleModel(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)simpleModel_M, 0,
                sizeof(RT_MODEL_simpleModel));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&simpleModel_M->solverInfo,
                          &simpleModel_M->Timing.simTimeStep);
    rtsiSetTPtr(&simpleModel_M->solverInfo, &rtmGetTPtr(simpleModel_M));
    rtsiSetStepSizePtr(&simpleModel_M->solverInfo,
                       &simpleModel_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&simpleModel_M->solverInfo, (&rtmGetErrorStatus
      (simpleModel_M)));
    rtsiSetRTModelPtr(&simpleModel_M->solverInfo, simpleModel_M);
  }

  rtsiSetSimTimeStep(&simpleModel_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&simpleModel_M->solverInfo,"FixedStepDiscrete");

  /* Initialize timing info */
  {
    int_T *mdlTsMap = simpleModel_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    simpleModel_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    simpleModel_M->Timing.sampleTimes = (&simpleModel_M->
      Timing.sampleTimesArray[0]);
    simpleModel_M->Timing.offsetTimes = (&simpleModel_M->
      Timing.offsetTimesArray[0]);

    /* task periods */
    simpleModel_M->Timing.sampleTimes[0] = (0.0);
    simpleModel_M->Timing.sampleTimes[1] = (0.01);

    /* task offsets */
    simpleModel_M->Timing.offsetTimes[0] = (0.0);
    simpleModel_M->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr(simpleModel_M, &simpleModel_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = simpleModel_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    simpleModel_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(simpleModel_M, -1);
  simpleModel_M->Timing.stepSize0 = 0.01;
  simpleModel_M->Timing.stepSize1 = 0.01;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    simpleModel_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(simpleModel_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(simpleModel_M->rtwLogInfo, (NULL));
    rtliSetLogT(simpleModel_M->rtwLogInfo, "");
    rtliSetLogX(simpleModel_M->rtwLogInfo, "");
    rtliSetLogXFinal(simpleModel_M->rtwLogInfo, "");
    rtliSetSigLog(simpleModel_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(simpleModel_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(simpleModel_M->rtwLogInfo, 0);
    rtliSetLogMaxRows(simpleModel_M->rtwLogInfo, 1000);
    rtliSetLogDecimation(simpleModel_M->rtwLogInfo, 1);
    rtliSetLogY(simpleModel_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(simpleModel_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(simpleModel_M->rtwLogInfo, (NULL));
  }

  /* External mode info */
  simpleModel_M->Sizes.checksums[0] = (1985251451U);
  simpleModel_M->Sizes.checksums[1] = (3128440421U);
  simpleModel_M->Sizes.checksums[2] = (3849208798U);
  simpleModel_M->Sizes.checksums[3] = (3411995153U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[1];
    simpleModel_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(simpleModel_M->extModeInfo,
      &simpleModel_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(simpleModel_M->extModeInfo,
                        simpleModel_M->Sizes.checksums);
    rteiSetTPtr(simpleModel_M->extModeInfo, rtmGetTPtr(simpleModel_M));
  }

  simpleModel_M->solverInfoPtr = (&simpleModel_M->solverInfo);
  simpleModel_M->Timing.stepSize = (0.01);
  rtsiSetFixedStepSize(&simpleModel_M->solverInfo, 0.01);
  rtsiSetSolverMode(&simpleModel_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  simpleModel_M->ModelData.blockIO = ((void *) &simpleModel_B);
  (void) memset(((void *) &simpleModel_B), 0,
                sizeof(BlockIO_simpleModel));

  /* parameters */
  simpleModel_M->ModelData.defaultParam = ((real_T *)&simpleModel_P);

  /* states (dwork) */
  simpleModel_M->Work.dwork = ((void *) &simpleModel_DWork);
  (void) memset((void *)&simpleModel_DWork, 0,
                sizeof(D_Work_simpleModel));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    simpleModel_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 14;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.B = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.P = &rtPTransTable;
  }

  /* Initialize Sizes */
  simpleModel_M->Sizes.numContStates = (0);/* Number of continuous states */
  simpleModel_M->Sizes.numY = (0);     /* Number of model outputs */
  simpleModel_M->Sizes.numU = (0);     /* Number of model inputs */
  simpleModel_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  simpleModel_M->Sizes.numSampTimes = (2);/* Number of sample times */
  simpleModel_M->Sizes.numBlocks = (3);/* Number of blocks */
  simpleModel_M->Sizes.numBlockIO = (2);/* Number of block outputs */
  simpleModel_M->Sizes.numBlockPrms = (5);/* Sum of parameter "widths" */
  return simpleModel_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/
