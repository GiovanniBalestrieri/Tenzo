#ifndef tenzo_h
#define  tenzo_h

#define MAX_TASKS 32

#include "Ctx.h"
#include "Serial.h"
#include "Propulsion.h"
#include "Ux.h"
#include "Sensors.h"
#include "NonLinearPid.h"
#include "ControlPid.h"
#include "FlightParams.h"
#include "tenzo_timer.h"
#include "MedianFilter.h"
#include "FreeSixIMU.h"
#include "FIMU_ADXL345.h"
#include "FIMU_ITG3200.h"
#include "Scheduler.h"
#include "Sonar.h"
#include "RTClib.h"
#include "Logs.h"

volatile int bestId;

#endif
