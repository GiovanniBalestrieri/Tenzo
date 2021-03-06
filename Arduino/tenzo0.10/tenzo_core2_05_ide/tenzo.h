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
//#include "FreeSixIMU.h"
#include "Scheduler.h"
#include "Sonar.h"

#ifndef RTC_OFF
  #include "RTClib.h"
#endif

#ifndef LOGGER_OFF
  #include "Logs.h"
#endif

#include "MelodyUtils.h"
#include "Pitches.h"
#include "Inertial.h"

volatile int bestId;

#endif
