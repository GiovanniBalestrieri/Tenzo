

#ifndef FlightParams_h
  #define FlightParams_h

/**
 * Modes
 */
int connAck = 0;
int takeOff = 0;
int hovering = 0;
int landed = 1;
int tracking = 0;
int warning = 0;

/**
 * VTOL settings
 */
 // Take Off settings
int rampTill = 1100; // rampTill = 1270;
int idle = 1000;
int motorRampDelayFast = 2;
int motorRampDelayMedium = 5;
int motorRampDelaySlow = 15;
int motorRampDelayVerySlow = 20;

// Safe Mode: after timeToLand ms tenzo will land automatically
unsigned long timeToLand = 20000;
boolean autoLand = false;
boolean landing = false;
int landSpeed = 1;
// Landing protocol variables
unsigned long timerStart;
unsigned long checkpoint;



// Keep track of the state
boolean initializing = false;
boolean initialized = false;
boolean sendStatesRemote = false;

#endif
