#define SECOND_US 1000000
#define MAIN_LOOP_DISP_PERIOD 1000000
float NUM_BLADES = 2;
#define SEC_IN_MIN 60
#define MAX_TASKS 10


volatile int bestId;

volatile double rev_sec = 0;
volatile double rev_min = 0;
volatile double rad_sec = 0;
volatile boolean setupOk = false;
boolean working = false;
const byte interruptPin = 2;
const byte servoPin = 8;
volatile byte state = LOW;
volatile long counter = 0;
volatile long lastCounter = 0;
unsigned long timeTracker = 0;

boolean printRevSec = true;
boolean printRevMin = true;
boolean printRadSec = true;

boolean printRev = false;

// servo 
boolean initialize = false;
boolean initialized = false;
boolean initializing = false;
boolean land = false;
boolean landed = true;
boolean landing = false;

volatile int currentUs = 0;
boolean printServoSignal = false;
boolean test = false;
boolean testing = false;

boolean printScheduler = false;
boolean verboseSerial = false;
unsigned long timerSec = 0;
unsigned long secRoutine = 0;

// SignalGenerator
volatile  int signalCounter = 0;
volatile  int signalInitializeSequence = 0;
volatile  int signalLandSequence = 0;
volatile  int signalPhase1Sequence = 0;
volatile  int signalPhase2Sequence = 0;
volatile  int signalPhase3Sequence = 0;
boolean firstTest = true;

// Optical Encoder
volatile boolean lowState = false; // not used
volatile boolean highState = false; // not used
volatile boolean statePin = false;
volatile float dt= 0;



volatile int  REF_SIGNAL = 1450;
volatile int MAX_SIGNAL = 2000;
volatile int  MIN_SIGNAL = 1000;
volatile int  MIN_SIGNAL0 = 1350;
