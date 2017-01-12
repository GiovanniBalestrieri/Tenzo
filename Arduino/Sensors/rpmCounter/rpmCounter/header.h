#define SECOND_US 1000000
#define MAIN_LOOP_DISP_PERIOD 1000000
#define NUM_BLADES 2
#define SEC_IN_MIN 60
#define MAX_TASKS 10


volatile int bestId;

volatile int rev_sec = 0;
volatile int rev_min = 0;
volatile float rad_sec = 0;
boolean working = false;
const byte interruptPin = 2;
const byte servoPin = 8;
volatile byte state = LOW;
volatile long counter = 0;
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

boolean printScheduler = true;
unsigned long timerSec = 0;
unsigned long secRoutine = 0;

// SignalGenerator
volatile  int signalCounter = 0;
volatile  int signalInitializeSequence = 0;
volatile  int signalLandSequence = 0;
volatile  int signalPhase1Sequence = 0;
volatile  int signalPhase2Sequence = 0;
volatile  int signalPhase3Sequence = 0;


#define REF_SIGNAL 1300
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 700
