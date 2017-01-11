#define SECOND_US 1000000
#define NUM_BLADES 2
#define SEC_IN_MIN 60

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

boolean printRev = true;

// servo 
boolean initialize = false;
boolean initialized = false;
boolean land = false;
boolean landed = true;

int currentUs = 0;
boolean printServoSignal = false;
boolean test = false;


#define REF_SIGNAL 1300
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 700
