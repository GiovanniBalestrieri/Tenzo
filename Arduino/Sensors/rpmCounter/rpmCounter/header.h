#define SECOND_US 1000000
#define NUM_BLADES 2
#define SEC_IN_MIN 60

volatile int rev_sec = 0;
volatile int rev_min = 0;
volatile float rad_sec = 0;
boolean working = false;
const byte interruptPin = 2;
volatile byte state = LOW;
volatile long counter = 0;
unsigned long timeTracker = 0;

boolean printRevSec = true;
boolean printRevMin = true;
boolean printRadSec = true;
