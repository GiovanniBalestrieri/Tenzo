
long reading = 0, readingM1 = 0;

const int FILTERING = 1;

int pushPin = A0;
int pushValue;
int pushTreshold = 30;
bool statoPush, stato;

float reads[10];
double sonarTimer = 0;

// #TUNE
int thresholdMax = 110;
int thresholdMin = 5;

long int contSonarRoutine = 0;
long int maxsonarTimer = 0;
double sonarTimeTot = 0;
boolean modeD = false, coro = false, cora = false;

//Filter #TUNE
float alpha = 0.95;

byte lowB, highB;
int us;
//Servo viber;


long duration, distance;
