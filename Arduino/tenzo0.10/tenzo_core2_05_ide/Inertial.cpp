/*
  Inertial.h - Library to handle inertial sensors and estimates
  configures Accelerometers, Gyroscopes, Magnetometers, Barometers, Gps
  Created by Giovanni Balestrieri - UserK, March 3, 2017.
  www.userk.co.uk
*/


#include "Inertial.h"
//#include "Ctx.h"
#include "CommunicationUtils.h"
#include "MedianFilter.h"

// Gyro
#define L3G4200D_Address 105
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

#define scale2000 70

#define STATUS_REG 0x27
#define ZOR_REG 0b01000000
#define ZDA_REG 0b00000100
#define YOR_REG 0b00100000
#define YDA_REG 0b00000010
#define XOR_REG 0b00010000
#define XDA_REG 0b00000001



// 3.3 Fully loaded Tenzo V2.2
int xRawMin = 409;
int xRawMax = 278;
// 
int yRawMin = 404;
int yRawMax = 272;
// 
int zRawMin = 419;
int zRawMax = 288;

/*
 * Constructor: 
 *              Loads configuration values from ctx
 */
Inertial::Inertial(){
  // Load accelerometer pins
  _axPin = ACC_X_AXIS_PIN;
  _ayPin = ACC_Y_AXIS_PIN;
  _azPin = ACC_Z_AXIS_PIN;

  _gyroBiasTempX = 0;
  _gyroBiasTempY = 0;
  _gyroBiasTempZ = 0;
  _gyroBiasX = 0;
  _gyroBiasY = 0;
  _gyroBiasZ = 0;

  dt = 0;

  // Init MedianFilters
   MedianFilter medianGyroX(3,0);
   MedianFilter medianGyroY(3,0);
   MedianFilter medianGyroZ(3,0);
}

/*
 * Initialize sensors
 */
void Inertial::init(){

  // Init Accelerometer
  this->setupAccelerometer();
  Serial.println("[ Ok ] Acc");

  // Init Gyroscope
  this->setupGyroscope();
  Serial.println("[ Ok ] Gyro");

  // Init Magnetometer
  
}

/**
 * Returns phi, theta, psi (roll, pitch,yaw) estimates
 */
 void Inertial::getYawPitchRoll(float *angles) {

    dt = micros() - dt;
    // update phiAcc and thetaAcc
    this->estAngleFromAcc();

    // Estimate Angles 
    angles[0] = (angles[0]  + _wx*(float)dt/1000000.0)*k_compl_filter_gyro + _phiAcc*k_compl_filter_acc;
    angles[1] = (angles[1]  + _wy*(float)dt/1000000.0)*k_compl_filter_gyro + _thetaAcc*k_compl_filter_acc;  
    dt = micros();
 }

 /**
 * Returns phi, theta, psi (roll, pitch,yaw) estimates
 */
 void Inertial::estAngleFromAcc(){ 
  if (filterAcc)
  {
    _phiAcc = (atan2(-_aF[0],-_aF[2])) * RAD_TO_DEG;
    _thetaAcc = (atan2(-_aF[1],-_aF[2])) * RAD_TO_DEG;
  }
  else 
  {
    _phiAcc = (atan2(-_ax,-_az)) * RAD_TO_DEG;
    _thetaAcc = (atan2(-_ay,-_az)) * RAD_TO_DEG;
  }
 }


/**
 * Initialize Magnetometer
 */
void Inertial::setupMagnetometer() {
  // CMPS10
  // No preconfig required  
}

/**
 * Initialize Gyroscope
 */
void Inertial::setupGyroscope() {
  
  // Configure L3G4200 
  this->setupL3G4200D(gyroSensibility); 
  
  //wait for the sensor to be ready   
  delay(500); 
  
  // Compute gyro bias  
  this->calcGyroBias();
}

/**
 * Computes Gyro bias
 */
void Inertial::setupL3G4200D(int scale) {
  //From  Jim Lindblom of Sparkfun's code

    // Enable x, y, z and turn off power down:
  //writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);
  // 400Hz and 1Hz cutoff frq of the HPF
  //writeRegister(L3G4200D_Address, CTRL_REG1, 0b01011111);
  // 200Hz 
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b01001111);

  // If you'd like to adjust/use the HPF:
  // High pass filter cut off frecuency configuration
  // ODR 200Hz, cutoff 0.02Hz 1001
  //writeRegister(L3G4200D_Address, CTRL_REG2, 0b00001001);
  // ODR 200Hz, cutoff 1 Hz 0100
  //writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000100);
  // ODR 200Hz, cutoff 0.02Hz 1001
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00001001);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  //writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range:
  if(scale == 250)
  {
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }
  else if(scale == 500)
  {
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }
  else
  {
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00010000);
  //writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
  
}

/**
 * Computes Gyro bias
 */
void Inertial::calcGyroBias() {
  
  for (int i = 0; i<gyroBiasSamples; i++)
  {
    delay(5);
    this->getAngularVel(); 
    _gyroBiasTempX = _gyroBiasTempX + _wx;
    _gyroBiasTempY = _gyroBiasTempY + _wy;
    _gyroBiasTempZ = _gyroBiasTempZ + _wz;
   
  }
  _gyroBiasX = _gyroBiasTempX / gyroBiasSamples;
  _gyroBiasY = _gyroBiasTempY / gyroBiasSamples;
  _gyroBiasZ = _gyroBiasTempZ / gyroBiasSamples;
  
  _initializedGyroCalib = true;
}

void Inertial::getAngularVel() {
  // Get Data if available
  byte statusflag = readRegister(L3G4200D_Address, STATUS_REG);
  while(!(statusflag & ZDA_REG) && (statusflag & ZOR_REG)&&!(statusflag & YDA_REG) && (statusflag & YOR_REG)&& !(statusflag & XDA_REG) && (statusflag & XOR_REG)) 
  {
    statusflag = readRegister(L3G4200D_Address, STATUS_REG);
  }
  

  xMSB = readRegister(L3G4200D_Address, 0x29);
  xLSB = readRegister(L3G4200D_Address, 0x28);
  xC = ((xMSB << 8) | xLSB);
  //medianGyroX.in(xC);
  //_wx = medianGyroX.out();    
    _wx = xC;

  yMSB = readRegister(L3G4200D_Address, 0x2B);
  yLSB = readRegister(L3G4200D_Address, 0x2A);
  yC = ((yMSB << 8) | yLSB);
  //medianGyroY.in(yC);
  //_wy = medianGyroY.out();     
    _wy = yC;

  zMSB = readRegister(L3G4200D_Address, 0x2D);
  zLSB = readRegister(L3G4200D_Address, 0x2C);
  zC = ((zMSB << 8) | zLSB);
  //medianGyroZ.in(zC);
  //_wz = medianGyroZ.out();  
    _wz = zC;   
  
  if (_initializedGyroCalib)
    this->removeBiasAndScaleGyroData();
    
   if (filterGyro)
   {
      _wF[0] = _wy;
      _wF[1] = _wy;
      _wF[2] = _wz;
      wFilter(_wF);
   }

}

/**
 * Returns angular velocities in a float array
 */
float Inertial::getAngularVel(int axis){
  float res;
  if (axis == XAXIS)
     res = _wx;
  else if (axis == YAXIS)
     res = _wy;
  else if (axis == ZAXIS)
     res = _wz;
  return res;
}



void Inertial::wFilter(float val[])
{
  val[0] = (1-alphaW)*val[0] + alphaW*_wFm1[0];
  val[1] = (1-alphaW)*val[1] + alphaW*_wFm1[1];
  val[2] = (1-alphaW)*val[2] + alphaW*_wFm1[2];
  
  _wFm1[0] = val[0];
  _wFm1[1] = val[1];
  _wFm1[2] = val[2];
}

void Inertial::accFilter(float val[])
{
  val[0] = (1-alphaA)*val[0] + alphaA*_aFm1[0];
  val[1] = (1-alphaA)*val[1] + alphaA*_aFm1[1];
  val[2] = (1-alphaA)*val[2] + alphaA*_aFm1[2];
  
  _aFm1[0] = val[0];
  _aFm1[1] = val[1];
  _aFm1[2] = val[2];
}

/**
 * Removes Bias and Scale Gyroscope values
 */
void Inertial::removeBiasAndScaleGyroData() {
  _wx = (_wx - _gyroBiasX)*scale2000/1000;
  _wy = (_wy - _gyroBiasY)*scale2000/1000;
  _wz = (_wz - _gyroBiasZ)*scale2000/1000;
}

/**
 * Initialize Accelerometer
 */
void Inertial::setupAccelerometer() {
  pinMode(ACC_X_AXIS_PIN,INPUT);
  pinMode(ACC_Y_AXIS_PIN,INPUT);
  pinMode(ACC_Z_AXIS_PIN,INPUT);
}

/**
 * Get Accelerometer's values
 */
void Inertial::getAcc() {

  // Read raw values
   _axRaw=analogRead(ACC_X_AXIS_PIN);
   _ayRaw=analogRead(ACC_Y_AXIS_PIN);
   _azRaw=analogRead(ACC_Z_AXIS_PIN);

  // convert Raw values
  float xAccScaled = map(_axRaw, xRawMin, xRawMax, -1000, 1000);
  float yAccScaled = map(_ayRaw, yRawMin, yRawMax, -1000, 1000);
  float zAccScaled = map(_azRaw, zRawMin, zRawMax, -1000, 1000);
  
  
  // re-scale to fractional Gs
  _ax = xAccScaled / 1000.0;
  _ay = yAccScaled / 1000.0;
  _az = zAccScaled / 1000.0;
   
   if (filterAcc)
   {
      _aF[0] = _ax;
      _aF[1] = _ay;
      _aF[2] = _az;
      accFilter(_aF);
   }
}

