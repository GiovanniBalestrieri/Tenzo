/*
  Inertial.h - Library to handle inertial sensors and estimates
  configures Accelerometers, Gyroscopes, Magnetometers, Barometers, Gps
  Created by Giovanni Balestrieri - UserK, March 3, 2017.
  www.userk.co.uk
*/
#ifndef Inertial_h
#define Inertial_h

#include "Arduino.h"
#include "MedianFilter.h"

extern int ACC_X_AXIS_PIN;
extern int ACC_Y_AXIS_PIN;
extern int ACC_Z_AXIS_PIN;
extern int gyroSensibility;
extern int gyroBiasSamples;
extern boolean filterGyro;
extern boolean filterAcc;
extern float alphaA;
extern float alphaW;
extern const float pi;
extern const float k_compl_filter_acc;
extern const float k_compl_filter_gyro;
extern unsigned long timerInertial1;
extern unsigned long timerInertial2;

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

class Inertial
{
  public:
    Inertial();
    void init();

    void setupGyroscope();
    void setupAccelerometer();
    void setupMagnetometer();

    void calcGyroBias();
    void getMagnetometerValues();
    void requestMagnetometerData();

    void filterValuesFirstOrder(float alpha);
    void filterValuesSecondOrder(float alpha);

    float getRollEst();
    float getPitchEst();
    float getYawEst();
    float getRollRaw();
    float getPitchRaw();
    float getYawRaw();
    void getYawPitchRoll(float *);

    // Accelerometer
    void getAcc();
    float getAccX();
    float getAccY();
    float getAccZ();
    float getAccXFilt();
    float getAccYFilt();
    float getAccZFilt();
    void estAngleFromAcc();

    // Gyroscope
    float getAngularVel(int axis);    
    //void getGyroValues(float *x[],int);    
    void getAngularVel();    
    void setupL3G4200D(int);
    boolean getGyroCalibrationStatus();
    void setGyroCalibrationStatus(boolean statusGyroCalib);
    void removeBiasAndScaleGyroData();
    void removeBiasAndScale();

    
    MedianFilter medianGyroX;
    MedianFilter medianGyroY;
    MedianFilter medianGyroZ;

    void wFilter(float val[]);
    void accFilter(float val[]);

    unsigned long dt;
    
    byte xMSB;
    byte xLSB;
    int xC;
    byte yMSB;
    byte yLSB;
    int yC;
    byte zMSB;
    byte zLSB;
    int zC;
  
  
   
  private:
    float _phi;
    float _phiEst;
    float _phiAcc;
    float _psi;
    float _psiEst;
    float _psiAcc;
    float _theta;
    float _thetaEst;
    float _thetaAcc;
    float _wx;
    float _wy;
    float _wz;


    // gyro bias
    int _gyroBiasTempX;
    int _gyroBiasTempY;
    int _gyroBiasTempZ;
    int _gyroBiasX;
    int _gyroBiasY;
    int _gyroBiasZ;
    boolean _initializedGyroCalib;
    
    float _ax;
    float _ay;
    float _az;
    float _axRaw;
    float _ayRaw;
    float _azRaw;
    float _xPose;
    float _yPose;
    float _zPose;
    float _wF[3] = {0,0,0};
    float _aF[3] = {0,0,0};
    float _wFm1[3] = {0,0,0};
    float _aFm1[3] = {0,0,0};
    
    int _axPin;
    int _ayPin;
    int _azPin;
};
#endif
