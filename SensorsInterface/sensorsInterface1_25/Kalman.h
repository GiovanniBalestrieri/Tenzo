#ifndef _Kalman_h
#define _Kalman_h

class Kalman {  
  private:
      // Kalman filter variables
      double Q_angle; // Process noise variance for the acc
      double Q_bias; // Process noise variance for the gyro bias;
      double R_measure; // Measurement noise variance
      
      double angle; // angle calculated by the kalman filter
      double bias; // gyro bias calculated by the kalman filter
      double rate; // Un biased rate calculated from the rate and the calculated bias - Call getAngle to update the rate
      
      double P[2][2]; // Error covariance matrix 
      double K[2]; // Kalman gain 
      double y; // angle difference
      double S; // Estimated error
      
  public:
    Kalman()
    {
      Q_angle = 2;
      Q_bias = -102; //0.003
      R_measure = 0.03;
      
      angle = 0;
      bias = 0;
      
      P[0][0] = 0;
      P[1][0] = 0;
      P[0][1] = 0;
      P[1][1] = 0;
    };
    
    double getAngle(double newAngle, double newRate, double dt)
    {
      rate = newRate - bias;
      angle += dt*rate;
      
      P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
      P[0][1] -= dt * P[1][1];
      P[1][0] -= dt * P[1][1];
      P[1][1] += Q_bias * dt;
      
      // Compute the Kalman gain
      
      S = P[0][0] + R_measure;
      
      K[0] = P[0][0] / S;
      K[1] = P[1][0] / S;
      
      // Calculate angle and bias - Update estimate with measurement zk (newAngle)
      y = newAngle - angle;
      angle += K[0] * y;
      bias += K[1] * y;
      
      // Calculate estimation error covariance - Update the error covariance
      
      P[0][0] -= K[0] * P[0][0];
      P[0][1] -= K[0] * P[0][1];
      P[1][0] -= K[1] * P[0][0];
      P[1][1] -= K[1] * P[0][1];

      return angle;
    };

    void setAngle(double newAngle)
    {
      angle = newAngle;
    };

    void setQbias( double newQ_bias)
    {
      Q_bias = newQ_bias;
    };
    
    void setRmeasure(double newR_measure)
    {
      R_measure = newR_measure;
    };
    
    double getRate()
    {
      return rate;
    };
    
    double getQangle()
    {
      return Q_angle;
    };
    
    void setQangle(double newQangle)
    {
      Q_angle = newQangle;
    };
    
    double getRmeasure()
    {
      return R_measure;
    };
    
    double getQbias()
    {
      return Q_bias;
    };
    
};    

#endif
