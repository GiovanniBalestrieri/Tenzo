/* Librerie ROS */
//#include <ros.h>
//#include <sensor_msgs/Imu.h>

/* Librerie IMU */
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
//#include <MPU6050.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
int16_t g[3];
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


/* ROS */
//ros::NodeHandle nh;
//ros::sensor_msgs::Imu imu_msg,
//ros::Publisher pub_imu("imu_data", &imu_msg);
//sensor_msgs.Imu imu_msg;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    /* ROS */
    //nh.initNode();
    //nh.advertise(pub_imu);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    //mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        
        mpuIntStatus = mpu.getIntStatus();
        
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {// ERROR!
      Serial.println("ERROR!");
    }

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    if (!dmpReady) return;

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
          mpu.resetFIFO();
          Serial.println(F("FIFO overflow!"));
          
      } else if (mpuIntStatus & 0x02) {
        
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGyro(g, fifoBuffer);
        //mpu.dmpGetGravity(&gravity, &q);
        //mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        Serial.print("accel\t");
        Serial.print(aa.x);
        Serial.print("\t");
        Serial.print(aa.y);
        Serial.print("\t");
        Serial.println(aa.z);
        Serial.print("gyro\t");
        Serial.print(g[0]);
        Serial.print("\t");
        Serial.print(g[1]);
        Serial.print("\t");
        Serial.println(g[2]);
        
        /* ROS */
        /*
        sensor_msgs/Imu
	std_msgs/Header
		uint32 seq
		time stamp
		string frame_id
		string child_frame_id
	geometry_msgs/Quaternion
		float64 x
		float64 y
		float64 z
		float64 w
	float64[9] (Orientation_covariance)
	geometry_msgs/Vector3 (angular_velocity)
		float64 x
		float64 y
		float64 z
	float64[9] (angular_velocity_covariance)
	geometry_msgs/Vector3 (linear_acceleration)
		float64 x
		float64 y
		float64 z	
	float64[9] (linear_acceleration_covariance)
        */
        //imu_msg.orientation.x = q.x;
        //imu_msg.orientation.y = q.y;
        //imu_msg.orientation.z = q.z;
        //imu_msg.orientation.w = q.w;
        //setVelTransform(imu_msg.angular_velocity, g);
        //setAccTransform(imu_msg.linear_acceleration, aa);
        //imu_msg.orientation_covariance = {0.0, 0.0, 0.0,
        //                                  0.0, 0.0, 0.0,
        //                                  0.0, 0.0, 0.0}
        // . . .
        //pub_imu.publish( &str_msg );
        //nh.spinOnce();
    }
}
