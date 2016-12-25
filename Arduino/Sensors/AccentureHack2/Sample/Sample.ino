//The 10 Dof sample sketch for reading the BMP085 and IMUs raw data

#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <HMC5883L.h>
#include "settings.h"
#include <BMP085.h>
#include <SoftwareSerial.h>

SoftwareSerial blu(10, 11); // RX, TX

float angles[3]; // yaw pitch roll
float heading;
BMP085 dps = BMP085();
long Temperature = 0, Pressure = 0, Altitude = 0;
float temp_bias = 36.53f;
float temp;
boolean printSerial = false;

int luxVal;


// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

HMC5883L compass;
// Record any errors that may occur in the compass.
int error = 0;

void setup() {

  Serial.begin(115200);
  Wire.begin();
  delay(1000);
  dps.init();
  dps.dumpCalData();
  delay(5000);

  delay(5);
  sixDOF.init(); //init the Acc and Gyro
  delay(5);
  compass = HMC5883L(); // init HMC5883

  error = compass.SetScale(1.3); // Set the scale of the compass.
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if (error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));


  pinMode(0, INPUT);
  setupBlu();

  Serial.print("Welcome to Fujin v: ");
  Serial.println(VERSION);
  blu.print("Welcome to Fujin v: ");
  blu.println(VERSION);

}

void loop() {
  Serial.print("t_raw\t");
  Serial.print(Temperature);
  dps.getTemperature(&Temperature);

  Serial.print("\t\t\tTemp:\t");
  Serial.println(Temperature / 340.0 + temp_bias);
  dps.getPressure(&Pressure);
  dps.getAltitude(&Altitude);

  //sixDOF.getEuler(angles);

  //getHeading();
  readLux();

  SerialRoutine();

  delay(300);
}


void setupBlu()
{
  Serial.println("Enter AT commands:");
  blu.begin(9600);  // HC-05 default speed in AT command more
}


void getHeading() {
  // Retrive the raw values from the compass (not scaled).
  MagnetometerRaw raw = compass.ReadRawAxis();
  // Retrived the scaled values from the compass (scaled to the configured scale).
  MagnetometerScaled scaled = compass.ReadScaledAxis();

  // Values are accessed like so:
  int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)

  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  heading = atan2(scaled.YAxis, scaled.XAxis);

  float declinationAngle = 0.0457;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  heading = heading * 180 / M_PI;
}

void printData() {

  Serial.print("Eular Angle: ");
  Serial.print(angles[0]);
  Serial.print("  ");
  Serial.print(angles[1]);
  Serial.print("  ");
  Serial.print(angles[2]);
  Serial.print("  ");
  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.print("  ");
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.print("C");
  Serial.print("  ");
  Serial.print("Altitude: ");
  Serial.print(Altitude);
  Serial.print("cm");
  Serial.print("  ");
  Serial.print("Pressure: ");
  Serial.print(Pressure);
  Serial.println(" Pa");



  blu.print("Eular Angle: ");
  blu.print(angles[0]);
  blu.print("  ");
  blu.print(angles[1]);
  blu.print("  ");
  blu.print(angles[2]);
  blu.print("  ");
  blu.print("Heading: ");
  blu.print(heading);
  blu.print("  ");
  blu.print("Temperature: ");
  blu.print(temp);
  blu.print("C");
  blu.print("  ");
  blu.print("Altitude: ");
  blu.print(Altitude);
  blu.print("cm");
  blu.print("  ");
  blu.print("Pressure: ");
  blu.print(Pressure);
  blu.println(" Pa");
}

// # LUX
void readLux()
{
  luxVal = analogRead(0);
  if (verbosityLevel >= 3)
    Serial.println("Ok LUx");
}

// #SERIAL
void SerialRoutine()
{
  // TODO
  if (Serial.available() || blu.available())
  {
    char t = Serial.read();
    char b = blu.read();
    if (t == 'c' || b == 'c')
    {
      printCSV = !printCSV;
      if (printCSV)
      {
        Serial.println("- CSV mode = TRUE");
        blu.println("- CSV mode = TRUE");
      }
      else
      {
        Serial.println("- CSV mode = FALSE");
        blu.println("- CSV mode = FALSE");
      }
    }
    else if (t == 'n' || b == 'n')
    {
      printSerial = !printSerial;
      if (printSerial)
      {
        Serial.println("- Normal mode = TRUE");
        blu.println("- Normal mode = TRUE");
      }
      else
      {
        Serial.println("- Normal mode = FALSE");
        blu.println("- Normal mode = FALSE");
      }
    }
    else if (t == '+' || b == '+')
    {
      verbosityLevel++;
      Serial.println("- Verbosity level changed :]\n");
      Serial.print(verbosityLevel);
    }
    else if (t == 'e')
    {
      //setupEsp();
    }
    else if (t == '-' || b == '-')
    {
      verbosityLevel--;
      Serial.println("- Verbosity level changed :]\n");
      Serial.print(verbosityLevel);
    }
    else if (t == 'p' || b == 'p')
    {
      verbosity = !verbosity;
      if (verbosity)
        Serial.println("- Verbosity = TRUE");
      else
        Serial.println("- Verbosity = FALSE");
    }
    else if (t == 'd' || b == 'd')
    {
      if (serial) {
        //
      }
      else if (bluetooth) {
        Serial.println("- Verbosity = FALSE");

      }
      printData();
    }
    else if (t == 'h' || b == 'h')
    {
      Serial.println("\n");
      Serial.println("Help:");
      Serial.println("Fujin returns three float values in order:");
      Serial.println("Temperature - Humidity - Euler - Lux");
      Serial.println("");

      Serial.println("+/- : adjust verbosity level");
      Serial.println("c : toggle CSV mode");
      Serial.println("n : toggle Normal mode");
      Serial.println("s : toggle SD verbosity");
      Serial.println("p : toggle general verbosity");
      Serial.println();
      Serial.print("l: print LUX ");
      Serial.println("\td: print ALL_DATA history");
      Serial.println("t: print TMP ");
      Serial.println("h: print HUM ");
      Serial.print("a: print Euler [°]");
      Serial.println("\td: print ALL_DATA history");


      blu.println("\n");
      blu.println("Help:");
      blu.println("Fujin returns three float values in order:");
      blu.println("Temperature - Humidity - Lux");
      blu.println("");

      blu.println("+/- : adjust verbosity level");
      blu.println("c : toggle CSV mode");
      blu.println("n : toggle Normal mode");
      blu.println("s : toggle SD verbosity");
      blu.println("p : toggle general verbosity");
      blu.println();
      blu.print("l: print LUX history");
      blu.println("\td: print ALL_DATA history");
      blu.println("t: print TMP history");
      blu.println("h: print HUM history");
      blu.println("m: print Altitude cm\t Pressure Pa");
      blu.print("a: print Euler [°]");
      blu.println("\td: print ALL_DATA history");

    } else if (t == 'a' || b == 'a') {
      Serial.print("a,");
      Serial.print(angles[0]);
      Serial.print(",");
      Serial.print(angles[1]);
      Serial.print(",");
      Serial.print(angles[2]);
      Serial.println(",z");


      blu.print("a,");
      blu.print(angles[0]);
      blu.print(",");
      blu.print(angles[1]);
      blu.print(",");
      blu.print(angles[2]);
      blu.println(",z");
    } else if (t == 'l' || b == 'l') {
      Serial.print("l,");
      Serial.print(luxVal);
      Serial.println(",z");

      blu.print("l,");
      blu.print(luxVal);
      blu.println(",z");
    } else if (t == 't' || b == 't') {
      Serial.print("l,");
      Serial.print(temp);
      Serial.println(",z");

      blu.print("l,");
      blu.print(temp);
      blu.println(",z");
    } else if (t == 'm' || b == 'm') {
      Serial.print("z,");
      Serial.print(Altitude);
      Serial.print(",");
      Serial.print(Pressure);
      Serial.println(",z");


      blu.print("z,");
      blu.print(Altitude);
      blu.print(",");
      blu.print(Pressure);
      blu.println(",z");
    }

  }
}
