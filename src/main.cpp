#include <Arduino.h>
#include <PID_v1.h>
#include <stdlib.h>
#include <VL53L1X.h>
#include <Wire.h>

#include "MPU9250.h"
#include "quaternionFilters.h"

#include <Servo.h>

// Declare laser sensors
VL53L1X sensor1;
VL53L1X sensor2;

// Declare IMU
#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

// Declare servo for motors
Servo motorLeft;
Servo motorRight;

// @TODO: Adjust tolerance to viable numbers
#define STABILITY_TOLERANCE 200     // Assume degrees
#define ANGLE_TOLERANCE 7           // Assume degrees
#define FRONT_DISTANCE_TOLERANCE 10 // Assume cm
#define SIDE_DISTANCE_TOLERANCE 10  // Assume cm

// Global Variables
double tileDistance = 0;
double angleTare = 0;
double stabilityTare = 0;
int turnCount = 0;
int jaggedValue = 5;
int motorzerooffset = 90;
bool reachedGoal = false;
int zero = 0;
int motorValue = 0;

// PID Stuff
double angleInputDifference = 0;
double angleOutputPID = 0;
PID anglePID(&angleInputDifference, &angleOutputPID, 0, 1, 10, 25, DIRECT);

double frontDistanceDifference = 0;
double frontDistanceOutputPID = 0;
PID distanceFrontPID(&frontDistanceDifference, &frontDistanceOutputPID, 0, 1, 10, 25, DIRECT);

double sideDistanceDifference = 0;
double sideDistanceOutputPID = 0;
PID distanceSidePID(&sideDistanceDifference, &sideDistanceOutputPID, 0, 1, 10, 25, DIRECT);

// Scale Values
// @TODO: input motor characterization scale here
const double distanceScale = 1.00;
const double angleScale = 1.00;

// Sensor Setup
void setUpLaserSensors()
{
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);

  pinMode(4, INPUT_PULLUP);
  delay(150);
  Serial.println("00");
  sensor1.setTimeout(500);
  if (!sensor1.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1)
      ;
  }
  sensor1.setAddress(0x01);

  pinMode(5, INPUT_PULLUP);
  delay(150);
  Serial.println("01");
  sensor2.setTimeout(500);
  if (!sensor2.init())
  {
    Serial.println("Failed to detect and initialize sensor2!");
    while (1)
      ;
  }
  sensor2.setAddress(0x02);

  sensor1.setDistanceMode(VL53L1X::Long);
  sensor2.setDistanceMode(VL53L1X::Long);

  sensor1.setMeasurementTimingBudget(50000);
  sensor2.setMeasurementTimingBudget(50000);

  sensor1.startContinuous(50);
  sensor2.startContinuous(50);
}

void setUpIMU()
{
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0], 1);
    Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1], 1);
    Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2], 1);
    Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3], 1);
    Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4], 1);
    Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5], 1);
    Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);

    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    //    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(myIMU.magBias[0]);
    Serial.println(myIMU.magBias[1]);
    Serial.println(myIMU.magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(myIMU.magScale[0]);
    Serial.println(myIMU.magScale[1]);
    Serial.println(myIMU.magScale[2]);
    //    delay(2000); // Add delay to see results before serial spew of data

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
}

void setUpMotors()
{
  // set pin, min and max values for each motor servo
  // TODO: Update to actual pins
  motorLeft.attach(8, 1000, 2000);
  motorRight.attach(9, 1000, 2000);
}

// Sensor Update
void readLaserSensors()
{
  sensor1.read();
  sensor2.read();
}

void updateIMU()
{
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readMagData(myIMU.magCount); // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
    // Serial.print("Mag Yaw, Pitch, Roll: ");
    // Serial.print(myIMU.mx, 2); // top/down
    // Serial.print(", ");
    // Serial.print(myIMU.my, 2); // right/left
    // Serial.print(", ");
    // Serial.println(myIMU.mz, 2); // sideways
  }
}

double getStableSensor()
{
  return myIMU.mx;
}

double getCompassSensor()
{
  return (((int)(0.4 * ((myIMU.my) + 1000))) % 360) - angleTare;
}

double getFrontDistanceSensor()
{
  return sensor1.ranging_data.range_mm;
}
double getLeftDistanceSensor()
{
  return sensor2.ranging_data.range_mm;
}

// Motor stuff
void stopRobot()
{
  // int sensorValue = 0;
  // if (Serial.available() > 0)
  // {
  //   sensorValue = Serial.parseInt();
  // }
  // TODO: Stop Robot
  motorLeft.write(90);
  motorRight.write(90);
}
void driveForward(int value)
{
  motorLeft.write(50);
  motorRight.write(50);
  delay(100);
  motorLeft.write(value);
  motorRight.write(value);
}

void setup()
{
  // Serial stuff
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  //Configuration of PID's
  anglePID.SetMode(AUTOMATIC);
  distanceFrontPID.SetMode(AUTOMATIC);
  distanceSidePID.SetMode(AUTOMATIC);

  setUpLaserSensors();
  setUpIMU();
  updateIMU();

  // set values
  tileDistance = getLeftDistanceSensor() * 2;
  angleTare = getCompassSensor();
  stabilityTare = getStableSensor();

  // testing
  tileDistance = 50;
}

bool hasGoalReached()
{
  // Processed Sensor Values
  double stabilityValue = (getStableSensor() - stabilityTare);
  double angleValue = getCompassSensor();
  double tileDistanceFrontVal = getFrontDistanceSensor();
  double tileDistanceSideVal = getLeftDistanceSensor();

  // Desired Point Difference
  angleInputDifference = angleValue - ((turnCount * 90) % 360);
  frontDistanceDifference = tileDistanceFrontVal - ((tileDistance * ((turnCount + 1) / 4)) + (tileDistance / 2.0));
  sideDistanceDifference = tileDistanceSideVal - ((tileDistance * ((turnCount) / 4)) + (tileDistance / 2.0));

  // Compute PID
  anglePID.Compute();
  distanceFrontPID.Compute();
  distanceSidePID.Compute();

  // Vehicle Tilted
  if (abs(stabilityValue) > STABILITY_TOLERANCE)
  {
    Serial.println("UNSTABLE");
    // @TODO: Set both left and right motors to a sensible blind forward distance that we know works
    return false;
  }

  // Vehicle Heading drifting
  if (abs(angleInputDifference) > ANGLE_TOLERANCE)
  {
    Serial.println("ANGLE HEADING FUCKED");
    // @TODO: set right and left motor to opposite magnitude
    // leftmotor=(angleScale*angleOutputPID)+motorzerooffset;
    // rightmotor=-(angleScale*angleOutputPID)+motorzerooffset;
    return false;
  }

  // Vehicle Drifting right/left too much **NOTE: to Tarnpreet Side Tolereance should be large**
  if (abs(sideDistanceDifference) > SIDE_DISTANCE_TOLERANCE)
  {
    Serial.println("SIDE DISTANCE FUCKED");
    // @TODO: set right and left motor to same magnitude
    // leftmotor=(distanceScale*sideDistanceOutputPID)+motorzerooffset;
    // rightmotor=(distanceScale*sideDistanceOutputPID)+jaggedvalue+motorzerooffset;
    return false;
  }

  // Vehicle Front Travel Value
  if (abs(frontDistanceDifference) > FRONT_DISTANCE_TOLERANCE)
  {
    Serial.println("FRONT DISTANCE FUCKED");
    // @TODO: set right and left motor to same magnitude
    // leftmotor=(distanceScale*frontDistanceOutputPID)+motorzerooffset;
    // rightmotor=(distanceScale*frontDistanceOutputPID)+motorzerooffset;
    return false;
  }

  // Increment TurnCount for next position
  if (turnCount != 10)
  {
    turnCount++;

    return false;
  }

  return true;
}

void loop()
{
  // keep moving till goal reached
  if (!reachedGoal)
  {
    //delay(1000);
    readLaserSensors();
    updateIMU();

    Serial.println("Scale: PID OUTPUTS" + String(angleOutputPID));
    Serial.println();
    Serial.println("Scale: PID OUTPUTS" + String(frontDistanceOutputPID));
    Serial.println();
    Serial.println("Scale: PID OUTPUTS" + String(sideDistanceOutputPID));
    Serial.println();

    Serial.println("Rawvalue: " + String(myIMU.my));
    Serial.println();

    Serial.println("Compass: " + String(getCompassSensor()));
    Serial.println();

    Serial.println("Angle Sensor PID Difference: " + String(angleInputDifference));
    Serial.println();

    Serial.println("Front Sensor PID Difference: " + String(frontDistanceDifference));
    Serial.println();

    Serial.println("Side Sensor PID Difference: " + String(sideDistanceDifference));
    Serial.println();

    Serial.println("Turncount:" + String(turnCount));

    reachedGoal = hasGoalReached();
  }
  else
  {
    stopRobot();
  }
}
