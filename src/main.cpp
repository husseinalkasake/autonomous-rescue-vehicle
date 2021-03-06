/* LIBRARIES */
#include <Arduino.h>
#include <stdlib.h>
#include <VL53L1X.h>
#include <Wire.h>
#include "MPU9250.h"
#include "quaternionFilters.h"
#include <Servo.h>

/* MACROS */
#define WIRE_CLOCK 400000 // Define Wire Clock
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0 // Define Addresses
#define STABILITY_TOLERANCE 200      // Assume degrees
#define ANGLE_TOLERANCE 5           // Assume degrees
#define FRONT_DISTANCE_TOLERANCE 70 // Assume cm
#define SIDE_DISTANCE_TOLERANCE 70  // Assume cm
#define MOTOR_MAX 110
#define MOTOR_MIN 70
#define TILE_DISTANCE 250
#define GAP_DISTANCE 50
#define SIDE_GAP_DISTANCE 100
#define JAGGED_VALUE 5
#define MOTOR_ZERO_OFFSET 90
#define DISTANCE_SCALE 1000
#define ANGLE_SCALE 360
#define TIME_OUT 100

/* SENSOR DECLARATION */
// Declare laser sensors
VL53L1X frontSensor;
VL53L1X leftSensor;

// Declare IMU
MPU9250 myIMU(MPU9250_ADDRESS, Wire, WIRE_CLOCK);

// Declare servo for motors
Servo motorLeft;
Servo motorRight;

/* GlOBALS */
// Parameters
double angleTare = 0;
double stabilityTare = 0;
int turnCount = 0;
bool reachedGoal = false;

// Difference Stuff
double angleInputDifference = 0;
double frontDistanceDifference = 0;
double sideDistanceDifference = 0;


/* COMPONENTS SETUP */
// ToF Laser Sensor Setup
void setUpLaserSensors()
{
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);

  pinMode(11, INPUT_PULLUP);
  delay(TIME_OUT*2);
  Serial.println("00");
  frontSensor.setTimeout(500);
  if (!frontSensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1)
      ;
  }
  frontSensor.setAddress(0x01);

  pinMode(12, INPUT_PULLUP);
  delay(TIME_OUT*2);
  Serial.println("01");
  leftSensor.setTimeout(500);
  if (!leftSensor.init())
  {
    Serial.println("Failed to detect and initialize leftSensor!");
    while (1)
      ;
  }
  leftSensor.setAddress(0x02);

  frontSensor.setDistanceMode(VL53L1X::Long);
  leftSensor.setDistanceMode(VL53L1X::Long);

  frontSensor.setMeasurementTimingBudget(50000);
  leftSensor.setMeasurementTimingBudget(50000);

  frontSensor.startContinuous(50);
  leftSensor.startContinuous(50);
}

// IMU Setup (from library example code)
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

// Motor Setup
void setUpMotors()
{
  // set pin, min and max values for each motor servo
  // TODO: Update to actual pins
  motorLeft.attach(9, 1000, 2000);
  motorRight.attach(10, 1000, 2000);
}

/* UPDATE COMPONENTS */
// ToF Sensor Update
void readLaserSensors()
{
  frontSensor.read();
  leftSensor.read();
}

// IMU Sensor Update
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
  }
}

/* GETTERS FOR SENSOR VALUE */
// Get Stability Angle
double getStableSensor()
{
  // TODO: TEST TO SEE IF X OR Y AXIS SHOULD BE USED
  return (atan2(myIMU.mz, myIMU.mx) * 180. / PI) + 180 - stabilityTare;
}

// Get Compass Angle
double getCompassSensor()
{
  return (atan2(myIMU.my, myIMU.mx) * 180. / PI) + 180 - angleTare;
}

// Get Front Distance Sensor Value
double getFrontDistanceSensor()
{
  return frontSensor.ranging_data.range_mm;
}

// Get Left Distance Sensor Value
double getLeftDistanceSensor()
{
  return leftSensor.ranging_data.range_mm;
}

/* MOTOR FUNCTIONS*/
// Saturation Filter for motor values to make sure motor doesn't get burnt
int safetyMotor(int value)
{
  if (value > MOTOR_MAX)
  {
    return MOTOR_MAX;
  }
  if (value < MOTOR_MIN)
  {
    return MOTOR_MIN;
  }
  return value;
}

// Drive Function that writes to motors
void drive(int leftValue, int rightValue)
{
  leftValue = safetyMotor(leftValue);
  rightValue = safetyMotor(rightValue);

  // Set actual motor values after delay
  motorLeft.write(leftValue);
  delay(TIME_OUT);
  motorRight.write(rightValue);
  Serial.println("MOTOR LEFT VALUE" + String(leftValue));
  Serial.println("MOTOR RIGHT VALUE" + String(rightValue));
}

// Stop the Motors
void stopRobot()
{
  drive(MOTOR_ZERO_OFFSET, MOTOR_ZERO_OFFSET);
}

/* SCALE FUNCTIONS */
// Map distance difference value to motor value
long getDistanceToMotorValue(double differenceFromDesired)
{
  return map(differenceFromDesired, -DISTANCE_SCALE, DISTANCE_SCALE, -MOTOR_ZERO_OFFSET, MOTOR_ZERO_OFFSET);
}

// Map angle difference value to motor value
long getAngleToMotorValue(double differenceFromDesired)
{
  return map(differenceFromDesired, -ANGLE_SCALE, ANGLE_SCALE, -MOTOR_ZERO_OFFSET, MOTOR_ZERO_OFFSET);
}

/* MAIN CONTROLLER FUNCTION*/
bool hasGoalReached()
{
  // Desired Point Differences
  angleInputDifference = getCompassSensor() - ((turnCount * 90) % 360);
  delay(TIME_OUT);
  frontDistanceDifference = getFrontDistanceSensor() - ((TILE_DISTANCE * ((turnCount + 1) / 4)) + GAP_DISTANCE);
  delay(TIME_OUT);
  sideDistanceDifference = getLeftDistanceSensor() - ((TILE_DISTANCE * ((turnCount) / 4)) + SIDE_GAP_DISTANCE);
  delay(TIME_OUT);

  // Vehicle Tilted
  if (abs(getStableSensor()) > STABILITY_TOLERANCE)
  {
    Serial.println("UNSTABLE");
    // Set both left and right motors to a sensible blind forward distance that we know works
    drive(MOTOR_ZERO_OFFSET + 10, MOTOR_ZERO_OFFSET - 10);
    return false;
  }

  // Vehicle Heading drifting
  if (abs(angleInputDifference) > ANGLE_TOLERANCE)
  {
    // Set right and left motor to opposite magnitude (one is reversed so same value)
    Serial.println("HEADING OFF");
    long motorValue = getAngleToMotorValue(angleInputDifference);
    drive(MOTOR_ZERO_OFFSET + motorValue, MOTOR_ZERO_OFFSET + motorValue);
    return false;
  }

  // Vehicle Drifting right/left too much **NOTE: to Tarnpreet Side Tolereance should be large**
  if (abs(sideDistanceDifference) > SIDE_DISTANCE_TOLERANCE)
  {
    // Set right and left motor to same magnitude (one is reversed so one is opposite value + jagged value)
    Serial.println("SIDE DISTANCE OFF");
    long motorValue = getDistanceToMotorValue(sideDistanceDifference);
    drive(MOTOR_ZERO_OFFSET + motorValue, MOTOR_ZERO_OFFSET - motorValue - JAGGED_VALUE);
    return false;
  }

  // Vehicle Front Travel Value
  if (abs(frontDistanceDifference) > FRONT_DISTANCE_TOLERANCE)
  {
    // Set right and left motor to same magnitude
    Serial.println("FRONT DISTANCE OFF");
    long motorValue = getDistanceToMotorValue(frontDistanceDifference);
    drive(MOTOR_ZERO_OFFSET + motorValue, MOTOR_ZERO_OFFSET - motorValue);
    return false;
  }

  // Increment turn count for next position
  if (turnCount != 10)
  {
    turnCount++;
    return false;
  }
  return true;
}

/* ARDUINO MAIN FUNCTIONS */
void setup()
{
  // Serial stuff setup
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(WIRE_CLOCK);

  // sensor setup
  setUpLaserSensors();
  setUpIMU();
  setUpMotors();
  drive(MOTOR_ZERO_OFFSET,MOTOR_ZERO_OFFSET);
  delay(TIME_OUT*50);
  updateIMU();
  delay(TIME_OUT*10);
  readLaserSensors();

  // set values
  delay(TIME_OUT*10);
  angleTare = getCompassSensor();
  stabilityTare = getStableSensor();
}

void loop()
{
  // keep moving till goal reached
  if (!reachedGoal)
  {
    // TODO: COMMENT OUT DELAY AND PRINTS FOR FINAL CODE
    delay(TIME_OUT);
    readLaserSensors();
    delay(TIME_OUT);
    updateIMU();

    Serial.println("Angle Sensor PID Difference: " + String(angleInputDifference));
    Serial.println();
    Serial.println("Front Sensor PID Difference: " + String(frontDistanceDifference));
    Serial.println();
    Serial.println("Side Sensor PID Difference: " + String(sideDistanceDifference));
    Serial.println();
    Serial.println("Turncount:" + String(turnCount));

    delay(TIME_OUT*2);
    reachedGoal = hasGoalReached();
  }
  else
  {
    stopRobot();
  }
}
