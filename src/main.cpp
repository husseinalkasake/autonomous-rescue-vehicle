/* LIBRARIES */
#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* MACROS */
#define WIRE_CLOCK 400000                // Define Wire Clock
#define BNO055_SAMPLERATE_DELAY_MS (100) // Define Delay between IMU Samples
#define BNO055_ADDRESS 0x28              // Define IMU Address

// Motor Pins
#define MOTOR_LEFT_ENABLE_PIN 45
#define MOTOR_LEFT_PIN_1 30
#define MOTOR_LEFT_PIN_2 31
#define MOTOR_RIGHT_ENABLE_PIN 44
#define MOTOR_RIGHT_PIN_1 28
#define MOTOR_RIGHT_PIN_2 29

// Laser Sensor Pins
#define FRONT_LASER_SENSOR_ADDRESS 0x05
#define FRONT_LASER_SENSOR_PIN 6
#define LEFT_LASER_SENSOR_ADDRESS 0x10
#define LEFT_LASER_SENSOR_PIN 7

// Tolerances/Values
#define STABILITY_TOLERANCE 200     // Assume degrees
#define ANGLE_TOLERANCE 5           // Assume degrees
#define FRONT_DISTANCE_TOLERANCE 70 // Assume mm
#define SIDE_DISTANCE_TOLERANCE 70  // Assume mm
#define MOTOR_MAX 120
#define MOTOR_MIN 70
#define TILE_DISTANCE 250
#define GAP_DISTANCE 50
#define SIDE_GAP_DISTANCE 100
#define JAGGED_VALUE 5
#define DISTANCE_SCALE 1000
#define ANGLE_SCALE 360
#define TIME_OUT 100

/* SENSOR DECLARATION */
// Declare laser sensors
VL53L1X frontSensor;
VL53L1X leftSensor;

// Declare IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS);
sensors_event_t imuEvent;

/* GlOBALS */
// Parameters
int turnCount = 0;
bool reachedGoal = false;

// Difference Stuff
double angleInputDifference = 0;
double frontDistanceDifference = 0;
// double sideDistanceDifference = 0;

/* COMPONENTS SETUP */
// ToF Laser Sensor Setup
void setUpLaserSensors()
{
  pinMode(LEFT_LASER_SENSOR_PIN, OUTPUT);
  pinMode(FRONT_LASER_SENSOR_PIN, OUTPUT);
  digitalWrite(LEFT_LASER_SENSOR_PIN, LOW);
  digitalWrite(FRONT_LASER_SENSOR_PIN, LOW);

  pinMode(LEFT_LASER_SENSOR_PIN, INPUT_PULLUP);
  delay(150);
  Serial.println("Left Sensor");
  leftSensor.setTimeout(1000);
  if (!leftSensor.init())
  {
    Serial.println("Failed to detect and initialize left sensor!");
    while (1)
      ;
  }
  leftSensor.setAddress(LEFT_LASER_SENSOR_ADDRESS);

  pinMode(FRONT_LASER_SENSOR_PIN, INPUT_PULLUP);
  delay(150);
  Serial.println("Front Sensor");
  frontSensor.setTimeout(1000);
  if (!frontSensor.init())
  {
    Serial.println("Failed to detect and initialize front sensor!");
    while (1)
      ;
  }
  frontSensor.setAddress(FRONT_LASER_SENSOR_ADDRESS);

  //Adjustable parameters for TOF sensors
  leftSensor.setDistanceMode(VL53L1X::Long);
  frontSensor.setDistanceMode(VL53L1X::Long);
  leftSensor.setMeasurementTimingBudget(100000);
  frontSensor.setMeasurementTimingBudget(100000);
  leftSensor.startContinuous(100);
  frontSensor.startContinuous(100);
}

// IMU Setup
void setUpIMU()
{
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
  {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
}

// Motor Setup
void setUpMotors()
{
  pinMode(MOTOR_LEFT_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_PIN_1, OUTPUT);
  pinMode(MOTOR_LEFT_PIN_2, OUTPUT);
  pinMode(MOTOR_RIGHT_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN_1, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN_2, OUTPUT);
}

/* UPDATE COMPONENTS */
// ToF Sensor Update
void readLaserSensors()
{
  if (leftSensor.dataReady())
  {
    leftSensor.read(false);
  }
  if (frontSensor.dataReady())
  {
    frontSensor.read(false);
  }
  Serial.print("range: ");
  Serial.print(leftSensor.ranging_data.range_mm);
  Serial.print("\trange: ");
  Serial.print(frontSensor.ranging_data.range_mm);
}

// IMU Sensor Update
// Helper Function for IMU
void displayCalStatus()
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
void readIMU()
{
  bno.getEvent(&imuEvent);

  /* Display the floating point data */
  Serial.print("\tX: ");
  Serial.print(imuEvent.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(imuEvent.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(imuEvent.orientation.z, 4);

  //Display Calibration Status
  displayCalStatus();
  Serial.println("");
}

/* GETTERS FOR SENSOR VALUE */
// Get Stability Angle
double getStableSensor()
{
  return imuEvent.orientation.y;
}

// Get Compass Angle
double getCompassSensor()
{
  return imuEvent.orientation.x;
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
  // set direction of motor based on value
  if (leftValue < 0)
  {
    // motor left pin setup to move backwards
    digitalWrite(MOTOR_LEFT_PIN_1, LOW);
    digitalWrite(MOTOR_LEFT_PIN_2, HIGH);
  }
  else
  {
    // motor left pin setup to move forwards
    digitalWrite(MOTOR_LEFT_PIN_1, HIGH);
    digitalWrite(MOTOR_LEFT_PIN_2, LOW);
  }

  if (rightValue < 0)
  {
    // motor right pin setup to move backwards
    digitalWrite(MOTOR_RIGHT_PIN_1, HIGH);
    digitalWrite(MOTOR_RIGHT_PIN_2, LOW);
  }
  else
  {
    // motor right pin setup to move forwards
    digitalWrite(MOTOR_RIGHT_PIN_1, LOW);
    digitalWrite(MOTOR_RIGHT_PIN_2, HIGH);
  }

  leftValue = safetyMotor(abs(leftValue));
  rightValue = safetyMotor(abs(rightValue));

  // Set actual motor values
  analogWrite(MOTOR_LEFT_ENABLE_PIN, leftValue);
  analogWrite(MOTOR_RIGHT_ENABLE_PIN, rightValue);

  Serial.println("MOTOR LEFT VALUE: " + String(leftValue));
  Serial.println("MOTOR RIGHT VALUE: " + String(rightValue));
}

// Stop the Motors
void stopRobot()
{
  // TODO: CONFIRM WITH TARN IF U CAN DO THIS
  drive(0, 0);
}

/* SCALE FUNCTIONS */
// Map distance difference value to motor value
long getDistanceToMotorValue(double differenceFromDesired)
{
  return map(differenceFromDesired, 0, DISTANCE_SCALE, MOTOR_MIN, MOTOR_MAX);
}

// Map angle difference value to motor value
long getAngleToMotorValue(double differenceFromDesired)
{
  long value = map(abs(differenceFromDesired), 0, ANGLE_SCALE, MOTOR_MIN, MOTOR_MAX);
  if (differenceFromDesired < 0)
  {
    return -value;
  }
  else
  {
    return value;
  }
}

/* MAIN CONTROLLER FUNCTION*/
bool hasGoalReached()
{
  // Desired Point Differences
  angleInputDifference = getCompassSensor() - ((turnCount * 90) % 360);
  frontDistanceDifference = getFrontDistanceSensor() - ((TILE_DISTANCE * ((turnCount + 1) / 4)) + GAP_DISTANCE);
  // sideDistanceDifference = getLeftDistanceSensor() - ((TILE_DISTANCE * ((turnCount) / 4)) + SIDE_GAP_DISTANCE);

  // Vehicle Tilted
  // if (abs(getStableSensor()) > STABILITY_TOLERANCE)
  // {
  //   Serial.println("UNSTABLE");
  //   return false;
  // }

  // Vehicle Heading drifting
  if (abs(angleInputDifference) > ANGLE_TOLERANCE)
  {
    // Set right and left motor to opposite magnitude (one is reversed so same value)
    Serial.println("HEADING OFF");
    long motorValue = getAngleToMotorValue(angleInputDifference);
    drive(-motorValue, motorValue);
    return false;
  }

  // Vehicle Drifting right/left too much **NOTE: to Tarnpreet Side Tolereance should be large**
  // if (abs(sideDistanceDifference) > SIDE_DISTANCE_TOLERANCE)
  // {
  //   // Set right and left motor to same magnitude (one is reversed so one is opposite value + jagged value)
  //   Serial.println("SIDE DISTANCE OFF");
  //   long motorValue = getDistanceToMotorValue(sideDistanceDifference);
  //   drive(MOTOR_ZERO_OFFSET + motorValue, MOTOR_ZERO_OFFSET - motorValue - JAGGED_VALUE);
  //   return false;
  // }

  // Vehicle Front Travel Value
  if (abs(frontDistanceDifference) > FRONT_DISTANCE_TOLERANCE)
  {
    // Set right and left motor to same magnitude
    Serial.println("FRONT DISTANCE OFF");
    long motorValue = getDistanceToMotorValue(frontDistanceDifference);
    drive(motorValue, motorValue);
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
  Wire.setClock(400000);

  // sensor setup
  setUpLaserSensors();
  setUpIMU();
  setUpMotors();
}

void loop()
{
  // keep moving till goal reached
  if (!reachedGoal)
  {
    // TODO: COMMENT OUT DELAY AND PRINTS FOR FINAL CODE
    readLaserSensors();
    readIMU();

    Serial.println("Angle Sensor PID Difference: " + String(angleInputDifference));
    Serial.println();
    Serial.println("Front Sensor PID Difference: " + String(frontDistanceDifference));
    Serial.println();
    // Serial.println("Side Sensor PID Difference: " + String(sideDistanceDifference));
    // Serial.println();
    Serial.println("Turncount:" + String(turnCount));

    reachedGoal = hasGoalReached();
    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
  else
  {
    stopRobot();
  }
}
