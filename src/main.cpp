#include <Arduino.h>
#include <PID_v1.h>
#include <stdlib.h>

// @TODO: Adjust tolerance to viable numbers
#define STABILITY_TOLERANCE 0.1    // Assume degrees
#define ANGLE_TOLERANCE 1          // Assume degrees
#define FRONT_DISTANCE_TOLERANCE 1 // Assume cm
#define SIDE_DISTANCE_TOLERANCE 1  // Assume cm

// Scale Values
// @TODO: input motor characterization scale here
const double distanceScale = 1.00;
const double angleScale = 1.00;

// Sensor Mocks
// TODO: RETURN ACTUAL SENSOR DATA
double getStableSensor()
{
  return 0;
}
double getCompassSensor()
{
  return 0;
}
double getFrontDistanceSensor()
{
  return 0;
}
double getLeftDistanceSensor()
{
  return 0;
}

// Global Variables
double tileDistance = 0;
double angleTare = 0;
double stabilityTare = 0;
int turnCount = 0;
bool reachedGoal = false;

// PID Stuff
double angleInputDifference = 0;
double angleOutputPID = 0;
PID anglePID(&angleInputDifference, &angleOutputPID, 0, 2, 5, 1, DIRECT);

double frontDistanceDifference = 0;
double frontDistanceOutputPID = 0;
PID distanceFrontPID(&frontDistanceDifference, &frontDistanceOutputPID, 0, 2, 5, 1, DIRECT);

double sideDistanceDifference = 0;
double sideDistanceOutputPID = 0;
PID distanceSidePID(&sideDistanceDifference, &sideDistanceOutputPID, 0, 2, 5, 1, DIRECT);

// Motor stuff
void stopRobot()
{
  // TODO: Stop Robot
}
void driveForward()
{
  // TODO: Blind forward motor value
}
void rightTurnRobot()
{
  stopRobot();

  // TODO: right turn robot

  // turn until angle matches expected one after turn
  while (
      abs(getAngleSensor() - angleTare - (((turnCount * 90) % 360) + 90)) >
      ANGLE_TOLERANCE)
  {
  }

  stopRobot();
  driveForward();

  // increment turn count
  turnCount++;
}

void setup()
{
  //Configuration of PID's
  anglePID.SetMode(AUTOMATIC);
  distanceFrontPID.SetMode(AUTOMATIC);
  distanceSidePID.SetMode(AUTOMATIC);

  // set values
  tileDistance = getLeftDistanceSensor() * 2;
  angleTare = getCompassSensor();
  // TODO: set stability tare from angle sensor
}

bool hasGoalReached()
{
  // Processed Sensor Values
  double stabilityValue = (getStableSensor() - stabilityTare);
  double angleValue = (getCompassSensor() - angleTare);
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
    // @TODO: Set both left and right motors to a sensible blind forward distance that we know works
    return false;
  }

  // Vehicle Heading drifting
  if (abs(angleInputDifference) > ANGLE_TOLERANCE)
  {
    // @TODO: set right and left motor to opposite magnitude
    // leftmotor=angleScale*angleOutputPID;
    // rightmotor=-(angleScale*angleOutputPID);
    return false;
  }

  // Vehicle Drifting right/left too much **NOTE: to Tarnpreet Side Tolereance should be large**
  if (abs(sideDistanceDifference) > SIDE_DISTANCE_TOLERANCE)
  {
    // @TODO: set right and left motor to same magnitude
    // leftmotor=distanceScale*sideDistanceOutputPID;
    // rightmotor=distanceScale*sideDistanceOutputPID+jaggedvalue;
    return false;
  }

  // Vehicle Front Travel Value
  if (abs(frontDistanceDifference) > FRONT_DISTANCE_TOLERANCE)
  {
    // @TODO: set right and left motor to same magnitude
    // leftmotor=distanceScale*frontDistanceOutputPID;
    // rightmotor=distanceScale*frontDistanceOutputPID;
    return false;
  }

  // Increment TurnCount for next position
  if (turnCount != 10)
  {
    rightTurnRobot();
    return false;
  }

  return true;
}

void loop()
{
  // keep moving till goal reached
  if (!reachedGoal)
  {
    reachedGoal = hasGoalReached();
  }
}