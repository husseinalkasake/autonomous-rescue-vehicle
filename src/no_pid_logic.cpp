// #include <Arduino.h>
// #include <stdlib.h>

// /// Non PID Traversal Logic.
// /// Just for reference. Probably won't be used

// // Tolerances
// #define ANGLE_TOLERANCE_VALUE 0.05
// #define DISTANCE_TOLERANCE_VALUE 0.25

// // Global Variables
// double tileDistance = 0;
// double angleTare = 0;
// int turnCount = 0;

// // sensor mocks
// // TODO: RETURN ACTUAL SENSOR VALUE
// double getLeftSensorDistance()
// {
//     return 0;
// }
// double getFrontSensorDistance()
// {
//     return 0;
// }
// double getAngleSensor()
// {
//     return 0;
// }
// // return pointing down sensor (probably not needed)
// // TODO (if needed): return actual value
// bool approachingDrop()
// {
//     return false;
// }

// // Motor Mock
// void driveForward()
// {
//     // TODO: set motors to forward speed
// }
// void stopRobot()
// {
//     // TODO: stop robot
// }
// void rightTurnRobot()
// {
//     stopRobot();

//     // TODO: right turn robot

//     // turn until angle matches expected one after turn
//     while (
//         abs(getAngleSensor() - angleTare - (((turnCount * 90) % 360) + 90)) >
//         ANGLE_TOLERANCE_VALUE)
//     {
//     }

//     stopRobot();
//     driveForward();

//     // increment turn count
//     turnCount++;
// }
// void selfAlign()
// {
//     double startingAngle = getAngleSensor();
//     bool shouldTurnLeft = startingAngle > (turnCount * 90) % 360;
//     if (shouldTurnLeft)
//     {
//         // TODO: TURN MOTOR LEFT
//     }
//     else
//     {
//         // TODO: TURN MOTOR RIGHT
//     }

//     while (
//         abs(getAngleSensor() - angleTare - ((turnCount * 90) % 360)) >
//         ANGLE_TOLERANCE_VALUE)
//     {
//     }

//     stopRobot();
//     driveForward();
// }

// void setup()
// {
//     // default variables to start course
//     tileDistance = 2 * getLeftSensorDistance();
//     angleTare = getAngleSensor();
//     turnCount = 0;
// }

// bool hasReached = false;
// bool hasReachedGoal()
// {
//     // slow down if approaching drop
//     driveForward();

//     // self align if angle is too far off desired angle
//     if (
//         abs(getAngleSensor() - angleTare - ((turnCount * 90) % 360)) >
//         ANGLE_TOLERANCE_VALUE)
//     {
//         selfAlign();
//         return false;
//     }

//     // keep going if robot hasn't reached turning point
//     if (
//         abs(
//             getFrontSensorDistance() -
//             (tileDistance * ((turnCount + 1) / 4) + tileDistance / 2)) > DISTANCE_TOLERANCE_VALUE)
//     {
//         return false;
//     }

//     // stop if got to center of course
//     if (turnCount == 10)
//     {
//         stopRobot();
//         return true;
//     }
//     // turn if reached known turning point
//     else
//     {
//         rightTurnRobot();
//     }
//     return false;
// }

// void loop()
// {
//     if (!hasReached)
//     {
//         hasReached = hasReachedGoal();
//     }
// }