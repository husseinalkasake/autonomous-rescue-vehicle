// #include <PID_v1.h>
// #include <stdlib.h>
// #include <Wire.h>
// #include <VL53L1X.h>

// VL53L1X sensor1;
// VL53L1X sensor2;
// VL53L1X sensor3;
// VL53L1X sensor4;

// // @TODO: Adjust tolerance to viable numbers
// #define STABILITYTOLERANCE 0.1   // Assume degrees
// #define ANGLETOLERANCE 1         // Assume degrees
// #define FRONTDISTANCETOLERANCE 1 // Assume cm
// #define LEFTDISTANCETOLERANCE 1  // Assume cm

// // Global Constants
// double TILEDISTANCE = 0;
// double ANGLETARE = 0;
// double STABILITYTARE = 0;
// int TURNCOUNT = 0;

// bool hasGoalReached(double FlightofTimeFront, double FlightofTimeLeft, double CompassSensor, double StableSensor)
// {
//     // Scales TODO@input motor characterization scale here
//     double distanceScale = 1.00;
//     double angleScale = 1.00;

//     // Processed Sensor Values
//     double ValStability = (StableSensor - STABILITYTARE);
//     double ValAngle = (CompassSensor - ANGLETARE);
//     double ValTileDistanceFront = FlightofTimeFront;
//     double ValTileDistanceLeft = FlightofTimeLeft;

//     // Desired Objective Values
//     double DesiredAngle = (TURNCOUNT * 90) % 360;
//     double DesiredTileDistanceFront = (TILEDISTANCE * ((TURNCOUNT + 1) / 4)) + (TILEDISTANCE / 2.0);
//     double DesiredTileDistanceLeft = (TILEDISTANCE * ((TURNCOUNT) / 4)) + (TILEDISTANCE / 2.0);

//     // Desired Point Difference
//     double DifferenceAngleInput = ValAngle - DesiredAngle;
//     double DifferenceFrontDistanceInput = ValTileDistanceFront - DesiredTileDistanceFront;
//     double DifferenceSideDistanceInput = ValTileDistanceLeft - DesiredTileDistanceLeft;

//     // Outputs for PID's
//     double PIDAngleOutput = 0;
//     double PIDFrontDistanceOutput = 0;
//     double PIDSideDistanceOutput = 0;

//     //Declaration of PID's
//     PID anglePID(&DifferenceAngleInput, &PIDAngleOutput, 0, 2, 5, 1, DIRECT);
//     PID distanceFrontPID(&DifferenceFrontDistanceInput, &PIDFrontDistanceOutput, 0, 2, 5, 1, DIRECT);
//     PID distanceSidePID(&DifferenceSideDistanceInput, &PIDSideDistanceOutput, 0, 2, 5, 1, DIRECT);

//     //Configuration of PID's
//     anglePID.SetMode(AUTOMATIC);
//     distanceFrontPID.SetMode(AUTOMATIC);
//     distanceSidePID.SetMode(AUTOMATIC);

//     anglePID.Compute();
//     distanceFrontPID.Compute();
//     distanceSidePID.Compute();

//     // Vehicle Tilted
//     if (abs(ValStability) > STABILITYTOLERANCE)
//     {
//         // @TODO: Set both left and right motors to a sensible blind forward distance that we know works
//         return false;
//     }

//     // Vehicle Heading drifting
//     if (abs(DifferenceAngleInput) > ANGLETOLERANCE)
//     {
//         // @TODO: set right and left motor to opposite magnitude
//         // leftmotor=angleScale*PIDAngleOutput;
//         // rightmotor=-(angleScale*PIDAngleOutput);
//         return false;
//     }

//     // Vehicle Drifting right/left too much **NOTE: to Tarnpreet Side Tolereance should be large**
//     if (abs(DifferenceSideDistanceInput) > LEFTDISTANCETOLERANCE)
//     {
//         // @TODO: set right and left motor to same magnitude
//         // leftmotor=distanceScale*PIDFrontDistanceOutput;
//         // rightmotor=distanceScale*PIDFrontDistanceOutput+jaggedvalue;
//         return false;
//     }

//     // Vehicle Front Travel Value
//     if (abs(DifferenceFrontDistanceInput) > FRONTDISTANCETOLERANCE)
//     {
//         // @TODO: set right and left motor to same magnitude
//         // leftmotor=distanceScale*PIDFrontDistanceOutput;
//         // rightmotor=distanceScale*PIDFrontDistanceOutput;
//         return false;
//     }

//     // Increment TurnCount for next position
//     if (TURNCOUNT != 10)
//     {
//         TURNCOUNT++;
//         return false;
//     }

//     return true;
// }

// int main()
// {
//     bool Stop = false;
//     double FlightofTimeFront = 0;
//     double FlightofTimeLeft = 0;
//     double CompassSensor = 0;
//     double StableSensor = 0;

//     // TILEDISTANCE=0; @TODO: Pull in left sensor and multiply by two
//     // ANGLETARE=0; @TODO: Pull in Angle sensor at start
//     // STABILITYTARE=0; @TODO: Pull in Angle sensor at start

//     // MAIN BLACKBOX FUNCTION
//     while (Stop)
//     {

//         // Pull sensor data here @TODO: pull in real time sensor data here
//         // double FlightofTimeFront
//         // double FlightofTimeLeft;
//         // double CompassSensor;
//         // double StableSensor;

//         // Black Box
//         Stop = hasGoalReached(FlightofTimeFront, FlightofTimeLeft, CompassSensor, StableSensor);
//     }

//     // @TODO: Set both motor speed to zero

//     return 0;
// }
