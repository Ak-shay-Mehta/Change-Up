#include "math.h"
#include "v5.h"
#include "v5_vcs.h"
#include "vex_global.h"
#include "vex_triport.h"

class Robot {
private:
  // Controller Declaration
  vex::controller Controller;
  // Chassis Declaration
  vex::motor LeftChassisFront;
  vex::motor RightChassisFront;
  vex::motor LeftChassisBack;
  vex::motor RightChassisBack;
  // Body Declaration
  vex::motor primaryIntake;
  vex::motor sorter;
  vex::motor leftRoller;
  vex::motor rightRoller;
  // Sensor Declaration
  vex::encoder Encoder;
  vex::inertial Inertial;
  // PID K, P, and D constant Values
  const float gyro_Kp = 1.6;
  const float gyro_Ki = 0.05;
  const float gyro_Kd = 0.09;
  const float movement_Kp = 3;
  const float movement_Ki = 0.1;
  const float movement_Kd = 1.4;
  const float leftChassis_Kp = 0.4;
  const float leftChassis_Ki = 0;
  const float leftChassis_Kd = 0.1;
  const float rightChassis_Kp = 0.4;
  const float rightChassis_Ki = 0;
  const float rightChassis_Kd = 0.1;
  // Odometry Constants
  const float distanceBetweenWheels = 42;
  const float totalTicksInRevolution = 900;
  const float r = 6;
  // Threshold Constants
  const int turnThreshold = 10;
  const int smallerTurnThreshold = 3;
  const int movementThreshold = 2;
  // Power Enumeration
  enum Power {
    LOW_POWER = 10,
    MEDIUM_POWER = 50,
    HIGH_POWER = 100,
    MAX_POWER = 200
  };
  // PID Condition
  bool areWeThereYet = false;
  // PID Calculated K, P, and D values
  float gyroP;
  float gyroI;
  float gyroD;
  float movementP;
  float movementI;
  float movementD;
  float leftChassisP;
  float leftChassisI;
  float leftChassisD;
  float rightChassisP;
  float rightChassisI;
  float rightChassisD;
  // Last Errpr PID Values
  float lastgyroError;
  float lastMovementError;
  float lastLeftChassisError;
  float lastRightChassisError;
  // Requested PID Values
  float gyroRequestedValue;
  float movementRequestedValue;
  float leftChassisRequestedValue;
  float rightChassisRequestedValue;
  // Position Variables
  float currentX;
  float currentTheta;
  float currentY;
  // Curve peak
  bool capLeft = false;
  int finalCap;
  // Odometry Rotation Values
  float currentRotationLeft;
  float currentRotationRight;
  // Odometry Displacements
  float leftDisplacement;
  float rightDisplacement;
  float centerDisplacement;
  // Odometry Ticks
  float currentLeftTick;
  float deltaLeftTick;
  float currentRightTick;
  float deltaRightTick;
  // Odometry Previous Calculated Values
  float lastRightTick = 0;
  float lastRotationLeft = 0;
  float lastRotationRight = 0;
  float lastX = 0;
  float lastY = 0;
  float lastTheta = 0;
  float lastLeftTick = 0;
  // Max Turn PID Value
  int maxTurnPower = 140;
  // Odometry Functions
  void odometryCalculationLoop();
  void moveForwardWithoutCheck(int moveForwardValue, float theta,
                               int powerValue);
  void moveBackwardWithoutCheck(int moveBackwardValue, float theta,
                                int powerValue);
  void resetChassis();
  void checkTurns(int theta);
  void turnController();
  void curveController();
public:
  vex::brain Brain;
  // Constructor
  Robot();
  Robot(vex::brain Brain, vex::controller Controller, vex::motor LeftChassisFront, vex::motor RightChassisFront, vex::motor LeftChassisBack, vex::motor RightChassisBack, vex::motor primaryIntake, vex::motor sorter, vex::motor leftRoller, vex::motor rightRoller, vex::encoder Encoder, vex::inertial Inertial);
  // Wait Time
  void wait(int waitTime);
  //UI variables
  int indexer = 0;
  int autonomousSelection = 0;
  // Movement
  void moveForward(int moveForwardValue, float theta, int powerValue);
  void moveBackward(int moveBackwardValue, float theta, int powerValue);
  void turnPID(float requestedValue);
  void curvePID(float leftRequestedValue, float rightRequestedValue, int cap);
  // Curve Switcher
  void switchCurveLeft();
  void switchCurveRight();
  void rollerSpin(int rollerRotation, int rollerPower);
  void intakeSpin(int rollerRotation, int rollerPower);
  // Odometry Init Function
  void odometryInit();
  // Autonomous
  void runSkills();
  void runTwoGoal();
  void runOneGoal();
  void runThreeGoal();
  // User Control
  void userControl();
};