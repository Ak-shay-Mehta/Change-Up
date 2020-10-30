#include "robot.h"

// Constructor Implementation
Robot::Robot(vex::brain Brain, vex::controller Controller,
             vex::motor LeftChassisFront, vex::motor RightChassisFront,
             vex::motor LeftChassisBack, vex::motor RightChassisBack,
             vex::motor primaryIntake, vex::motor sorter, vex::motor leftRoller,
             vex::motor rightRoller, vex::encoder Encoder,
             vex::inertial Inertial)
    : LeftChassisFront(LeftChassisFront), RightChassisFront(RightChassisFront),
      LeftChassisBack(LeftChassisBack), RightChassisBack(RightChassisBack),
      primaryIntake(primaryIntake), sorter(sorter), leftRoller(leftRoller),
      rightRoller(rightRoller), Encoder(Encoder), Inertial(Inertial),
      Brain(Brain) {}

void Robot::wait(int waitTime) { vex::task::sleep(waitTime); }
void Robot::odometryInit() {
  LeftChassisFront.resetRotation();
  RightChassisFront.resetRotation();
  LeftChassisBack.resetRotation();
  RightChassisBack.resetRotation();
  lastRotationLeft = 0;
  lastRotationRight = 0;
  lastX = 0;
  lastY = 0;
  lastTheta = 0;
  lastLeftTick = 0;
  lastRightTick = 0;
}

void Robot::odometryCalculationLoop() {
  currentLeftTick = LeftChassisFront.rotation(vex::rotationUnits::raw);
  deltaLeftTick = currentLeftTick - lastLeftTick;
  currentRightTick = RightChassisFront.rotation(vex::rotationUnits::raw);
  deltaRightTick = currentRightTick - lastRightTick;

  currentRotationLeft =
      2 * 3.14159265358979 * r * (deltaLeftTick / totalTicksInRevolution);
  currentRotationRight =
      2 * 3.14159265358979 * r * (deltaRightTick / totalTicksInRevolution);

  leftDisplacement = currentRotationLeft - lastRotationLeft;
  rightDisplacement = currentRotationRight - lastRotationRight;
  centerDisplacement = ((rightDisplacement + leftDisplacement) / 2);

  currentX = lastX + (centerDisplacement * cos(lastTheta));
  currentY = lastY + (centerDisplacement * sin(lastTheta));
  currentTheta = Inertial.rotation(vex::rotationUnits::deg);
}

void Robot::moveForwardWithoutCheck(int moveForwardValue, float theta,
                                    int powerValue) // 50
{
  while (currentX < moveForwardValue - movementThreshold) {
    checkTurns(theta);
    odometryCalculationLoop();
    LeftChassisBack.spin(vex::directionType::fwd, powerValue,
                         vex::velocityUnits::pct);
    LeftChassisFront.spin(vex::directionType::fwd, powerValue,
                          vex::velocityUnits::pct);
    RightChassisBack.spin(vex::directionType::fwd, powerValue,
                          vex::velocityUnits::pct);
    RightChassisFront.spin(vex::directionType::fwd, powerValue,
                           vex::velocityUnits::pct);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
  }
  resetChassis();
}
void Robot::moveBackwardWithoutCheck(int moveBackwardValue, float theta,
                                     int powerValue) {
  while (currentX > moveBackwardValue + movementThreshold) {
    checkTurns(theta);
    odometryCalculationLoop();
    LeftChassisBack.spin(vex::directionType::fwd, -powerValue,
                         vex::velocityUnits::pct);
    LeftChassisFront.spin(vex::directionType::fwd, -powerValue,
                          vex::velocityUnits::pct);
    RightChassisBack.spin(vex::directionType::fwd, -powerValue,
                          vex::velocityUnits::pct);
    RightChassisFront.spin(vex::directionType::fwd, -powerValue,
                           vex::velocityUnits::pct);
  }
  resetChassis();
}
void Robot::moveBackward(int moveBackwardValue, float theta, int powerValue) {
  while (currentX > moveBackwardValue + movementThreshold) {
    checkTurns(theta);
    odometryCalculationLoop();
    LeftChassisBack.spin(vex::directionType::fwd, -powerValue,
                         vex::velocityUnits::pct);
    LeftChassisFront.spin(vex::directionType::fwd, -powerValue,
                          vex::velocityUnits::pct);
    RightChassisBack.spin(vex::directionType::fwd, -powerValue,
                          vex::velocityUnits::pct);
    RightChassisFront.spin(vex::directionType::fwd, -powerValue,
                           vex::velocityUnits::pct);
  }
  resetChassis();
  wait(100);
  moveBackwardWithoutCheck(moveBackwardValue, theta, LOW_POWER);
  resetChassis();
}
void Robot::moveForward(int moveForwardValue, float theta, int powerValue) {
  while (currentX < moveForwardValue - movementThreshold) {
    checkTurns(theta);
    odometryCalculationLoop();
    LeftChassisBack.spin(vex::directionType::fwd, powerValue,
                         vex::velocityUnits::pct);
    LeftChassisFront.spin(vex::directionType::fwd, powerValue,
                          vex::velocityUnits::pct);
    RightChassisBack.spin(vex::directionType::fwd, powerValue,
                          vex::velocityUnits::pct);
    RightChassisFront.spin(vex::directionType::fwd, powerValue,
                           vex::velocityUnits::pct);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
  }
  resetChassis();
  wait(100);
  moveBackwardWithoutCheck(moveForwardValue, theta, LOW_POWER);
  resetChassis();
}
void Robot::resetChassis() {
  LeftChassisBack.stop(vex::brakeType::brake);
  LeftChassisFront.stop(vex::brakeType::brake);
  RightChassisBack.stop(vex::brakeType::brake);
  RightChassisFront.stop(vex::brakeType::brake);
}
void Robot::checkTurns(int theta) {
  if (currentTheta < theta - turnThreshold) {
    while (currentTheta < theta - turnThreshold) {
      odometryCalculationLoop();
      turnPID(theta);
    }
  }
  if (currentTheta > theta + turnThreshold) {
    while (currentTheta > theta + turnThreshold) {
      odometryCalculationLoop();
      turnPID(theta);
    }
  }
}
void Robot::turnController() {
  float gyroSensorCurrentValue;
  float gyroError;
  float gyroDrive;

  Brain.resetTimer();
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);

  while ((!areWeThereYet)) {
    odometryCalculationLoop();
    Brain.Screen.print(Brain.timer(vex::timeUnits::msec));

    // Read the sensor value and scale
    gyroSensorCurrentValue = Inertial.rotation(vex::rotationUnits::deg);

    // calculate error
    gyroError = gyroRequestedValue - gyroSensorCurrentValue;
    if (gyroError < smallerTurnThreshold &&
        gyroError > -smallerTurnThreshold) {
      resetChassis();
      break;
    }
    Brain.Screen.setCursor(1, 1);

    // calculate drive
    gyroP = (gyro_Kp * gyroError);

    static float gyroI = 0;
    gyroI += gyroError * gyro_Ki;
    if (gyroI > 1)
      gyroI = 1;
    if (gyroI < -1)
      gyroI = -1;

    gyroD = gyroError - lastgyroError;

    gyroDrive = gyroP + gyroI;

    // limit drive
    if (gyroDrive > maxTurnPower)
      gyroDrive = maxTurnPower;
    if (gyroDrive < -maxTurnPower)
      gyroDrive = -maxTurnPower;

    // send to motor
    int powerValue = gyroDrive;

    LeftChassisFront.spin(vex::directionType::fwd, powerValue,
                          vex::velocityUnits::rpm);
    LeftChassisBack.spin(vex::directionType::fwd, powerValue,
                         vex::velocityUnits::rpm);
    RightChassisFront.spin(vex::directionType::fwd, -powerValue,
                           vex::velocityUnits::rpm);
    RightChassisBack.spin(vex::directionType::fwd, -powerValue,
                          vex::velocityUnits::rpm);

    lastgyroError = gyroError;
    wait(10);
  }
  resetChassis();
}
void Robot::turnPID(float requestedValue) {
  gyroRequestedValue = requestedValue;
  turnController();
}
void Robot::curveController() {

  float leftChassisSensorCurrentValue;
  float leftChassisError;
  float leftChassisDrive;
  float rightChassisSensorCurrentValue;
  float rightChassisError;
  float rightChassisDrive;

  while ((!areWeThereYet)) {
    odometryCalculationLoop();
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 7);
    Brain.Screen.print(Brain.timer(vex::timeUnits::msec));
    Brain.Screen.setCursor(1, 1);

    // Read the sensor value and scale
    leftChassisSensorCurrentValue =
        (LeftChassisFront.rotation(vex::rotationUnits::raw) +
         LeftChassisBack.rotation(vex::rotationUnits::raw)) /
        2;
    rightChassisSensorCurrentValue =
        (RightChassisFront.rotation(vex::rotationUnits::raw) +
         RightChassisBack.rotation(vex::rotationUnits::raw)) /
        2;

    // calculate error
    rightChassisError =
        rightChassisRequestedValue - rightChassisSensorCurrentValue;
    leftChassisError =
        leftChassisRequestedValue - leftChassisSensorCurrentValue;
    if (rightChassisError < 20 && rightChassisError > -20) {
      RightChassisFront.spin(vex::directionType::fwd, 0,
                             vex::velocityUnits::rpm);
      RightChassisBack.spin(vex::directionType::fwd, 0,
                            vex::velocityUnits::rpm);
    }
    if (leftChassisError < 20 && leftChassisError > -20) {
      LeftChassisBack.spin(vex::directionType::fwd, 0, vex::velocityUnits::rpm);
      LeftChassisFront.spin(vex::directionType::fwd, 0,
                            vex::velocityUnits::rpm);
    }
    if (rightChassisError < 20 && rightChassisError > -20 &&
        leftChassisError < 10 && leftChassisError > -10) {
      resetChassis();
      break;
    }
    Brain.Screen.setCursor(1, 1);

    // calculate drive
    leftChassisP = (leftChassis_Kp * leftChassisError);
    rightChassisP = (rightChassis_Kp * rightChassisError);

    rightChassisI += rightChassisError * rightChassis_Ki;
    if (rightChassisI > 1)
      rightChassisI = 1;
    if (rightChassisI < -1)
      rightChassisI = -1;

    leftChassisI += leftChassisError * leftChassis_Ki;
    if (leftChassisI > 1)
      leftChassisI = 1;
    if (leftChassisI < -1)
      leftChassisI = -1;

    leftChassisD = leftChassisError - lastLeftChassisError;
    rightChassisD = rightChassisError - lastRightChassisError;

    rightChassisDrive = rightChassisP + rightChassisI;
    leftChassisDrive = leftChassisP + leftChassisI;

    // limit drive
    if (capLeft) {
      if (leftChassisDrive > finalCap)
        leftChassisDrive = finalCap;
      if (leftChassisDrive < -finalCap)
        leftChassisDrive = -finalCap;
      if (rightChassisDrive > MAX_POWER)
        rightChassisDrive = MAX_POWER;
      if (rightChassisDrive < -MAX_POWER)
        rightChassisDrive = -MAX_POWER;
    } else {
      if (leftChassisDrive > MAX_POWER)
        leftChassisDrive = MAX_POWER;
      if (leftChassisDrive < -MAX_POWER)
        leftChassisDrive = -MAX_POWER;
      if (rightChassisDrive > finalCap)
        rightChassisDrive = finalCap;
      if (rightChassisDrive < -finalCap)
        rightChassisDrive = -finalCap;
    }

    // send to motor
    int rightx = rightChassisDrive;
    int leftx = leftChassisDrive;

    LeftChassisFront.spin(vex::directionType::fwd, leftx,
                          vex::velocityUnits::rpm);
    LeftChassisBack.spin(vex::directionType::fwd, leftx,
                         vex::velocityUnits::rpm);
    RightChassisFront.spin(vex::directionType::fwd, rightx,
                           vex::velocityUnits::rpm);
    RightChassisBack.spin(vex::directionType::fwd, rightx,
                          vex::velocityUnits::rpm);

    lastRightChassisError = rightChassisError;
    lastLeftChassisError = leftChassisError;
  }
  resetChassis();
}
void Robot::curvePID(float leftRequestedValue, float rightRequestedValue,
                     int cap) {
  leftChassisRequestedValue = leftRequestedValue;
  rightChassisRequestedValue = rightRequestedValue;
  finalCap = cap;
  curveController();
}
void Robot::switchCurveLeft() {
  capLeft = false;
  resetChassis();
}
void Robot::switchCurveRight() {
  capLeft = true;
  resetChassis();
}
void Robot::rollerSpin(int rollerRotation, int rollerPower) {
  leftRoller.startRotateFor(rollerRotation, vex::rotationUnits::raw,
                            rollerPower, vex::velocityUnits::pct);
  rightRoller.startRotateFor(rollerRotation, vex::rotationUnits::raw,
                             rollerPower, vex::velocityUnits::pct);
}
void Robot::intakeSpin(int intakeRotation, int intakePower) {
  primaryIntake.startRotateFor(-intakeRotation, vex::rotationUnits::raw,
                               intakePower, vex::velocityUnits::pct);
  sorter.startRotateFor(intakeRotation, vex::rotationUnits::raw, intakePower,
                        vex::velocityUnits::pct);
}
void Robot::runSkills() {
      //////////////////////////////////////////////First Score//////////////////////////////////////////
      rollerSpin(6400, HIGH_POWER);
      moveForward(60,0,30);//Custom Power
      wait(500);
      turnPID(126.41);
      wait(500);
      moveForward(140,126.41,MEDIUM_POWER);
      wait(500);
      moveForward(141,126.41,MEDIUM_POWER);
      rollerSpin(300, HIGH_POWER);
      intakeSpin(1000, HIGH_POWER);
      wait(1000);
       //////////////////////////////////////////////Second Score//////////////////////////////////////////
      moveBackward(90,126.41,30);//Custom Power
      wait(300);
      turnPID(91);
      moveBackward(0,91,MEDIUM_POWER);
      wait(300);
      turnPID(178);
      rollerSpin(1500, HIGH_POWER);
      moveForward(23,178,MEDIUM_POWER);
      rollerSpin(300, HIGH_POWER);
      intakeSpin(2400, HIGH_POWER);
      wait(900);
       //////////////////////////////////////////////Third Score//////////////////////////////////////////
      moveBackward(11,178,MEDIUM_POWER);
      turnPID(269.85);
      rollerSpin(6500, HIGH_POWER);
      intakeSpin(400, HIGH_POWER);
      moveForward(122,267.85,MEDIUM_POWER);
      turnPID(226.35);
      rollerSpin(2500, HIGH_POWER);
      intakeSpin(600, HIGH_POWER);
      moveForward(166,226.35,MEDIUM_POWER);
      rollerSpin(1000, HIGH_POWER);
      intakeSpin(3000, HIGH_POWER);
      wait(1500);
      odometryInit();
      odometryCalculationLoop();
       //////////////////////////////////////////////Fourth Score//////////////////////////////////////////
       moveBackward(-73,226.35,MEDIUM_POWER);
       wait(300);
       turnPID(264);
       rollerSpin(8000, 100);
       moveForward(-7,262,MEDIUM_POWER);
       rollerSpin(1000, 100);
       intakeSpin(800, 100);
       moveBackward(-96,262,MEDIUM_POWER);
       wait(300);
       turnPID(322);
       rollerSpin(7500, 100);
       moveForward(-2,322,MEDIUM_POWER);
       wait(300);
       turnPID(269);
       moveForward(21,269,MEDIUM_POWER);
       rollerSpin(300, HIGH_POWER);
       intakeSpin(2000, 100);
       wait(1200);
       //////////////////////////////////////////////Fifth Score//////////////////////////////////////////
       moveBackward(-19,269,MEDIUM_POWER);
       wait(300);
       turnPID(332);
       rollerSpin(1000, 100);
       moveForward(158,332,MEDIUM_POWER);
       rollerSpin(300, HIGH_POWER);
       intakeSpin(2500, 100);
       odometryInit();
       odometryCalculationLoop();
       wait(2400);
       //////////////////////////////////////////////Sixth Score//////////////////////////////////////////
       moveBackward(-32, 332, MEDIUM_POWER);
       wait(300);
       turnPID(437.06);
       wait(200);
       rollerSpin(5000, 100);
       moveForward(82, 437.06, MEDIUM_POWER);
       wait(300);
       turnPID(360.68);
       wait(300);
       moveForward(115, 360.68, MEDIUM_POWER);
       rollerSpin(300, HIGH_POWER);
       intakeSpin(4000, 100);
       wait(5000);
}
void Robot::userControl() {
  if (Controller.ButtonLeft.pressing()) {
    Inertial.calibrate();
    odometryInit();
    odometryCalculationLoop();
    wait(3000);                             
  }
  if (Controller.ButtonX.pressing()) {
    odometryCalculationLoop();
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("(");
    Controller.Screen.print(currentX);
    Controller.Screen.print(", ");
    Controller.Screen.print(currentY);
    Controller.Screen.print(", ");
    Controller.Screen.print(currentTheta);
    Controller.Screen.print(")");
    Inertial.calibrate();
  }
  // User control code here, inside the loop
  if (Controller.ButtonR1.pressing()) {
    primaryIntake.spin(vex::directionType::rev, HIGH_POWER,
                       vex::velocityUnits::pct);
    sorter.spin(vex::directionType::fwd, HIGH_POWER, vex::velocityUnits::pct);
  } else if (Controller.ButtonR2.pressing()) {
    primaryIntake.spin(vex::directionType::rev, HIGH_POWER,
                       vex::velocityUnits::pct);
    sorter.spin(vex::directionType::rev, HIGH_POWER, vex::velocityUnits::pct);
  } else if (Controller.ButtonY.pressing()) {
    primaryIntake.spin(vex::directionType::fwd, HIGH_POWER,
                       vex::velocityUnits::pct);
    sorter.spin(vex::directionType::rev, HIGH_POWER, vex::velocityUnits::pct);
  } else {
    primaryIntake.stop(vex::brakeType::brake);
    sorter.stop(vex::brakeType::brake);
  }
  if (Controller.ButtonL1.pressing()) {
    leftRoller.spin(vex::directionType::fwd, HIGH_POWER,
                    vex::velocityUnits::pct);
    rightRoller.spin(vex::directionType::fwd, HIGH_POWER,
                     vex::velocityUnits::pct);
  } else if (Controller.ButtonL2.pressing()) {
    leftRoller.spin(vex::directionType::rev, HIGH_POWER,
                    vex::velocityUnits::pct);
    rightRoller.spin(vex::directionType::rev, HIGH_POWER,
                     vex::velocityUnits::pct);
  } else {
    leftRoller.stop();
    rightRoller.stop();
  }
  if (Controller.ButtonA.pressing()) {
    odometryInit();
    odometryCalculationLoop();
  }
  RightChassisBack.spin(vex::directionType::fwd,
                        Controller.Axis3.position() -
                            Controller.Axis1.position(),
                        vex::velocityUnits::pct);

  LeftChassisFront.spin(vex::directionType::fwd,
                        Controller.Axis3.position() +
                            Controller.Axis1.position(),
                        vex::velocityUnits::pct);
  RightChassisFront.spin(
      vex::directionType::fwd,
      (Controller.Axis3.position() - Controller.Axis1.position()),
      vex::velocityUnits::pct);
  LeftChassisBack.spin(
      vex::directionType::fwd,
      (Controller.Axis3.position() + Controller.Axis1.position()),
      vex::velocityUnits::pct);
}
void Robot::runTwoGoal() {
  odometryInit();
  odometryCalculationLoop();
  rollerSpin(3220, HIGH_POWER);
  curvePID(100, 900, 150);
  odometryInit();
  odometryCalculationLoop();
  moveForward(38, 0, 30); // Custom 30 Power
  intakeSpin(700, HIGH_POWER);
  wait(600);
  odometryInit();
  odometryCalculationLoop();
  wait(200);
  moveBackward(-53, 0, 60); // Custom 60 Power
  wait(200);
  turnPID(-46);
  wait(200);
  odometryInit();
  odometryCalculationLoop();
  intakeSpin(550, HIGH_POWER);
  rollerSpin(500, HIGH_POWER);
  moveBackward(-75, 0, 60); // Custom 60 Power
  wait(200);
  turnPID(-117);
  odometryInit();
  odometryCalculationLoop();
  wait(200);
  moveForward(35, 0, 80);
  intakeSpin(2400, HIGH_POWER);
  wait(700);
  wait(3000);
}
void Robot::runOneGoal() {
  odometryInit();
  odometryCalculationLoop();
  rollerSpin(3220, HIGH_POWER);
  curvePID(100, 900, 150);
  odometryInit();
  odometryCalculationLoop();
  moveForward(38, 0, 30); // Custom 30 Power
  intakeSpin(700, HIGH_POWER);
  wait(600);
  wait(3000);
}
void Robot::runThreeGoal() {
  odometryInit();
  rollerSpin(3220, HIGH_POWER);
  curvePID(100, 900, 150);
  odometryInit();
  moveForward(38, 0, 30); // Custom 30 Power
  intakeSpin(700, HIGH_POWER);
  wait(600);
  odometryInit();
  wait(200);
  moveBackward(-53, 0, 60); // Custom 60 Power
  wait(200);
  turnPID(-46);
  wait(200);
  odometryInit();
  intakeSpin(550, HIGH_POWER);
  rollerSpin(500, HIGH_POWER);
  moveBackward(-75, 0, 60); // Custom 60 power
  wait(200);
  turnPID(-117);
  odometryInit();
  wait(200);
  moveForward(35, 0, 80); // Custom 80 power
  intakeSpin(2400, HIGH_POWER);
  wait(700);
  moveBackward(0, 0, 60); // Custom 60 power
  wait(400);
  turnPID(-189);
  odometryInit();
  wait(400);
  rollerSpin(6500, HIGH_POWER);
  moveForward(163, 0, 60); // Custom 60 power
  intakeSpin(2400, HIGH_POWER);
  wait(3000);
}