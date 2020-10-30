/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Akshay.Mehta                                             */
/*    Created:      Sat Nov 30 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include "ui.h"
// Autonomous state plus different buttons from UI
void runautonomous(void) {
  bool threeGoal = buttons[4].state;
  bool twoGoal = buttons[5].state;
  bool oneGoal = buttons[6].state;
  bool isSkills = buttons[2].state;
  if (threeGoal) {
    robot.runThreeGoal();
  } else if (twoGoal) {
    robot.runTwoGoal();
  } else if (oneGoal) {
    robot.runOneGoal();
  } else if (isSkills) {
    robot.runSkills();
  }
}

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    robot.userControl();
  }
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(runautonomous);
  Competition.drivercontrol(usercontrol);
  robot.Brain.Screen.pressed(buttonCallback);
  robot.Brain.Screen.released(buttonReleased);
  robot.Brain.Screen.pressed(userTouchCallbackPressed);
  robot.Brain.Screen.released(userTouchCallbackReleased);
  drawLoadingPage();
  drawHomePage();

  // Prevent main from exiting with an infinite loop.
  while (1) {
    robot.wait(100); // Sleep the task for a short amount of time to
  }
}