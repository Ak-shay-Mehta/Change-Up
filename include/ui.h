#include "robot.h"
vex::brain Brain;
vex::competition Competition;
vex::controller Controller;
// Chassis Declaration
vex::motor LeftChassisFront = vex::motor(vex::PORT18);
vex::motor RightChassisFront = vex::motor(vex::PORT20, true);
vex::motor LeftChassisBack = vex::motor(vex::PORT17, true);
vex::motor RightChassisBack = vex::motor(vex::PORT19);
// Body Declaration
vex::motor primaryIntake = vex::motor(vex::PORT14);
vex::motor sorter = vex::motor(vex::PORT13);
vex::motor leftRoller = vex::motor(vex::PORT11, true);
vex::motor rightRoller = vex::motor(vex::PORT12);
// Sensor Declaration
vex::encoder Encoder = vex::encoder(Brain.ThreeWirePort.A);
vex::inertial Inertial = vex::inertial(vex::PORT16);
Robot robot(Brain, Controller, LeftChassisFront, RightChassisFront,
            LeftChassisBack, RightChassisBack, primaryIntake, sorter,
            leftRoller, rightRoller, Encoder, Inertial);
            
// UI Functions
void displayButtonControls(int index, bool pressed);
int findButton(int16_t xpos, int16_t ypos);
void drawBackButton(int x, int y, int width, int height);
void drawWiringPage();
void drawHomePage();
void buttonReleased();
void buttonCallback();
void initButtons();
void userTouchCallbackPressed();
void userTouchCallbackReleased();
typedef struct _button {
  int xpos;
  int ypos;
  int width;
  int height;
  bool state;
  vex::color offColor;
  vex::color onColor;
  const char *label;
} button;
button buttons[] = {{30, 30, 45, 45, false, 0xE00000, 0x0000E0, "Ally"},
                    {150, 30, 45, 45, false, 0x303030, 0x7fff00, "Far"},
                    {270, 30, 45, 45, false, 0x303030, 0x7fff00, "Skills"},
                    {390, 30, 45, 45, false, 0x303030, 0x7fff00, "Def"},
                    {30, 90, 45, 45, false, 0x404040, 0x7fff00, "3 Goal"},
                    {150, 90, 45, 45, false, 0x404040, 0x7fff00, "2 Goal"},
                    {270, 90, 45, 45, false, 0x404040, 0x7fff00, "1 Goal"},
                    {390, 90, 45, 45, false, 0x404040, 0x7fff00, "4"},
                    {30, 150, 45, 45, false, 0x404040, 0x7fff00, "5"},
                    {150, 150, 45, 45, false, 0x404040, 0x7fff00, "6"},
                    {270, 150, 45, 45, false, 0x404040, 0x7fff00, "7"},
                    {390, 150, 45, 45, false, 0x404040, 0x7fff00, "8"}};
void buttonReleased() {}
void buttonCallback() {
  while (Brain.Screen.pressing()) {
    if (Brain.Screen.xPosition() < 100 && Brain.Screen.yPosition() < 100 &&
        robot.indexer == 0) {
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.clearScreen();
      Brain.Screen.setPenWidth(1);
      Brain.Screen.setFont(vex::fontType::mono20);
      Brain.Screen.setPenColor(vex::color::white);
      Brain.Screen.setCursor(0, 1);
      {
        int index;
        int xpos = Brain.Screen.xPosition();
        int ypos = Brain.Screen.yPosition();

        if ((index = findButton(xpos, ypos)) >= 0) {
          displayButtonControls(index, true);
        }
      }
      {
        int index;
        int xpos = Brain.Screen.xPosition();
        int ypos = Brain.Screen.yPosition();

        if ((index = findButton(xpos, ypos)) >= 0) {
          // clear all buttons to false, ie. unselected
          //      initButtons();

          // now set this one as true
          if (buttons[index].state == true) {
            buttons[index].state = false;
          } else {
            buttons[index].state = true;
          }

          // save as auton selection
          robot.autonomousSelection = index;

          displayButtonControls(index, false);
        }
      }

      drawBackButton(195, 200, 90, 40);
      robot.indexer = 2;
    }
    if (Brain.Screen.xPosition() > 329 && Brain.Screen.yPosition() < 100 &&
        robot.indexer == 0) {
      drawWiringPage();
      drawBackButton(370, 190, 100, 40);
      robot.indexer = 1;
    }
    if (Brain.Screen.xPosition() > 370 && Brain.Screen.yPosition() > 190 &&
        robot.indexer == 1) {
      drawHomePage();
      robot.indexer = 0;
    }
    if (Brain.Screen.xPosition() > 195 && Brain.Screen.yPosition() > 200 &&
        robot.indexer == 2 && Brain.Screen.xPosition() < 295) {
      drawHomePage();
      Brain.Screen.pressed(buttonCallback);
      Brain.Screen.released(buttonReleased);
      robot.indexer = 0;
    }
  }
}
void drawHomePage() {
  Brain.Screen.pressed(buttonCallback);
  Brain.Screen.released(buttonReleased);
  Brain.Screen.clearScreen("#1A5276");
  Brain.Screen.setPenWidth(2);
  Brain.Screen.setPenColor(vex::color::black);
  Brain.Screen.setFillColor(vex::color::white);
  Brain.Screen.drawCircle(429, 50, 50);
  Brain.Screen.drawCircle(429, 189, 50);
  Brain.Screen.drawCircle(50, 189, 50);
  Brain.Screen.drawCircle(50, 50, 50);
  Brain.Screen.setFillColor("#1A5276");
  Brain.Screen.setPenColor(vex::color::white);
  Brain.Screen.printAt(127, 125, "Hello, We are team 1028Z");
  Brain.Screen.setFillColor(vex::color::white);
  Brain.Screen.setPenColor("#9A7D0A");
  Brain.Screen.setFont(vex::fontType::prop40);
  Brain.Screen.printAt(39, 65, "A");
  Brain.Screen.setFillColor(vex::color::white);
  Brain.Screen.setPenColor("#9A7D0A");
  Brain.Screen.setFont(vex::fontType::prop40);
  Brain.Screen.printAt(414, 67, "W");
}
void drawWiringPage() {

  Brain.Screen.setPenWidth(1);
  Brain.Screen.setFont(vex::fontType::mono20);
  Brain.Screen.setPenColor(vex::color::white);
  Brain.Screen.setCursor(0, 1);
  Brain.Screen.clearScreen("#1A5276");
  Brain.Screen.setFillColor("#1A5276");
  Brain.Screen.printAt(1, 15, "Wiring:");
  Brain.Screen.printAt(1, 37, "Left Chassis - Port 2");
  Brain.Screen.printAt(1, 59, "Intake - Port 5");
  Brain.Screen.printAt(1, 81, "Ball Graber - Port 7");
  Brain.Screen.printAt(1, 103, "Left Chassis Back - Port 8");
  Brain.Screen.printAt(1, 125, "Second Catapult - Port 9");
  Brain.Screen.printAt(1, 147, "Right Chassis - Port 10");
  Brain.Screen.printAt(1, 169, "Vision Sensor - Port 16");
  Brain.Screen.printAt(1, 191, "Right Chassis Back - Port 17");
  Brain.Screen.printAt(1, 213, "Catapult - Port 20");
  Brain.Screen.printAt(1, 235, "Gyro - Port B");
  Brain.Screen.pressed(buttonCallback);
  Brain.Screen.released(buttonReleased);
}
void drawBackButton(int x, int y, int width, int height) {
  Brain.Screen.setPenWidth(1);
  Brain.Screen.setFont(vex::fontType::mono20);
  Brain.Screen.setPenColor(vex::color::white);
  Brain.Screen.drawRectangle(x, y, width, height, vex::color::red);
  Brain.Screen.setFillColor(vex::color::red);
  Brain.Screen.printAt(x + 27, y + 27, "BACK");
}
void displayButtonControls(int index, bool pressed);

int findButton(int16_t xpos, int16_t ypos) {
  int nButtons = sizeof(buttons) / sizeof(button);

  for (int index = 0; index < nButtons; index++) {
    button *pButton = &buttons[index];
    if (xpos < pButton->xpos || xpos > (pButton->xpos + pButton->width))
      continue;

    if (ypos < pButton->ypos || ypos > (pButton->ypos + pButton->height))
      continue;

    return (index);
  }
  return (-1);
}

void initButtons() {
  int nButtons = sizeof(buttons) / sizeof(button);

  for (int index = 0; index < nButtons; index++) {
    buttons[index].state = false;
  }
}
void userTouchCallbackPressed() {
  if (robot.indexer == 2) {
    int index;
    int xpos = Brain.Screen.xPosition();
    int ypos = Brain.Screen.yPosition();

    if ((index = findButton(xpos, ypos)) >= 0) {
      displayButtonControls(index, true);
    }
  }
}

void userTouchCallbackReleased() {
  if (robot.indexer == 2) {
    int index;
    int xpos = Brain.Screen.xPosition();
    int ypos = Brain.Screen.yPosition();

    if ((index = findButton(xpos, ypos)) >= 0) {
      // clear all buttons to false, ie. unselected
      //      initButtons();

      // now set this one as true
      if (buttons[index].state == true) {
        buttons[index].state = false;
      } else {
        buttons[index].state = true;
      }

      // save as auton selection
      robot.autonomousSelection = index;

      displayButtonControls(index, false);
    }
  }
}
void displayButtonControls(int index, bool pressed) {
  vex::color c;
  Brain.Screen.setPenColor(vex::color(0xe0e0e0));

  for (int i = 0; i < sizeof(buttons) / sizeof(button); i++) {

    if (buttons[i].state)
      c = buttons[i].onColor;
    else
      c = buttons[i].offColor;

    Brain.Screen.setFillColor(c);

    // button fill
    if (i == index && pressed == true) {
      Brain.Screen.drawRectangle(buttons[i].xpos, buttons[i].ypos,
                                 buttons[i].width, buttons[i].height, c);
    } else
      Brain.Screen.drawRectangle(buttons[i].xpos, buttons[i].ypos,
                                 buttons[i].width, buttons[i].height);

    // outline
    Brain.Screen.drawRectangle(buttons[i].xpos, buttons[i].ypos,
                               buttons[i].width, buttons[i].height,
                               vex::color::transparent);

    // draw label
    if (buttons[i].label != NULL)
      Brain.Screen.printAt(buttons[i].xpos + 8,
                           buttons[i].ypos + buttons[i].height - 8,
                           buttons[i].label);
  }
}
void drawLoadingPage() {
  int coordX = 50;
  int coordY = 50;
  Brain.Screen.setFillColor(vex::color::red);
  Brain.Screen.setFillColor(vex::color::black);
  while (coordX != 429) {
    Brain.Screen.drawCircle(coordX, coordY, 50);
    coordX++;
    robot.wait(1);
    Brain.Screen.clearScreen();
  }
  while (coordY != 189) {
    Brain.Screen.clearScreen();
    Brain.Screen.drawCircle(coordX, coordY, 50);
    Brain.Screen.drawCircle(429, 50, 50);
    coordY++;
    robot.wait(1);
  }
  while (coordX != 50) {
    Brain.Screen.clearScreen();
    Brain.Screen.drawCircle(coordX, coordY, 50);
    Brain.Screen.drawCircle(429, 50, 50);
    Brain.Screen.drawCircle(429, 189, 50);
    coordX--;
    robot.wait(1);
  }
  while (coordY != 50) {
    Brain.Screen.clearScreen();
    Brain.Screen.drawCircle(429, 50, 50);
    Brain.Screen.drawCircle(429, 189, 50);
    Brain.Screen.drawCircle(50, 189, 50);
    Brain.Screen.drawCircle(coordX, coordY, 50);
    coordY--;
  }
}