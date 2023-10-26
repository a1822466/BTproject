#ifndef YOUBOTCLASS_CPP
#define YOUBOTCLASS_CPP
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Node.hpp>

#include <cmath>
#include <vector>
#include <array>
#include <string>
#include <iostream>

using namespace webots;
using namespace std;

class YoubotClass {
  public:
    YoubotClass(Supervisor *robot);
    void moveStraight(double mod);
    void Strafe(double mod);
    void Turn(double mod);
    void grip(bool closed);
    void stop();
    void armPosition(int pos);
    const double* getPos(string object);
  protected:
    Supervisor *mRobot;
    Motor *mwheels[4];
    Motor *marm[5];
    Motor *mgripper[2];
    std::array<std::array<double, 4>, 6> heights= {{
        {-0.97, -1.55, -0.61, 0.0}, // ARM_FRONT_FLOOR
        {-0.62, -0.98, -1.53, 0.0}, // ARM_FRONT_PLATE
        {1.57,-2.635,1.78,0.0}, // RESET ARM
        {0.678, 0.682, 1.74, 0.0},  // ARM_BACK_PLATE_HIGH
        {0.92, 0.42, 1.78, 0.0},    // ARM_BACK_PLATE_LOW
        {-0.4, -1.2, -1.57, 1.57} // ARM_HANOI_PREPARE
    }};
    //void setArmToPosition(int armIndex, double position);
};

YoubotClass::YoubotClass(Supervisor *robot) : mRobot(robot) {
  // cout << "constructing\n";
  char wheelNames[4][8] = {"wheel1","wheel2","wheel3","wheel4"};
  char armNames[5][8] = {"arm1","arm2","arm3","arm4","arm5"};
  char gripperNames[5][18] = {"finger::left","finger::right"};
  
  for (int i = 0; i < 4; i++){
       mwheels[i] = robot->getMotor(wheelNames[i]);
       mwheels[i]->setPosition(INFINITY);
       mwheels[i]->setVelocity(0.0);
  }
  for (int i = 0; i < 5; i++){
       marm[i] = robot->getMotor(armNames[i]);
  }
  for (int i = 0; i < 2; i++){
       mgripper[i] = robot->getMotor(gripperNames[i]);
  }
  for (int i = 0; i < 2; i++){
       // cout << "setting grip " << endl;
       mgripper[i]->setPosition(0.0);
  }
  for (int i = 1; i < 5; i++){
       // cout << "setting to " << heights[2][i-1] << endl;
       marm[i]->setPosition(heights[2][i-1]);
  }
}

void YoubotClass::moveStraight(double mod) {
  double dir = 3.0 * mod;
  for(int i=0;i<4;i++){
    mwheels[i]->setVelocity(dir);
  }
  return;
}

void YoubotClass::Strafe(double mod) {
  double dir = 3.0 * mod;
  mwheels[0]->setVelocity(-dir);
  mwheels[1]->setVelocity(dir);  
  mwheels[2]->setVelocity(dir);  
  mwheels[3]->setVelocity(-dir);
  return;
}

void YoubotClass::Turn(double mod) {
  double dir = 3.0 * mod;
  mwheels[0]->setVelocity(-dir);
  mwheels[1]->setVelocity(dir);  
  mwheels[2]->setVelocity(-dir);  
  mwheels[3]->setVelocity(dir);
  return;
}

void YoubotClass::grip(bool closed) {
  double fingerPos=0.025;
  if(closed){
    fingerPos=0.0;
  }
  mgripper[0]->setPosition(fingerPos);
  mgripper[1]->setPosition(fingerPos);
  return;
}

void YoubotClass::armPosition(int pos) {
  for (int i = 1; i < 5; i++){
   //cout << "setting to " << heights[key-49][i-1] << endl;
   marm[i]->setPosition(heights[pos][i-1]);
  }
  return;
}

void YoubotClass::stop() {
  for(int i=0;i<4;i++){
    mwheels[i]->setVelocity(0.0);
  }
  return;
}

const double* YoubotClass::getPos(string object){
  Node* obj;
  if(object=="box"){
    obj = mRobot->getFromDef("kukabox");
  } else {
    obj = mRobot->getFromDef("youBot");
  }

  const double *position = obj->getPosition();  
  return position;
}

#endif