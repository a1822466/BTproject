#ifndef YOUBOTCLASS_CPP
#define YOUBOTCLASS_CPP
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Node.hpp>
#include <webots/InertialUnit.hpp>

#include <cmath>
#include <vector>
#include <array>
#include <string>
#include <bits/stdc++.h>
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
    double* getArmPosition();
    double findDist(double* Point1, double* Point2);
    double findAngle(double* Vec1, double* Vec2);
    const double* getPos(string object);
    const double* getAngle(string object);
    void endSim();
    InertialUnit *InerUnit;
    DistanceSensor *distSensor;
  protected:
    Supervisor *mRobot;
    Motor *mwheels[4];
    Motor *marm[5];
    Motor *mgripper[2];
    PositionSensor *mArmSensor[5];
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
  
  InerUnit = robot->getInertialUnit("InUnit");
  InerUnit->enable(32);
  distSensor = robot->getDistanceSensor("distSens");
  distSensor->enable(32);
  
  for (int i = 0; i < 4; i++){
       mwheels[i] = robot->getMotor(wheelNames[i]);
       mwheels[i]->setPosition(INFINITY);
       mwheels[i]->setVelocity(0.0);
  }
  for (int i = 0; i < 5; i++){
       marm[i] = robot->getMotor(armNames[i]);
       string current = armNames[i];
       current += "sensor";
       mArmSensor[i] = robot->getPositionSensor(current);
       mArmSensor[i]->enable(32);
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
  double dir = 2.0 * mod;
  for(int i=0;i<4;i++){
    mwheels[i]->setVelocity(dir);
  }
  return;
}

void YoubotClass::Strafe(double mod) {
  double dir = 2.0 * mod;
  mwheels[0]->setVelocity(-dir);
  mwheels[1]->setVelocity(dir);  
  mwheels[2]->setVelocity(dir);  
  mwheels[3]->setVelocity(-dir);
  return;
}

void YoubotClass::Turn(double mod) {
  double dir = 1.0 * mod;
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

double* YoubotClass::getArmPosition() {
  double* armPos = new double[5];
  //cout << "arm value is ";
  for (int i=1;i<5;i++){
   armPos[i] = mArmSensor[i]->getValue();
   //cout << armPos[i] << " ";
  }
  //cout << endl;
  return armPos;
}

void YoubotClass::stop() {
  for(int i=0;i<4;i++){
    mwheels[i]->setVelocity(0.0);
  }
  return;
}

void YoubotClass::endSim() {
  mRobot->simulationSetMode(Supervisor::SIMULATION_MODE_PAUSE);
  return;
}

const double* YoubotClass::getPos(string object){
  Node* obj;
  if(object=="box1"){
    obj = mRobot->getFromDef("kukabox1");
  } else if(object=="box2"){
    obj = mRobot->getFromDef("kukabox2");
  } else if(object=="box3"){
    obj = mRobot->getFromDef("kukabox3");
  } else if(object=="box4"){
    obj = mRobot->getFromDef("kukabox4");
  } else if(object=="bot") {
    obj = mRobot->getFromDef("youBot");
  } else if(object=="playerBot") {
    obj = mRobot->getFromDef("playerBot");
  } else {
    obj = mRobot->getFromDef("playerBot");
  }

  const double *position = obj->getPosition();  
  return position;
}

const double* YoubotClass::getAngle(string object){
  const double* yaw = InerUnit->getRollPitchYaw();
  // cout << "got inerunit data, which is " << yaw[0] << " " << yaw[1] << " " << yaw[2] << endl;
  return yaw;
}

double YoubotClass::findDist(double* Point1, double* Point2){
  double dist;
  double xDist = Point1[0] - Point2[0];
  double yDist = Point1[1] - Point2[1];
  dist = sqrt(pow(xDist,2)+ pow(yDist,2));
  return dist;
}

double YoubotClass::findAngle(double* Vec1, double* Vec2){
  double angle;
  double mag1 = sqrt((pow(Vec1[0],2)+ pow(Vec1[1],2)));
  double mag2 = sqrt((pow(Vec2[0],2)+ pow(Vec2[1],2)));
  
  double dotProd = (Vec1[0] * Vec2[0]) + (Vec1[1] * Vec2[1]);
  angle = acos( dotProd / (mag1*mag2));
  return angle;
}

#endif