#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>
#include <webots/Keyboard.hpp>
#include "YouBotClass.cpp"
#include <cmath>
#include <vector>
#include <array>
#include <string>
#include <iostream>

using namespace webots;
using namespace std;

int main(int argc, char **argv) {
  Supervisor *robot = new Supervisor();
  std::array<std::array<double, 4>, 6> heights;
  heights = {{
  {-0.97, -1.55, -0.61, 0.0}, // ARM_FRONT_FLOOR
  {-0.62, -0.98, -1.53, 0.0}, // ARM_FRONT_PLATE
  {1.57,-2.635,1.78,0.0}, // RESET ARM
  {0.678, 0.682, 1.74, 0.0},  // ARM_BACK_PLATE_HIGH
  {0.92, 0.42, 1.78, 0.0},    // ARM_BACK_PLATE_LOW
  {-0.4, -1.2, -1.57, 1.57} // ARM_HANOI_PREPARE
  }};
  Keyboard kb;
  
  YoubotClass *newBot = new YoubotClass(robot);
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  
  kb.enable(timeStep);
  
  // cout << "entering loop\n";
  while (robot->step(timeStep) != -1) {
        int key=kb.getKey();
        // cout << key << endl;
        if(key==315){
          newBot->moveStraight(1);
        } else if(key==317){
          newBot->moveStraight(-1);
        }  else if(key==314){ //left
          newBot->Strafe(-1);
        } else if(key==316){ // right
          newBot->Strafe(1);
        } else if(key==65){ // A
          newBot->Turn(-1);
        } else if(key==68){ // D
          newBot->Turn(1);
        } else if(key==81){ // Q
          newBot->grip(0);
        } else if(key==69){ // E
          newBot->grip(1);
        } else if(key<55 && key >48){ // other
          newBot->armPosition(key-49);
        } else if(key==89){ // other
          const double* position;
          position = newBot->getPos("box");
          cout << "box pos is " << position[0] << " " << position[1] << " "<< position[2] << endl;
        } else if(key==85){ // other
          const double* position;
          position = newBot->getPos("bot");
          cout << "bot pos is " << position[0] << " " << position[1] << " "<< position[2] << endl;
        } else {
          newBot->stop();
        }
        
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}