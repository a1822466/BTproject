#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>
#include <webots/Keyboard.hpp>
#include <webots/InertialUnit.hpp>
#include "YouBotClass.cpp"
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include "Nodes.cpp"
#include <cmath>
#include <vector>
#include <array>
#include <string>
#include <iostream>

using namespace webots;
using namespace std;
using namespace BT;

int main(int argc, char **argv) {
  BehaviorTreeFactory factory;
  // GripperInterface gripper;
  
  Supervisor *robot = new Supervisor();
  Keyboard kb;
  
  YoubotClass *newBot = new YoubotClass(robot);
  factory.registerNodeType<Wait>("Wait", newBot);
  factory.registerNodeType<atMiddle>("atMiddle", newBot);
  factory.registerNodeType<getPositions>("getPositions", newBot);
  factory.registerNodeType<showPositions>("showPositions", newBot);
  factory.registerNodeType<chooseTarget>("chooseTarget", newBot);
  factory.registerNodeType<atTarget>("atTarget", newBot);
  factory.registerNodeType<alignWithBox>("alignWithBox", newBot);
  factory.registerNodeType<frontClear>("frontClear", newBot);
  factory.registerNodeType<frontNotClear>("frontNotClear", newBot);
  factory.registerNodeType<holdingTarget>("holdingTarget", newBot);
  factory.registerNodeType<holdingAnyTarget>("holdingAnyTarget", newBot);
  factory.registerNodeType<targetsRetrieved>("targetsRetrieved");
  // factory.registerNodeType<moveForward>("moveForward", newBot);
  factory.registerNodeType<TurnToBox>("TurnToBox", newBot);
  factory.registerNodeType<MoveToBox>("MoveToBox", newBot);
  factory.registerNodeType<SetArmPos>("ArmGrabBox", newBot,1);
  factory.registerNodeType<SetArmPos>("ArmDropBox", newBot,5);
  factory.registerNodeType<GripperInterface>("CloseGripper", newBot,1);
  factory.registerNodeType<GripperInterface>("OpenGripper", newBot,0);
  factory.registerNodeType<EndSimulationAfter>("EndSimulationAfter10s", newBot,30);
  factory.registerNodeType<EndSimulationAfter>("EndSimulation", newBot,0);
  
  auto tree = factory.createTreeFromFile("./finalTree.xml");
  
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  //cout << "step is " << timeStep << endl;
  
  kb.enable(timeStep);
  
  // cout << "entering loop\n";
  while (robot->step(timeStep) != -1) {
     tree.rootNode()->executeTick();   
        
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
