#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>
#include <webots/Keyboard.hpp>

#include "YouBotClass.cpp"

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include <cmath>
#include <vector>
#include <array>
#include <string>
#include <iostream>
#include <chrono>
#include <numeric>
#include <algorithm>

using namespace webots;
using namespace std;
using namespace BT;
using namespace std::chrono_literals;

class frontClear : public SyncActionNode {
private:
  YoubotClass* robot;

public:
  frontClear(const std::string& name, const NodeConfig& config)
    : SyncActionNode(name, config){}
  frontClear(const string& name, const BT::NodeConfig& config, YoubotClass* theRobot)
    : SyncActionNode(name,config), robot(theRobot){}

  static PortsList providedPorts(){ return {}; }

  NodeStatus tick() override{
    double distance = robot->distSensor->getValue();
    // cout << "dist val is " << distance << endl;
    if(distance < 950) {
      // cout << "SOMETHING IN THE WAY\n";
      robot->stop();
      return NodeStatus::FAILURE;
    } else {
      return NodeStatus::SUCCESS;
    }
    return NodeStatus::SUCCESS;
   
  }
};

class frontNotClear : public SyncActionNode {
private:
  YoubotClass* robot;

public:
  frontNotClear(const std::string& name, const NodeConfig& config)
    : SyncActionNode(name, config){}
  frontNotClear(const string& name, const BT::NodeConfig& config, YoubotClass* theRobot)
    : SyncActionNode(name,config), robot(theRobot){}

  static PortsList providedPorts(){ return {}; }

  NodeStatus tick() override{
    // cout << "called front not clear\n";
    double distance = robot->distSensor->getValue();
    // cout << "dist val is " << distance << endl;
    if(distance < 950) {
      cout << "SOMETHING IN THE WAY\n";
      robot->stop();
      return NodeStatus::SUCCESS;
    } else {
      return NodeStatus::FAILURE;
    }
    return NodeStatus::FAILURE;
   
  }
};

class alignWithBox: public BT::StatefulActionNode{
  private:
    YoubotClass* robot;
  public:
    alignWithBox(const string& name, const BT::NodeConfig& config):StatefulActionNode(name,config){}    
    alignWithBox(const string& name, const BT::NodeConfig& config, YoubotClass* theRobot):StatefulActionNode(name,config),robot(theRobot){}
    static PortsList providedPorts() { return {InputPort<string>("target")}; }
    
    BT::NodeStatus onStart() override { return NodeStatus::RUNNING; }    
    
    BT::NodeStatus onRunning() override {
      Expected<string> tB = getInput<string>("target");
      string targetBox = tB.value();
      double angleBox=0;
      double strafemod = 1;
      if(targetBox == "box1"){
        angleBox = -0.78539;
      } else if(targetBox == "box2"){
        angleBox = -2.35619;
        strafemod=-1;
      } else if(targetBox == "box3"){
        angleBox = 2.35619;
      } else if(targetBox == "box4"){
        angleBox = 0.78539;
        strafemod=-1;
      }
      const double* angleRads = robot->getAngle("bot");
      double currentAngle = angleRads[2];
      // cout << "trying to turn from angle to ideal " << currentAngle << " " << angleBox << endl;
      if(currentAngle > angleBox+0.01){
        robot->Turn(1);
        return NodeStatus::RUNNING;
      } else if(currentAngle < angleBox-0.01){
        robot->Turn(-1);
        return NodeStatus::RUNNING;
      } else if(currentAngle > angleBox-0.01 && currentAngle < angleBox+0.01){
        cout << "aligned angle with box\n";
        
        const double* targetPos = robot->getPos(targetBox);
        double* targetP = const_cast<double*>(targetPos);
        const double* botPos = robot->getPos("bot");
        double* botP = const_cast<double*>(botPos);
        
        double distX = abs(targetP[0] - botP[0]);
        double distY = abs(targetP[1] - botP[1]);
        // cout << "dist X and Y are" << distX << " " << distY << endl;
        
        if(distY > distX+0.008){
          robot->Strafe(strafemod);
          return NodeStatus::RUNNING;
        } else if(distX > distY+0.008){
          robot->Strafe(-strafemod);
          return NodeStatus::RUNNING;
        } else if (distX <  distY+0.008 && distY < distX+0.008) {
          robot->stop();
          cout << "position aligned!\n";
          double dist = robot->findDist(targetP,botP);
          // cout << "dist to box is " << dist << endl;
          if(dist>0.399){
            robot->moveStraight(1);
            return NodeStatus::RUNNING;
          } else if (dist<0.396){
            robot->moveStraight(-1);
            return NodeStatus::RUNNING;
          } else if (dist >= 0.396 && dist <= 0.399){
            cout << "robot fully aligned with box!\n";
            return NodeStatus::SUCCESS;
          }
          
        }
      }
      return NodeStatus::FAILURE;

    }

    void onHalted() override {printf("[ MoveToBox: ABORTED ]");}    
};

class atMiddle : public SyncActionNode {
private:
  YoubotClass* robot;

public:
  atMiddle(const std::string& name, const NodeConfig& config)
    : SyncActionNode(name, config){}

  atMiddle(const string& name, const BT::NodeConfig& config, YoubotClass* theRobot)
    : SyncActionNode(name,config), robot(theRobot){}

  static PortsList providedPorts(){ return {}; }

  NodeStatus tick() override{
    double* selfPos = const_cast<double*>(robot->getPos("bot"));
    // cout << "robot pos is " << selfPos[0] << " " << selfPos[1] << endl;
    if(selfPos[0]<0.4 && selfPos[0] > -0.4 && selfPos[1]<0.4 && selfPos[1] > -0.4){
      cout << "robot in the middle\n";
      robot->stop();
      return NodeStatus::SUCCESS;
    } else{
      cout << "robot not in the middle\n";
      return NodeStatus::FAILURE;
    }
  }
};

class atTarget : public SyncActionNode {
private:
  YoubotClass* robot;
public:
  atTarget(const std::string& name, const NodeConfig& config)
    : SyncActionNode(name, config){}

  atTarget(const string& name, const BT::NodeConfig& config, YoubotClass* theRobot)
    : SyncActionNode(name,config), robot(theRobot){}

  static PortsList providedPorts(){ return { InputPort<string>("target") }; }

  NodeStatus tick() override{
    Expected<string> t= getInput<string>("target");
    string target = t.value();
    double* targetP = new double[3];
    if(target=="middle"){
      targetP[0]=0.0;
      targetP[1]=0.0;
    } else {
      const double* targetPos = robot->getPos(target);
      targetP = const_cast<double*>(targetPos);
    }
    const double* botPos = robot->getPos("bot");
    double* botP = const_cast<double*>(botPos);
    
    double dist = robot->findDist(targetP,botP);
    // cout << "distance in check func is " << dist << endl;
    if(dist < 0.404){
      // cout << "close to to target!\n";
      robot->stop();
      return NodeStatus::SUCCESS;
    }
    // cout << "not next to target \n";
    return NodeStatus::FAILURE;
  }
};

class holdingTarget : public SyncActionNode {
private:
  YoubotClass* robot;
public:
  holdingTarget(const std::string& name, const NodeConfig& config)
    : SyncActionNode(name, config){}
  holdingTarget(const string& name, const BT::NodeConfig& config, YoubotClass* theRobot)
    : SyncActionNode(name,config), robot(theRobot){}

  static PortsList providedPorts(){ return { InputPort<string>("target") }; }

  NodeStatus tick() override{
    Expected<string> t= getInput<string>("target");
    string target = t.value();
    const double* targetPos = robot->getPos(target);
    double* targetP = const_cast<double*>(targetPos);
    
    const double* botPos = robot->getPos("bot");
    double* botP = const_cast<double*>(botPos);
    
    double dist = robot->findDist(targetP,botP);
    // cout << "distance in pickup func is " << dist << endl;
    // cout << "height is " << dist << endl;
    if(dist < 0.3 && targetP[2]<0.18 && targetP[2]>0.14){
      // cout << "holding target!\n";
      return NodeStatus::SUCCESS;
    }
    // cout << "not holding target \n";
    return NodeStatus::FAILURE;
  }
};

class holdingAnyTarget : public SyncActionNode {
private:
  YoubotClass* robot;
public:
  holdingAnyTarget(const std::string& name, const NodeConfig& config)
    : SyncActionNode(name, config){}
  holdingAnyTarget(const string& name, const BT::NodeConfig& config, YoubotClass* theRobot)
    : SyncActionNode(name,config), robot(theRobot){}

  static PortsList providedPorts(){ return {}; }

  NodeStatus tick() override{
    const double* botPos = robot->getPos("bot");
    double* botP = const_cast<double*>(botPos);
    
    double dist = 0;
    // cout << "distance in pickup func is " << dist << endl;
    // cout << "height is " << dist << endl;
    for(int i=1;i<5;i++){
      string currentCheck = "box";
      currentCheck.append(to_string(i));
      cout << "currently checking if holding " << currentCheck << endl;
      
      const double* targetPos = robot->getPos(currentCheck);
      double* targetP = const_cast<double*>(targetPos);
      dist = robot->findDist(targetP,botP);
      
      if(dist < 0.3 && targetP[2]<0.18 && targetP[2]>0.14){
        return NodeStatus::SUCCESS;
      }
    }
    // cout << "not holding target \n";
    return NodeStatus::FAILURE;
  }
};

class targetsRetrieved : public SyncActionNode {
public:
  targetsRetrieved(const std::string& name, const NodeConfig& config)
    : SyncActionNode(name, config){}

  static PortsList providedPorts(){
      return { InputPort<double**>("boxes") };}
  
    NodeStatus tick() override{      
      auto boxespos = getInput<double**>("boxes");
      double** boxesPos = boxespos.value();
      
      bool finished = 1;
      cout <<"checking boxes\n";
      for(int i=0;i<4;i++){
        // cout << "box " << i+1 << " is at pos " << boxesPos[i][0] << " " << boxesPos[i][1] << endl;
        if(boxesPos[i][0] >0.4 || boxesPos[i][1] >0.4 || boxesPos[i][0] <-0.4 ||boxesPos[i][1] <-0.4 ){
          // cout << "box " << i+1 << " is outside zone at pos " << boxesPos[i][0] << " " << boxesPos[i][1] << endl;
          finished = 0;
        }
      }
      // cout << "finished is " << finished << endl;
      if(finished==1){
        cout << "all targets retrieved! ending sim.\n";
        return NodeStatus::SUCCESS;
      } else {
        return NodeStatus::FAILURE;
      }
    }
};

// Wait for 2 in-simulation seconds/ 64 ticks
class Wait: public BT::StatefulActionNode{
  private:
    YoubotClass* robot;
    int timer = 0;
  public:
    Wait(const string& name, const BT::NodeConfig& config):StatefulActionNode(name,config){}
    Wait(const string& name, const BT::NodeConfig& config, YoubotClass* theRobot):StatefulActionNode(name,config),robot(theRobot){}
    static PortsList providedPorts() { return {}; } 
       
    BT::NodeStatus onStart() override {      
       timer=0;
       // cout << "starting wait\n";
       return NodeStatus::RUNNING;      
    }    
    
    BT::NodeStatus onRunning() override {
      timer++;
      if(timer>64){
        // cout << "done waiting\n";
        return NodeStatus::SUCCESS;
      } else {return NodeStatus::RUNNING;}
    }

    void onHalted() override {printf("[ Wait: ABORTED ]");}    
};

// Move forward until close enough to the box
class MoveToBox: public BT::StatefulActionNode{
  private:
    YoubotClass* robot;
    double dist = 0;
  public:
    MoveToBox(const string& name, const BT::NodeConfig& config):StatefulActionNode(name,config){}    
    MoveToBox(const string& name, const BT::NodeConfig& config, YoubotClass* theRobot):StatefulActionNode(name,config),robot(theRobot){}
    static PortsList providedPorts() { return {InputPort<string>("target")}; }
    
    BT::NodeStatus onStart() override { return NodeStatus::RUNNING; }    
    
    BT::NodeStatus onRunning() override {
      Expected<string> tB = getInput<string>("target");
      string targetBox = tB.value();
      robot->moveStraight(1);
      return NodeStatus::RUNNING;
    }

    void onHalted() override {printf("[ MoveToBox: ABORTED ]");}    
};

//Turn in the direction of target box
class TurnToBox: public BT::StatefulActionNode{
  private:
    YoubotClass* robot;
    double dist = 0;
  public:
    TurnToBox(const string& name, const BT::NodeConfig& config):StatefulActionNode(name,config){}    
    TurnToBox(const string& name, const BT::NodeConfig& config, YoubotClass* theRobot):StatefulActionNode(name,config),robot(theRobot){}
    static PortsList providedPorts() { return {InputPort<string>("target")}; }
    
    BT::NodeStatus onStart() override { 
      cout << "called turn node\n";
      return NodeStatus::RUNNING; 
    }    
    
    BT::NodeStatus onRunning() override {
      Expected<string> tB = getInput<string>("target");
      string targetBox = tB.value();
      const double* angleRads = robot->getAngle("bot");
      double currentAngle = angleRads[2];
      double* targetPos = new double[3];
      double* botPos = const_cast<double*>(robot->getPos("bot"));
      if(targetBox == "middle"){
        targetPos[0]=0;
        targetPos[1]=0;
      } else {
        targetPos = const_cast<double*>(robot->getPos(targetBox));
        // cout << "target box is at coords " << targetPos[0] << " " << targetPos[1] << endl;
      }
      // cout << "self is at coords " << botPos[0] << " " << botPos[1] << endl;
      double xDist = targetPos[0] - botPos[0];
      double yDist = targetPos[1] - botPos[1];
      double* botToBox = new double[2];
      botToBox[0] = xDist;
      botToBox[1] = yDist;
      double* dirVector = new double[2];
      dirVector[0] = cos(currentAngle);
      dirVector[1] = sin(currentAngle);
      double turnAngle = robot->findAngle(botToBox,dirVector);
      // cout << "turn angle with function is " << turnAngle << endl;
      double crossProduct = botToBox[0] * dirVector[1] - botToBox[1] * dirVector[0];
      // cout << "cross prod is " << crossProduct << endl;
      if(turnAngle < 0.01 && turnAngle > -0.01){
        return NodeStatus::SUCCESS;
      } else {
        if(crossProduct < 0){
          robot->Turn(-2);
        } else {
          robot->Turn(2);
        }
      }
      return NodeStatus::RUNNING;
    }

    void onHalted() override {printf("[ TurnToBox: ABORTED ]");}    
};

// Set the arm to a position as defined in the heights array
class SetArmPos: public BT::StatefulActionNode{
  private:
    YoubotClass* robot;
    std::array<std::array<double, 4>, 6> heights= {{
        {-0.97, -1.55, -0.61, 0.0}, // ARM_FRONT_FLOOR
        {-0.62, -0.98, -1.53, 0.0}, // ARM_FRONT_PLATE
        {1.57,-2.635,1.78,0.0}, // RESET ARM
        {0.678, 0.682, 1.74, 0.0},  // ARM_BACK_PLATE_HIGH
        {0.92, 0.42, 1.78, 0.0},    // ARM_BACK_PLATE_LOW
        {-0.4, -1.2, -1.57, 1.57} // ARM_HANOI_PREPARE
    }};
    int armPos=0;
    double* currentPos = 0 ;
    int timer = 0;
  public:
    SetArmPos(const string& name, const BT::NodeConfig& config):StatefulActionNode(name,config){}
    SetArmPos(const string& name, const BT::NodeConfig& config, YoubotClass* theRobot, int pos):StatefulActionNode(name,config),robot(theRobot), armPos(pos){}
    static PortsList providedPorts() { return {}; }
    
    BT::NodeStatus onStart() override {
      timer=0;
      // double* currentPos = robot->getArmPosition();
      // cout << "arm pos before moving " << currentPos[0] << " " << currentPos[1] << " " << currentPos[2] << " " << currentPos[3] << " " << currentPos[4] << endl;  
      return NodeStatus::RUNNING;    
    }
    
    BT::NodeStatus onRunning() override {
      currentPos = robot->getArmPosition();
      if(timer == 54){
        // cout << "arm moved to position!\n";
        // cout << "current arm pos " << currentPos[0] << " " << currentPos[1] << " " << currentPos[2] << " " << currentPos[3] << " " << currentPos[4] << endl;
        return NodeStatus::SUCCESS;
      } else {
        robot->armPosition(armPos-1);
        timer++;
        return NodeStatus::RUNNING;
      }
    }

    void onHalted() override {printf("[ SetArmPos: ABORTED ]");}   
};

// Open or close gripper
class GripperInterface: public BT::StatefulActionNode{
  private:
    YoubotClass* robot;
    bool closed = 0;
    int timer = 0;
  public:
    GripperInterface(const string& name, const BT::NodeConfig& config):StatefulActionNode(name,config){}
    GripperInterface(const string& name, const BT::NodeConfig& config, YoubotClass* theRobot, bool gripSetting):StatefulActionNode(name,config),robot(theRobot), closed(gripSetting){}
    static PortsList providedPorts() { return {}; }
    
    BT::NodeStatus onStart() override {
      // std::cout << "GripperInterface Activated" << std::endl;
      timer=0;
      return NodeStatus::RUNNING;
    }
    

    BT::NodeStatus onRunning() override {
      if(timer == 24){return NodeStatus::SUCCESS;
      } else {
        robot->grip(closed);
        timer++;
        return NodeStatus::RUNNING;
      }
    }

    void onHalted() override {printf("[ GripperInterface: ABORTED ]");}   
};

// Ends simulation after delay seconds/ delay*32 ticks
class EndSimulationAfter: public BT::StatefulActionNode{
  private:
    YoubotClass* robot;
    int timer = 0;
    int _delay = 0;
  public:
    EndSimulationAfter(const std::string& name, const BT::NodeConfig& config):BT::StatefulActionNode(name, config) {}
    EndSimulationAfter(const std::string& name, const BT::NodeConfig& config, YoubotClass* theRobot, int delay):BT::StatefulActionNode(name, config), robot(theRobot), _delay(delay){}
    static PortsList providedPorts() { return {}; }
    
    BT::NodeStatus onStart() override {
      // std::cout << "GripperInterface Activated" << std::endl;
      return NodeStatus::RUNNING;
    }
    
    BT::NodeStatus onRunning() override {
      if(timer == (_delay*32)){
        std::cout << "Ending Simulation!" << std::endl;
        robot->endSim();
        return NodeStatus::SUCCESS;
      } else {
        timer++;
        return NodeStatus::RUNNING;
      }
    }

    void onHalted() override {printf("[ EndSimulationAfter: ABORTED ]");}    
};

class getPositions : public SyncActionNode {
  private:
    YoubotClass* robot;
  public:
    getPositions(const std::string& name, const NodeConfig& config):SyncActionNode(name, config){}
    getPositions(const string& name, const BT::NodeConfig& config, YoubotClass* theRobot):SyncActionNode(name,config), robot(theRobot){}
    static PortsList providedPorts(){
      return { OutputPort<double**>("boxes"),
      OutputPort<double*>("bt")
    };}
      
    NodeStatus tick() override{
      const double* pos1 = robot->getPos("box1");
      const double* pos2 = robot->getPos("box2");
      const double* pos3 = robot->getPos("box3");
      const double* pos4 = robot->getPos("box4");
      const double* botpos = robot->getPos("playerBot");
      // cout << "as retreived, box 1 is at " << pos1[0] << " " << pos1[1] << endl;
      // cout << "as retreived, box 2 is at " << pos2[0] << " " << pos2[1] << endl;
      // cout << "as retreived, box 3 is at " << pos3[0] << " " << pos3[1] << endl;
      // cout << "as retreived, box 4 is at " << pos4[0] << " " << pos4[1] << endl;
      double* posBox1 = const_cast<double*>(pos1);
      double* posBox2 = const_cast<double*>(pos2);
      double* posBox3 = const_cast<double*>(pos3);
      double* posBox4 = const_cast<double*>(pos4);
      double* posBot = const_cast<double*>(botpos);
      
      double** boxPositions = new double*[4];
      boxPositions[0]= posBox1;
      boxPositions[1]= posBox2;
      boxPositions[2]= posBox3;
      boxPositions[3]= posBox4;
      setOutput("boxes",boxPositions);
      setOutput("bt",posBot);
      return NodeStatus::SUCCESS;
    }
};


class showPositions : public SyncActionNode {
  private:
    YoubotClass* robot;
  public:
    showPositions(const std::string& name, const NodeConfig& config):SyncActionNode(name, config){}
    showPositions(const string& name, const BT::NodeConfig& config, YoubotClass* theRobot):SyncActionNode(name,config), robot(theRobot){}
    static PortsList providedPorts(){
      return { InputPort<double**>("boxes"), 
      InputPort<double*>("bt")};
    }
  
    NodeStatus tick() override{
      auto boxespos = getInput<double**>("boxes");
      auto botpos = getInput<double*>("bt");
      
      double** boxesPos = boxespos.value();
      double* botPos = botpos.value();
      
      cout << "Box 1 is at " << boxesPos[0][0] << " " << boxesPos[0][1] << " " << boxesPos[0][2] << endl;
      cout << "Box 2 is at " << boxesPos[1][0] << " " << boxesPos[1][1] << " " << boxesPos[1][2] << endl;
      cout << "Box 3 is at " << boxesPos[2][0] << " " << boxesPos[2][1] << " " << boxesPos[2][2] << endl;
      cout << "Box 4 is at " << boxesPos[3][0] << " " << boxesPos[3][1] << " " << boxesPos[3][2] << endl;
  
      cout << "Bot is at " << botPos[0] << " " << botPos[1] << " " << botPos[2] << endl;
  
      return NodeStatus::SUCCESS;
    }
};

class chooseTarget : public SyncActionNode {
  private:
    YoubotClass* robot;
  public:
    chooseTarget(const std::string& name, const NodeConfig& config):SyncActionNode(name, config){}
    chooseTarget(const string& name, const BT::NodeConfig& config, YoubotClass* theRobot):SyncActionNode(name,config), robot(theRobot){}
    static PortsList providedPorts(){
      return { OutputPort<string>("target") };}
  
    NodeStatus tick() override{
      double* playerPos = const_cast<double*>(robot->getPos("playerBot"));
      cout << "player is at " << playerPos[0] << " " << playerPos[1] << " " << playerPos[2] << endl;
      if(playerPos[0]<0.4 && playerPos[0] > -0.4 && playerPos[1]<0.4 && playerPos[1] > -0.4){
        // cout << "player in the middle\n";
        return NodeStatus::FAILURE;
      } else{
        double* box1Pos = const_cast<double*>(robot->getPos("box1"));
        double* box2Pos = const_cast<double*>(robot->getPos("box2"));
        double* box3Pos = const_cast<double*>(robot->getPos("box3"));
        double* box4Pos = const_cast<double*>(robot->getPos("box4"));\
        double** boxesPos = new double*[4];
        boxesPos[0] = box1Pos;
        boxesPos[1] = box2Pos;
        boxesPos[2] = box3Pos;
        boxesPos[3] = box4Pos;
        double box1Dist = robot->findDist(box1Pos,playerPos);
        double box2Dist = robot->findDist(box2Pos,playerPos);
        double box3Dist = robot->findDist(box3Pos,playerPos);
        double box4Dist = robot->findDist(box4Pos,playerPos);
        
        double maxDist = max({box1Dist,box2Dist,box3Dist,box4Dist});
        double minDist = min({box1Dist,box2Dist,box3Dist,box4Dist});
        // cout << "max dist is " << max({box1Dist,box2Dist,box3Dist,box4Dist}) << endl;
        // cout << "min dist is " << min({box1Dist,box2Dist,box3Dist,box4Dist}) << endl;
        double options[4] = {box1Dist,box2Dist,box3Dist,box4Dist};
        double maxVal=0;;
        for(int i=0;i<4;i++){
          if(boxesPos[i][0] < 0.4 && boxesPos[i][1] < 0.4 && boxesPos[i][0] > -0.4 && boxesPos[i][1] > -0.4 ){
            cout << "box " << i+1 << " is in the middle, so removing it from options.\n";
            options[i]=0;
          }
          if(options[i]== minDist){
            // cout << "deleting a min value " << options[i] << endl;
            options[i]=0;
          }
          if(options[i]>maxVal){
            cout << "new max is at " << i+1 << endl;
            maxVal=options[i];
          }
        }
        // options.erase(remove(options.begin(), options.end(), maxDist));  
        // cout << "removed.\n";
        // cout << "max Val and min Dist are " << maxVal << " " << minDist << endl;
        if (maxVal==0 || maxVal == minDist){
          cout << "no valid targets!\n";
          return NodeStatus::FAILURE;
        }         
        string maxBox;
        
        if(box1Dist == maxVal){
          maxBox="box1";
        } else if(box2Dist == maxVal){
          maxBox="box2";
        } else if(box3Dist == maxVal){
          maxBox="box3";
        } else if(box4Dist == maxVal){
          maxBox="box4";
        }
        setOutput("target",maxBox);
        cout << "successfully set target to " << maxBox << endl;
        return NodeStatus::SUCCESS;
      }
    }
};
