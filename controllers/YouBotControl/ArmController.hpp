#ifndef ARM_CONTROLLER_HPP
#define ARM_CONTROLLER_HPP

#include <webots/motor.hpp>
#include <webots/robot.hpp>

class ArmController {
public:
  enum Height {
    ARM_FRONT_FLOOR,
    ARM_FRONT_PLATE,
    ARM_FRONT_CARDBOARD_BOX,
    ARM_RESET,
    ARM_BACK_PLATE_HIGH,
    ARM_BACK_PLATE_LOW,
    ARM_HANOI_PREPARE,
    ARM_MAX_HEIGHT
  };

  enum Orientation {
    ARM_BACK_LEFT,
    ARM_LEFT,
    ARM_FRONT_LEFT,
    ARM_FRONT,
    ARM_FRONT_RIGHT,
    ARM_RIGHT,
    ARM_BACK_RIGHT,
    ARM_MAX_SIDE
  };

  ArmController();
  void init();
  void reset();
  void setHeight(Height height);
  void setOrientation(Orientation orientation);
  void increaseHeight();
  void decreaseHeight();
  void increaseOrientation();
  void decreaseOrientation();
  // Define other public member functions as needed

private:
  WbDeviceTag arm_elements[5];
  Height current_height;
  Orientation current_orientation;
  Height new_height;
  Orientation new_orientation;
  // Define other private member variables as needed
};

#endif // ARM_CONTROLLER_HPP