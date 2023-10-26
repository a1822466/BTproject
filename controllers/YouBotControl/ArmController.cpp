#include "ArmController.hpp"

ArmController::ArmController() {
  // Constructor implementation
}

void ArmController::init() {
  arm_elements[ARM1] = wb_robot_get_device("arm1");
  arm_elements[ARM2] = wb_robot_get_device("arm2");
  arm_elements[ARM3] = wb_robot_get_device("arm3");
  arm_elements[ARM4] = wb_robot_get_device("arm4");
  arm_elements[ARM5] = wb_robot_get_device("arm5");

  wb_motor_set_velocity(arm_elements[ARM2], 0.5);

  setHeight(ARM_RESET);
  setOrientation(ARM_FRONT);
}

void ArmController::reset() {
  setHeight(ARM_RESET);
  setOrientation(ARM_FRONT);
}

void ArmController::setHeight(Height height) {
  switch (height) {
    case ARM_FRONT_FLOOR:
      wb_motor_set_position(arm_elements[ARM2], -0.97);
      wb_motor_set_position(arm_elements[ARM3], -1.55);
      wb_motor_set_position(arm_elements[ARM4], -0.61);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_FRONT_PLATE:
      wb_motor_set_position(arm_elements[ARM2], -0.62);
      wb_motor_set_position(arm_elements[ARM3], -0.98);
      wb_motor_set_position(arm_elements[ARM4], -1.53);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_FRONT_CARDBOARD_BOX:
      wb_motor_set_position(arm_elements[ARM2], 0.0);
      wb_motor_set_position(arm_elements[ARM3], -0.77);
      wb_motor_set_position(arm_elements[ARM4], -1.21);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_RESET:
      wb_motor_set_position(arm_elements[ARM2], 1.57);
      wb_motor_set_position(arm_elements[ARM3], -2.635);
      wb_motor_set_position(arm_elements[ARM4], 1.78);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_BACK_PLATE_HIGH:
      wb_motor_set_position(arm_elements[ARM2], 0.678);
      wb_motor_set_position(arm_elements[ARM3], 0.682);
      wb_motor_set_position(arm_elements[ARM4], 1.74);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_BACK_PLATE_LOW:
      wb_motor_set_position(arm_elements[ARM2], 0.92);
      wb_motor_set_position(arm_elements[ARM3], 0.42);
      wb_motor_set_position(arm_elements[ARM4], 1.78);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_HANOI_PREPARE:
      wb_motor_set_position(arm_elements[ARM2], -0.4);
      wb_motor_set_position(arm_elements[ARM3], -1.2);
      wb_motor_set_position(arm_elements[ARM4], -M_PI_2);
      wb_motor_set_position(arm_elements[ARM5], M_PI_2);
      break;
    default:
      fprintf(stderr, "arm_height() called with a wrong argument\n");
      return;
  }
  current_height = height;
}

void ArmController::setOrientation(Orientation orientation) {
  switch (orientation) {
    case ARM_BACK_LEFT:
      wb_motor_set_position(arm_elements[ARM1], -2.949);
      break;
    case ARM_LEFT:
      wb_motor_set_position(arm_elements[ARM1], -M_PI_2);
      break;
    case ARM_FRONT_LEFT:
      wb_motor_set_position(arm_elements[ARM1], -0.2);
      break;
    case ARM_FRONT:
      wb_motor_set_position(arm_elements[ARM1], 0.0);
      break;
    case ARM_FRONT_RIGHT:
      wb_motor_set_position(arm_elements[ARM1], 0.2);
      break;
    case ARM_RIGHT:
      wb_motor_set_position(arm_elements[ARM1], M_PI_2);
      break;
    case ARM_BACK_RIGHT:
      wb_motor_set_position(arm_elements[ARM1], 2.949);
      break;
    default:
      fprintf(stderr, "arm_set_side() called with a wrong argument\n");
      return;
  }
  current_orientation = orientation;
}

// Define other member function implementations as needed