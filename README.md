# Project Info
This project will not be functional unless the user has installed the open-source BehaviourTree.cpp C++ library by Davide Faconti onto their computer, as well as the Webots simulator to run the simulation. By default, the makefile for the ObjectYoubot controller attempts to read the library from the /libraries folder that would be present in the main project directory. If the library is not installed here, the makefile found in the ObjectYoubot controller folder will need to have the library path reconfigured to match your installation.

The user should open the BTandHuman.wbt world file for the finished product of a user-controlled and BT-controlled robot working together, or the onehuman.wbt file for the user-controlled robot only.

The BT robot should be using the ObjectYoubot.cpp file as its controller, and the user controlled robot should use the YoubotTeleoperation.cpp file as its controller.
The robot with the latter controller should be controllable using keyboard inputs, with arrows keys for movement, A and D for turning, Q and E for opening and closing gripper, and the numbers 1-5 for each arm position.
