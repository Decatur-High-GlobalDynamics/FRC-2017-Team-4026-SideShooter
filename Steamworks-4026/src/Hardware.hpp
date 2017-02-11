//
//  Hardware.hpp
//  RobotBehavior
//
//  Created by Aaron Hillegass on 1/24/17.
//  Copyright © 2017 FRC4026. All rights reserved.
//

#ifndef Hardware_hpp
#define Hardware_hpp
#include <Victor.h>
#include <Joystick.h>
#include <stdio.h>

// Hardware has the sensors and actuators

class Hardware
{
public:
	VictorSP rightDriveMotor;
	VictorSP leftDriveMotor;
	Joystick driveLeftStick;
	Joystick driveRightStick;
	Hardware();
};
#endif /* Hardware_hpp */
