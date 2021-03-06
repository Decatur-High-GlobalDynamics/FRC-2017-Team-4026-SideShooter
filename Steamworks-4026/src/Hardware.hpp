//
//  Hardware.hpp
//  RobotBehavior
//
//  Created by Aaron Hillegass on 1/24/17.
//  Copyright © 2017 FRC4026. All rights reserved.
//

#ifndef Hardware_hpp
#define Hardware_hpp
#include <WPILib.h>
#include <CANTalon.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

// Hardware has the sensors and actuators

float smoothJoystick(float raw);
float convertDriveTicksToInches(int encTicks);


class Hardware
{
public:
    // Actuators
    VictorSP rightDriveMotor { 0 };
    VictorSP leftDriveMotor { 1 };
    CANTalon shooterWheelFront { 1 };
    CANTalon shooterWheelBack { 5 };
    CANTalon ballIntakeRoller1 { 2 };
    CANTalon ballIntakeRoller2 { 4 };
    CANTalon gearCatcherScrew { 3 };
	Servo agitatorServo { 5 };
    Servo shooterServo { 4 };
    
	AnalogInput wallDistanceSensorR { 2 };
	AnalogInput wallDistanceSensorL { 1 };
	DigitalInput gearCatcherLimitLeft { 1 };
	DigitalInput gearCatcherLimitRight { 0 };


    // Driver inputs
    frc::Joystick driveLeftStick { 1 };
    frc::Joystick driveRightStick { 0 };
    frc::Joystick manipulatorStick { 2 };
    
    // Sensors
    AnalogGyro driveGyro { 0 };
    
    // System state
    bool driveReverse;
    bool isGyroResetTelop;
    
    Hardware();
};
#endif /* Hardware_hpp */
