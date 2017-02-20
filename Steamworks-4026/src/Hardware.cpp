//
//  Hardware.cpp
//  RobotBehavior
//
//  Created by Aaron Hillegass on 1/24/17.
//  Copyright © 2017 FRC4026. All rights reserved.
//

#include "Hardware.hpp"
#define DRIVE_TICKSPERREV 1000


float smoothJoystick(float raw)
{
    return powf(raw, 3);
}
float convertDriveTicksToInches(int encTicks)
{
	return (float)((float)encTicks / DRIVE_TICKSPERREV) * 3.14 * 4.0;
}

Hardware::Hardware()
{
    //Configure Shooter Talons
    shooterWheelFront.SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
    shooterWheelFront.ConfigNominalOutputVoltage(+0.0f, -0.0f);
    shooterWheelFront.ConfigPeakOutputVoltage(+12.0f, -12.0f);  //Modify this to allow for just forward or just backward spin
    shooterWheelFront.SetTalonControlMode(CANTalon::TalonControlMode::kSpeedMode);
    shooterWheelFront.SetSensorDirection(false);
    shooterWheelFront.SetAllowableClosedLoopErr(0);
    shooterWheelFront.SelectProfileSlot(0);
    shooterWheelFront.SetF(0.0416);
    shooterWheelFront.SetP(0.0);
    shooterWheelFront.SetI(0.0);
    shooterWheelFront.SetD(0.0);
    //shooterWheelFront.Set(0.0);
    
    shooterWheelBack.SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
    shooterWheelBack.ConfigNominalOutputVoltage(+0.0f, -0.0f);
    shooterWheelBack.ConfigPeakOutputVoltage(+12.0f, -12.0f); //Modify this to allow for just forward or just backward spin
    shooterWheelBack.SetTalonControlMode(CANTalon::TalonControlMode::kSpeedMode);
    shooterWheelBack.SetSensorDirection(false);
    shooterWheelBack.SetAllowableClosedLoopErr(0);
    shooterWheelBack.SelectProfileSlot(0);
    shooterWheelBack.SetF(0.0416);
    shooterWheelBack.SetP(0.0);
    shooterWheelBack.SetI(0.0);
    shooterWheelBack.SetD(0.0);
    //shooterWheelBack.Set(0.0);
    
    shooterServo.Set(1.0);
    driveReverse = false;
    isGyroResetTelop = false;
    driveGyro.Reset();


}



