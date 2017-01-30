#include "DriveStraightBehavior.hpp"

DriveStraightBehavior::DriveStraightBehavior()
{

}

void DriveStraightBehavior::start(Hardware *io, unsigned long millis)
{
    Behavior::start(io, millis);
    // This is where you should read the gyro and put that in initialGyroDirection
}

BehaviorExit DriveStraightBehavior::continueOperating(Hardware *hw, unsigned long millis)
{
	// Run superclass's implementation and check result
    BehaviorExit exitCode = Behavior::continueOperating(hw, millis);
    if (exitCode != BehaviorIncomplete) {
        return exitCode;
    }

    // Is the button released?
    if (!hw->driveRightStick.GetTrigger()) {

    	// I'm done!
    	return BehaviorComplete;
    }

    // Get joystick values
	float right = hw->driveRightStick.GetY();

	// Set power to motors
	hw->leftDriveMotor.Set(right);
	hw->rightDriveMotor.Set(-right);

	// I'm not done!
	return BehaviorIncomplete;
}
