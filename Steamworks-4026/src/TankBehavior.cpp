#include "TankBehavior.hpp"


// The constructor
TankBehavior::TankBehavior()
{

}

// Called again and again
BehaviorExit TankBehavior::continueOperating(Hardware *hw, unsigned long millis)
{
	// Run superclass's implementation and check result
    BehaviorExit exitCode = Behavior::continueOperating(hw, millis);
    if (exitCode != BehaviorIncomplete) {
        return exitCode;
    }

    // Transition to driving straight?
    if (hw->driveRightStick.GetTrigger()) {

    	// I'm done!
    	return BehaviorComplete;
    }

    // Get joystick values
	float right = hw->driveRightStick.GetY();
	float left = hw->driveLeftStick.GetY();

	// Set power to motors
	hw->leftDriveMotor.Set(left);
	hw->rightDriveMotor.Set(-right);

	// I'm not done!
	return BehaviorIncomplete;
}
