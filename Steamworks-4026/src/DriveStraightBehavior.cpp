#include "DriveStraightBehavior.hpp"
#include "Hardware.hpp"


void DriveStraightBehavior::start(Hardware *hw, unsigned long millis)
{
    Behavior::start(hw, millis);
    targetAngle = hw->driveGyro.GetAngle();
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

    // Get the left joystick value
    float leftJoystickRaw = hw->driveLeftStick.GetY();
    float currentAngle = hw->driveGyro.GetAngle();
    
    // Could smooth or reverse
    float desiredVelocity = leftJoystickRaw;

    float error = targetAngle - currentAngle;
    
    // The abs(error) should aways be less than or equal to 180 degrees
    while (error > 180.0) {
        error = error - 360.0;
    }
    while (error < -180) {
        error = error + 360.0;
    }
    
    float correctionFactor = (error/75.0);
    
    float leftDriveVel, rightDriveVel;
    
    // FIXME: Make sure the sign is right on all these
    if (hw->driveReverse) {
        leftDriveVel = desiredVelocity + correctionFactor;
        rightDriveVel = -desiredVelocity + correctionFactor;
    } else {
        leftDriveVel = -desiredVelocity - correctionFactor;
        rightDriveVel = desiredVelocity - correctionFactor;
    }
    
    hw->leftDriveMotor.Set(leftDriveVel);
    hw->rightDriveMotor.Set(rightDriveVel);
    
  	return BehaviorIncomplete;
}
