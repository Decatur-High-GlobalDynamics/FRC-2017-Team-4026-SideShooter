#include "WaitForSpeedBehavior.hpp"

#define MARGIN (10.0)


// The constructor
WaitForSpeedBehavior::WaitForSpeedBehavior(float desiredFrontSpeed, float desiredBackSpeed)
{
    // Just note the desired speeds
    targetFrontSpeed = desiredFrontSpeed;
    targetBackSpeed = desiredBackSpeed;
}

void WaitForSpeedBehavior::start(Hardware *hw, unsigned long millis)
{
    Behavior::start(hw, millis);
    
    // Reset F
    shooterWheelFront.SetF(0.0416);
    shooterWheelFront.SetF(0.0416);
    
    // Start the motors
    hw->shooterWheelFront.Set(targetFrontSpeed);
    hw->shooterWheelBack.Set(targetBackSpeed);
}

// Called again and again
BehaviorExit WaitForSpeedBehavior::continueOperating(Hardware *hw, unsigned long millis)
{
	// Run superclass's implementation and check result
    BehaviorExit exitCode = Behavior::continueOperating(hw, millis);
    if (exitCode != BehaviorIncomplete) {
        return exitCode;
    }

    // Button released?
    if (!(hw->manipulatorStick.GetRawButton(1))) {
        
        // I'm done!
        return BehaviorComplete;
    }

    // Read speeds from the encoders
    double actualFrontSpeed = hw->shooterWheelFront.GetSpeed();
    double actualBackSpeed = hw->shooterWheelBack.GetSpeed();
    
    // FIXME: This conversion....
    double actualFrontRPM = actualFrontSpeed * 6.8;
    double actualBackRPM = actualBackSpeed * 6.8;
    
    // Is it close enough to the desired speeds?
    if (fabs(actualFrontRPM - targetFrontSpeed) < MARGIN &&
        fabs(actualBackRPM - targetBackSpeed) < MARGIN) {
        return BehaviorComplete;
    }

	// I'm not done!
	return BehaviorIncomplete;
}
