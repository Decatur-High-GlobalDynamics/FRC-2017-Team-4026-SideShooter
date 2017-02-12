#include "ShootFuelBehavior.hpp"


// The constructor
ShootFuelBehavior::ShootFuelBehavior(float desiredSpeedFront, float desiredSpeedBack)
{
    targetFrontSpeed = desiredSpeedFront;
    targetBackSpeed = desiredSpeedBack;
}



// Called again and again
BehaviorExit ShootFuelBehavior::continueOperating(Hardware *hw, unsigned long millis)
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
    
    // We could tweak these as we see the balls
    // missing the target
    hw->shooterWheelFront.Set(targetFrontSpeed);
    hw->shooterWheelBack.Set(targetBackSpeed);
    
	// I'm not done!
	return BehaviorIncomplete;
}


void ShootFuelBehavior::cleanUp(Hardware *hw, unsigned long millis)
{
    Behavior::cleanUp(hw, millis);
    
    // Close the door
    hw->shooterServo.Set(1.0);
    
    // Let the motors slow to a stop
    hw->shooterWheelFront.SetF(0.0);
    hw->shooterWheelBack.SetF(0.0);
}
