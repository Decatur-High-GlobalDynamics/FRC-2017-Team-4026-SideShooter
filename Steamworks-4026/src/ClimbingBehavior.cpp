#include "ClimbingBehavior.hpp"

    BehaviorExit ClimbingBehavior::continueOperating(Hardware *hw, unsigned long millis)
    {
    	// Run superclass's implementation and check result
        BehaviorExit exitCode = Behavior::continueOperating(hw, millis);
        if (exitCode != BehaviorIncomplete) {
            return exitCode;
        }

		if(hw->manipulatorStick.GetRawButton(8))
		{
			hw->ballIntakeRoller1.Set(-1.0);
			hw->ballIntakeRoller2.Set(-1.0);
		}
		else if(hw->manipulatorStick.GetRawButton(7))
		{
			hw->ballIntakeRoller1.Set(-0.3);
			hw->ballIntakeRoller2.Set(-0.3);
		}
		else
		{
			hw->ballIntakeRoller1.Set(0.0);
			hw->ballIntakeRoller2.Set(0.0);
		}
		return BehaviorIncomplete;
    }
