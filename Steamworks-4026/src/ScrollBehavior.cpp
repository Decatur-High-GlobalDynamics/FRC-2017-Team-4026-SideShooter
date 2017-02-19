
#include "ScrollBehavior.hpp"

    BehaviorExit ScrollBehavior::continueOperating(Hardware *hw, unsigned long millis)
    {
    	// Run superclass's implementation and check result
        BehaviorExit exitCode = Behavior::continueOperating(hw, millis);
        if (exitCode != BehaviorIncomplete) {
            return exitCode;
        }

       /*
        if (hw->manipulatorStick.GetRawButton()) {
        	return BehaviorComplete;
        	}
        */
		if(hw->manipulatorStick.GetRawButton(5) && hw->gearCatcherLimitLeft.Get())
		{
			hw->gearCatcherScrew.Set(0.7);
		}
		else if(hw->manipulatorStick.GetRawButton(6) && hw->gearCatcherLimitRight.Get())
		{
			hw->gearCatcherScrew.Set(-0.7);
		}
		else
		{
			hw->gearCatcherScrew.Set(0.0);
		}
		return BehaviorIncomplete;
    }
