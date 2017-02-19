#include "WaitForButtonBehavior.hpp"


// Which joystick? Which button?
WaitForButtonBehavior::WaitForButtonBehavior(Joystick *js, int bIndex)
{
    joystick = js;
    buttonIndex = bIndex;
}

BehaviorExit WaitForButtonBehavior::continueOperating(Hardware *hw, unsigned long millis)
{
	// Run superclass's implementation and check result
    BehaviorExit exitCode = Behavior::continueOperating(hw, millis);
    if (exitCode != BehaviorIncomplete) {
        return exitCode;
    }

    // Is the button pressed?
    if (joystick->GetRawButton(buttonIndex)) {
    	fprintf(stderr, "Button pressed!\n");
    	// I'm done!
    	return BehaviorComplete;
    }

	// I'm not done!
	return BehaviorIncomplete;
}
