
#ifndef WaitForButtonBehavior_hpp
#define WaitForButtonBehavior_hpp

#include <stdio.h>
#include <WPILib.h>
#include "Behavior.hpp"

// This behavior lets the robot be driven like a WaitForButton using two joysticks
class WaitForButtonBehavior : public Behavior
{

public:
    Joystick *joystick;
    int buttonIndex;
    WaitForButtonBehavior(Joystick *js, int bIndex);
    BehaviorExit continueOperating(Hardware *io, unsigned long millis);
};


#endif
