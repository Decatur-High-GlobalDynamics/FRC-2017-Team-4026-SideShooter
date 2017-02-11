#ifndef WaitForSpeedBehavior_hpp
#define WaitForSpeedBehavior_hpp

#include <stdio.h>
#include "Behavior.hpp"

// This behavior waits for shooter motors to get to speed
// or for the button to be released
class WaitForSpeedBehavior : public Behavior
{

public:
    float targetFrontSpeed;
    float targetBackSpeed;
    WaitForSpeedBehavior(float desiredFrontSpeed, float desiredBackSpeed);
    void start(Hardware *io, unsigned long millis);
    BehaviorExit continueOperating(Hardware *hw, unsigned long millis);
};


#endif
