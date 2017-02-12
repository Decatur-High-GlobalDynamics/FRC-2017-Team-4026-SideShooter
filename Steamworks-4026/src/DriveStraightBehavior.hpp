#ifndef DriveStraightBehavior_hpp
#define DriveStraightBehavior_hpp

#include <stdio.h>
#include "Behavior.hpp"

// This behavior drives the robot straight ahead at a rate defined
// by the right joystick Y value.  When the right trigger is released, this
// behavior completes

class DriveStraightBehavior : public Behavior
{
public:
	float targetAngle; // Where was the bot pointed when the behavior started?
    void start(Hardware *io, unsigned long millis); // Called automatically when the behavior starts
    BehaviorExit continueOperating(Hardware *io, unsigned long millis); // Called repeatedly
};


#endif
