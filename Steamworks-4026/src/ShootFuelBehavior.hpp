#ifndef ShootFuelBehavior_hpp
#define ShootFuelBehavior_hpp

#include <stdio.h>
#include "Behavior.hpp"

// This behavior lets the robot shoot fuel
class ShootFuelBehavior : public Behavior
{

public:
    float targetFrontSpeed, targetBackSpeed;
    ShootFuelBehavior(float desiredSpeedFront, float desiredSpeedBack);
    void start(Hardware *io, unsigned long millis);
    BehaviorExit continueOperating(Hardware *io, unsigned long millis);
    void cleanUp(Hardware *io, unsigned long millis);
};


#endif
