#ifndef TankBehavior_hpp
#define TankBehavior_hpp

#include <stdio.h>
#include "Behavior.hpp"

// This behavior lets the robot be driven like a tank using two joysticks
class TankBehavior : public Behavior
{

public:
    TankBehavior();
    BehaviorExit continueOperating(Hardware *io, unsigned long millis);
};


#endif
