#ifndef ShootFuelBehavior_hpp
#define ShootFuelBehavior_hpp

#include "Behavior.hpp"
#include "Hardware.hpp"

// This behavior lets the robot shoot fuel
class ShootFuelBehavior : public Behavior
{

public:
    float targetFrontSpeed;
    float targetBackSpeed;
    void start(Hardware *hw, unsigned long millis);
    BehaviorExit continueOperating(Hardware *io, unsigned long millis);
    void cleanUp(Hardware *io, unsigned long millis);
};


#endif
