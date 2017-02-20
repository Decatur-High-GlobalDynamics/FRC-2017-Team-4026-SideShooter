#ifndef ClimbingBehavior_hpp
#define ClimbingBehavior_hpp

#include "Behavior.hpp"
#include "Hardware.hpp"

// This behavior lets the robot climb the rop
class ClimbingBehavior : public Behavior
{

public:
    BehaviorExit continueOperating(Hardware *io, unsigned long millis);
};


#endif
