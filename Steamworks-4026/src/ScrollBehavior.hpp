#ifndef ScrollBehavior_hpp
#define ScrollBehavior_hpp

#include "Behavior.hpp"
#include "Hardware.hpp"

// This behavior lets the robot shoot fuel
class ScrollBehavior : public Behavior
{

public:
    BehaviorExit continueOperating(Hardware *io, unsigned long millis);
};


#endif
