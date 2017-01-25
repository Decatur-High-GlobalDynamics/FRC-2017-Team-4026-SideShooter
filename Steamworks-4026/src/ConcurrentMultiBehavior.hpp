//
//  ConcurrentMultiBehavior.hpp
//  RobotBehavior
//
//  Created by Aaron Hillegass on 1/24/17.
//  Copyright © 2017 FRC4026. All rights reserved.
//

#ifndef ConcurrentMultiBehavior_hpp
#define ConcurrentMultiBehavior_hpp

#include <stdio.h>
#include <vector>
#include "Behavior.hpp"

class ConcurrentMultiBehavior : public Behavior
{
private:
    std::vector<Behavior *> behaviors;
    std::vector<Behavior *> activeBehaviors;
public:
    ~ConcurrentMultiBehavior();
    void appendBehavior(Behavior *behavior);
    void start(Hardware *io, unsigned long millis);
    BehaviorExit continueOperating(Hardware *io, unsigned long millis);
    virtual void cleanUp(Hardware *io, unsigned long millis);
};

#endif /* ConcurrentMultiBehavior_hpp */
