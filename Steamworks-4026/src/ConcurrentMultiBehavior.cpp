//
//  ConcurrentMultiBehavior.cpp
//  RobotBehavior
//
//  Created by Aaron Hillegass on 1/24/17.
//  Copyright © 2017 FRC4026. All rights reserved.
//

#include "ConcurrentMultiBehavior.hpp"

void ConcurrentMultiBehavior::appendBehavior(Behavior *behavior)
{
    behaviors.push_back(behavior);
}

void ConcurrentMultiBehavior::start(Hardware *io, unsigned long millis)
{
    Behavior::start(io, millis);

    // Copy all behaviors into 'activeBehaviors'
    activeBehaviors.clear();
    for (std::vector<Behavior *>::iterator it=behaviors.begin(); it!=behaviors.end(); ++it) {
        Behavior *b = *it;
        b->start(io, millis);
        activeBehaviors.push_back(b);
    }
}

BehaviorExit ConcurrentMultiBehavior::continueOperating(Hardware *io, unsigned long millis)
{
    BehaviorExit exitCode = Behavior::continueOperating(io, millis);
    if (exitCode != BehaviorIncomplete) {
        return exitCode;
    }
    
    std::vector<Behavior *>::iterator it=activeBehaviors.begin();
    while (it!=activeBehaviors.end()) {
        Behavior *b = *it;
        exitCode = b->continueOperating(io, millis);
        
        // If it is done, remove it from active
        if (exitCode != BehaviorIncomplete) {
            it = activeBehaviors.erase(it);
        } else {
            ++it;
        }
    }

    if (activeBehaviors.empty()) {
        return BehaviorComplete;
    } else {
        return BehaviorIncomplete;
    }
}

void ConcurrentMultiBehavior::cleanUp(Hardware *io, unsigned long millis)
{
    Behavior::cleanUp(io, millis);
    
    for (std::vector<Behavior *>::iterator it=behaviors.begin(); it!=behaviors.end(); ++it) {
        Behavior *b = *it;
        b->cleanUp(io, millis);
    }
}

ConcurrentMultiBehavior::~ConcurrentMultiBehavior() {
    for (std::vector<Behavior *>::iterator it=behaviors.begin(); it!=behaviors.end(); ++it) {
        Behavior *b = *it;
        delete b;
    }
    fprintf(stderr, "Deallocating: %s\n", name.c_str());

}
