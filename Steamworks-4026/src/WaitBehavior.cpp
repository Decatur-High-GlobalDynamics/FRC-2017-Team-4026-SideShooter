//
//  WaitBehavior.cpp
//  RobotBehavior
//
//  Created by Aaron Hillegass on 1/24/17.
//  Copyright © 2017 FRC4026. All rights reserved.
//

#include "WaitBehavior.hpp"
#include <stdio.h>

WaitBehavior::WaitBehavior(unsigned long milliseconds)
{
    waitTime = milliseconds;
}

void WaitBehavior::start(Hardware *io, unsigned long millis)
{
    Behavior::start(io, millis);
    fprintf(stderr, "%lu: Starting: %s\n", millis, name.c_str());
}

BehaviorExit WaitBehavior::continueOperating(Hardware *io, unsigned long millis)
{
    BehaviorExit exitCode = Behavior::continueOperating(io, millis);
    if (exitCode != BehaviorIncomplete) {
        return exitCode;
    }

    unsigned long elapsedTime = millisSinceStart(millis);
    if (elapsedTime < waitTime) {
        return BehaviorIncomplete;
    } else {
        fprintf(stderr, "%lu: Completed: %s\n", millis, name.c_str());
        return BehaviorComplete;
    }
}

void WaitBehavior::cleanUp(Hardware *io, unsigned long millis)
{
    Behavior::cleanUp(io, millis);
    fprintf(stderr, "%lu: Cleaned up: %s\n", millis, name.c_str());
}

WaitBehavior::~WaitBehavior() {
    fprintf(stderr, "Deallocating: %s\n", name.c_str());
}



