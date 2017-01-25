//
//  Behavior.cpp
//  RobotBehavior
//
//  Created by Aaron Hillegass on 1/24/17.
//  Copyright © 2017 FRC4026. All rights reserved.
//

#include "Behavior.hpp"
#include "time.h"

Behavior::~Behavior()
{
    
}
// Returns the difference between startTime and later
unsigned long Behavior::millisSinceStart(unsigned long later)
{
    return later - startTime;
}

// 'cancel' sets 'isCanceled' to true.
void Behavior::cancel(Hardware *io)
{
    isCanceled = true;
}

void Behavior::start(Hardware *io, unsigned long millis)
{
    startTime = millis;
    isCanceled = false;
}

// 'continueOperating' does the work of the behavior.
// It returns 'BehaviorIncomplete' if it wants to be called again.
// Always check to see if isCanceled.
BehaviorExit Behavior::continueOperating(Hardware *io, unsigned long millis)
{
    if (isCanceled) {
        return BehaviorCanceled;
    } else {
        return BehaviorIncomplete;
    }
}

// 'cleanUp'
void Behavior::cleanUp(Hardware *io, unsigned long millis)
{
    
}
