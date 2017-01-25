//
//  Behavior.hpp
//  RobotBehavior
//
//  Created by Aaron Hillegass on 1/24/17.
//  Copyright © 2017 FRC4026. All rights reserved.
//

#ifndef Behavior_hpp
#define Behavior_hpp

#include <stdio.h>
#include "Hardware.hpp"
#include <string>

typedef enum BehaviorExit
{
    BehaviorIncomplete = -1,
    BehaviorComplete = 0,
    BehaviorTimeout = 1,
    BehaviorCanceled = 2
} BehaviorExit;

#define BEHAVIOR_EXIT_OPTION_COUNT 3

// This is class is for subclassing
class Behavior
{
private:
    unsigned long startTime;
    
public:
    
    virtual ~Behavior();
    // Has this behavior been canceled?
    bool isCanceled;
    
    std::string name;
    
    // Returns the difference between startTime and later
    unsigned long millisSinceStart(unsigned long later);
    
    // 'cancel' sets 'isCanceled' to true.
    virtual void cancel(Hardware *io);
    
    // These methods are for overriding
    
    // 'start' gets called once immediately before the first call
    // to 'continueOperating'. Default implementation just sets
    // startTime to millis.
    virtual void start(Hardware *io, unsigned long millis);
    
    // 'continueOperating' does the work of the behavior.
    // It returns 'BehaviorIncomplete' if it wants to be called again.
    // Always check to see if isCanceled.
    virtual BehaviorExit continueOperating(Hardware *io, unsigned long millis);
    
    // 'cleanUp'
    virtual void cleanUp(Hardware *io, unsigned long millis);
};

#endif /* Behavior_hpp */
