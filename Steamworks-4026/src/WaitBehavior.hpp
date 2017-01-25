//
//  WaitBehavior.hpp
//  RobotBehavior
//
//  Created by Aaron Hillegass on 1/24/17.
//  Copyright © 2017 FRC4026. All rights reserved.
//

#ifndef WaitBehavior_hpp
#define WaitBehavior_hpp

#include <stdio.h>
#include "Behavior.hpp"

class WaitBehavior : public Behavior
{
protected:
    unsigned long waitTime;
public:
    WaitBehavior(unsigned long milliseconds);
    void start(Hardware *io, unsigned long millis);
    BehaviorExit continueOperating(Hardware *io, unsigned long millis);
    void cleanUp(Hardware *io, unsigned long millis);
    ~WaitBehavior();
};


#endif /* WaitBehavior_hpp */
