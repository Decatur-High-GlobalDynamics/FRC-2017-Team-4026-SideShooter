//
//  StateMachine.hpp
//  RobotBehavior
//
//  Created by Aaron Hillegass on 1/24/17.
//  Copyright © 2017 FRC4026. All rights reserved.
//

#ifndef StateMachine_hpp
#define StateMachine_hpp

#include <stdio.h>

#include "Behavior.hpp"
#include <vector>

class StateNode
{
public:
    Behavior *currentBehavior;
    StateNode *possibleNextBehaviors[BEHAVIOR_EXIT_OPTION_COUNT];
};

class StateMachine
{
protected:
    bool nodeNeedsStart;
    std::vector<StateNode *> nodes;
public:
    StateNode *initialNode;
    StateNode *currentNode;
    
    // Creates a node, wires the preceeding node to it
    StateNode *appendBehavior(Behavior *b);
    
    // Runs the currentNode, updates as necessary
    void execute(Hardware *hardware, unsigned long millis);
    
    // Deletes all behaviors and nodes
    void clear();
};

#endif /* StateMachine_hpp */
