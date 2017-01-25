//
//  StateMachine.cpp
//  RobotBehavior
//
//  Created by Aaron Hillegass on 1/24/17.
//  Copyright © 2017 FRC4026. All rights reserved.
//

#include "StateMachine.hpp"
#include <set>

// Creates a node, wires the preceeding node to it
StateNode *StateMachine::appendBehavior(Behavior *b)
{
    StateNode *newNode = new StateNode();
    newNode->currentBehavior = b;
    
    // Is this the first node?
    if (nodes.empty()) {
        fprintf(stderr, "Adding first node: \"%s\"\n", b->name.c_str());
        initialNode = newNode;
        currentNode = newNode;
        nodeNeedsStart = true;
    } else {
        StateNode *previousNode = nodes.back();
        fprintf(stderr, "Adding \"%s\" after \"%s\"\n", b->name.c_str(), previousNode->currentBehavior->name.c_str());
        
        for (int i = 0; i < BEHAVIOR_EXIT_OPTION_COUNT; i++) {
            previousNode->possibleNextBehaviors[i] = newNode;
        }
    }
    nodes.push_back(newNode);
    
    return newNode;
}

// Runs the currentNode, updates as necessary
void StateMachine::execute(Hardware *hardware, unsigned long millis)
{
    if (!currentNode) {
        fprintf(stderr, "No current node to execute\n");
        return;
    }
    
    Behavior *currentBehavior = currentNode->currentBehavior;
    
    if (nodeNeedsStart) {
        currentBehavior->start(hardware, millis);
        nodeNeedsStart = false;
    }
    
    //fprintf(stderr, "Running behavior: %s\n", currentBehavior->name.c_str());
    BehaviorExit exitCode = currentBehavior->continueOperating(hardware, millis);
    
    // Is it time to switch to a new node?
    if (exitCode != BehaviorIncomplete) {
        currentBehavior->cleanUp(hardware, millis);
        currentNode = currentNode->possibleNextBehaviors[exitCode];
        nodeNeedsStart = true;
    }
    
}

// Deletes all behaviors and nodes
void StateMachine::clear()
{
    // Behaviors may appear in more than one node
    // so gather in a set so only deleted once
    std::set<Behavior *> behaviorSet;
    while (!nodes.empty()) {
        StateNode *node = nodes.back();
        nodes.pop_back();
        Behavior *behavior = node->currentBehavior;
        behaviorSet.insert(behavior);
        delete node;
    }
    while (!behaviorSet.empty()) {
        Behavior *behavior = *behaviorSet.begin();
        behaviorSet.erase(behavior);
        delete behavior;
    }
    
    
}
