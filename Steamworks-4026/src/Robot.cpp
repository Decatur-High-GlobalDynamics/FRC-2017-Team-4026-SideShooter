#include <iostream>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <time.h>
#include "StateMachine.hpp"
#include "DriveStraightBehavior.hpp"
#include "TankBehavior.hpp"
#include "Hardware.hpp"

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

struct timespec processStart;
unsigned long millisSinceStart() {
    struct timespec currentTimeSpec;
    clock_gettime(CLOCK_REALTIME, &currentTimeSpec);
    unsigned long result = (currentTimeSpec.tv_sec - processStart.tv_sec) * 1000;
    result += (currentTimeSpec.tv_nsec - processStart.tv_nsec) / 1000000;
    return result;
}

class Robot: public frc::IterativeRobot {

public:

    // This has all the sensors and actuators
    Hardware *hardware;
    std::vector<StateMachine *> activeStateMachines;

	void RobotInit() {
		hardware = new Hardware();
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit() override {

	}

	void AutonomousPeriodic() {


	}

	void TeleopInit() {
	    fprintf(stderr, "*** Configuring ***\n");
	    StateMachine *stateMachine = new StateMachine();

	    /* This is where we create behavior instances */

	    // Starts of tank driving
	    TankBehavior *tankBehavior;
	    tankBehavior = new TankBehavior();
	    tankBehavior->name = "Tank Driving";
	    StateNode *firstNode = stateMachine->appendBehavior(tankBehavior);

	    // If the right trigger is pushed, it transitions to straigh driving
	    DriveStraightBehavior *driveStraight = new DriveStraightBehavior();
	    driveStraight->name = "Drive Straight";
	    StateNode *lastNode =stateMachine->appendBehavior(driveStraight);

	    // Loop it! If the right trigger is released go back to tank driving
	    lastNode->possibleNextBehaviors[BehaviorComplete] = firstNode;

	    // Schedule this state machine
	    activeStateMachines.push_back(stateMachine);

	    // Note when we started this process
	    clock_gettime(CLOCK_REALTIME, &processStart);	}

	void TeleopPeriodic() {

		// Step through all the scheduled state machines, executing each
		std::vector<StateMachine *>::iterator it = activeStateMachines.begin();
	    while (it < activeStateMachines.end()) {
	    	StateMachine *stateMachine = *it;
	        stateMachine->execute(hardware, millisSinceStart());
	    }
	}

	void TestPeriodic() {
		//lw->Run();
	}

};

START_ROBOT_CLASS(Robot)
