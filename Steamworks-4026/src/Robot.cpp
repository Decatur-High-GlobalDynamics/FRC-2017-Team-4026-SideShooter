#include <iostream>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <time.h>
#include "StateMachine.hpp"
#include "DriveStraightBehavior.hpp"
#include "WaitForButtonBehavior.hpp"
#include "WaitForSpeedBehavior.hpp"
#include "ShootFuelBehavior.hpp"
#include "TankBehavior.hpp"
#include "Hardware.hpp"

#include <IterativeRobot.h>
//#include <SampleRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
//#include <WPILib.h>
//#include <CANTalon.h>

struct timespec processStart;
unsigned long millisSinceStart() {
    struct timespec currentTimeSpec;
    clock_gettime(CLOCK_REALTIME, &currentTimeSpec);
    unsigned long result = (currentTimeSpec.tv_sec - processStart.tv_sec) * 1000;
    result += (currentTimeSpec.tv_nsec - processStart.tv_nsec) / 1000000;
    return result;
}

class Robot: public frc::IterativeRobot {
//class Robot: public frc::SampleRobot {

public:

    // This has all the sensors and actuators
    Hardware *hardware;
    std::vector<StateMachine *> activeStateMachines;
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameGear1= "Gear Location 1";
	const std::string autoNameGear2 = "Gear Location 2";
	const std::string autoNameGear3 = "Gear Location 3";
 
	void RobotInit() {
        chooser.AddDefault(autoNameDefault, autoNameDefault);
        chooser.AddObject(autoNameGear1, autoNameGear1);
        chooser.AddObject(autoNameGear2, autoNameGear2);
        chooser.AddObject(autoNameGear3, autoNameGear3);
        frc::SmartDashboard::PutData("Auto Modes", &chooser);
		hardware = new Hardware();

		CameraServer::GetInstance()->StartAutomaticCapture();
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

		StateMachine *driveStateMachine = new StateMachine();

	    /* This is where we create behavior instances */

	    // Starts of tank driving
	    TankBehavior *tankBehavior;
	    tankBehavior = new TankBehavior();
	    tankBehavior->name = "Tank Driving";
	    StateNode *firstNode = driveStateMachine->appendBehavior(tankBehavior);

	    // If the right trigger is pushed, it transitions to straigh driving
	    DriveStraightBehavior *driveStraight = new DriveStraightBehavior();
	    driveStraight->name = "Drive Straight";
	    StateNode *lastNode =driveStateMachine->appendBehavior(driveStraight);

	    // Loop it! If the right trigger is released go back to tank driving
	    lastNode->possibleNextBehaviors[BehaviorComplete] = firstNode;

	    // Schedule this state machine
	    activeStateMachines.push_back(driveStateMachine);

        
        // Create a second state machine for shooting
        StateMachine *shootStateMachine = new StateMachine();

        // Wait for a button to be pressed
        WaitForButtonBehavior *wfb = new WaitForButtonBehavior(&(hardware->manipulatorStick), 1);
        wfb->name = "Wait for shoot button";
        StateNode *firstShootNode = shootStateMachine->appendBehavior(wfb);
        
        // Wait for the shooters to get up to speed or button release
        WaitForSpeedBehavior *wfs = new WaitForSpeedBehavior(-3200.0, 3400.0);
        wfs->name = "Wait for shooters to get to speed";
        shootStateMachine->appendBehavior(wfs);
        
        // Open the gates, maintain speed, check for button release
        ShootFuelBehavior *shoot = new ShootFuelBehavior();
        shoot->targetFrontSpeed = -3200.0;
        shoot->targetBackSpeed = 3400.0;
        shoot->name = "Shoot fuel";
        StateNode *lastShootNode = shootStateMachine->appendBehavior(shoot);

        // Loop it! If button is released go back to waiting
        lastShootNode->possibleNextBehaviors[BehaviorComplete] = firstShootNode;

        // Schedule this state machine
        activeStateMachines.push_back(shootStateMachine);

	    // Note when we started this process
	    clock_gettime(CLOCK_REALTIME, &processStart);
	};

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
