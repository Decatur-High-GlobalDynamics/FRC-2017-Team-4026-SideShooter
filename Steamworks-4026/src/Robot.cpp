#include <iostream>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <time.h>
#include "StateMachine.hpp"
#include "WaitBehavior.hpp"
#include "Hardware.hpp"
#include "ConcurrentMultiBehavior.hpp"

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
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);

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
		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
		    fprintf(stderr, "*** Configuring ***\n");
		    StateMachine *stateMachine = new StateMachine();

		    /* This is where we create behavior instances */
		    WaitBehavior *wb;
		    wb = new WaitBehavior(5000);
		    wb->name = "5 second wait";
		    StateNode *firstNode = stateMachine->appendBehavior(wb);

		    wb = new WaitBehavior(2000);
		    wb->name = "2 second wait";
		    stateMachine->appendBehavior(wb);

		    ConcurrentMultiBehavior *concurrentMultiBehavior = new ConcurrentMultiBehavior();
		    concurrentMultiBehavior->name = "Timer Multi Behavior";

		    wb = new WaitBehavior(500);
		    wb->name = "0.5 second wait";
		    concurrentMultiBehavior->appendBehavior(wb);

		    wb = new WaitBehavior(700);
		    wb->name = "0.7 second wait (just .2 seconds later)";
		    concurrentMultiBehavior->appendBehavior(wb);

		    wb = new WaitBehavior(1000);
		    wb->name = "1.0 second wait (just .3 seconds after that)";
		    concurrentMultiBehavior->appendBehavior(wb);

		    stateMachine->appendBehavior(concurrentMultiBehavior);

		    wb = new WaitBehavior(1500);
		    wb->name = "1.5 second wait";
		    StateNode *lastNode = stateMachine->appendBehavior(wb);

		    // For fun, let's make it loop!
		    lastNode->possibleNextBehaviors[BehaviorComplete] = firstNode;

		    activeStateMachines.push_back(stateMachine);

		    // Note when we started this process
		    clock_gettime(CLOCK_REALTIME, &processStart);


		}
	}

	void AutonomousPeriodic() {
		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			std::vector<StateMachine *>::iterator it = activeStateMachines.begin();

		    while (it < activeStateMachines.end()) {
		    	StateMachine *stateMachine = *it;
		        stateMachine->execute(hardware, millisSinceStart());
		        struct timespec waitSpec;
		        waitSpec.tv_sec = 0;
		        waitSpec.tv_nsec = 10000000;
		        nanosleep(&waitSpec, NULL);
		    }
		}
	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {

	}

	void TestPeriodic() {
		lw->Run();
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
};

START_ROBOT_CLASS(Robot)
