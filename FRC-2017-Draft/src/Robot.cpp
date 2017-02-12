#include <iostream>
#include <memory>
#include <string>

#include <math.h>
#include <WPILib.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <CANTalon.h>
#include <CameraServer.h>
#define PI 3.14159265
#define USE_DRIVE_TIMER 1
#define DRIVE_TICKSPERREV 1000
#define SERVO_UP 0.2

/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will
 * automatically call your Autonomous and OperatorControl methods at the right
 * time as controlled by the switches on the driver station or the field
 * controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
class Robot: public frc::SampleRobot {
	//frc::RobotDrive myRobot { 0, 1 }; // robot drive system
	VictorSP rightDriveMotor { 0 };
	VictorSP leftDriveMotor { 1 };
	CANTalon shooterWheelFront { 1 };
	CANTalon shooterWheelBack { 5 };
	CANTalon ballIntakeRoller1 { 2 };
	CANTalon ballIntakeRoller2 { 4 };
	CANTalon gearCatcherScrew { 3 };
	Servo shooterServo { 4 };
	Servo agitatorServo { 5 };

	frc::Joystick driveLeftStick { 0 };
	frc::Joystick driveRightStick { 1 };
	frc::Joystick manipulatorStick { 2 };

	AnalogGyro driveGyro { 0 };
	AnalogInput wallDistanceSensorR { 2 };
	AnalogInput wallDistanceSensorL { 1 };
	DigitalInput gearCatcherLimitLeft { 1 };
	DigitalInput gearCatcherLimitRight { 0 };

	Timer autoDriveTimer;
	Timer agitatorTimer;

	bool driveReverse;
	bool isGyroResetTelop;
	bool agitatorUp;
	int autoState;

	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameGear1= "Gear Location 1";
	const std::string autoNameGear2 = "Gear Location 2";
	const std::string autoNameGear3 = "Gear Location 3";

public:
	Robot() {
		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls
		driveReverse = false;
		isGyroResetTelop = false;
		autoState = 0;
		agitatorUp = false;
		//myRobot.SetExpiration(0.1);
	}

	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameGear1, autoNameGear1);
		chooser.AddObject(autoNameGear2, autoNameGear2);
		chooser.AddObject(autoNameGear3, autoNameGear3);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);

		//Configure Shooter Talons
		shooterWheelFront.SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
		shooterWheelFront.ConfigNominalOutputVoltage(+0.0f, -0.0f);
		shooterWheelFront.ConfigPeakOutputVoltage(+12.0f, -12.0f);  //Modify this to allow for just forward or just backward spin
		shooterWheelFront.SetTalonControlMode(CANTalon::TalonControlMode::kSpeedMode);
		shooterWheelFront.SetSensorDirection(false);
		shooterWheelFront.SetAllowableClosedLoopErr(0);
		shooterWheelFront.SelectProfileSlot(0);
		shooterWheelFront.SetF(0.0416);
		shooterWheelFront.SetP(0.0);
		shooterWheelFront.SetI(0.0);
		shooterWheelFront.SetD(0.0);
		//shooterWheelFront.Set(0.0);

		shooterWheelBack.SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
		shooterWheelBack.ConfigNominalOutputVoltage(+0.0f, -0.0f);
		shooterWheelBack.ConfigPeakOutputVoltage(+12.0f, -12.0f); //Modify this to allow for just forward or just backward spin
		shooterWheelBack.SetTalonControlMode(CANTalon::TalonControlMode::kSpeedMode);
		shooterWheelBack.SetSensorDirection(false);
		shooterWheelBack.SetAllowableClosedLoopErr(0);
		shooterWheelBack.SelectProfileSlot(0);
		shooterWheelBack.SetF(0.0416);
		shooterWheelBack.SetP(0.0);
		shooterWheelBack.SetI(0.0);
		shooterWheelBack.SetD(0.0);
		//shooterWheelBack.Set(0.0);

		shooterServo.Set(1.0);
		agitatorServo.Set(0.9);
		driveReverse = true;
		driveGyro.Reset();

		//autoDriveTimer = new Timer();
		//agitatorTimer = new Timer();

		CameraServer::GetInstance()->StartAutomaticCapture();
	}

	/*
	 * Smooth joystick input for driving
	 */
	float smoothJoyStick(float joyInput)
	{
		return powf(joyInput,3);
	}

	/*
	 * Switch the front and back of the robot
	 */
	void toggleDriveDirection()
	{
		if(driveLeftStick.GetRawButton(3))
		{
			driveReverse=false;
		}
		else if(driveLeftStick.GetRawButton(2))
		{
			driveReverse=true;
		}
	}

	/*
	 * Allow for robot to drive straight using Gyro feedback
	 */
	void keepDriveStraight(float leftDriveVel, float rightDriveVel, float targetAngle)
	{
		float error = 0, correctionFactor;
		error = targetAngle - driveGyro.GetAngle();
		correctionFactor = (error/75.0);

		if(targetAngle > (driveGyro.GetAngle() - 0.5) || targetAngle < (driveGyro.GetAngle() + 0.5))
		{
			leftDriveMotor.Set((-leftDriveVel) + correctionFactor);
			rightDriveMotor.Set(rightDriveVel + correctionFactor);
		}
		else
		{
			leftDriveMotor.Set(-leftDriveVel);
			rightDriveMotor.Set(rightDriveVel);
		}
	}

	/*
	 * All for human control of drive train
	 */
	void tankDrive()
	{
		toggleDriveDirection();
		//float right = smoothJoyStick(driveRightStick.GetY());
		//float left = smoothJoyStick(driveLeftStick.GetY());
		float right = driveRightStick.GetY();
		float left = driveLeftStick.GetY();

		if(!driveRightStick.GetTrigger())
		{
			if (driveReverse)
			{
				leftDriveMotor.Set(-right);
				rightDriveMotor.Set(left);
			}
			else
			{
				leftDriveMotor.Set(left);
				rightDriveMotor.Set(-right);
			}
			isGyroResetTelop = false;
		}
		else
		{
			if(isGyroResetTelop == false)
			{
				driveGyro.Reset();
				isGyroResetTelop = true;
			}
			if (driveReverse)
			{
				keepDriveStraight(driveRightStick.GetY(), driveRightStick.GetY(), 0);
			}
			else
			{
				keepDriveStraight(-driveRightStick.GetY(), -driveRightStick.GetY(), 0);
			}
		}
	}

	/*
	 * Simple manual shooter control.  A more complex shooting scheme will be needed to step through opening the door and waiting until the shooter is at speed.
	 */
	void shootFuel()
	{
		if(manipulatorStick.GetY() > 0.1 || manipulatorStick.GetY() < -0.1)
		{
			shooterWheelFront.SetP(0.047);
			shooterWheelFront.SetI(0.0);
			shooterWheelFront.SetD(1.2);

			shooterWheelBack.SetP(0.047);
			shooterWheelBack.SetI(0.0);
			shooterWheelBack.SetD(1.2);

			shooterServo.Set(SERVO_UP);
			shooterWheelFront.Set(-1.0*manipulatorStick.GetY() * 3400);
			shooterWheelBack.Set(manipulatorStick.GetY()* 3600);
		}
		else if(manipulatorStick.GetRawButton(1))
		{
			//CHANGE VALUES BELOW TO REFLECT ACTUAL SHOOTING SPEED
			shooterWheelFront.SetP(0.047);
			shooterWheelFront.SetI(0.0);
			shooterWheelFront.SetD(1.2);

			shooterWheelBack.SetP(0.047);
			shooterWheelBack.SetI(0.0);
			shooterWheelBack.SetD(1.2);

			shooterServo.Set(SERVO_UP);
			if(!agitatorUp && (agitatorTimer.Get() > 4.0))
			{
				agitatorServo.Set(0.2);
				agitatorTimer.Reset();
				agitatorTimer.Start();
				agitatorUp = true;
			}
			else if(agitatorTimer.Get() > 2.0)
			{
				agitatorServo.Set(0.9);
				agitatorUp = false;
			}

			//shooterWheelFront.Set(-.6);
			//shooterWheelBack.Set(0.6);
			//shooterWheelBack.PIDWrite(1);
			//shooterWheelFront.PIDWrite(1);
			shooterWheelFront.Set(-3400.0);
			shooterWheelBack.Set(3600.0);
		}
		/*else if(manipulatorStick.GetRawButton(2))
		{
			//CHANGE VALUES BELOW TO REFLECT ACTUAL SHOOTING SPEED
			shooterServo.Set(SERVO_UP);
			shooterWheelFront.Set(-0.65);
			shooterWheelBack.Set(0.65);
		}
		else if(manipulatorStick.GetRawButton(3))
		{
			//CHANGE VALUES BELOW TO REFLECT ACTUAL SHOOTING SPEEDvv
			shooterServo.Set(SERVO_UP);
			shooterWheelFront.Set(-0.7);
			shooterWheelBack.Set(0.7);
		}
		else if(manipulatorStick.GetRawButton(4))
		{
			//CHANGE VALUES BELOW TO REFLECT ACTUAL SHOOTING SPEED
			shooterServo.Set(SERVO_UP);
			shooterWheelFront.Set(-0.75);
			shooterWheelBack.Set(0.75);
		}*/
		else
		{
			//Stop shooting
			shooterServo.Set(1.0);

			shooterWheelFront.SetP(0.0);
			shooterWheelFront.SetI(0.0);
			shooterWheelFront.SetD(0.0);
			shooterWheelBack.SetP(0.0);
			shooterWheelBack.SetI(0.0);
			shooterWheelBack.SetD(0.0);

			shooterWheelFront.Set(0.0);
			shooterWheelBack.Set(0.0);

			agitatorUp = false;
			agitatorTimer.Reset();
			agitatorTimer.Start();
		}
	}

	/*
	 * Simple manual gear catcher control using manipulator stick X axis
	 */
	void controlGearCatcher()
	{
		//Should set a dead-zone for this despite the speed controllers having one built in
		//gearCatcherScrew.Set(manipulatorStick.GetX());
		if(manipulatorStick.GetRawButton(5) && gearCatcherLimitLeft.Get())
		{
			gearCatcherScrew.Set(0.7);
		}
		else if(manipulatorStick.GetRawButton(6))
		{
			gearCatcherScrew.Set(-0.7);
		}
		else
		{
			gearCatcherScrew.Set(0.0);
		}
	}

	/*
	 * Simple manual intake control (doubles as robot climb control for the moment)
	 */
	void controlBallIntake()
	{
		if(manipulatorStick.GetRawButton(7))
		{
			ballIntakeRoller1.Set(1.0);
			ballIntakeRoller2.Set(1.0);
		}
		else
		{
			ballIntakeRoller1.Set(0.0);
			ballIntakeRoller2.Set(0.0);
		}
	}

	/*
	 * Stop the drive motors
	 */
	void stopRobotDrive()
	{
		leftDriveMotor.Set(0);
		rightDriveMotor.Set(0);
	}

	/*
	 * For use during autonomous.  Call before initiating an automated drive command.
	 */
	void resetDrive(bool isTimerBased)
	{
		if(isTimerBased)
		{
			autoDriveTimer.Reset();
			autoDriveTimer.Start();
		}
		else
		{
			//leftDriveEncoder.Reset();
			//rightDriveEncoder.Reset();
		}
	}

	/*
	 * Perform the conversion of encoder ticks to inches
	 */
	float convertDriveTicksToInches(int encTicks)
	{
		return (float)((float)encTicks / DRIVE_TICKSPERREV) * 3.14 * 4.0;
	}

	/*
	 * Used during autonomous to turn the robot to a specified angle.
	 */
	bool turnGyro(float rAngle)
	{
		float error = 0;
		//Positive gyro angle means turning left
		if(rAngle < driveGyro.GetAngle())
		{
			error = fabs(rAngle) - driveGyro.GetAngle();
			if(driveGyro.GetAngle() <= fabs(rAngle) && fabs(error) > 2.0)
			{
				//turn left
				leftDriveMotor.Set((error/140) + 0.2); //0.8
				rightDriveMotor.Set((error/140) + 0.2); //0.8
			}
			else
			{
				stopRobotDrive();
				return true;
			}
		}
		else if(rAngle > driveGyro.GetAngle())
		{
			error = -rAngle - driveGyro.GetAngle();
			if(driveGyro.GetAngle() >= -rAngle && fabs(error) > 2.0)
			{
				//turn right
				leftDriveMotor.Set((error/140) - 0.2); //-0.8
				rightDriveMotor.Set((error/140) - 0.2); //-0.8
			}
			else
			{
				stopRobotDrive();
				return true;
			}
		}
		else
		{
			stopRobotDrive();
			return true;
		}

		return false;
	}

	/*
	 * Used during autonomous to drive the robot fwd or back to a location
	 * Prior to calling this function you must call resetDrive
	 */
	bool autoDriveRobot(float velocityLeft, float velocityRight, float timeSec, float distanceInch, bool isTimerBased)
	{
		if(isTimerBased)
		{
			if(autoDriveTimer.Get() <= timeSec)
			{
				//leftDriveMotor.Set(-velocityLeft);
				//rightDriveMotor.Set(velocityRight);
				keepDriveStraight(velocityLeft, velocityRight, 0);
			}
			else
			{
				stopRobotDrive();
				return true;
			}
		}
		else
		{
			/*if(fabs(convertDriveTicksToInches(rightDriveEncoder.GetRaw())) < fabs(distanceInch))
			{
				leftDriveMotor.Set(-velocityLeft);
				rightDriveMotor.Set(velocityRight);
			}
			else
			{
				stopRobotDrive();
				return true;
			}*/
		}
		return false;
	}

	/*
	 * Used to calculate the robot distance from the wall
	 */
	double CalculateWallDistanceR(bool averaged = true)
	{
		double rawVoltage;
		static double wallDistance;

		if(averaged)
			rawVoltage = (double)(wallDistanceSensorR.GetAverageVoltage());
		else
			rawVoltage = (double)(wallDistanceSensorR.GetVoltage());

		//MB1013
		double VFiveMM = 0.00488; //Old numbers 0.0048359375;  //((4.952 / 5120) * 5);
		wallDistance = ((rawVoltage * 5 * 0.0393) / VFiveMM);  //Units inch

		//MB1030
		//double VFiveMM = 0.009671875;
		//crateDistance = rawVoltage / VFiveMM;

		return wallDistance;
	}

	/*
	 * Used to calculate the robot distance from the wall
	*/
	double CalculateWallDistanceL(bool averaged = true)
	{
		double rawVoltage;
		static double wallDistance;

		if(averaged)
			rawVoltage = (double)(wallDistanceSensorL.GetAverageVoltage());
		else
			rawVoltage = (double)(wallDistanceSensorL.GetVoltage());

		//MB1013
		double VFiveMM = 0.00488; //Old numbers 0.0048359375;  //((4.952 / 5120) * 5);
		wallDistance = ((rawVoltage * 5 * 0.0393) / VFiveMM);  //Units inch

		return wallDistance;
	}

	/*
	 * Update the SmartDashboard
	 */
	void updateDashboard()
	{
		SmartDashboard::PutNumber("Wall Distance Right: ", CalculateWallDistanceR(false));
		SmartDashboard::PutNumber("Wall Distance Left: ", CalculateWallDistanceL(false));
		SmartDashboard::PutNumber("Gyro Reading: ", driveGyro.GetAngle());

		SmartDashboard::PutNumber("GearLimitLeft: ", gearCatcherLimitLeft.Get());
		SmartDashboard::PutNumber("GearLimitRight: ", gearCatcherLimitRight.Get());
	}

	/*
	 * Do nothing
	 */
	void doNothingAutonomous()
	{
		switch(autoState)
		{
			default:
				stopRobotDrive();
				break;
		}
	}

	/*
	 * Align robot parallel with wall
	 */
	bool performRobotFaceAlignment()
	{
		float angleToTurnDegrees = 0.0;
		try
		{
			angleToTurnDegrees = (atan((CalculateWallDistanceR(false) - CalculateWallDistanceL(false)) / 22.625) * 180.0) / PI;
		}
		catch(...)
		{
			angleToTurnDegrees = 0.0;
		}
		SmartDashboard::PutNumber("Angle Off Parallel: ", angleToTurnDegrees);

		return false;
		//return turnGyro(angleToTurnDegrees);
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
	void Autonomous() {
		auto autoSelected = chooser.GetSelected();
		// std::string autoSelected = frc::SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		while (IsAutonomous() && IsEnabled())
		{
			if(autoSelected == autoNameGear1){
				//lowBarAutonomous();
			}
			else if (autoSelected == autoNameGear2)
			{
				//moatRampartAutonomous();
			}
			else if (autoSelected == autoNameGear3)
			{
				//lowBarScoreAutonomous();
			}
			else
			{
				//Default Auto goes here
				doNothingAutonomous();
			}
			updateDashboard();
			Wait(0.005);				// wait for a motor update time
		}
	}

	/*
	 * Runs the motors with arcade steering.
	 */
	void OperatorControl() override {
		//myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled())
		{
			tankDrive();
			shootFuel();
			controlGearCatcher();
			controlBallIntake();
			updateDashboard();

			performRobotFaceAlignment(); //temp..to be used during auto
			// wait for a motor update time
			frc::Wait(0.005);
		}
	}

	/*
	 * Runs during test mode
	 */
	void Test() override {

	}
};

START_ROBOT_CLASS(Robot)
