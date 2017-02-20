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
#define SERVO_DOWN 1.0
#define USE_DRIVE_TIMER 1

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
	DigitalInput photoElectric {2};
	DigitalInput gearCatcherLimitLeft { 1 };
	DigitalInput gearCatcherLimitRight { 0 };

	Timer autoDriveTimer;
	Timer agitatorTimer;
	Timer genericTimer;

	bool driveReverse;
	bool isGyroResetTelop;
	bool agitatorUp;
	bool genericTimerStarted;
	int autoState;
	int gearCatcherState;
	double avgShooterVelocityError;

	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameGear1RED= "RED: Gear Location 1";
	const std::string autoNameGear2RED = "RED: Gear Location 2";
	const std::string autoNameGear3RED = "RED: Gear Location 3";
	const std::string autoNameTwoHopperRED = "RED: Two Hopper";

public:
	Robot() {
		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls
		driveReverse = false;
		isGyroResetTelop = false;
		autoState = 0;
		gearCatcherState = 0;
		agitatorUp = false;
		genericTimerStarted = false;
		avgShooterVelocityError = 0.0;
		//myRobot.SetExpiration(0.1);
	}

	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameGear1RED, autoNameGear1RED);
		chooser.AddObject(autoNameGear2RED, autoNameGear2RED);
		chooser.AddObject(autoNameGear3RED, autoNameGear3RED);
		chooser.AddObject(autoNameTwoHopperRED, autoNameTwoHopperRED);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);

		//Configure Shooter Talons
		shooterWheelFront.SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
		shooterWheelFront.ConfigNominalOutputVoltage(+0.0f, -0.0f);
		shooterWheelFront.ConfigPeakOutputVoltage(+12.0f, -12.0f);  //Modify this to allow for just forward or just backward spin
		shooterWheelFront.SetTalonControlMode(CANTalon::TalonControlMode::kSpeedMode);
		shooterWheelFront.SetSensorDirection(false);
		shooterWheelFront.SetAllowableClosedLoopErr(0);
		shooterWheelFront.SelectProfileSlot(0);
		shooterWheelFront.SetF(0.02497); //0.0416
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
		shooterWheelBack.SetF(0.02497);
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
	 * Returns true when time in seconds has expired
	 */
	bool WaitAsyncUntil(double timeInSec, bool resetTimer)
	{
		if(!genericTimerStarted)
		{
			genericTimer.Reset();
			genericTimer.Start();
			genericTimerStarted = true;
		}

		if(genericTimer.Get() >= timeInSec)
		{
			if(resetTimer)
				genericTimerStarted = false;
			return true;
		}

		return false;
	}

	/*
	 * Smooth joystick input for driving
	 */
	float smoothJoyStick(float joyInput)
	{
		return powf(joyInput,2);
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
	 * Spin shooter wheels up to speed with door closed.
	 * Return true when wheels are at speed
	 */

	bool spinShooterWheels(double frontWheel, double backWheel)
	{

		shooterWheelFront.SetP(0.047);
		shooterWheelFront.SetI(0.0);
		shooterWheelFront.SetD(1.2);

		shooterWheelBack.SetP(0.047);
		shooterWheelBack.SetI(0.0);
		shooterWheelBack.SetD(1.2);

		shooterWheelFront.Set(-1.0 * frontWheel);
		shooterWheelBack.Set(backWheel);

		avgShooterVelocityError = (shooterWheelFront.GetClosedLoopError() + shooterWheelBack.GetClosedLoopError()) / 2.0;

		if(avgShooterVelocityError < 500)
			return true;

		return false;
	}

	/*
	 * Stop shooter wheels from spinning
	 */
	void stopShooter()
	{
		shooterServo.Set(SERVO_DOWN);

		shooterWheelFront.SetP(0.0);
		shooterWheelFront.SetI(0.0);
		shooterWheelFront.SetD(0.0);
		shooterWheelBack.SetP(0.0);
		shooterWheelBack.SetI(0.0);
		shooterWheelBack.SetD(0.0);

		shooterWheelFront.Set(0.0);
		shooterWheelBack.Set(0.0);
	}

	/*
	 * Perform task of shooting fuel
	 */
	void shootFuel()
	{
		if(spinShooterWheels(3600.0, 3400.0))
			shooterServo.Set(SERVO_UP);
		else
			shooterServo.Set(SERVO_DOWN);

		if(!agitatorUp && (agitatorTimer.Get() > 0.5))
		{
			agitatorServo.Set(0.2);
			agitatorTimer.Reset();
			agitatorTimer.Start();
			agitatorUp = true;
		}
		else if(agitatorTimer.Get() > 0.5)
		{
			agitatorServo.Set(0.9);
			agitatorTimer.Reset();
			agitatorTimer.Start();
			agitatorUp = false;
		}
	}

	/*
	 * Simple manual shooter control.  A more complex shooting scheme will be needed to step through opening the door and waiting until the shooter is at speed.
	 */
	void shootFuelControl()
	{
		if(manipulatorStick.GetY() > 0.1 || manipulatorStick.GetY() < -0.1)
		{
			if(spinShooterWheels(manipulatorStick.GetY() * 3600.0, manipulatorStick.GetY() * 3400.0))
				shooterServo.Set(SERVO_UP);
			else
				shooterServo.Set(SERVO_DOWN);
		}
		else if(manipulatorStick.GetRawButton(1))
		{
			shootFuel();
		}
		else
		{
			//Stop shooting
			stopShooter();

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
		if(manipulatorStick.GetRawButton(3))
		{
			findGearCatcherLift();
		}
		else
		{
			if(manipulatorStick.GetRawButton(5) && gearCatcherLimitLeft.Get())
			{
				gearCatcherScrew.Set(0.7);
			}
			else if(manipulatorStick.GetRawButton(6) && gearCatcherLimitRight.Get())
			{
				gearCatcherScrew.Set(-0.7);
			}
			else
			{
				gearCatcherScrew.Set(0.0);
			}

			gearCatcherState = 0;
		}
	}

	/*
	 * Simple manual intake control (doubles as robot climb control for the moment)
	 */
	void controlBallIntake()
	{
		if(manipulatorStick.GetRawButton(8))
		{
			ballIntakeRoller1.Set(-1.0);
			ballIntakeRoller2.Set(-1.0);
		}
		else if(manipulatorStick.GetRawButton(7))
		{
			ballIntakeRoller1.Set(-0.3);
			ballIntakeRoller2.Set(-0.3);
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
			resetGyroAngle();
		}
		else
		{
			//leftDriveEncoder.Reset();
			//rightDriveEncoder.Reset();
			//resetGyroAngle();
		}
	}

	/*
	 * Reset the Gyro position
	 */
	void resetGyroAngle()
	{
		driveGyro.Reset();
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
		SmartDashboard::PutNumber("photoElectric: ", photoElectric.Get());

		SmartDashboard::PutNumber("Avg Shooter Vel Error: ", avgShooterVelocityError);
	}

	/*
	 * Align robot parallel with wall
	 */
	float performRobotFaceAlignment()
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

		return angleToTurnDegrees;
	}

	/*
	 * Automatically find the retro-reflective tape for the gear lift
	 */
	bool findGearCatcherLift()
	{

		switch(gearCatcherState)
		{
			case 0:
				if (gearCatcherLimitRight.Get()){
					gearCatcherScrew.Set(-0.7);
				}
				else
				{
					gearCatcherState++;
					gearCatcherScrew.Set(0.0);
				}
				break;
			case 1:
				if (photoElectric.Get() && gearCatcherLimitLeft.Get())
				{
					gearCatcherScrew.Set(0.4);
				}
				else
				{
					gearCatcherState++;
					gearCatcherScrew.Set(0.0);
				}
				break;
			case 2:
				if (!photoElectric.Get())
				{
					gearCatcherScrew.Set(0.0);
					return true;
				}
				else
				{
					gearCatcherState = 0;
				}
				break;

			default:
				gearCatcherScrew.Set(0.0);
				break;
		}

		return false;
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
	 * Get both middle and close hopper of balls (RED)
	 * I've assumed that negative angles will turn clockwise relative to the gear catcher being the front
	 * I've assume positive drive motor speeds will drive the robot in reverse (relative to the gear catcher being the front)
	 */
	void score_RED_TwoHopper_Autonomous()
	{
		switch(autoState)
		{
			case 0:
				resetDrive(USE_DRIVE_TIMER);
				autoState++;
				break;
			case 1:
				//Drive the robot in reverse to get to middle hopper
				if(autoDriveRobot(0.8, 0.8, 2.5, 0, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 2:
				//Turn intake side towards hopper
				if(turnGyro(-90.0))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 3:
				//Drive the robot reverse to trigger hopper
				if(autoDriveRobot(0.5, 0.5, 0.5, 0, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 4:
				//Wait some time for the hopper to empty into robot
				Wait(1.0);
				resetDrive(USE_DRIVE_TIMER);
				autoState++;
				break;
			case 5:
				//Drive the robot forward away from hopper
				if(autoDriveRobot(-0.5, -0.5, 0.5, 0, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 6:
				//Turn gear catcher towards alliance station
				if(turnGyro(90.0))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 7:
				//Drive the robot in forward to get to close hopper
				if(autoDriveRobot(0.8, 0.8, 1.0, 0, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 8:
				//Turn shooter to face boiler
				if(turnGyro(-60.0))
				{
					resetDrive(USE_DRIVE_TIMER);

					agitatorUp = false;
					agitatorTimer.Reset();
					agitatorTimer.Start();

					autoState++;
				}
				break;
			case 9:
				shootFuel();
				if(WaitAsyncUntil(4.0, true))
				{
					stopShooter();
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 10:
				//Turn towards close hopper
				if(turnGyro(-30.0))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 11:
				//Drive the robot reverse to trigger hopper
				if(autoDriveRobot(0.5, 0.5, 0.5, 0, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 12:
				//Wait some time for the hopper to empty into robot
				Wait(1.0);
				resetDrive(USE_DRIVE_TIMER);
				autoState++;
				break;
			case 13:
				//Drive the robot forward away from hopper
				if(autoDriveRobot(-0.5, -0.5, 0.5, 0, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 14:
				//Turn shooter to face boiler
				if(turnGyro(30.0))
				{
					resetDrive(USE_DRIVE_TIMER);

					agitatorUp = false;
					agitatorTimer.Reset();
					agitatorTimer.Start();

					autoState++;
				}
				break;
			case 15:
				shootFuel();
				if(WaitAsyncUntil(5.0, true))
				{
					stopShooter();
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			default:
				stopRobotDrive();
				break;
		}
	}

	/*
	 * Score gear on peg RED (Gear position 1 is closest to the boiler)
	 * I've assumed that negative angles will turn clockwise relative to the gear catcher being the front
	 * I've assume positive drive motor speeds will drive the robot in reverse (relative to the gear catcher being the front)
	 */
	void score_GearPosition1_Autonomous(bool isRed)
	{
		float angleErrorFromUltrasonics = 0.0;
		float angleToTurnForAirship = -90.0; //135

		if(!isRed)
			angleToTurnForAirship *= -1.0;

		switch(autoState)
		{
			case 0:
				resetDrive(USE_DRIVE_TIMER);
				autoState++;
				break;
			case 1:
				//Drive the robot in reverse
				if(autoDriveRobot(0.5, 0.5, 0.75, 0, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 2:
				if(turnGyro(angleToTurnForAirship))
				{
					Wait(0.5);
					//resetDrive(USE_DRIVE_TIMER);
					//autoState++;
				}
				break;
			case 3:
				//Drive the robot forward to get a bit closer to airship
				if(autoDriveRobot(-0.5, -0.5, 0.5, 0, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 4:
				//Align the robot face parallel to airship wall
				angleErrorFromUltrasonics = performRobotFaceAlignment();
				if(fabs(angleErrorFromUltrasonics) > 20.0)	//Assume that if the angle returned is a large value there has been an error with the sensors
				{
					autoState = -1;	//Abort
				}
				else
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 5:
				if(turnGyro(angleErrorFromUltrasonics))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 6:
				//Search for the peg using the photoelectric sensor
				if(findGearCatcherLift())
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				else if(WaitAsyncUntil(2.0, true))
				{
					autoState = -1; //Abort due to timeout
				}
				break;
			case 7:
				//Drive the robot forward to get the gear on the peg
				if(autoDriveRobot(-0.3, -0.3, 1.0, 0, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 8:
				//Wait some time for the pilot to remove the gear.  Would be better to have a sensor for this.
				Wait(3.0);
				resetDrive(USE_DRIVE_TIMER);
				autoState++;
				break;
			case 9:
				//Drive the robot reverse
				if(autoDriveRobot(0.5, 0.5, 1.0, 0, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			default:
				stopRobotDrive();
				break;
		}
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
			if(autoSelected == autoNameGear1RED){
				score_GearPosition1_Autonomous(false); //temp BLUE
			}
			else if (autoSelected == autoNameGear2RED)
			{
				//moatRampartAutonomous();
			}
			else if (autoSelected == autoNameGear3RED)
			{
				//lowBarScoreAutonomous();
			}
			else if (autoSelected == autoNameTwoHopperRED)
			{
				score_RED_TwoHopper_Autonomous();
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
			shootFuelControl();
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
