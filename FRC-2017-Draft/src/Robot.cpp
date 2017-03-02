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
#define GRAVITY_IN_S2 385.827
#define SHOOTER_ANGLE_DEGREES 76
#define SHOOTER_WHEEL_DIAMETER_INCH 2.375
#define SHOOTER_PCT_EFFICIENCY 97
#define DRIVE_TICKSPERREV 64
#define SERVO_UP 0.2
#define SERVO_DOWN 1.0
#define USE_DRIVE_TIMER 0
#define MAX_BATTERY 12.0

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
	Talon agitatorMotor { 2 };

	frc::Joystick driveLeftStick { 0 };
	frc::Joystick driveRightStick { 1 };
	frc::Joystick manipulatorStick { 2 };

	AnalogGyro driveGyro { 0 };
	AnalogInput wallDistanceSensorS { 3 };
	AnalogInput wallDistanceSensorR { 2 };
	AnalogInput wallDistanceSensorL { 1 };
	DigitalInput photoElectric {2};
	DigitalInput photoElectricShooter {3};
	DigitalInput gearCatcherLimitLeft { 1 };
	DigitalInput gearCatcherLimitRight { 0 };
	Encoder rightDriveEncoder {8 , 9, false};

	Timer autoDriveTimer;
	Timer agitatorTimer;
	Timer genericTimer;

	bool stoleDriveTrainControl;	//Set to true if an autonomous function is controlling the drive train during tele-op
	bool driveReverse;
	bool isGyroResetTelop;
	bool agitatorUp;
	bool genericTimerStarted;
	int autoState;
	int gearCatcherState;
	int shootFuelState;
	double avgShooterVelocityError;
	double gyroKi; //Integrator term for gyro
	//const double shootSpeedArray[3] = {1000.0, 3600.0, 4000.0};
	//const double shootDistanceInchArray[3] = {36.0, 118.0, 160.0};

	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameGear1 = "Gear Location 1";
	const std::string autoNameGear2 = "Gear Location 2";
	const std::string autoNameGear3 = "Gear Location 3";
	const std::string autoNameTwoHopper = "Two Hopper";

public:
	Robot() {
		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls
		stoleDriveTrainControl = false;
		driveReverse = false;
		isGyroResetTelop = false;
		autoState = 0;
		gearCatcherState = 0;
		shootFuelState = 0;
		agitatorUp = false;
		genericTimerStarted = false;
		avgShooterVelocityError = 0.0;
		gyroKi = 0.0;
		//myRobot.SetExpiration(0.1);
	}

	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameGear1, autoNameGear1);
		chooser.AddObject(autoNameGear2, autoNameGear2);
		chooser.AddObject(autoNameGear3, autoNameGear3);
		chooser.AddObject(autoNameTwoHopper, autoNameTwoHopper);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);

		//Configure Shooter Talons
		shooterWheelFront.SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
		shooterWheelFront.ConfigNominalOutputVoltage(+0.0f, -0.0f);
		shooterWheelFront.ConfigPeakOutputVoltage(+0.0f, -12.0f);  //Modify this to allow for just forward or just backward spin
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
		shooterWheelBack.ConfigPeakOutputVoltage(+12.0f, -0.0f); //Modify this to allow for just forward or just backward spin
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
			leftDriveMotor.Set(((-leftDriveVel) + correctionFactor) * batteryCompensationPct());
			rightDriveMotor.Set((rightDriveVel + correctionFactor) * batteryCompensationPct());
		}
		else
		{
			leftDriveMotor.Set(-leftDriveVel * batteryCompensationPct());
			rightDriveMotor.Set(rightDriveVel * batteryCompensationPct());
		}
	}

	/*
	 * Determine whether we should keep driving straight in tele-op
	 */
	bool shouldIHelpDriverDriveStraight()
	{
		float right = driveRightStick.GetY();
		float left = driveLeftStick.GetY();
		float diff = fabs(right-left);
		bool sameSign = ((right < 0.0 && left < 0.0) || (right > 0.0 && left > 0.0))  ? true : false;

		if(sameSign && (diff < 0.2))
			return true;

		return false;
	}

	/*
	 * All for human control of drive train
	 */
	void tankDrive()
	{
		toggleDriveDirection();
		//float right = smoothJoyStick(driveRightStick.GetY());
		//float left = smoothJoyStick(driveLeftStick.GetY());
		double right = driveRightStick.GetY();
		double left = driveLeftStick.GetY();
		double avgStick = (right + left) / 2.0;

		if(!driveRightStick.GetTrigger() && !shouldIHelpDriverDriveStraight())
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
				keepDriveStraight(avgStick, avgStick, 0);
			}
			else
			{
				keepDriveStraight(-avgStick, -avgStick, 0);
			}
		}
	}

	/*
	 * Spin shooter wheels up to speed with door closed.
	 * Return true when wheels are at speed
	 */
	bool spinShooterWheels(double frontWheel, double backWheel)
	{
		//Ensure we do not tell the shooter to spin backwards
		if(frontWheel < 0.0)
			frontWheel = 0.0;
		if(backWheel < 0.0)
			backWheel = 0.0;

		shooterWheelFront.SetP(0.047);
		shooterWheelFront.SetI(0.0);
		shooterWheelFront.SetD(1.2); //1.2

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

		agitatorMotor.Set(0.0);
	}

	/*
	 * Perform task of shooting fuel
	 */
	void shootFuel(bool useDistanceSensor)
	{
		if(useDistanceSensor)
		{	double shootRPM = calculateShotSpeedBasedOnDistance();

			if(shootRPM != 0.0) //0.0 indicated error
			{
				if(spinShooterWheels(shootRPM, shootRPM - 200.0))
					shooterServo.Set(SERVO_UP);
				else
					shooterServo.Set(SERVO_DOWN);
			}
		}
		else
		{
			if(spinShooterWheels(3400.0, 3600.0)) //3600, 3400
				shooterServo.Set(SERVO_UP);
			else
				shooterServo.Set(SERVO_DOWN);
		}

		if(!agitatorUp && (agitatorTimer.Get() > 1.0))
		{
			agitatorServo.Set(0.2);
			agitatorMotor.Set(1.0);
			agitatorTimer.Reset();
			agitatorTimer.Start();
			agitatorUp = true;
		}
		else if(agitatorTimer.Get() > 2.0)
		{
			agitatorServo.Set(0.9);
			agitatorMotor.Set(-1.0);
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
		double angleBoilerFoundDeg = 0.0;

		if(manipulatorStick.GetY() > 0.1 || manipulatorStick.GetY() < -0.1)
		{
			/*
			if(spinShooterWheels(manipulatorStick.GetY() * 3600.0, manipulatorStick.GetY() * 3400.0))
				shooterServo.Set(SERVO_UP);
			else
				shooterServo.Set(SERVO_DOWN);
			 */
		}
		else if(manipulatorStick.GetRawButton(1))
		{
			stoleDriveTrainControl = true;
			switch(shootFuelState)
			{
				case 0:
					resetDrive(USE_DRIVE_TIMER);
					shootFuelState++;
					break;

				case 1:
					//Search for boiler turning max 90 deg clockwise (assuming gear catcher is the front)
					if(turnGyro(-90.0, 0.3))
					{
						Wait(0.25);
						resetDrive(USE_DRIVE_TIMER);
						shootFuelState = -1;
					}
					if(!photoElectricShooter.Get())
					{
						stopRobotDrive();

						angleBoilerFoundDeg = driveGyro.GetAngle();

						agitatorUp = false;
						agitatorTimer.Reset();
						agitatorTimer.Start();

						shootFuelState++;
					}
					break;

				case 2:
					//Attempt to keep the robot pointing in the correct direction
					if(!photoElectricShooter.Get() && turnGyro(angleBoilerFoundDeg, 0.3))
					{
						shootFuel(false); //Use the distance sensor to adjust shot speed set to true
					}
					else
					{
						//Robot lost sight of the boiler with the photoelectric sensor so stop shooting
						stopShooter();
						//shootFuelState = -1;
					}
					break;

				default:
					stopRobotDrive();
					stopShooter();
					break;
			}
		}
		else if(manipulatorStick.GetRawButton(2))
		{
			shootFuel(false);	//For manual emergency
		}
		else
		{
			//Stop shooting
			stopShooter();

			shootFuelState = 0;
			stoleDriveTrainControl = false;

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
			rightDriveEncoder.Reset();
			resetGyroAngle();
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
	bool turnGyro(float rAngle, float maxTurnSpeed = 0.5)
	{
		float error = 0.0;
		float speedToSet = 0.0;
		//Positive gyro angle means turning left
		if(rAngle < driveGyro.GetAngle())
		{
			//Start accumulating error if the rate of turning is < 2 deg/sec
			if(driveGyro.GetRate() < 2.0)
			{
				gyroKi += 0.001;
				if(gyroKi > 0.2) //Cap the integral term
					gyroKi = 0.2;
			}

			error = fabs(rAngle) - driveGyro.GetAngle();
			if(driveGyro.GetAngle() <= fabs(rAngle) && fabs(error) > 2.0)
			{
				//turn left
				speedToSet = (error/270) + 0.2 + gyroKi; //140 0.2
				if(fabs(speedToSet) > maxTurnSpeed)
					speedToSet = maxTurnSpeed * (speedToSet < 0.0 ? -1.0 : 1.0);
				leftDriveMotor.Set(speedToSet * batteryCompensationPct()); //0.8
				rightDriveMotor.Set(speedToSet * batteryCompensationPct()); //0.8
			}
			else
			{
				gyroKi = 0.0;
				stopRobotDrive();
				//if(WaitAsyncUntil(0.5,true))
					return true;
			}
		}
		else if(rAngle > driveGyro.GetAngle())
		{
			//Start accumulating error if the rate of turning is < 2 deg/sec
			if(driveGyro.GetRate() < 2.0)
			{
				gyroKi += 0.001;
				if(gyroKi > 0.2) //Cap the integral term
					gyroKi = 0.2;
			}

			error = -rAngle - driveGyro.GetAngle();
			if(driveGyro.GetAngle() >= -rAngle && fabs(error) > 2.0)
			{
				//turn right
				speedToSet = (error/270) - 0.2 - gyroKi;
				if(fabs(speedToSet) > maxTurnSpeed)
					speedToSet = maxTurnSpeed * (speedToSet < 0.0 ? -1.0 : 1.0);
				leftDriveMotor.Set(speedToSet * batteryCompensationPct()); //-0.8
				rightDriveMotor.Set(speedToSet * batteryCompensationPct()); //-0.8
			}
			else
			{
				gyroKi = 0.0;
				stopRobotDrive();
				//if(WaitAsyncUntil(0.5,true))
					return true;
			}
		}
		else
		{
			gyroKi = 0.0;
			stopRobotDrive();
			return true;
		}

		return false;
	}

	/*
	 * Used during autonomous to drive the robot fwd or back to a location
	 * Prior to calling this function you must call resetDrive
	 */
	bool autoDriveRobot(float velocityLeft, float velocityRight, float timeSec, float targetDistanceInch, bool isTimerBased)
	{
		double err = 0.0;
		double driveDistInch = 0.0;
		double percentPower = 0.0;
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
			driveDistInch = fabs(convertDriveTicksToInches(rightDriveEncoder.Get()));
			if(driveDistInch < fabs(targetDistanceInch))
			{
				//leftDriveMotor.Set(-velocityLeft);
				//rightDriveMotor.Set(velocityRight);
				err = fabs(targetDistanceInch) - driveDistInch;
				percentPower = (err / fabs(targetDistanceInch));

				if(err <= 48.0)	//If within 24" start slowing down
				{
					velocityLeft *= percentPower;
					velocityRight *= percentPower;

					if(velocityLeft < 0.0 && velocityLeft > -0.2)
						velocityLeft = -0.2;
					else if(velocityLeft > 0.0 && velocityLeft < 0.2)
						velocityLeft = 0.2;
					if(velocityRight < 0.0 && velocityRight > -0.2)
						velocityRight = -0.2;
					else if(velocityRight > 0.0 && velocityRight < 0.2)
						velocityRight = 0.2;
				}

				keepDriveStraight(velocityLeft, velocityRight, 0);
			}
			else
			{
				stopRobotDrive();
				return true;
			}
		}
		return false;
	}

	/*
	 * Calculate the shooter speed based on ultrasonic measured distance
	 */
	double calculateShotSpeedBasedOnDistance()
	{
		double currentShooterDistanceInch = CalculateWallDistanceShooter(false) + 17.5;

		//Minimum shot distance
		if(currentShooterDistanceInch < 24.0)
			return 0.0;

		double shooterVelocity = 0.0;
		double shooterCalculatedRPM = 0.0;
		shooterVelocity = sqrt(((currentShooterDistanceInch * 2.0) * GRAVITY_IN_S2) / sin(2.0 * SHOOTER_ANGLE_DEGREES * PI / 180.0));
		shooterCalculatedRPM = (shooterVelocity * 60.0)/ (SHOOTER_WHEEL_DIAMETER_INCH * PI) * (SHOOTER_PCT_EFFICIENCY / 100.0);

		SmartDashboard::PutNumber("Calculated Shot Speed (RPM): ", shooterCalculatedRPM);

		return shooterCalculatedRPM;
	}

	/*
	 * Used to calculate the robot distance from the wall
	 */
	double CalculateWallDistanceShooter(bool averaged = true)
	{
		double rawVoltage;
		static double wallDistance;

		if(averaged)
			rawVoltage = (double)(wallDistanceSensorS.GetAverageVoltage());
		else
			rawVoltage = (double)(wallDistanceSensorS.GetVoltage());

		//MB1030
		double VFiveMM = 0.009671875;
		wallDistance = rawVoltage / VFiveMM;

		return wallDistance;
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
		//wallDistance = rawVoltage / VFiveMM;

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
	 * Calculate a battery compensation percentage to multiply pwm output by
	 */
	double batteryCompensationPct()
	{
		double batteryScaleFactor = 0.0;
		batteryScaleFactor = MAX_BATTERY / DriverStation::GetInstance().GetBatteryVoltage();

		return batteryScaleFactor;
	}

	/*
	 * Update the SmartDashboard
	 */
	void updateDashboard()
	{
		SmartDashboard::PutNumber("Wall Distance Right: ", CalculateWallDistanceR(false));
		SmartDashboard::PutNumber("Wall Distance Left: ", CalculateWallDistanceL(false));
		SmartDashboard::PutNumber("Wall Distance Shooter: ", CalculateWallDistanceShooter(false));
		SmartDashboard::PutNumber("Gyro Reading: ", driveGyro.GetAngle());

		SmartDashboard::PutNumber("GearLimitLeft: ", gearCatcherLimitLeft.Get());
		SmartDashboard::PutNumber("GearLimitRight: ", gearCatcherLimitRight.Get());
		SmartDashboard::PutNumber("photoElectric: ", photoElectric.Get());
		SmartDashboard::PutNumber("photoElectricShooter: ", photoElectricShooter.Get());

		SmartDashboard::PutNumber("Avg Shooter Vel Error: ", avgShooterVelocityError);

		SmartDashboard::PutNumber("Drive Encoder Ticks: ", rightDriveEncoder.Get());
		SmartDashboard::PutNumber("Drive Encoder Inch: ", convertDriveTicksToInches(rightDriveEncoder.Get()));

		SmartDashboard::PutNumber("Battery Scaling Factor: ", batteryCompensationPct());
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
					gearCatcherScrew.Set(0.0);
					gearCatcherState++;
				}
				break;
			case 1:
				if (photoElectric.Get() && gearCatcherLimitLeft.Get())
				{
					gearCatcherScrew.Set(0.4);
				}
				else
				{
					gearCatcherScrew.Set(0.0);

					if(!gearCatcherLimitLeft.Get())
						gearCatcherState = 0;
					else
						return true;
						//gearCatcherState++;
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
	void score_TwoHopper_Autonomous(bool isRED)
	{
		double angleMultiplier = 1.0;
		float farHopperDistance = 285.0 + 52.0; //Red Alliance
		if(!isRED)
		{
			//Blue
			angleMultiplier = -1.0;
			farHopperDistance = 285.0;
		}

		switch(autoState)
		{
			case 0:
				resetDrive(USE_DRIVE_TIMER);
				autoState++;
				break;
			case 1:
				//Drive the robot in reverse to get to middle hopper
				if(autoDriveRobot(0.8, 0.8, 0, farHopperDistance, USE_DRIVE_TIMER))
				{
					Wait(0.25);
					//resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 2:
				//Turn intake side towards hopper
				if(turnGyro(-90.0 * angleMultiplier))
				{
					Wait(0.25);
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 3:
				//Drive the robot reverse to trigger hopper
				if(autoDriveRobot(0.5, 0.5, 1.5, 84, true))
				{
					resetDrive(true);
					autoState++;
				}
				break;
			case 4:
				//Wait some time for the hopper to empty into robot
				if(WaitAsyncUntil(1.0, true))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 5:
				//Drive the robot forward away from hopper
				if(autoDriveRobot(-0.5, -0.5, 0, 74, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 6:
				//Turn gear catcher towards alliance station
				if(turnGyro(90.0 * angleMultiplier))
				{
					Wait(0.5);
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 7:
				//Drive the robot in forward to get to close hopper
				if(autoDriveRobot(0.8, 0.8, 0, farHopperDistance - 177.0, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 8:
				if(turnGyro(-135.0, 0.3))
				{
					Wait(0.25);
					resetDrive(USE_DRIVE_TIMER);
					autoState = -1;
				}
				if(!photoElectricShooter.Get())
				{
					stopRobotDrive();

					agitatorUp = false;
					agitatorTimer.Reset();
					agitatorTimer.Start();

					autoState++;
				}
				break;
			case 9:
				shootFuel(false);
				if(WaitAsyncUntil(4.0, true))
				{
					stopShooter();
					resetDrive(USE_DRIVE_TIMER);
					//autoState++;
					autoState = -1;
				}
				break;
			case 10:
				//Turn towards close hopper
				if(turnGyro(-30.0))
				{
					Wait(0.5);
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
					Wait(0.5);
					resetDrive(USE_DRIVE_TIMER);

					agitatorUp = false;
					agitatorTimer.Reset();
					agitatorTimer.Start();

					autoState++;
				}
				break;
			case 15:
				shootFuel(false);
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
	void score_GearPosition1_Autonomous(bool isRED, bool shootFuelAfterGear = true)
	{
		float angleErrorFromUltrasonics = 0.0;
		float angleToTurn = -120.0; //For RED -120
		float angleForBoiler = 110.0; //90
		float distanceToDrive = 70.0; //For RED. should be 103

		if(!isRED)
		{
			angleToTurn *= -1.0;
		}

		switch(autoState)
		{
			case 0:
				resetDrive(USE_DRIVE_TIMER);
				autoState++;
				break;
			case 1:
				//Drive the robot in reverse
				if(autoDriveRobot(0.5, 0.5, 0, distanceToDrive, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
					//autoState= -1;
				}
				break;
			case 2:
				if(turnGyro(angleToTurn, 0.35))
				{
					Wait(0.25);
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 3:
				//Home gear catcher
				gearCatcherState = 0;
				findGearCatcherLift();

				//Drive the robot forward to get a bit closer to airship
				if(autoDriveRobot(-0.4, -0.4, 1.3, 30, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					//autoState++;
					autoState = 6;//6
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
					resetDrive(true);
					autoState++;
				}
				/*else if(WaitAsyncUntil(5.0, true))
				{
					autoState = -1; //Abort due to timeout
				}*/
				break;
			case 7:
				//Drive the robot forward to get the gear on the peg
				if(autoDriveRobot(-0.3, -0.3, 0.5, 14, true))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 8:
				//Wait some time for the pilot to remove the gear.  Would be better to have a sensor for this.
				if(WaitAsyncUntil(2.0, true))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 9:
				//Drive the robot reverse
				if(autoDriveRobot(0.4, 0.4, 0.6, 36, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);

					if(shootFuelAfterGear)
						autoState++;
					else
						autoState = -1;
				}
				break;
			case 10:
				if(turnGyro(angleForBoiler))
				{
					Wait(0.25);

					//agitatorUp = false;
					//agitatorTimer.Reset();
					//agitatorTimer.Start();

					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 11:
				if(turnGyro(-1.0 * angleForBoiler, 0.3))
				{
					Wait(0.25);
					resetDrive(USE_DRIVE_TIMER);
					autoState = -1;
				}
				if(!photoElectricShooter.Get())
				{
					stopRobotDrive();

					agitatorUp = false;
					agitatorTimer.Reset();
					agitatorTimer.Start();

					autoState++;
				}
				break;
			case 12:
				shootFuel(false);
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
	 * Score gear on peg RED (Gear position 2 is middle peg)
	 * I've assumed that negative angles will turn clockwise relative to the gear catcher being the front
	 * I've assume positive drive motor speeds will drive the robot in reverse (relative to the gear catcher being the front)
	 */
	void score_GearPosition2_Autonomous()
	{
		float distanceToDrive = 48.0;

		switch(autoState)
		{
			case 0:
				resetDrive(USE_DRIVE_TIMER);
				autoState++;
				break;
			case 1:
				//Home gear catcher
				gearCatcherState = 0;
				findGearCatcherLift();

				//Drive the robot forward
				if(autoDriveRobot(-0.5, -0.5, 0, distanceToDrive, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
					//autoState= -1;
				}
				break;
			case 2:
				//Search for the peg using the photoelectric sensor
				if(findGearCatcherLift())
				{
					resetDrive(true);
					autoState++;
				}
				/*else if(WaitAsyncUntil(5.0, true))
				{
					autoState = -1; //Abort due to timeout
				}*/
				break;
			case 3:
				//Drive the robot forward to get the gear on the peg
				if(autoDriveRobot(-0.3, -0.3, 0.5, 14, true))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 4:
				//Wait some time for the pilot to remove the gear.  Would be better to have a sensor for this.
				if(WaitAsyncUntil(3.0, true))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 5:
				//Drive the robot reverse
				if(autoDriveRobot(0.4, 0.4, 0.6, 36, USE_DRIVE_TIMER))
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
		DriverStation::Alliance allianceColor;
		bool isAllianceRED = false;
		auto autoSelected = chooser.GetSelected();
		// std::string autoSelected = frc::SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		gearCatcherState = 0;
		autoState = 0;

		//Determine current alliance color
		allianceColor = DriverStation::GetInstance().GetAlliance();
		if(allianceColor == DriverStation::Alliance::kRed)
		{
			isAllianceRED = true;
		}

		while (IsAutonomous() && IsEnabled())
		{
			if(autoSelected == autoNameGear1)
			{
				score_GearPosition1_Autonomous(isAllianceRED);
			}
			else if (autoSelected == autoNameGear2)
			{
				score_GearPosition2_Autonomous();
			}
			else if (autoSelected == autoNameGear3)
			{
				score_GearPosition1_Autonomous(!isAllianceRED, false);
			}
			else if (autoSelected == autoNameTwoHopper)
			{
				score_TwoHopper_Autonomous(isAllianceRED);
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
		driveGyro.Reset();
		while (IsOperatorControl() && IsEnabled())
		{
			if(!stoleDriveTrainControl)
				tankDrive();
			shootFuelControl();
			controlGearCatcher();
			controlBallIntake();
			updateDashboard();

			//performRobotFaceAlignment(); //temp..to be used during auto
			calculateShotSpeedBasedOnDistance();
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
