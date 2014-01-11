#include "_Robot.h"

// can only be called once
START_ROBOT_CLASS(Robot);

// Robot class constructor
Robot::Robot() :
	JagFL(22, CANJaguar::kPosition),
	JagFR(21, CANJaguar::kPosition),
	JagBL(23, CANJaguar::kPosition),
	JagBR(24, CANJaguar::kPosition)
{
	// set up joysticks
	RealJoy1 = new Joystick(1);
	RealJoy2 = new Joystick(2);
	Joystick1 = new SimpleJoystick(RealJoy1);
	Joystick2 = new SimpleJoystick(RealJoy2);
	
	// set up drive class
	MechanumDrive = new RobotDrive(&JagFL, &JagFR, &JagBL, &JagBR);
	
	return;
}

// Robot class destructor
Robot::~Robot()
{
	delete Joystick1;
	delete Joystick2;
	delete RealJoy1;
	delete RealJoy2;
	delete MechanumDrive;
	
	return;
}

/**
 * Robot-wide initialization code should go here.
 * 
 * Use this method for default Robot-wide initialization which will
 * be called when the robot is first powered on.  It will be called exactly 1 time.
 */
void Robot::RobotInit()
{
	this->GetWatchdog().SetEnabled(false);
	this->GetWatchdog().SetExpiration(1000.0);
	
	return;
}

/**
 * Initialization code for disabled mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters disabled mode. 
 */
void Robot::DisabledInit()
{
	this->GetWatchdog().SetEnabled(false);
	this->GetWatchdog().Feed();
	this->GetWatchdog().SetExpiration(500);
	
	this->MechanumDrive->StopMotor();
	
	return;
}

/**
 * Periodic code for disabled mode should go here.
 * 
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in disabled mode.
 */
void Robot::DisabledPeriodic()
{
	this->GetWatchdog().Feed();
		
	this->Joystick1->Update();
	this->Joystick2->Update();
	
	return;
}

/**
 * Initialization code for autonomous mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters autonomous mode.
 */
void Robot::AutonomousInit()
{
	this->GetWatchdog().SetEnabled(true);
	this->GetWatchdog().Feed();
	this->GetWatchdog().SetExpiration(0.5);

	return;
}

/**
 * Periodic code for autonomous mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in autonomous mode.
 */
void Robot::AutonomousPeriodic()
{
	this->GetWatchdog().Feed();
	
	// TODO: autonomous shooting code
	
	// drive forwards half speed
	MechanumDrive->Drive(-0.5, 0.0); 
	// for 2 seconds
	Wait(2.0);
	// stop robot
	MechanumDrive->Drive(0.0, 0.0);
}

/**
 * Initialization code for teleop mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters teleop mode.
 */
void Robot::TeleopInit()
{
	this->GetWatchdog().SetExpiration(1000.0);
	this->GetWatchdog().SetEnabled(true);
	this->GetWatchdog().Feed();
	
	return;
}

/**
 * Periodic code for teleop mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in teleop mode.
 */
void Robot::TeleopPeriodic()
{
	this->GetWatchdog().Feed();
	this->Joystick1->Update();
	this->Joystick2->Update();
	
	// Get driving joystick position
	float direction = this->RealJoy1->GetDirectionDegrees();
	float rotation = this->RealJoy1->GetTwist();
	
	// Set the throttle
	bool turbo = Joystick1->Toggled(BUTTON_8);
	double throttleMag = this->RealJoy1->GetThrottle();
	float stickMag = this->RealJoy1->GetMagnitude();
	float totalMag = throttleMag * stickMag;
	if (turbo)
		totalMag *= 1.5;

	// drive
	if(!Joystick1->Toggled(BUTTON_7))
		MechanumDrive->MecanumDrive_Polar(totalMag, direction, rotation);
	
	return;
}
