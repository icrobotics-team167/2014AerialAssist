#include "_Robot.h"

// can only be called once
START_ROBOT_CLASS(Robot);

// Robot class constructor
Robot::Robot() :
	JagFL(22,CANJaguar::kPosition),
	JagFR(21,CANJaguar::kPosition),
	JagBL(23,CANJaguar::kPosition),
	JagBR(24,CANJaguar::kPosition)
{
	// set up joysticks
	RealJoy1 = new Joystick(1);
	RealJoy2 = new Joystick(2);
	Joystick1 = new SimpleJoystick(RealJoy1);
	Joystick2 = new SimpleJoystick(RealJoy2);
	
	// set up drive class
	UINT16 encoder_lines = 250;
	MechanumDrive = new MechanumWheels(&JagFL, &JagFR, &JagBL, &JagBR, encoder_lines);
	MechanumDrive->SetMaxVoltage(10.0);
	MechanumDrive->StopJags();
	MechanumDrive->Init(true);
	
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

	this->MechanumDrive->Disable();
	
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
	
	this->MechanumDrive->Enable();
	MechanumDrive->SetMaxVoltage(7.0);
		
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
	
	this->MechanumDrive->Enable();
	
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
	float x = this->RealJoy1->GetAxis(Joystick::kXAxis);
	float y = this->RealJoy1->GetAxis(Joystick::kYAxis);
	float z = Vector3::GetRotation(x, y);
	
	//---------------------------------------------------------------------
	// Set the throttle
	//---------------------------------------------------------------------
	bool turbo = Joystick1->Toggled(BUTTON_8);
	double throttle_mag = this->RealJoy1->GetAxis(Joystick::kThrottleAxis) * 7.0;
	
	if(Joystick1->Pressed(BUTTON_3))
		throttle_mag = this->RealJoy1->GetAxis(Joystick::kThrottleAxis) * 12.0;
	
	float abs_x = std::abs(x), abs_y = std::abs(y);
	double stick_mag = max(abs_x, abs_y);
	double outputVolts = throttle_mag * stick_mag;
	
	if(outputVolts < 1)
		outputVolts = 1.0;
	
	if(turbo)
		outputVolts *= 1.5;
	
	if(outputVolts > 12.0)
		outputVolts = 12.0;
	
	MechanumDrive->SetMaxVoltage(outputVolts);
	
	//---------------------------------------------------------------------
	// determine direction
	//---------------------------------------------------------------------
	MechanumWheels::DriveDir dir = MechanumWheels::Stop;
	
	/*
	 * Rotation code: buttons 4 and 5 can be pressed to toggle rotation.
	 * Then, if one of those buttons is pressed and the joystick is moved 
	 * right, the robot will rotate right (clockwise).
	 * If the joystick is moved left, the robot will rotate left (counterclockwise).
	 */
	if(Joystick1->Pressed(BUTTON_4) || Joystick1->Pressed(BUTTON_5))
	{
		if(x > .1)
			dir = MechanumWheels::RotateRight;
		else if (x < -.1)
			dir = MechanumWheels::RotateLeft;
	}
	else if(Vector3::GetMagnitude(x, y) < 0.25)
		// stop
		dir = MechanumWheels::Stop;
	else if(z < 30 && z >= 0.0)
		// right
		dir = MechanumWheels::Right;
	else if(z < 75 && z >= 30)
		// back right (diagonal)
		dir = MechanumWheels::BackRight;
	else if(z < 105 && z >= 75)
		// reverse
		dir = MechanumWheels::Reverse;
	else if(z < 165 && z >= 105)
		// back left (diagonal)
		dir = MechanumWheels::BackLeft;
	else if(z < 195 && z >= 165)
		// left
		dir = MechanumWheels::Left;
	else if(z < 255 && z >= 195)
		// forward left (diagonal)
		dir = MechanumWheels::FwdLeft;
	else if(z < 285 && z >= 255)
		// forward
		dir = MechanumWheels::Forward;
	else if(z < 330 && z >= 285)
		// forward right (diagonal)
		dir = MechanumWheels::FwdRight;
	else if(z <= 360 && z >= 330)
		// right
		dir = MechanumWheels::Right;
	else
		dir = MechanumWheels::Stop;

	if(!Joystick1->Toggled(BUTTON_7))
		MechanumDrive->Update(dir);
	
	MechanumDrive->CheckComplete();
	MechanumWheels::DriveDir task = MechanumDrive->CurrentTask;
		
	if(task == MechanumWheels::ManualDrive ||
		task == MechanumWheels::TaskFinished || 
		task == MechanumWheels::Stop)
	{
		MechanumDrive->Update(dir);
	}
	else
	{
		// currently in a task
		MechanumDrive->FeedJags();
	}
	
	return;
}
