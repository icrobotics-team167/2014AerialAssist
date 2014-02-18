#include "_Robot.h"

// can only be called once
START_ROBOT_CLASS(Robot);

// Robot class constructor
Robot::Robot() :
	JagFL(21, CANJaguar::kPosition),
	JagFR(22, CANJaguar::kPosition),
	JagBL(23, CANJaguar::kPosition),
	JagBR(24, CANJaguar::kPosition),
	VicCatapult(1),
	JagRoller(25, CANJaguar::kPercentVbus),
	JagRollerArm(26, CANJaguar::kPercentVbus),
	
	ArmDownSwitch(3),
	ArmUpSwitch(2),
	
	CatapultPhotoEye(1),
	
	ExtraCockWait(),
	ShootWait()
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
	
	state = Default;
	
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
	
	if (MechanumDrive)
		this->MechanumDrive->Disable();
	
	// reset timers
	ShootWait.Stop();
	ShootWait.Reset();
	ExtraCockWait.Stop();
	ExtraCockWait.Reset();
	
	// reset all button toggles
	this->Joystick1->DisableToggleAll();
	this->Joystick2->DisableToggleAll();
	
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
	this->GetWatchdog().SetExpiration(1000.0);
	this->GetWatchdog().SetEnabled(true);
	this->GetWatchdog().Feed();

	if (MechanumDrive)
		this->MechanumDrive->Enable();
	
	this->MechanumDrive->SetMaxVoltage(7.0);
	
	// cock the catapult extra
	VicCatapult.Set(-1);
	Wait(0.01);
	VicCatapult.Set(0);
	
	// get the vision target (to check if it's hot)
	autoTarget = getBestTarget(true, false);
	
	// drive for 2 seconds
	Timer AutoDriveTimer;
	
	while (AutoDriveTimer.Get() < 2000)
	{
		if (AutoDriveTimer.Get() == 0)
			AutoDriveTimer.Start();
		
		// drive forward
		MechanumDrive->CheckComplete();
		MechanumWheels::DriveDir task = MechanumDrive->CurrentTask;
		
		if (task == MechanumWheels::ManualDrive ||
			task == MechanumWheels::TaskFinished || 
			task == MechanumWheels::Stop)
		{
			MechanumDrive->Update(MechanumWheels::Forward);
		}
		else
		{
			// currently in a task
			MechanumDrive->FeedJags();
		}
	}
	
	MechanumDrive->Update(MechanumWheels::Stop);
	AutoDriveTimer.Stop();
	
	// put the arm down
	Timer AutoArmDownTimer;
	bool arm_down = !ArmDownSwitch.Get();
	
	while (AutoArmDownTimer.Get() < 500 && !arm_down)
	{
		if (AutoArmDownTimer.Get() == 0)
			AutoArmDownTimer.Start();
		
		arm_down = !ArmDownSwitch.Get();
		
		JagRollerArm.Set(-0.5);
	}
	
	JagRollerArm.Set(0);
	AutoArmDownTimer.Stop();
	
	// if the target is NOT hot, wait until it is before shooting
	if (!autoTarget.hot)
		Wait(1.0);
	
	// shoot
	VicCatapult.Set(-1);
	Wait(0.01);
	VicCatapult.Set(0);

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

	return;
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
	
	// reset timers
	ShootWait.Stop();
	ShootWait.Reset();
	ExtraCockWait.Stop();
	ExtraCockWait.Reset();
	
	// reset button toggles
	this->Joystick1->DisableToggleAll();
	this->Joystick2->DisableToggleAll();
	
	return;
}

void Robot::SetPIDs()
{
	DriverStation * ds = DriverStation::GetInstance();
	
	float P = ds->GetAnalogIn(1);
 	float I = ds->GetAnalogIn(2);
	float D = ds->GetAnalogIn(3);
 	
 	for (int wheel = 0; wheel < 4; wheel++)
 	{
 		if (ds->GetDigitalIn(wheel + 1))
 			MechanumDrive->SetPID(wheel, P, I, D);
 	}
 	
 	return;
}

/**
 * Periodic code for teleop mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in teleop mode.
 * 
 * TeleopPeriodic is called about once every 20 ms; it is synchronized with updates from the Driver Station
 */
void Robot::TeleopPeriodic()
{	
	this->GetWatchdog().Feed();
	this->Joystick1->Update();
	this->Joystick2->Update();
	
	// -------------------------------------------------
	// get distance to a vision target if one is in view
	// -------------------------------------------------
	
	// only spend time doing distance calculation if the driver wants it
	if (Joystick1->Pressed(BUTTON_11))
	{
		TargetReport target = getBestTarget(false, true);
		SmartDashboard::PutNumber("distance", target.distance);
		
		// todo "in shooting range" Dashboard variable?
	}
	else
		SmartDashboard::PutNumber("distance", -1.0);

	//-------------------------
	// drive logic (input side)
	//-------------------------
	
	// get joystick position
	float x = this->RealJoy1->GetAxis(Joystick::kXAxis);
	float y = -this->RealJoy1->GetAxis(Joystick::kYAxis);
	float z = Vector3::GetRotation(x, y);
	
	// raw axis 3 is the twist axis on the Logitech Extreme 3D Pro joystick
	// we use the raw axis because the default mappings are incorrect
	float twist = this->RealJoy1->GetRawAxis(3);
	
	float thumbStickX = this->RealJoy1->GetRawAxis(5);
	// note: -1 if pushed forward, 1 if pulled back
	float thumbStickY = this->RealJoy1->GetRawAxis(6);
	
	// Set the throttle
	bool turbo = Joystick1->Toggled(BUTTON_8);
	
	/*
	 * raw axis 4 is the throttle axis on the Logitech Extreme 3D Pro joystick
	 * we use the raw axis because the default mappings are incorrect
	 * the throttle, by default, returns values from -1.0 at the plus position to 1.0 at the minus position
	 * we first multiply by -6.0 to get values from -6.0 at the minus position to 6.0 at the plus position
	 * we then add 6.0 to get final voltage values from 0.0 (off) at minus position to 12.0 (full throttle) at the plus position
	 */
	double throttle_mag = this->RealJoy1->GetRawAxis(4) * -6.0 + 6.0;
	SmartDashboard::PutNumber("throttle", throttle_mag);

	float abs_x = abs(x), abs_y = abs(y), abs_twist = abs(twist);
	
	double outputVolts = throttle_mag;
	
	if (abs_twist > 0.4)
		// if the joystick is twisted, calculate final voltage with throttle * twist
		outputVolts *= abs_twist * 0.8;
	else if (!Joystick1->Pressed(BUTTON_3) && !Joystick1->Pressed(BUTTON_4))
		// otherwise, if we are not turning get the larger of the x and y values of the joystick posistion,
		// and multiply that by the throttle to get final voltage
		outputVolts *= max(abs_x, abs_y);
	// note: if we are turning, the rate of turning depends only on the throttle setting
	
	if (outputVolts < 1)
		outputVolts = 1.0;
	
	if (turbo)
		outputVolts *= 1.5;
	
	if (outputVolts > 12.0)
		outputVolts = 12.0;
	
	if (thumbStickX != 0 && thumbStickY != 0 && x == 0 && y == 0)
		outputVolts = 8.0;

	MechanumDrive->SetMaxVoltage(outputVolts);
	
	// determine direction
	MechanumWheels::DriveDir dir = MechanumWheels::Stop;

	if (abs_twist > 0.4)
	{
		if (twist > 0)
		{
			// rotate right
			dir = MechanumWheels::RotateLeft;
			if (outputVolts < 2.0)
				outputVolts = 2.0;
		}
		else
		{
			// rotate left
			dir = MechanumWheels::RotateRight;
			if (outputVolts < 2.0)
				outputVolts = 2.0;
		}
	}
	else if (Joystick1->Pressed(BUTTON_4))
	{
			// rotate right
			dir = MechanumWheels::RotateLeft;
	}
	else if (Joystick1->Pressed(BUTTON_3))
	{
		// rotate left
		dir = MechanumWheels::RotateRight;
	}
	else if (thumbStickX == 1 && thumbStickY == -1)
	{
		// forward right diagonal
		dir = MechanumWheels::FwdRight;
	}
	else if (thumbStickX == -1 && thumbStickY == -1)
	{
		// forward right diagonal
		dir = MechanumWheels::FwdLeft;
	}
	else if (thumbStickX == 1 && thumbStickY == 1)
	{
		// forward right diagonal
		dir = MechanumWheels::BackRight;
	}
	else if (thumbStickX == -1 && thumbStickY == 1)
	{
		// forward right diagonal
		dir = MechanumWheels::BackLeft;
	}
	else if (Vector3::GetMagnitude(x, y) < 0.25)
	{
		// stop
		dir = MechanumWheels::Stop;
	}
	else if (z >= 225 && z < 315)
	{
		// forward
		dir = MechanumWheels::Forward;
	}
	else if ((z >= 315 && z <= 360) || (z >= 0 && z < 45))
	{
		// right
		dir = MechanumWheels::Left;
	}
	else if(z >= 45 && z < 135)
	{
		// backward
		dir = MechanumWheels::Reverse;
	}
	else if (z >= 135 && z < 225)
	{
		// left
		dir = MechanumWheels::Right;
	}
	else
	{
		// stop
		dir = MechanumWheels::Stop;
	}
	
	// ----------------
	// catapult control
	// ----------------
	
	// CANNOT decock catapult
	// Note: catapult motor should be run backwards to cock the catapult (pull it back)
	
	// set cocked status of catapult based on current state of photo eye
	bool photoeye_tripped = !CatapultPhotoEye.Get();
	
	switch (state)
	{
	case Off_Not_Cocked:
		VicCatapult.Set(0);
		
		SmartDashboard::PutBoolean("catapult cocked", false);
		
		if (Joystick2->Released(BUTTON_7))
			state = On_Not_Cocked_1;
		else if (Joystick2->Released(BUTTON_4))
			state = On_Not_Cocked_2;
		
		break;
		
	case On_Not_Cocked_1:
		VicCatapult.Set(-1);
		
		SmartDashboard::PutBoolean("catapult cocked", false);
		
		if (Joystick2->Released(BUTTON_7) && !photoeye_tripped)
			state = Off_Not_Cocked;
		else if (photoeye_tripped)
			state = Off_Cocked;
		
		break;
		
	case On_Not_Cocked_2:
		VicCatapult.Set(-1);
		
		SmartDashboard::PutBoolean("catapult cocked", false);
		
		if (photoeye_tripped)
			state = Extra_Cock_Time;
		else if (Joystick2->Released(BUTTON_4) && !photoeye_tripped)
			state = Off_Not_Cocked;
		
		break;
		
	case Extra_Cock_Time:
		VicCatapult.Set(-1);
		
		SmartDashboard::PutBoolean("catapult cocked", false);
		
		if (ExtraCockWait.Get() == 0)
			ExtraCockWait.Start();
		else if (ExtraCockWait.Get() >= 10)
		{
			ExtraCockWait.Stop();
			ExtraCockWait.Reset();
			state = Off_Cocked;
		}
		
		break;
		
	case Off_Cocked:
		VicCatapult.Set(0);
		
		SmartDashboard::PutBoolean("catapult cocked", true);
		
		if (Joystick2->Released(BUTTON_1))
			state = Firing;
		
		break;
		
	case Firing:
		VicCatapult.Set(-1);
		
		SmartDashboard::PutBoolean("catapult cocked", true);
		
		if (Joystick2->Released(BUTTON_1) && ShootWait.Get() < 20)
			state = Off_Cocked;
		else if (ShootWait.Get() == 0)
			ShootWait.Start();
		else if (ShootWait.Get() >= 20)
		{
			ShootWait.Stop();
			ShootWait.Reset();
			state = Off_Not_Cocked;
		}
		
		break;
		
	default:
		VicCatapult.Set(0);
		
		if (photoeye_tripped)
			state = Off_Cocked;
		else
			state = Off_Not_Cocked;
		
		break;
	}

	// --------------
	// roller control
	// --------------
	if (Joystick2->Pressed(BUTTON_12))
	{
		// tell the Jaguar to turn forward to pull ball in at 100% voltage forwards
		JagRoller.Set(1.0);
	}
	else if (Joystick2->Pressed(BUTTON_11))
	{
		// tell the Jaguar to turn backwards to push ball out at 100% voltage backwards
		JagRoller.Set(-1.0);
	}
	else
		JagRoller.Set(0);

	// ------------------
	// roller arm control
	// ------------------
	
	bool arm_up = !ArmUpSwitch.Get();
	bool arm_down = !ArmDownSwitch.Get();
	
	float armJoyY = -this->RealJoy2->GetAxis(Joystick::kYAxis);
	
	if (armJoyY < 0 && !arm_up)
	{
		// tell the Jaguar to lift arm at 50% voltage backwards
		JagRollerArm.Set(-0.5 * armJoyY);
	}
	else if (armJoyY > 0 && !arm_down)
	{
		// tell the Jaguar to put down arm at 50% voltage forwards
		JagRollerArm.Set(-0.5 * armJoyY);
	}
	else
		JagRollerArm.Set(0);
	
	// drive
	MechanumDrive->CheckComplete();
	MechanumWheels::DriveDir task = MechanumDrive->CurrentTask;
	
	if (task == MechanumWheels::ManualDrive ||
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
