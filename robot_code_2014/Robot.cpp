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
	ShootWait(),
	
	CockedLights(1, Relay::kForwardOnly)
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
	
	catapult_state = Start;
	
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
	
	JagFL.ChangeControlMode(CANJaguar::kPosition);
	JagFR.ChangeControlMode(CANJaguar::kPosition);
	JagBL.ChangeControlMode(CANJaguar::kPosition);
	JagBR.ChangeControlMode(CANJaguar::kPosition);
	
	JagFR.DisableControl();
	JagFL.DisableControl();
	JagBR.DisableControl();
	JagBL.DisableControl();
	
	// reset timers
	ShootWait.Stop();
	ShootWait.Reset();
	ExtraCockWait.Stop();
	ExtraCockWait.Reset();
	
	// reset all button toggles
	this->Joystick1->DisableToggleAll();
	this->Joystick2->DisableToggleAll();
	
	// turn off lights
	CockedLights.Set(Relay::kOff);
	
	catapult_state = Start;
	
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
	
	this->MechanumDrive->SetMaxVoltage(9.0);
	
	// cock the catapult extra
	VicCatapult.Set(-1);
	Wait(0.55);
	VicCatapult.Set(0);
	
	// turn on the lights for extra awesome
	CockedLights.Set(Relay::kOn);

	// get the vision target (to check if it's hot)
	//autoTarget = getBestTarget(true, false);
	
	// todo remove
	//SmartDashboard::PutBoolean("target hot", autoTarget.hot);

	// drive for 1 second
	Timer AutoDriveTimer;
	
	while (AutoDriveTimer.Get() < 1)
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
			MechanumDrive->Update(MechanumWheels::Reverse);
		}
		else
		{
			// currently in a task
			MechanumDrive->FeedJags();
		}
	}
	
	MechanumDrive->StopJags();
	MechanumDrive->Disable();
	AutoDriveTimer.Stop();
	
	Timer AutoArmUpTimer;
	bool arm_up = !ArmUpSwitch.Get();
	
	while (AutoArmUpTimer.Get() < 0.3 && !arm_up)
	{
		if (AutoArmUpTimer.Get() == 0)
			AutoArmUpTimer.Start();
		
		arm_up = !ArmUpSwitch.Get();
		
		JagRollerArm.Set(0.3);
	}
	
	JagRollerArm.Set(0);
	AutoArmUpTimer.Stop();
	
	Wait(1);
	
	// put the arm down
	Timer AutoArmDownTimer;
	bool arm_down = !ArmDownSwitch.Get();
	
	while (AutoArmDownTimer.Get() < 0.5 && !arm_down)
	{
		if (AutoArmDownTimer.Get() == 0)
			AutoArmDownTimer.Start();
		
		arm_down = !ArmDownSwitch.Get();
		
		JagRollerArm.Set(-0.5);
	}
	
	JagRollerArm.Set(0);
	AutoArmDownTimer.Stop();
	
	// if the target is NOT hot, wait until it is before shooting
	/*
	if (!autoTarget.hot)
		Wait(3.0);
	*/
	
	// shoot
	VicCatapult.Set(-1);
	Wait(1.5);
	VicCatapult.Set(0);
	
	// turn off the lights
	CockedLights.Set(Relay::kOff);

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
	
	JagFL.ChangeControlMode(CANJaguar::kVoltage);
	JagFR.ChangeControlMode(CANJaguar::kVoltage);
	JagBL.ChangeControlMode(CANJaguar::kVoltage);
	JagBR.ChangeControlMode(CANJaguar::kVoltage);
	
	JagFL.EnableControl(0.0f);
	JagFR.EnableControl(0.0f);
	JagBL.EnableControl(0.0f);
	JagBR.EnableControl(0.0f);
	
	// reset timers
	ShootWait.Stop();
	ShootWait.Reset();
	ExtraCockWait.Stop();
	ExtraCockWait.Reset();
	
	// reset button toggles
	this->Joystick1->DisableToggleAll();
	this->Joystick2->DisableToggleAll();
	
	// turn off lights
	CockedLights.Set(Relay::kOff);
	
	catapult_state = Start;
	
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
	/*
	if (Joystick1->Pressed(BUTTON_11))
	{
		TargetReport target = getBestTarget(false, true);
		SmartDashboard::PutNumber("distance", target.distance);
		
		// todo "in shooting range" Dashboard variable?
	}
	else
		SmartDashboard::PutNumber("distance", -1.0);
	*/

	//-------------------------
	// drive logic (input side)
	//-------------------------
	
	// get joystick position
	float x = this->RealJoy1->GetAxis(Joystick::kXAxis);
	float y = -this->RealJoy1->GetAxis(Joystick::kYAxis);
	float z = Vector3::GetRotation(x, y);
	
	// raw axis 3 is the twist axis on the Logitech Extreme 3D Pro joystick
	// we use the raw axis because the default mappings are incorrect
	//float twist = this->RealJoy1->GetRawAxis(3);
	
	//float thumbStickX = this->RealJoy1->GetRawAxis(5);
	// note: -1 if pushed forward, 1 if pulled back
	//float thumbStickY = this->RealJoy1->GetRawAxis(6);
	
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

	float abs_x = abs(x), abs_y = abs(y);
	
	double outputVolts = throttle_mag;
	
	if (!Joystick1->Pressed(BUTTON_5) && !Joystick1->Pressed(BUTTON_6))
		// if we are not turning get the larger of the x and y values of the joystick posistion,
		// and multiply that by the throttle to get final voltage
		outputVolts *= max(abs_x, abs_y);
	// note: if we are turning, the rate of turning depends only on the throttle setting
	
	if (outputVolts < 1)
		outputVolts = 1.0;
	
	if (turbo)
		outputVolts *= 1.5;
	
	if (outputVolts > 12.0)
		outputVolts = 12.0;
	
	/*
	if (thumbStickX != 0 && thumbStickY != 0 && abs_x < 0.2 && abs_y < 0.2)
		outputVolts = 8.0;
	*/

	// feed the jaguars
	JagFL.Feed();
	JagFR.Feed();
	JagBL.Feed();
	JagBR.Feed();

	if (Joystick1->Pressed(BUTTON_5))
	{
		// rotate left
		JagFL.Set(outputVolts);
		JagBL.Set(outputVolts);
		JagFR.Set(-outputVolts);
		JagBR.Set(-outputVolts);
	}
	else if (Joystick1->Pressed(BUTTON_6))
	{
		// rotate right
		JagFL.Set(-outputVolts);
		JagBL.Set(-outputVolts);
		JagFR.Set(outputVolts);
		JagBR.Set(outputVolts);
	}
	/*
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
	*/
	else if (Vector3::GetMagnitude(x, y) < 0.25)
	{
		// stop
		JagFL.Set(0);
		JagFR.Set(0);
		JagBL.Set(0);
		JagBR.Set(0);
	}
	else if (z >= 225 && z < 315)
	{
		// forward
		JagFL.Set(outputVolts);
		JagBL.Set(outputVolts);
		JagFR.Set(outputVolts);
		JagBR.Set(outputVolts);
	}
	else if ((z >= 315 && z <= 360) || (z >= 0 && z < 45))
	{
		// right
		JagFR.Set(outputVolts);
		JagBR.Set(-outputVolts);
		JagFL.Set(-outputVolts);
		JagBL.Set(outputVolts);
	}
	else if(z >= 45 && z < 135)
	{
		// backward
		JagFL.Set(-outputVolts);
		JagBL.Set(-outputVolts);
		JagFR.Set(-outputVolts);
		JagBR.Set(-outputVolts);
	}
	else if (z >= 135 && z < 225)
	{
		// left
		JagFR.Set(-outputVolts);
		JagBR.Set(outputVolts);
		JagFL.Set(outputVolts);
		JagBL.Set(-outputVolts);
	}
	else
	{
		// stop
		JagFL.Set(0);
		JagFR.Set(0);
		JagBL.Set(0);
		JagBR.Set(0);
	}
	
	// ----------------
	// catapult control
	// ----------------
	
	// CANNOT decock catapult
	// Note: catapult motor should be run backwards to cock the catapult (pull it back)
	
	// set cocked status of catapult based on current catapult_state of photo eye
	bool photoeye_tripped = !CatapultPhotoEye.Get();
	
	switch (catapult_state)
	{
	case Off_Not_Cocked:
		VicCatapult.Set(0);
		
		CockedLights.Set(Relay::kOff);
		SmartDashboard::PutBoolean("catapult status", false);
		
		if (Joystick2->Released(BUTTON_7))
			catapult_state = On_Not_Cocked_1;
		else if (Joystick2->Released(BUTTON_1))
			catapult_state = On_Not_Cocked_2;
		
		break;
		
	case On_Not_Cocked_1:
		VicCatapult.Set(-1);
		
		CockedLights.Set(Relay::kOff);
		SmartDashboard::PutBoolean("catapult status", false);
		
		if (Joystick2->Released(BUTTON_7) && !photoeye_tripped)
			catapult_state = Off_Not_Cocked;
		else if (photoeye_tripped)
			catapult_state = Off_Cocked;
		
		break;
		
	case On_Not_Cocked_2:
		VicCatapult.Set(-1);
		
		CockedLights.Set(Relay::kOff);
		SmartDashboard::PutBoolean("catapult status", false);
		
		if (photoeye_tripped)
			catapult_state = Extra_Cock_Time;
		else if (Joystick2->Released(BUTTON_1) && !photoeye_tripped)
			catapult_state = Off_Not_Cocked;
		
		break;
		
	case Extra_Cock_Time:
		VicCatapult.Set(-1);
		
		CockedLights.Set(Relay::kOff);
		SmartDashboard::PutBoolean("catapult status", false);
		
		if (ExtraCockWait.Get() == 0)
			ExtraCockWait.Start();
		else if (ExtraCockWait.Get() >= 0.55)
		{
			ExtraCockWait.Stop();
			ExtraCockWait.Reset();
			catapult_state = Off_Cocked;
		}
		
		break;
		
	case Off_Cocked:
		VicCatapult.Set(0);
		
		CockedLights.Set(Relay::kOn);
		SmartDashboard::PutBoolean("catapult status", true);
		
		if (Joystick2->Released(BUTTON_1))
			catapult_state = Firing;
		
		break;
		
	case Firing:
		VicCatapult.Set(-1);
		
		CockedLights.Set(Relay::kOn);
		SmartDashboard::PutBoolean("catapult status", true);
		
		if (Joystick2->Released(BUTTON_1) && ShootWait.Get() < 1.0)
			catapult_state = Off_Cocked;
		else if (ShootWait.Get() == 0)
			ShootWait.Start();
		else if (ShootWait.Get() >= 1.0)
		{
			ShootWait.Stop();
			ShootWait.Reset();
			catapult_state = Off_Not_Cocked;
		}
		
		break;
		
	default:
		VicCatapult.Set(0);
		
		CockedLights.Set(Relay::kOff);
		
		if (photoeye_tripped)
			catapult_state = Off_Cocked;
		else
			catapult_state = Off_Not_Cocked;
		
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

	return;
}
