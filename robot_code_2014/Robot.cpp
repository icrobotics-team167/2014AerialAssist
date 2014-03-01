#include "_Robot.h"

// can only be called once
START_ROBOT_CLASS(Robot);

// Robot class constructor
Robot::Robot() :
	VicFL(2),
	VicFR(3),
	VicBL(4),
	VicBR(5),
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
	
	// intialize drive class
	VictorWheels = new VictorDrive(&VicFL, &VicFR, &VicBL, &VicBR);
	
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
	delete VictorWheels;
	
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
	
	VictorWheels->Disable();
	
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
	VictorWheels->SetVoltagePercent(0.8);
	
	while (AutoDriveTimer.Get() < 1)
	{
		if (AutoDriveTimer.Get() == 0)
			AutoDriveTimer.Start();
		
		// drive forward
		VictorWheels->Forward();
	}
	
	VictorWheels->Stop();
	VictorWheels->Disable();
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
	
	// Set the throttle
	bool turbo = Joystick1->Toggled(BUTTON_8);
	
	/*
	 * raw axis 4 is the throttle axis on the Logitech Extreme 3D Pro joystick
	 * we use the raw axis because the default mappings are incorrect
	 * the throttle, by default, returns values from -1.0 at the plus position to 1.0 at the minus position
	 * we first multiply by -1.0 to get values from -1.0 at the minus position to 1.0 at the plus position
	 * we then add 1.0 and divide by 2 to get final voltage percentages from 0.0 (off) at minus position
	 * to 1.0 (full throttle) at the plus position
	 */
	double throttle_mag = (this->RealJoy1->GetRawAxis(4) * -1.0 + 1.0) / 2.0;
	SmartDashboard::PutNumber("throttle", throttle_mag);

	float abs_x = abs(x), abs_y = abs(y);
	
	double voltagePercent = throttle_mag;
	
	if (!Joystick1->Pressed(BUTTON_5) && !Joystick1->Pressed(BUTTON_6))
		// if we are not turning get the larger of the x and y values of the joystick posistion,
		// and multiply that by the throttle to get final voltage
		voltagePercent *= max(abs_x, abs_y);
	// note: if we are turning, the rate of turning depends only on the throttle setting
	
	if (voltagePercent < 1)
		voltagePercent = 0.1;
	
	if (turbo)
		voltagePercent *= 1.5;
	
	if (voltagePercent > 1.0)
		voltagePercent = 1.0;
	
	VictorWheels->SetVoltagePercent(voltagePercent);

	if (Joystick1->Pressed(BUTTON_5))
	{
		// rotate left
		VictorWheels->RotateLeft();
	}
	else if (Joystick1->Pressed(BUTTON_6))
	{
		// rotate right
		VictorWheels->RotateRight();
	}
	else if (Vector3::GetMagnitude(x, y) < 0.25)
	{
		// stop
		VictorWheels->Stop();
	}
	else if (z >= 225 && z < 315)
	{
		// forward
		VictorWheels->Forward();
	}
	else if ((z >= 315 && z <= 360) || (z >= 0 && z < 45))
	{
		// right
		VictorWheels->Right();
	}
	else if(z >= 45 && z < 135)
	{
		// backwards
		VictorWheels->Reverse();
	}
	else if (z >= 135 && z < 225)
	{
		// left
		VictorWheels->Left();
	}
	else
	{
		// stop
		VictorWheels->Stop();
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
