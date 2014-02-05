#include "_Robot.h"

// can only be called once
START_ROBOT_CLASS(Robot);

// Robot class constructor
Robot::Robot() :
	JagFL(22, CANJaguar::kPosition),
	JagFR(21, CANJaguar::kPosition),
	JagBL(23, CANJaguar::kPosition),
	JagBR(24, CANJaguar::kPosition)
	//JagCatapult(26, CANJaguar::kPercentVbus),
	//JagRoller(25, CANJaguar::kPercentVbus),
	//JagRollerArm(27, CANJaguar::kPercentVbus)
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

	// initialize camera
	Tilt = new Servo(1);
	camera_locked = true;
	
	// TODO initialize catapult booleans with switches
	
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
	
	Scores *scores;
	int verticalTargets[MAX_PARTICLES];
	int horizontalTargets[MAX_PARTICLES];
	int verticalTargetCount, horizontalTargetCount;
	
	//Threshold threshold(105, 137, 230, 255, 133, 183);	//HSV threshold criteria, ranges are in that order ie. Hue is 60-100
	// todo find correct threshold values
	Threshold threshold(88, 146, 125, 255, 36, 102);
	
	ParticleFilterCriteria2 criteria[] = {
			{IMAQ_MT_AREA, AREA_MINIMUM, 65535, false, false}
	};												//Particle filter criteria, used to filter out small particles
	AxisCamera &camera = AxisCamera::GetInstance();	//To use the Axis camera uncomment this line
	
	/**
	 * Do the image capture with the camera and apply the algorithm described above. This
	 * sample will either get images from the camera or from an image file stored in the top
	 * level directory in the flash memory on the cRIO. The file name in this case is "testImage.jpg"
	 */
	ColorImage *image;
	//image = new RGBImage("/testImage.jpg");		// get the sample image from the cRIO flash

	image = camera.GetImage();				//To get the images from the camera comment the line above and uncomment this one
	//image->Write("/image.bmp");
	BinaryImage *thresholdImage = image->ThresholdHSV(threshold);	// get just the green target pixels
	//thresholdImage->Write("/threshold.bmp");
	BinaryImage *filteredImage = thresholdImage->ParticleFilter(criteria, 1);	//Remove small particles
	//filteredImage->Write("Filtered.bmp");

	vector<ParticleAnalysisReport> *reports = filteredImage->GetOrderedParticleAnalysisReports();  //get a particle analysis report for each particle

	verticalTargetCount = horizontalTargetCount = 0;
	//Iterate through each particle, scoring it and determining whether it is a target or not
	
	// todo remove
	SmartDashboard::PutNumber("reports->size", (double) reports->size());
	
	if(reports->size() > 0)
	{
		scores = new Scores[reports->size()];
		for (unsigned int i = 0; i < MAX_PARTICLES && i < reports->size(); i++) {
			ParticleAnalysisReport *report = &(reports->at(i));
			
			//Score each particle on rectangularity and aspect ratio
			scores[i].rectangularity = scoreRectangularity(report);
			scores[i].aspectRatioVertical = scoreAspectRatio(filteredImage, report, true);
			scores[i].aspectRatioHorizontal = scoreAspectRatio(filteredImage, report, false);			
			
			//Check if the particle is a horizontal target, if not, check if it's a vertical target
			if(scoreCompare(scores[i], false))
			{
				printf("particle: %d  is a Horizontal Target centerX: %d  centerY: %d \n", i, report->center_mass_x, report->center_mass_y);
				horizontalTargets[horizontalTargetCount++] = i; //Add particle to target array and increment count
			} else if (scoreCompare(scores[i], true)) {
				printf("particle: %d  is a Vertical Target centerX: %d  centerY: %d \n", i, report->center_mass_x, report->center_mass_y);
				verticalTargets[verticalTargetCount++] = i;  //Add particle to target array and increment count
			} else {
				printf("particle: %d  is not a Target centerX: %d  centerY: %d \n", i, report->center_mass_x, report->center_mass_y);
			}
			printf("Scores rect: %f  ARvert: %f \n", scores[i].rectangularity, scores[i].aspectRatioVertical);
			printf("ARhoriz: %f  \n", scores[i].aspectRatioHorizontal);
		}

		//Zero out scores and set verticalIndex to first target in case there are no horizontal targets
		target.totalScore = target.leftScore = target.rightScore = target.tapeWidthScore = target.verticalScore = 0;
		target.verticalIndex = verticalTargets[0];
		for (int i = 0; i < verticalTargetCount; i++)
		{
			// todo remove
			SmartDashboard::PutString("doing vision processing", "processing");
			
			ParticleAnalysisReport *verticalReport = &(reports->at(verticalTargets[i]));
			for (int j = 0; j < horizontalTargetCount; j++)
			{
				ParticleAnalysisReport *horizontalReport = &(reports->at(horizontalTargets[j]));
				double horizWidth, horizHeight, vertWidth, leftScore, rightScore, tapeWidthScore, verticalScore, total;

				//Measure equivalent rectangle sides for use in score calculation
				imaqMeasureParticle(filteredImage->GetImaqImage(), horizontalReport->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &horizWidth);
				imaqMeasureParticle(filteredImage->GetImaqImage(), verticalReport->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &vertWidth);
				imaqMeasureParticle(filteredImage->GetImaqImage(), horizontalReport->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &horizHeight);
				
				//Determine if the horizontal target is in the expected location to the left of the vertical target
				leftScore = ratioToScore(1.2*(verticalReport->boundingRect.left - horizontalReport->center_mass_x)/horizWidth);
				//Determine if the horizontal target is in the expected location to the right of the  vertical target
				rightScore = ratioToScore(1.2*(horizontalReport->center_mass_x - verticalReport->boundingRect.left - verticalReport->boundingRect.width)/horizWidth);
				//Determine if the width of the tape on the two targets appears to be the same
				tapeWidthScore = ratioToScore(vertWidth/horizHeight);
				//Determine if the vertical location of the horizontal target appears to be correct
				verticalScore = ratioToScore(1-(verticalReport->boundingRect.top - horizontalReport->center_mass_y)/(4*horizHeight));
				total = leftScore > rightScore ? leftScore:rightScore;
				total += tapeWidthScore + verticalScore;
				
				//If the target is the best detected so far store the information about it
				if(total > target.totalScore)
				{
					target.horizontalIndex = horizontalTargets[j];
					target.verticalIndex = verticalTargets[i];
					target.totalScore = total;
					target.leftScore = leftScore;
					target.rightScore = rightScore;
					target.tapeWidthScore = tapeWidthScore;
					target.verticalScore = verticalScore;
				}
			}
			//Determine if the best target is a Hot target
			target.Hot = hotOrNot(target);
		}
		
		if(verticalTargetCount > 0)
		{
			//Information about the target is contained in the "target" structure
			//To get measurement information such as sizes or locations use the
			//horizontal or vertical index to get the particle report as shown below
			ParticleAnalysisReport *distanceReport = &(reports->at(target.verticalIndex));
			double distance = computeDistance(filteredImage, distanceReport);
			if(target.Hot)
			{
				printf("Hot target located \n");
				printf("Distance: %f \n", distance);
			} else {
				printf("No hot target present \n");
				printf("Distance: %f \n", distance);
			}
		}

		// be sure to delete images after using them
		delete filteredImage;
		delete thresholdImage;
		delete image;
		
		//delete allocated reports and Scores objects also
		delete scores;
		delete reports;
	}
	
	SmartDashboard::PutBoolean("target hot", target.Hot);
	// if the best target found is NOT hot, wait until it is before proceeding
	// with autonomous mode
	if (!target.Hot)
		Wait(5.0);
	
	// todo catapult will start decocked
	// todo rotate to face hot goal
	// shoot
	//JagCatapult.Set(1);
	//Wait(0.5);
	//JagCatapult.Set(0);
	//Wait(0.5);
	
	// drive forward into our zone
	MechanumDrive->SetMaxVoltage(4.0);
	MechanumDrive->Move2Loc(MechanumWheels::Forward, 4.0);

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
 */
void Robot::TeleopPeriodic()
{
	this->GetWatchdog().Feed();
	this->Joystick1->Update();
	this->Joystick2->Update();

	//-------------------------
	// drive logic (input side)
	//-------------------------
	
	// get joystick position
	float x = this->RealJoy1->GetAxis(Joystick::kXAxis);
	float y = this->RealJoy1->GetAxis(Joystick::kYAxis);
	float z = Vector3::GetRotation(x, y);
	
	// raw axis 3 is the twist axis on the Logitech Extreme 3D Pro joystick
	// we use the raw axis because the default mappings are incorrect
	float twist = this->RealJoy1->GetRawAxis(3);
	
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
		outputVolts *= abs_twist;
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
			
	MechanumDrive->SetMaxVoltage(outputVolts);

	// determine direction
	MechanumWheels::DriveDir dir = MechanumWheels::Stop;

	if (abs_twist > 0.4)
	{
		if (twist > 0)
		{
			// rotate right
			dir = MechanumWheels::RotateRight;
			if (outputVolts < 2.0)
				outputVolts = 2.0;
		}
		else
		{
			// rotate left
			dir = MechanumWheels::RotateLeft;
			if (outputVolts < 2.0)
				outputVolts = 2.0;
		}
	}
	else if (Joystick1->Pressed(BUTTON_4))
	{
			// rotate right
			dir = MechanumWheels::RotateRight;
	}
	else if (Joystick1->Pressed(BUTTON_3))
	{
		// rotate left
		dir = MechanumWheels::RotateLeft;
	}
	else if (Vector3::GetMagnitude(x, y) < 0.25)
	{
		// stop
		dir = MechanumWheels::Stop;
	}
	else if (z >= 247.5 && z < 292.5)
	{
		// forward
		dir = MechanumWheels::Forward;
	}
	else if (z >= 292.5 && z < 337.5)
	{
		// forward right diagonal
		dir = MechanumWheels::FwdRight;
	}
	else if ((z >= 337.5 && z <= 360) || (z >= 0 && z < 22.5))
	{
		// right
		dir = MechanumWheels::Right;
	}
	else if (z >= 22.5 && z < 67.5)
	{
		// backward right diagonal
		dir = MechanumWheels::BackRight;
	}
	else if(z >= 67.5 && z < 112.5)
	{
		// backward
		dir = MechanumWheels::Reverse;
	}
	else if (z >= 112.5 && z < 157.5)
	{
		// backward left diagonal
		dir = MechanumWheels::BackLeft;
	}
	else if (z >= 157.5 && z < 202.5)
	{
		// left
		dir = MechanumWheels::Left;
	}
	else if (z >= 202.5 && z < 247.5)
	{
		// forward left diagonal
		dir = MechanumWheels::FwdLeft;
	}
	else
	{
		// stop
		dir = MechanumWheels::Stop;
	}

	if (!Joystick1->Toggled(BUTTON_7))
		MechanumDrive->Update(dir);

	// camera control
	if (Joystick2->Pressed(BUTTON_10))
		camera_locked = !camera_locked;
	if (!camera_locked)
		ControlCamera();

	
	// todo probably move roller arm and roller at half speed
	
	// ----------------
	// catapult control
	// ----------------
	/*
	if (Joystick2->Pressed(BUTTON_3) && !catapult_cocked)
	{
		// tell the Jaguar to run the catapult cocking motor at 100% voltage forwards
		JagCatapult.Set(1);
	}
	else if (Joystick2->Pressed(BUTTON_2) && !catapult_decocked)
	{
		// tell the Jaguar to run the catapult cocking motor at 100% voltage backwards
		JagCatapult.Set(-1);
	}
	else if (Joystick2->Pressed(BUTTON_1) && catapult_cocked)
	{
		// shoot
		JagCatapult.Set(1);
		Wait(0.5);
		JagCatapult.Set(0);
	}
	else
		JagCatapult.Set(0);

	// --------------
	// roller control
	// --------------
	if (Joystick2->Pressed(BUTTON_4))
	{
		// tell the Jaguar to turn forward to pull ball in at 100% voltage forwards
		JagRoller.Set(1);
	}
	else if (Joystick2->Pressed(BUTTON_5))
	{
		// tell the Jaguar to turn backwards to push ball out at 100% voltage backwards
		JagRoller.Set(-1);
	}
	else
		JagRoller.Set(0);

	// ------------------
	// roller arm control
	// ------------------
	if (Joystick2->Pressed(BUTTON_6))
	{
		// tell the Jaguar to lift arm at 100% voltage backwards
		JagRollerArm.Set(-1);
	}
	else if (Joystick2->Pressed(BUTTON_7))
	{
		// tell the Jaguar to put down arm at 100% voltage forwards
		JagRollerArm.Set(1);
	}
	else
		JagRollerArm.Set(0);
	*/
	
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

//-------------------------------------------------------------------------
// Control Camera
//-------------------------------------------------------------------------
void Robot::ControlCamera()
{	
	// Get adjusted axis value from joystick
	int y_val = (int) (floor(RealJoy2->GetAxis(Joystick::kYAxis) * 4.0));
	
	// Check to make sure axis value is sufficiently large
	if(abs(y_val) < 2)
		y_val = 0;
	
	// Set position value for Tilt servo
	int tilt_pos = (int) Tilt->GetRaw();
	tilt_pos += y_val;
	
	// Make sure position value doesn't exceed max positions of camera
	tilt_pos = max(0, tilt_pos);
	tilt_pos = min(tilt_pos, 255);
	
	// Make sure there is a change in position before resetting it
	if (y_val != 0)
		Tilt->SetRaw((UINT8) tilt_pos);
}
