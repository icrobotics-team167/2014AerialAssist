#ifndef _ROBOT_H_
#define _ROBOT_H_
#include <cmath>
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"

//Camera constants used for distance calculation
// todo recalibrate view angle for Axix M1013
#define Y_IMAGE_RES 240		// Y Image resolution in pixels, should be 120, 240 or 480
#define VIEW_ANGLE 50.144	// Axis M1013

//Score limits used for target identification
#define RECTANGULARITY_LIMIT 40
#define ASPECT_RATIO_LIMIT 55

//Score limits used for hot target determination
#define TAPE_WIDTH_LIMIT 50
#define VERTICAL_SCORE_LIMIT 50
#define LR_SCORE_LIMIT 50

//Minimum area of particles to be considered
#define AREA_MINIMUM 150

//Maximum number of particles to process
#define MAX_PARTICLES 8

class Robot : public IterativeRobot
{
public:
	
	// tasks robot can do
	typedef enum {TaskFinished=12, Pickup=13, Shoot=14, Slowdown=15} Task;
	
	// states
	typedef enum { Off_Not_Cocked = 0, On_Not_Cocked_1 = 1, On_Not_Cocked_2 = 2,
					Extra_Cock_Time = 3, Off_Cocked = 4, Firing = 5, Default = 99 } CatapultState;
	
	Robot();
	~Robot();
	
	void AutonomousContinuous();
	void AutonomousInit();
	void AutonomousPeriodic();
	void DisabledContinuous();
	void DisabledInit();
	void DisabledPeriodic();
	void RobotInit();
	void TeleopContinuous();
	void TeleopInit();
	void TeleopPeriodic();
	
	void SetPIDs();
	
	// Vision processing methods and structures
	
	// Structure to represent the scores for the various tests used for target identification
	struct Scores
	{
		double rectangularity;
		double aspectRatioVertical;
		double aspectRatioHorizontal;
	};
	
	struct TargetReport
	{
		int verticalIndex;
		int horizontalIndex;
		bool hot;
		double totalScore;
		double leftScore;
		double rightScore;
		double tapeWidthScore;
		double verticalScore;
		double distance;
	};
	
	double computeDistance(BinaryImage *image, ParticleAnalysisReport *report);
	double scoreAspectRatio(BinaryImage *image, ParticleAnalysisReport *report, bool vertical);
	bool scoreCompare(Scores scores, bool vertical);
	double scoreRectangularity(ParticleAnalysisReport *report);
	double ratioToScore(double ratio);
	bool hotOrNot(TargetReport target);
	void LineUpWithGoal();
	TargetReport getBestTarget(bool setHot = false, bool setDistance = false);
	
protected:
	// driving jaguars
	Team167CanJaguar JagFL;
	Team167CanJaguar JagFR;
	Team167CanJaguar JagBL;
	Team167CanJaguar JagBR;
	
	// motor controllers for other components
	Victor VicCatapult;
	Team167CanJaguar JagRoller;
	Team167CanJaguar JagRollerArm;
	
	Joystick *RealJoy1;
	Joystick *RealJoy2;
	SimpleJoystick *Joystick1;
	SimpleJoystick *Joystick2;
	
	MechanumWheels *MechanumDrive;
	
	DigitalInput ArmDownSwitch;
	DigitalInput ArmUpSwitch;

	DigitalInput CatapultPhotoEye;
	
	Timer ExtraCockWait;
	Timer ShootWait;
	
	TargetReport autoTarget;
	
	CatapultState catapult_state;
	DigitalOutput CockedLights;
};

#endif
