#ifndef _ROBOT_H_
#define _ROBOT_H_
#include <cmath>
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"

class Robot : public IterativeRobot
{
public:
	
	// tasks robot can do
	typedef enum {TaskFinished=12, Pickup=13, Shoot=14, Slowdown=15} Task;
	
	// states
	typedef enum { Off_Not_Cocked = 0, On_Not_Cocked_1 = 1, On_Not_Cocked_2 = 2,
					Extra_Cock_Time = 3, Off_Cocked = 4, Firing = 5, Start = 99 } CatapultState;
	
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
	// driving Victors
	Victor VicFL;
	Victor VicFR;
	Victor VicBL;
	Victor VicBR;
	
	// motor controllers for other components
	Victor VicCatapult;
	Team167CanJaguar JagRoller;
	Team167CanJaguar JagRollerArm;
	
	Joystick *RealJoy1;
	Joystick *RealJoy2;
	SimpleJoystick *Joystick1;
	SimpleJoystick *Joystick2;
	
	VictorDrive *VictorWheels;
	
	DigitalInput ArmDownSwitch;
	DigitalInput ArmUpSwitch;

	DigitalInput CatapultPhotoEye;
	
	Timer ExtraCockWait;
	Timer ShootWait;
	
	TargetReport autoTarget;
	
	CatapultState catapult_state;
	Relay CockedLights;
};

#endif
