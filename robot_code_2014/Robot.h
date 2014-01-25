#ifndef _ROBOT_H_
#define _ROBOT_H_
#include <cmath>

class Robot:public IterativeRobot
{
public:
	
	//tasks robot can do
	typedef enum {TaskFinished=12, Pickup=13, Shoot=14, Slowdown=15} Task;
	
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
	
	void ControlCamera();
	void SetPIDs();
	
	// autonomous routines
	void SetPower();
	void AutoStatus();

	void LineUpWithGoal();
	
protected:
	//driving jaguars
	Team167CanJaguar JagFL;
	Team167CanJaguar JagFR;
	Team167CanJaguar JagBL;
	Team167CanJaguar JagBR;
	Team167CanJaguar JagCatapult;
	Team167CanJaguar JagRoller;
	
	Joystick *RealJoy1;
	Joystick *RealJoy2;
	SimpleJoystick *Joystick1;
	SimpleJoystick *Joystick2;
	MechanumWheels *MechanumDrive;

	Servo *Tilt;

	bool camera_locked;

	bool catapult_cocked;
	bool catapult_decocked;	
};

#endif
