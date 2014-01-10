#ifndef TEAM167_CANJAGUAR_H
#define TEAM167_CANJAGUAR_H

/**
 * Team167 Micro Jaguar Speed Control Mods
 */
class Team167CanJaguar : public CANJaguar
{
public:
	explicit Team167CanJaguar(UINT8 deviceNumber, CANJaguar::ControlMode controlMode = CANJaguar::kPercentVbus);

	// SpeedController interface
	
	// overloaded methods
	void Set(float value, UINT8 syncGroup=0);
	void SetForward();
	void SetReverse();
	double GetPosition();

	// Feeds the safety helper if it exists.
	void Feed(){if (m_safetyHelper) m_safetyHelper->Feed();};


	float rev;		// deals with backwards wired encoders 
	// Added by nick
};
#endif

