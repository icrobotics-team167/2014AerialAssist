#define PI 3.1415926535897932384
#include "WPILib.h"
#include "Math.h"

class KinectJoy 
{

public:
	KinectJoy();
	~KinectJoy();
	
	// enums
	typedef enum Hand{kRightHand = 0,kLeftHand = 1};
	typedef enum Axis{kXAxis = 0,kYAxis = 1,kZAxis = 2};
	
	
	void Update();
	
	float GetX();
	float GetY();
	float GetZ();
	
public:
	Vector3 Up;
	Vector3 Side;
	Vector3 Forward;
	
	float X;
	float Y;
	float Z;
protected:
	Kinect *kinect;

	void SetAngles(
			Skeleton::Joint first, 
			Skeleton::Joint middle, 
			Skeleton::Joint end);
	
	// Assumes that values are packed into X,Y of the Vector3 Class
	float GetAngleBetweenVectors(
			Vector3 newCoordFirst, 
			Vector3 newCoordEnd);
};
