#include "_Team167Lib.h"

#define RAD2DEG 57.2957795
#define DEG2RAD 1/RAD2DEG
/*         y   -z
           |  /
           | / 
           |/
-x ---------------------- x
          /|
         / |
        /  |
      z    -y
*/
//-------------------------------------------------------------------------
// KinectJoy is a joystick-like class for using the Kinect.
//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
// Kinect Joy Constructor
//-------------------------------------------------------------------------
KinectJoy::KinectJoy() 
{
	kinect = Kinect::GetInstance();
	X = 0;
	Y = 0;
	Z = 0;
}


//-------------------------------------------------------------------------
// Kinect Joy Destructor
//-------------------------------------------------------------------------
KinectJoy::~KinectJoy() 
{
	kinect = NULL;
}

//---------------------------------------------------------------------
//GetAngleBetweenVectors - gets the angle between two vectors
//							Assumes that values are packed into X,Y of 
//							the Vector3 Class
//---------------------------------------------------------------------
float KinectJoy::GetAngleBetweenVectors(Vector3 newCoordFirst,Vector3 newCoordEnd) 
{
	newCoordEnd *= 1.0f/newCoordEnd.length();
	newCoordFirst *= 1.0f/newCoordFirst.length();
	float angle = newCoordEnd.dot(newCoordFirst);
	
	return angle;
}

//---------------------------------------------------------------------
//SetAngles - sets the X and Y values for the joystick from three
//				given joints
//---------------------------------------------------------------------
void KinectJoy::SetAngles(Skeleton::Joint first, Skeleton::Joint middle, Skeleton::Joint end) 
{
	Vector3 x1(
					end.x - middle.x,
					end.y - middle.y,
					end.z - middle.z);
	
	Vector3 x2(
							first.x - middle.x,
							first.y - middle.y,
							first.z - middle.z);
	
	Vector3 y1(
				first.x - end.x,
				first.y - end.y,
				first.z - end.z);
	
	
	float dsEnd=0, dfEnd=0,duEnd=0;
	float dsFirst=0, dfFirst=0,duFirst=0;
	
	dsEnd = x1.dot(Side);
	dfEnd = x1.dot(Forward);
	duEnd = x1.dot(Up);
	
	dsFirst = x2.dot(Side);
	dfFirst = x2.dot(Forward);
	duFirst = x2.dot(Up);

	
	Vector3 newCoordEnd(dsEnd, duEnd, 0);
	Vector3 newCoordFirst(dsFirst, duFirst, 0);
	
	float dy_up = y1.dot(Up);
	float dy_side = y1.dot(Side);
	
	float y_val = (float)atan2(dy_up,dy_side);
	
	X = GetAngleBetweenVectors(newCoordFirst, newCoordEnd);
	Y = y_val;
	
}




//---------------------------------------------------------------------
//Update - updates the internal axes based off of the skeleton
//---------------------------------------------------------------------
void KinectJoy::Update() 
{

	if(kinect == NULL)	kinect = Kinect::GetInstance();
	//It is possible that in future versions or uses of this class that
	//if there isn't a connected kinect, problems occur. However, this
	//does not currently happen.
	
	
	//---------------------------------------------------------------------
	//get coordinate system through person
	//---------------------------------------------------------------------
	Skeleton::Joint centerShoulder, spine, leftShoulder, rightShoulder;
	centerShoulder = kinect->GetSkeleton().GetShoulderCenter();
	spine = kinect->GetSkeleton().GetSpine();
	leftShoulder = kinect->GetSkeleton().GetShoulderLeft();
	rightShoulder = kinect->GetSkeleton().GetShoulderRight();
	
	
	//---------------------------------------------------------------------
	// Make sure the Joints are being tracked
	//---------------------------------------------------------------------
	Skeleton::Joint * joint_list[4];
	joint_list[0] = &centerShoulder;
	joint_list[1] = &centerShoulder;
	joint_list[2] = &centerShoulder;
	joint_list[3] = &centerShoulder;
	bool all_tracked = true;
	
	for(int i=0;i<4;i++)
	{
		if(joint_list[i]->trackingState == Skeleton::kNotTracked)
			all_tracked = false;
		
		// clear the reference b/c we don't need it anymore.
		joint_list[i] = NULL;
	}
	
	
	
	//---------------------------------------------------------------------
	// Don't update if the you lack the entire skeleton
	//---------------------------------------------------------------------
	if(!all_tracked)
		return;
	
	
	
	//---------------------------------------------------------------------
	// Calculate Side Forward and Up vectors
	//---------------------------------------------------------------------
	Vector3 tempSide(rightShoulder.x - leftShoulder.x,
						rightShoulder.y - leftShoulder.y,
						rightShoulder.z - leftShoulder.z);
	
	Vector3 tempUp(centerShoulder.x - spine.x,
						centerShoulder.y - spine.y,
						centerShoulder.z - spine.z);
	
	Up = tempUp / tempUp.length();
	Side = tempSide / tempSide.length();
	

	Vector3 tempForward = Up.cross(Side);
	

	Forward = tempForward / tempForward.length();

	Skeleton::Joint shoulder, elbow, wrist;
	shoulder = kinect->GetSkeleton().GetShoulderRight();
	elbow = kinect->GetSkeleton().GetElbowRight();
	wrist = kinect->GetSkeleton().GetWristRight();
	
	
	SetAngles(wrist, elbow, shoulder);
}

//-------------------------------------------------------------------------
//GetX - returns the X value of the joystick. In current implementation,
//			this is the forward/backward angle of the elbow
//-------------------------------------------------------------------------
float KinectJoy::GetX() 
{
	return X;
}

//-------------------------------------------------------------------------
//GetY - returns the Y value of the joystick. In current implementation,
//			this is the up/down angle of the shoulder.
//-------------------------------------------------------------------------
float KinectJoy::GetY()
{
	return Y;
}

//-------------------------------------------------------------------------
//GetZ - returns the Z value of the joystick. In current implementation,
//			this is not implemented. Will always return 0.
//-------------------------------------------------------------------------
float KinectJoy::GetZ()
{
	return Z;
}
