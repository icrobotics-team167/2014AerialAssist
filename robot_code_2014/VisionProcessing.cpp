#include "_Robot.h"	

//Camera constants used for distance calculation
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

/**
 * Computes the estimated distance to a target using the height of the particle in the image. For more information and graphics
 * showing the math behind this approach see the Vision Processing section of the ScreenStepsLive documentation.
 * 
 * @param image The image to use for measuring the particle estimated rectangle
 * @param report The Particle Analysis Report for the particle
 * @return The estimated distance to the target in feet.
 */
double Robot::computeDistance(BinaryImage *image, ParticleAnalysisReport *report) {
	double rectLong, height;
	int targetHeight;
	
	imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
	//using the smaller of the estimated rectangle long side and the bounding rectangle height results in better performance
	//on skewed rectangles
	height = min((double) report->boundingRect.height, rectLong);
	targetHeight = 32;
	
	return Y_IMAGE_RES * targetHeight / (height * 12 * 2 * tan(VIEW_ANGLE * PI / (180 * 2)));
}

/**
 * Computes a score (0-100) comparing the aspect ratio to the ideal aspect ratio for the target. This method uses
 * the equivalent rectangle sides to determine aspect ratio as it performs better as the target gets skewed by moving
 * to the left or right. The equivalent rectangle is the rectangle with sides x and y where particle area= x*y
 * and particle perimeter= 2x+2y
 * 
 * @param image The image containing the particle to score, needed to perform additional measurements
 * @param report The Particle Analysis Report for the particle, used for the width, height, and particle number
 * @param outer	Indicates whether the particle aspect ratio should be compared to the ratio for the inner target or the outer
 * @return The aspect ratio score (0-100)
 */
double Robot::scoreAspectRatio(BinaryImage *image, ParticleAnalysisReport *report, bool vertical){
	double rectLong, rectShort, idealAspectRatio, aspectRatio;
	idealAspectRatio = vertical ? (4.0/32) : (23.5/4);	//Vertical reflector 4" wide x 32" tall, horizontal 23.5" wide x 4" tall
	
	imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
	imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
	
	//Divide width by height to measure aspect ratio
	if(report->boundingRect.width > report->boundingRect.height){
		//particle is wider than it is tall, divide long by short
		aspectRatio = ratioToScore(((rectLong/rectShort)/idealAspectRatio));
	} else {
		//particle is taller than it is wide, divide short by long
		aspectRatio = ratioToScore(((rectShort/rectLong)/idealAspectRatio));
	}
	return aspectRatio;		//force to be in range 0-100
}

/**
 * Compares scores to defined limits and returns true if the particle appears to be a target
 * 
 * @param scores The structure containing the scores to compare
 * @param vertical True if the particle should be treated as a vertical target, false to treat it as a horizontal target
 * 
 * @return True if the particle meets all limits, false otherwise
 */
bool Robot::scoreCompare(Scores scores, bool vertical){
	bool isTarget = true;

	isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
	if(vertical){
		isTarget &= scores.aspectRatioVertical > ASPECT_RATIO_LIMIT;
	} else {
		isTarget &= scores.aspectRatioHorizontal > ASPECT_RATIO_LIMIT;
	}

	return isTarget;
}

/**
 * Computes a score (0-100) estimating how rectangular the particle is by comparing the area of the particle
 * to the area of the bounding box surrounding it. A perfect rectangle would cover the entire bounding box.
 * 
 * @param report The Particle Analysis Report for the particle to score
 * @return The rectangularity score (0-100)
 */
double Robot::scoreRectangularity(ParticleAnalysisReport *report){
	if(report->boundingRect.width*report->boundingRect.height !=0){
		return 100*report->particleArea/(report->boundingRect.width*report->boundingRect.height);
	} else {
		return 0;
	}	
}	

/**
 * Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
 * linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside the range 0-2
 */
double Robot::ratioToScore(double ratio)
{
	return (max(0.0, min(100*(1-fabs(1-ratio)), 100.0)));
}

/**
 * Takes in a report on a target and compares the scores to the defined score limits to evaluate
 * if the target is a hot target or not.
 * 
 * Returns True if the target is hot. False if it is not.
 */
bool Robot::hotOrNot(TargetReport target)
{
	bool isHot = true;
	
	isHot &= target.tapeWidthScore >= TAPE_WIDTH_LIMIT;
	isHot &= target.verticalScore >= VERTICAL_SCORE_LIMIT;
	isHot &= (target.leftScore > LR_SCORE_LIMIT) | (target.rightScore > LR_SCORE_LIMIT);
	
	return isHot;
}

Robot::TargetReport Robot::getBestTarget(bool setHot, bool setDistance)
{
	TargetReport target;
	Scores *scores;
	int verticalTargets[MAX_PARTICLES];
	int horizontalTargets[MAX_PARTICLES];
	int verticalTargetCount, horizontalTargetCount;
	
	// Threshold threshold(105, 137, 230, 255, 133, 183); (FIRST-provided values)
	// HSV threshold criteria, ranges are in that order ie. Hue is 60-100
	// todo recalibrate threshold at competition
	Threshold threshold(55, 138, 248, 255, 34, 159);
	
	// Particle filter criteria, used to filter out small particles
	ParticleFilterCriteria2 criteria[] =
	{
		{IMAQ_MT_AREA, AREA_MINIMUM, 65535, false, false}
	};												
	AxisCamera &camera = AxisCamera::GetInstance();
	
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
				horizontalTargets[horizontalTargetCount++] = i; //Add particle to target array and increment count
			else if (scoreCompare(scores[i], true))
				verticalTargets[verticalTargetCount++] = i;  //Add particle to target array and increment count
		}

		//Zero out scores and set verticalIndex to first target in case there are no horizontal targets
		target.totalScore = target.leftScore = target.rightScore = target.tapeWidthScore = target.verticalScore = 0;
		target.verticalIndex = verticalTargets[0];
		for (int i = 0; i < verticalTargetCount; i++)
		{	
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
		}
		
		// determine if target is hot
		if (setHot)
			target.hot = hotOrNot(target);
		else
			target.hot = false;
		
		// calculate distance to target
		if (setDistance && verticalTargetCount > 0)
		{
			ParticleAnalysisReport *distanceReport = &(reports->at(target.verticalIndex));
			target.distance = computeDistance(filteredImage, distanceReport);
		}
		// default distance value if no target is found or we don't want to know distance
		else
			target.distance = 0.0;

		// be sure to delete images after using them
		delete filteredImage;
		delete thresholdImage;
		delete image;
		
		//delete allocated reports and Scores objects also
		delete scores;
		delete reports;
	}
	
	return target;
}
