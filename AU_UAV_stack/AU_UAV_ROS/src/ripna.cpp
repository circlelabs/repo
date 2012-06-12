/*
RIPNA.cpp

This is the implementation of RIPNA.h. 

*/


#include <math.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <tuple>

#include "AU_UAV_ROS/ripna.h"
#include "AU_UAV_ROS/standardFuncs.h"		// for PI, EARTH_RADIUS, MPS_SPEED
#include "AU_UAV_ROS/SimulatedPlane.h"		// for MAXIMUM_TURNING_ANGLE

#define CHECK_ZONE 4.0*MPS_SPEED
#define DANGER_ZEM 2.0*MPS_SPEED
#define MINIMUM_TURNING_RADIUS 28.64058013	
#define DESIRED_RADIUS 2.0*MPS_SPEED
#define LAMBDA 1.0

AU_UAV_ROS::waypoint calculateWaypoint(PlaneObject &plane1, std::map<int, PlaneObject> &planes){
	/* Find plane to avoid*/	
	std::tuple<int, double> greatestThreat = findGreatestThreat(plane1, planes);
	
	/* Unpack plane to avoid*/	
	int threatID = std::get<0>(greatestThreat);
	double threatZEM = std:get<1>(greatestThreat);
	bool turnRight = shouldTurnRight(plane1, planes[threatID]);
	double turningRadius = calculateTurningRadius(ZEM);
}

	
/* Function that returns the ID of the most dangerous neighboring plane and its ZEM */
std::tuple<int, double> findGreatestThreat(PlaneObject &plane1, std::map<int, PlaneObject> &planes){
	/* Set preliminary plane to avoid as non-existent*/
	int planeToAvoid = -1;
	/* Declare second plane and ID variable */
	PlaneObject plane2;
	int ID;
	/* Set the preliminary time-to-go to infinity*/
	double minimumTimeToGo = numeric_limits<double>::infinity();
	double mostDangerousZEM;
	/* Make a position vector representation of the current plane*/
	double magnitude2, direction2;	
	double magnitude = sqrt(pow(plane1.currentLoc.latitude,2)+pow(plane1.currentLoc.longitude,2));
	double direction = 2*PI+atan(plane1.currentLoc.latitude/plane1.currentLoc.longitude);	
	AU_UAV_ROS::mathVector p1(magnitude,direction);

	/* Make a heading vector representation of the current plane*/
	AU_UAV_ROS::mathVector d1(1.0,toCartesian(plane1.currentBearing));
		
	for ( iterator=planes.begin() ; iterator != planes.end(); iterator++ ){
		/* Unpacking plane to check*/		
		ID = (*iterator).first;
		plane2 = (*iterator).second;
		
		/* If it's not in the Check Zone, check the other plane*/
		if(plane1.findDistance(plane2) > CHECK_ZONE) continue;

		/* Making a position vector representation of plane2*/
		magnitude2 = sqrt(pow(plane2.currentLoc.latitude,2)+pow(plane2.currentLoc.longitude,2));
		direction2 = 2*PI+atan(plane2.currentLoc.latitude/plane2.currentLoc.longitude);	
		AU_UAV_ROS::mathVector p2(magnitude,direction);

		/* Make a heading vector representation of the current plane*/
		AU_UAV_ROS::mathVector d2(1.0,toCartesian(plane2.currentBearing));

		/* Compute Time To Go*/
		pDiff = p1-p2;
		dDiff = d1-d2;
		timeToGo = -1*pDiff.dotProduct(dDiff)/(MPS_SPEED*dDiff.dotProduct(dDiffs));

		/* Compute Zero Effort Miss*/
		zeroEffortMiss = sqrt(pDiff.dotProduct(pDiff) + 2*(MPS_SPEED*timeToGo)*pDiff.dotProduct(dDiff) + 
				pow(MPS_SPEED*timeToGo,2)*dDiff.dotproduct(dDiff));
		
		/* If the Zero Effort Miss is less than the minimum required separation, consider this plane as a candidate to avoid*/
		if(zeroEffortMiss <= DANGER_ZEM && timeToGo < minimumTimeToGo){
			planeToAvoid = ID;
			mostDangerousZEM = zeroEffortMiss;
		}
	}
	
	std::tuple<int, double> container(planeToAvoid, mostDangerousZEM);
	return container;

}

/* Returns true if the original plane (plane1) should turn right to avoid plane2, false if otherwise. Takes original plane and its greatest threat as parameters */
bool shouldTurnRight(PlaneObject &plane1, PlaneObject &plane2) {

	/* For checking whether the plane should turn right or left */
	double theta;
	double theta1;
	double theta2;
	bool turnRight;
	bool plane2OnRight;

	/* Calculate theta, theta1, and theta2 */
	theta = findAngle(plane1.currentLoc.latitude, plane1.currentLoc.longitude, plane2.currentLoc.latitude, plane2.currentLoc.longitude);
	theta1 = 90 - theta - plane1.currentBearing;
	theta2 = 90 + currentBearing + theta;

	theta = toCardinal(theta);
	plane2OnRight = theta - plane.currentBearing >= 0;
	
	if ((plane2OnRight && theta1 >= theta2) || (!plane2OnRight && theta1 <= theta2))
		turnRight = false;
	else if ((plane2OnRight && theta1 < theta2) || (!plane2OnRight && theta1 > theta2))
		turnRight = true;
	
	turnRight = (theta1 < theta2) ? true : false;
	return turnRight;		
}

/* */
double calculateTurningRadius(double ZEM){
	return MINIMUM_TURNING_RADIUS*exp(LAMBDA*ZEM/DESIRED_RADIUS);
}

/* create collision avoidance waypoint */


/* */
//TODO outline and simple cases for Dubins path
AU_UAV_ROS::waypoint takeDubinsPath(PlaneObject &plane1, AU_UAV_ROS::waypoint &wp){



}
