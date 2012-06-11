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
#define DESIRED_RADIUS 30.0
#define LAMBDA 1.0

AU_UAV_ROS::waypoint calculateWaypoint(PlaneObject &plane1, std::map<int, PlaneObject> &planes){
	/* Find plane to avoid*/	
	std::tuple<int, double, bool> greatestThreat = findGreatestThreat(plane1, planes);
	
	/* Unpack plane to avoid*/	
	int threatID = std::get<0>(greatestThreat);
	double threatZEM = std:get<1>(greatestThreat);
	bool turnRight = std::get<2>(greatestThreat);
		
	double turningRadius = calculateTurningRadius(ZEM);

}

	
/* */
std::tuple<int, double, bool> findGreatestThreat(PlaneObject &plane1, std::map<int, PlaneObject> &planes){
	/* Set preliminary plane to avoid as non-existent*/
	int planeToAvoid = -1;
	bool turnRight;
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
		int ID = (*iterator).first;
		PlaneObject plane2 = (*iterator).second;
		
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
	
	std::tuple<int, double, bool> container(planeToAvoid, mostDangerousZEM, turnRight);
	return container;

}

/* */
double calculateTurningRadius(double ZEM){
	return MINIMUM_TURNING_RADIUS*exp(LAMBDA*ZEM/DESIRED_RADIUS);
}


/* */
AU_UAV_ROS::waypoint takeDubinsPath(PlaneObject &plane1, AU_UAV_ROS::waypoint &wp){



}
