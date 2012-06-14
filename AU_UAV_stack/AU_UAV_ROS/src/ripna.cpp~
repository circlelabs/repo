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
#define DESIRED_SEPARATION 2.0*MPS_SPEED
#define LONG_TIME_STEP 1.2
#define LAMBDA 1.0

AU_UAV_ROS::waypoint findNewWaypoint(PlaneObject &plane1, std::map<int, PlaneObject> &planes){
	/* Find plane to avoid*/	
	std::tuple<int, double> greatestThreat = findGreatestThreat(plane1, planes);
	
	/* Unpack plane to avoid*/	
	int threatID = std::get<0>(greatestThreat);
	double threatZEM = std:get<1>(greatestThreat);
	
	/* If there is no plane to avoid, then take Dubin's path to the destination waypoint*/
	if ((threatID < 0) && (threatZEM < 0)) {	
		return takeDubinsPath(plane1);
	}
	
	/* If there is a plane to avoid, then figure out whcih direction it should turn*/
	bool turnRight = shouldTurnRight(plane1, planes[threatID]);

	/* Calculate turning radius to avoid collision*/
	double turningRadius = calculateTurningRadius(ZEM);

	/* Given turning radius and orientation of the plane, calculate next collision avoidance waypoint*/
	return calculateWaypoint(plane1, turningRadius, turnRight);
}

	
/* Function that returns the ID of the most dangerous neighboring plane and its ZEM */
std::tuple<int, double> findGreatestThreat(PlaneObject &plane1, std::map<int, PlaneObject> &planes){
	/* Set preliminary plane to avoid as non-existent and most dangerous ZEM as negative*/
	int planeToAvoid = -1;
	double mostDangerousZEM = -1;
	/* Declare second plane and ID variable */
	PlaneObject plane2;
	int ID;
	/* Set the preliminary time-to-go to infinity*/
	double minimumTimeToGo = numeric_limits<double>::infinity();
	double mostDangerousZEM;
	/* Make a position vector representation of the current plane*/
	double magnitude2, direction2;	
	double magnitude = sqrt(pow(plane1.getCurrentLoc().latitude,2)+pow(plane1.getCurrentLoc().longitude,2));
	double direction = 2*PI+atan(plane1.getCurrentLoc().latitude/plane1.getCurrentLoc().longitude);	
	AU_UAV_ROS::mathVector p1(magnitude,direction);

	/* Make a heading vector representation of the current plane*/
	AU_UAV_ROS::mathVector d1(1.0,toCartesian(plane1.getCurrentBearing()));
		
	for ( iterator=planes.begin() ; iterator != planes.end(); iterator++ ){
		/* Unpacking plane to check*/		
		ID = (*iterator).first;
		plane2 = (*iterator).second;
		
		/* If it's not in the Check Zone, check the other plane*/
		if(plane1.findDistance(plane2) > CHECK_ZONE || plane1.getID() == ID) continue;

		/* Making a position vector representation of plane2*/
		magnitude2 = sqrt(pow(plane2.getCurrentLoc().latitude,2)+pow(plane2.getCurrentLoc().longitude,2));
		direction2 = 2*PI+atan(plane2.getCurrentLoc().latitude/plane2.getCurrentLoc().longitude);	
		AU_UAV_ROS::mathVector p2(magnitude,direction);

		/* Make a heading vector representation of the current plane*/
		AU_UAV_ROS::mathVector d2(1.0,toCartesian(plane2.getCurrentBearing()));

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
	
	return std::tuple<int, double> (planeToAvoid, mostDangerousZEM);
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
	theta = findAngle(plane1.getCurrentLoc().latitude, plane1.getCurrentLoc().longitude, plane2.getCurrentLoc().latitude, plane2.getCurrentLoc().longitude);
	theta1 = 90 - theta - plane1.getCurrentBearing();
	theta2 = 90 + plane1.getCurrentBearing() + theta;

	theta = toCardinal(theta);
	plane2OnRight = theta - plane1.getCurrentBearing() >= 0;
	
	if ((plane2OnRight && theta1 >= theta2) || (!plane2OnRight && theta1 <= theta2))
		turnRight = false;
	else if ((plane2OnRight && theta1 < theta2) || (!plane2OnRight && theta1 > theta2))
		turnRight = true;
	
	turnRight = (theta1 < theta2) ? true : false;
	return turnRight;
}

/* Calculate the turning radius based on the zero effort miss*/
double calculateTurningRadius(double ZEM){
	return MINIMUM_TURNING_RADIUS*exp(LAMBDA*ZEM/DESIRED_SEPARATION);
}


/* Find the new collision avoidance waypoint for the plane to go to */
AU_UAV_ROS::waypoint calculateWaypoint(PlaneObject &plane1, double turningRadius, bool turnRight){
	double phi;	
	/* Calculate theta in degrees*/	
	double theta = MPS_SPEED*LONG_TIME_STEP/turningRadius*180/PI;
	/* Calculate signed supplement of bearing in degrees*/
	double bearingBar = calculateSupplement(plane1.getCurrentBearing());	
	/* Calculate center of turning circle*/
	turningCircleCenter = calculateCircleCenter(plane1, turnRight, turningRadius);
	/* Calculate phi in degrees if it's turning right or left*/
	if (turnRight) phi = bearingBar - theta;
	else phi = 180 + bearingBar + theta;
	
	/* Calculate the new waypoint destination*/
	AU_UAV_ROS::waypoint newDest;
	newDest.altitude = plane1.altitude;
	newDest.longitude = turningCircleCenter.longitude + turningRadius/DELTA_LON_TO_METERS*cos(phi);
	newDest.latitude = turningCircleCenter.latitude + turningRadius/DELTA_LAT_TO_METERS*sin(phi)
}


/* */
AU_UAV_ROS::waypoint takeDubinsPath(PlaneObject &plane1) {
	/* Initialize variables*/
	AU_UAV_ROS::coordinate circleCenter;
	AU_UAV_ROS::waypoint wp = plane1.getDestination();
	bool destOnRight;
	/* Calculate cartesian angle from plane to waypoint*/
	double wpBearing = findAngle(plane1.getCurrentLoc().latitude, plane1.getCurrentLoc().longitude, wp.latitude, wp.longitude);
	/* Calculate cartesian current bearing of plane (currentBearing is stored as Cardinal)*/
	double currentBearingCardinal = plane1.getCurrentBearing();	
	double currentBearingCartesian = toCartesian(currentBearingCardinal);
	
	if (abs(currentBearingCardinal) < 90)
	/* Figure out which side of the plane the waypoint is on*/		
		if ((wpBearing < currentBearingCartesian) && (wpBearing > manipulateAngle(currentBearingCartesian - 180)))
			destOnRight = true;
		else destOnRight = false;
	else
		if ((wpBearing > currentBearingCartesian) && (wpBearing < manipulateAngle(currentBearingCartesian - 180)))
			destOnRight = false;
		else desOnRight = true;
	/* Calculate the center of the circle of minimum turning radius on the side that the waypoint is on*/
	circleCenter = calculateCircleCenter(plane, MINIMUM_TURNING_RADIUS, destOnRight);

	/* If destination is inside circle, must fly opposite direction before we can reach destination*/
	if (findDistance(circleCenter.latitude, circleCenter.longitude, wp.latitude, wp.longitude) < MINIMUM_TURNING_RADIUS) {
		return calculateWaypoint(plane1, MINIMUM_TURNING_RADIUS, !destOnRight)
	}
	else {
		//we can simply pass the current waypoint because it's accessible
		return wp;
	}
}

AU_UAV_ROS::coordinate calculateCircleCenter(PlaneObject &plane, double turnRadius, bool turnRight) {
	AU_UAV_ROS::coordinate circleCenter;
	circleCenter.altitude = plane.getCurrentLoc().altitude;

	if (turnRight) {
		circleCenter.longitude = plane.getCurrentLoc().longitude + (turnRadius / DELTA_LON_TO_METERS) * cos(toCartesian(plane.getCurrentBearing()) - 90);
		circleCenter.latitude = plane.getCurrentLoc().latitude + (turnRadius / DELTA_LAT_TO_METERS) * sin(toCartesian(plane.getCurrentBearing()) - 90);
	}
	else {
		circleCenter.longitude = plane.getCurrentLoc().longitude + (turnRadius / DELTA_LON_TO_METERS) * cos(toCartesian(plane.getCurrentBearing()) + 90);
		circleCenter.latitude = plane.getCurrentLoc().latitude + (turnRadius / DELTA_LAT_TO_METERS) * sin(toCartesian(plane.getCurrentBearing()) + 90);
	}
	return circleCenter;
}
