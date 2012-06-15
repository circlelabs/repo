/*
RIPNA.cpp

This is the implementation of RIPNA.h. 

*/


#include <math.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <iostream>

#include "AU_UAV_ROS/ripna.h"
#include "AU_UAV_ROS/standardFuncs.h"		// for PI, EARTH_RADIUS, MPS_SPEED
#include "AU_UAV_ROS/SimulatedPlane.h"		// for MAXIMUM_TURNING_ANGLE

#define CHECK_ZONE 6.0*MPS_SPEED //meters
#define DANGER_ZEM 2.0*MPS_SPEED //meters
#define MINIMUM_TURNING_RADIUS 28.64058013 //meters	
#define DESIRED_SEPARATION 2.0*MPS_SPEED //meters
#define LAMBDA 0.1 //dimensionless
#define TIME_STEP 1.0 //seconds

/* This is the function called by collisionAvoidance.cpp which calls 
all necessary functions in order to find the new collision avoidance 
waypoint for the plane to travel to. If no collision avoidance 
maneuvers are necessary, the function returns the current destination 
waypoint. */

AU_UAV_ROS::waypoint AU_UAV_ROS::findNewWaypoint(PlaneObject &plane1, std::map<int, PlaneObject> &planes){
	/* Find plane to avoid*/
	AU_UAV_ROS::threatContainer greatestThreat = findGreatestThreat(plane1, planes);
	
	/* Unpack plane to avoid*/	
	int threatID = greatestThreat.planeID;
	double threatZEM = greatestThreat.ZEM;
	
	/* If there is no plane to avoid, then take Dubin's path to the 
	destination waypoint*/
	if ((threatID < 0) && (threatZEM < 0)) {
		return takeDubinsPath(plane1);
	}
	
	ROS_WARN("PLANE%d detected PLANE%d with a ZEM of %f", plane1.getID(), threatID, threatZEM);

	
	/* If there is a plane to avoid, then figure out which direction it 
	should turn*/
	bool turnRight = shouldTurnRight(plane1, planes[threatID]);

	/* Calculate turning radius to avoid collision*/
	double turningRadius = calculateTurningRadius(threatZEM);

	/* Given turning radius and orientation of the plane, calculate 
	next collision avoidance waypoint*/
	return calculateWaypoint(plane1, turningRadius, turnRight);
}

	
/* Function that returns the ID of the most dangerous neighboring plane and its ZEM */
AU_UAV_ROS::threatContainer AU_UAV_ROS::findGreatestThreat(PlaneObject &plane1, std::map<int, PlaneObject> &planes){
	/* Set reference for origin (Northwest corner of the course)*/
	AU_UAV_ROS::coordinate origin;
	origin.latitude = 32.606573;
	origin.longitude = -85.490356;
	origin.altitude = 400;
	/* Set preliminary plane to avoid as non-existent and most dangerous 
	ZEM as negative*/
	int planeToAvoid = -1;
	double mostDangerousZEM = -1;
	/* Set the preliminary time-to-go to infinity*/
	double minimumTimeToGo = std::numeric_limits<double>::infinity();
	/* Declare second plane and ID variable */
	PlaneObject plane2;
	int ID;
	/* Make a position vector representation of the current plane*/
	double magnitude2, direction2;
	double magnitude = findDistance(origin.latitude, origin.longitude, 
		plane1.getCurrentLoc().latitude, plane1.getCurrentLoc().longitude);
	double direction = findAngle(origin.latitude, origin.longitude, 
		plane1.getCurrentLoc().latitude, plane1.getCurrentLoc().longitude);
	AU_UAV_ROS::mathVector p1(magnitude,direction);

	/* Make a heading vector representation of the current plane*/
	AU_UAV_ROS::mathVector d1(1.0,toCartesian(plane1.getCurrentBearing()));
	
	/* Declare variables needed for this loop*/
	AU_UAV_ROS::mathVector pDiff;
	AU_UAV_ROS::mathVector dDiff;
	double timeToGo, zeroEffortMiss;
	std::map<int,AU_UAV_ROS::PlaneObject>::iterator it;
	for ( it=planes.begin() ; it!= planes.end(); it++ ){
		/* Unpacking plane to check*/		
		ID = (*it).first;
		plane2 = (*it).second;
		
		/* If it's not in the Check Zone, check the other plane*/
		if(plane1.findDistance(plane2) > CHECK_ZONE || plane1.getID() == ID) continue;

		/* Making a position vector representation of plane2*/
		magnitude2 = findDistance(origin.latitude, origin.longitude, 
			plane2.getCurrentLoc().latitude, plane2.getCurrentLoc().longitude);
		direction2 = findAngle(origin.latitude, origin.longitude, 
			plane2.getCurrentLoc().latitude, plane2.getCurrentLoc().longitude);
		AU_UAV_ROS::mathVector p2(magnitude2,direction2);

		/* Make a heading vector representation of the current plane*/
		AU_UAV_ROS::mathVector d2(1.0,toCartesian(plane2.getCurrentBearing()));

		/* Compute Time To Go*/
		pDiff = p1-p2;
		dDiff = d1-d2;
		timeToGo = -1*pDiff.dotProduct(dDiff)/(MPS_SPEED*dDiff.dotProduct(dDiff));

		/* Compute Zero Effort Miss*/
		zeroEffortMiss = sqrt(pDiff.dotProduct(pDiff) + 
			2*(MPS_SPEED*timeToGo)*pDiff.dotProduct(dDiff) + 
			pow(MPS_SPEED*timeToGo,2)*dDiff.dotProduct(dDiff));
		
		/* If the Zero Effort Miss is less than the minimum required 
		separation, consider this plane as a candidate to avoid*/
		if(zeroEffortMiss <= DANGER_ZEM && timeToGo < minimumTimeToGo){
			planeToAvoid = ID;
			mostDangerousZEM = zeroEffortMiss;
			minimumTimeToGo = timeToGo;
			ROS_WARN("potential time to go:%f", timeToGo);
		}
	}

	AU_UAV_ROS::threatContainer greatestThreat;
	greatestThreat.planeID = planeToAvoid;
	greatestThreat.ZEM = mostDangerousZEM;

	return greatestThreat;
}


/* Returns true if the original plane (plane1) should turn right to avoid plane2, 
false if otherwise. Takes original plane and its greatest threat as parameters */
bool AU_UAV_ROS::shouldTurnRight(PlaneObject &plane1, PlaneObject &plane2) {

	/* For checking whether the plane should turn right or left */
	double theta;
	double theta1;
	double theta2;
	bool turnRight;
	bool plane2OnRight;

	/* Calculate theta, theta1, and theta2. Theta is the cartesian angle
	from 0 degrees (due East) to plane2 (using plane1 as the origin). This 
	may be referred to as the LOS angle. */
	theta = findAngle(plane1.getCurrentLoc().latitude, plane1.getCurrentLoc().longitude, 
		plane2.getCurrentLoc().latitude, plane2.getCurrentLoc().longitude);
	theta1 = 90 - theta - plane1.getCurrentBearing();
	theta2 = 90 + theta + plane2.getCurrentBearing();
	
	/* Calculate which side of plane1 that plane2 is on. */
	double cardinalTheta = toCardinal(theta);
	plane2OnRight = cardinalTheta >= plane1.getCurrentBearing();
	
	/* Calculate which direction to turn*/
	if ((plane2OnRight && theta1 >= theta2) || (!plane2OnRight && theta1 <= theta2))
		turnRight = false;
	else if ((plane2OnRight && theta1 < theta2) || (!plane2OnRight && theta1 > theta2))
		turnRight = true;
	
	turnRight = (theta1 < theta2) ? true : false;
	return turnRight;
}

/* Calculate the turning radius based on the zero effort miss*/
double AU_UAV_ROS::calculateTurningRadius(double ZEM){
	return MINIMUM_TURNING_RADIUS*exp(LAMBDA*ZEM/DESIRED_SEPARATION);
}


/* Find the new collision avoidance waypoint for the plane to go to */
AU_UAV_ROS::waypoint AU_UAV_ROS::calculateWaypoint(PlaneObject &plane1, double turningRadius, bool turnRight){
	double phi;	
	/* Calculate theta in degrees*/	
	double theta = 2*asin(MPS_SPEED*TIME_STEP/(2*turningRadius))*180/PI;
	/* Calculate signed supplement of bearing in degrees*/
	double bearingBar = calculateSupplement(plane1.getCurrentBearing());	
	/* Calculate center of turning circle*/
	AU_UAV_ROS::coordinate turningCircleCenter = calculateCircleCenter(plane1, turnRight, turningRadius);
	
	/* Calculate phi in degrees if it's turning right or left*/
	if (turnRight) phi = bearingBar - theta;
	else phi = 180 + bearingBar + theta;
	
	/* Calculate the new waypoint destination*/
	AU_UAV_ROS::waypoint newDest;
	newDest.altitude = plane1.getCurrentLoc().altitude;
	newDest.longitude = turningCircleCenter.longitude + turningRadius/DELTA_LON_TO_METERS*cos(phi);
	newDest.latitude = turningCircleCenter.latitude + turningRadius/DELTA_LAT_TO_METERS*sin(phi);
	return newDest;
}


/* This function is calculates any maneuvers that are necessary for the 
current plane to avoid looping. Returns a waypoint based on calculations. 
If no maneuvers are necessary, then the function returns the current 
destination*/
AU_UAV_ROS::waypoint AU_UAV_ROS::takeDubinsPath(PlaneObject &plane1) {
	/* Initialize variables*/
	AU_UAV_ROS::coordinate circleCenter;
	AU_UAV_ROS::waypoint wp = plane1.getDestination();
	bool destOnRight;
	/* Calculate cartesian angle from plane to waypoint*/
	double wpBearing = findAngle(plane1.getCurrentLoc().latitude, 
		plane1.getCurrentLoc().longitude, wp.latitude, wp.longitude);
	/* Calculate cartesian current bearing of plane (currentBearing is stored as Cardinal)*/
	double currentBearingCardinal = plane1.getCurrentBearing();	
	double currentBearingCartesian = toCartesian(currentBearingCardinal);
	
	if (abs(currentBearingCardinal) < 90)
	/* Figure out which side of the plane the waypoint is on*/		
		if ((wpBearing < currentBearingCartesian) && 
				(wpBearing > manipulateAngle(currentBearingCartesian - 180)))
			destOnRight = true;
		else destOnRight = false;
	else
		if ((wpBearing > currentBearingCartesian) && 
				(wpBearing < manipulateAngle(currentBearingCartesian - 180)))
			destOnRight = false;
		else destOnRight = true;
	/* Calculate the center of the circle of minimum turning radius on the side that the waypoint is on*/
	circleCenter = calculateCircleCenter(plane1, MINIMUM_TURNING_RADIUS, destOnRight);

	/* If destination is inside circle, must fly opposite direction before we can reach destination*/
	if (findDistance(circleCenter.latitude, circleCenter.longitude, wp.latitude, wp.longitude) < 
			MINIMUM_TURNING_RADIUS) {
		return calculateWaypoint(plane1, MINIMUM_TURNING_RADIUS, !destOnRight);
	}
	else {
		//we can simply pass the current waypoint because it's accessible
		return wp;
	}
}

/* This function takes a plane, its turning radius, and the direction to turn 
and returns the center of the circle of its turning radius. */
AU_UAV_ROS::coordinate AU_UAV_ROS::calculateCircleCenter(PlaneObject &plane, double turnRadius, bool turnRight) {
	AU_UAV_ROS::coordinate circleCenter;
	circleCenter.altitude = plane.getCurrentLoc().altitude;
	
	if (turnRight) {
		double angle = (toCartesian(plane.getCurrentBearing()) - 90) * PI/180.0;
		double xdiff = turnRadius*cos(angle);
		double ydiff = turnRadius*sin(angle);
		circleCenter.longitude = plane.getCurrentLoc().longitude + xdiff/DELTA_LON_TO_METERS;
		circleCenter.latitude = plane.getCurrentLoc().latitude + ydiff/DELTA_LAT_TO_METERS; 
	}
	else {
		double angle = (toCartesian(plane.getCurrentBearing()) + 90) * PI/180.0;
		double xdiff = turnRadius*cos(angle);
		double ydiff = turnRadius*sin(angle);
		circleCenter.longitude = plane.getCurrentLoc().longitude + xdiff/DELTA_LON_TO_METERS;
		circleCenter.latitude = plane.getCurrentLoc().latitude + ydiff/DELTA_LAT_TO_METERS; 
	}
	
	return circleCenter;
}
