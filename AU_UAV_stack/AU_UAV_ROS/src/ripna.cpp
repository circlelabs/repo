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

#define PLANE_MAX_TURN_ANGLE 22.5 //degrees / sec
#define CHECK_ZONE 10.0*MPS_SPEED //meters
#define DANGER_ZEM 2.5*MPS_SPEED //meters
#define MINIMUM_TURNING_RADIUS 0.75*28.64058013 //meters	
#define DESIRED_SEPARATION 2.5*MPS_SPEED //meters
#define LAMBDA 0.1 //dimensionless
#define TIME_STEP 1.0 //seconds

/* This is the function called by collisionAvoidance.cpp which calls 
all necessary functions in order to find the new collision avoidance 
waypoint for the plane to travel to. If no collision avoidance 
maneuvers are necessary, the function returns the current destination 
waypoint. */

AU_UAV_ROS::waypointContainer AU_UAV_ROS::findNewWaypoint(PlaneObject &plane1, std::map<int, PlaneObject> &planes){
	/* Find plane to avoid*/
	AU_UAV_ROS::threatContainer greatestThreat = findGreatestThreat(plane1, planes);
	
	/* Unpack plane to avoid*/	
	int threatID = greatestThreat.planeID;
	double threatZEM = greatestThreat.ZEM;
	/*
	if (threatID != -1) {
	ROS_WARN("Distance between plane %d and plane %d = %f", plane1.getID(), 
		threatID, findDistance(plane1.getCurrentLoc().latitude, 
		plane1.getCurrentLoc().longitude, planes[threatID].getCurrentLoc().latitude, 
		planes[threatID].getCurrentLoc().longitude));
	}
	*/

	AU_UAV_ROS::waypointContainer newWaypoints; 	

	/* If there is no plane to avoid, then take Dubin's path to the 
	destination waypoint*/
	if ((threatID < 0) && (threatZEM < 0)) {
		newWaypoints.plane1WP = takeDubinsPath(plane1);
		newWaypoints.plane2ID = threatID;
		return newWaypoints;
	}
	
	/* If there is a plane to avoid, then figure out which direction it 
	should turn*/
	bool turnRight = shouldTurnRight(plane1, planes[threatID]);
	//ROS_WARN("Plane %d shouldTurnRight = %d", plane1.getID(), turnRight);	

	/* Calculate turning radius to avoid collision*/
	double turningRadius = calculateTurningRadius(threatZEM);
	//ROS_WARN("PLANE%d detected PLANE%d and wants to turn right %d with radius %f", plane1.getID(), threatID, turnRight, turningRadius);

	/* Given turning radius and orientation of the plane, calculate 
	next collision avoidance waypoint*/
	AU_UAV_ROS::waypoint plane1WP = calculateWaypoint(plane1, turningRadius, turnRight);
	newWaypoints.plane2ID = -1;
	if (findGreatestThreat(planes[threatID], planes).planeID == plane1.getID()) {
		AU_UAV_ROS::waypoint plane2WP = calculateWaypoint(planes[threatID], turningRadius, turnRight);
		ROS_WARN("Plane %d and Plane %d have each other as their greatest threats", threatID, plane1.getID());
		newWaypoints.plane2WP = plane2WP;
		newWaypoints.plane2ID = threatID;
	}
	newWaypoints.plane1WP = plane1WP;
	return newWaypoints;
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
		if(zeroEffortMiss <= DANGER_ZEM && timeToGo < minimumTimeToGo && timeToGo > 0){
			planeToAvoid = ID;
			mostDangerousZEM = zeroEffortMiss;
			minimumTimeToGo = timeToGo;
		}
	}

	AU_UAV_ROS::threatContainer greatestThreat;
	greatestThreat.planeID = planeToAvoid;
	greatestThreat.ZEM = mostDangerousZEM;
	
	ROS_WARN("TimeToGo for plane %d and plane %d is %f", plane1.getID(), planeToAvoid, minimumTimeToGo);
	return greatestThreat;
}


/* Returns true if the original plane (plane1) should turn right to avoid plane2, 
false if otherwise. Takes original plane and its greatest threat as parameters */
bool AU_UAV_ROS::shouldTurnRight(PlaneObject &plane1, PlaneObject &plane2) {

	/* For checking whether the plane should turn right or left */
	double theta, theta_prev, theta_prime, delta_theta, theta1, theta2;
	bool turnRight, plane2OnRight, plane1OnRight;
	double cartBearing1 = toCartesian(plane1.getCurrentBearing());
	double cartBearing2 = toCartesian(plane2.getCurrentBearing());

	/* Calculate theta, theta1, and theta2. Theta is the cartesian angle
	from 0 degrees (due East) to plane2 (using plane1 as the origin). This 
	may be referred to as the LOS angle. */
	theta = findAngle(plane1.getCurrentLoc().latitude, plane1.getCurrentLoc().longitude, 
		plane2.getCurrentLoc().latitude, plane2.getCurrentLoc().longitude);
	theta_prev = findAngle(plane1.getPreviousLoc().latitude, plane1.getPreviousLoc().longitude, 
		plane2.getPreviousLoc().latitude, plane2.getPreviousLoc().longitude);
	delta_theta = theta - theta_prev;
	theta_prime = manipulateAngle(theta+180.0);
	theta1 = cartBearing1 - theta;
	theta2 = cartBearing2 - theta_prime;
	
	/* Calculate which side of plane1 that plane2 is on, and also which side of plane2 plane 1 is on.*/
	plane2OnRight = theta1 >= 0; plane1OnRight = theta2 >= 0;
	//ROS_WARN("Plane %d: Bearing: %f plane2OnRight: %d", plane1.getID(), plane1.getCurrentBearing(), plane2OnRight);
	
	/* Calculate which direction to turn*/
	if (plane2OnRight && plane1OnRight) turnRight = false;
	else if (!plane2OnRight && !plane1OnRight) turnRight = true;
	else if (plane2OnRight && !plane1OnRight) turnRight = (theta1 < theta2) ? true : false;
	else if (!plane2OnRight && plane1OnRight) turnRight = (theta1 > theta2) ? true : false;
	
	return turnRight;
}

/* Calculate the turning radius based on the zero effort miss*/
double AU_UAV_ROS::calculateTurningRadius(double ZEM){
	double l = LAMBDA;
	double ds = DESIRED_SEPARATION;
	return MINIMUM_TURNING_RADIUS*exp(l*ZEM/ds);
}

/* Link to help calculate intersection of circles and therefore the new 
waypoint location: http://local.wasp.uwa.edu.au/~pbourke/geometry/2circle/ */

/* Find the new collision avoidance waypoint for the plane to go to */
AU_UAV_ROS::waypoint AU_UAV_ROS::calculateWaypoint(PlaneObject &plane1, double turningRadius, bool turnRight){
	
	AU_UAV_ROS::coordinate turningCircleCenter = calculateCircleCenter(plane1, turningRadius, turnRight);
	AU_UAV_ROS::waypoint wp;	
	double maxTurnAngle = PLANE_MAX_TURN_ANGLE;
	double r0 = turningRadius; //radius of turning circle
	double r1 = MPS_SPEED*TIME_STEP; //distance travelled in one time step
	double d = turningRadius; //distance between circle centers
	double a = (pow(r0, 2) - pow(r1, 2) + pow(d, 2)) / (2 * d);
	double h = sqrt(pow(r0, 2) - pow(a, 2));

	// Using turningCircleCenter as origin
	double x0 = 0;
	double y0 = 0;

	double x1 = (plane1.getCurrentLoc().longitude - turningCircleCenter.longitude) * DELTA_LON_TO_METERS;
	double y1 = (plane1.getCurrentLoc().latitude - turningCircleCenter.latitude) * DELTA_LAT_TO_METERS;

	double x2 = x0 + a * (x1 - x0) / d;
	double y2 = y0 + a * (y1 - y0) / d;

	double x3 = x2 + h * (y1 - y0) / d;
	double y3 = y2 - h * (x1 - x0) / d;

	if (fabs(manipulateAngle(toCartesian(plane1.getCurrentBearing()) - atan2((y3-y1),(x3-x1))*180/PI)) > maxTurnAngle) {
		x3 = x2 - h * (y1 - y0) / d;
		y3 = y2 + h * (x1 - x0) / d;
	}
	
	wp.latitude = turningCircleCenter.latitude + y3/DELTA_LAT_TO_METERS;
	wp.longitude = turningCircleCenter.longitude + x3/DELTA_LON_TO_METERS;
	wp.altitude = plane1.getCurrentLoc().altitude;	

	return wp;

}


/* This function is calculates any maneuvers that are necessary for the 
current plane to avoid looping. Returns a waypoint based on calculations. 
If no maneuvers are necessary, then the function returns the current 
destination*/
AU_UAV_ROS::waypoint AU_UAV_ROS::takeDubinsPath(PlaneObject &plane1) {
	/* Initialize variables*/
	AU_UAV_ROS::coordinate circleCenter;
	AU_UAV_ROS::waypoint wp = plane1.getDestination();
	double minTurningRadius = MINIMUM_TURNING_RADIUS;
	bool destOnRight;
	/* Calculate cartesian angle from plane to waypoint*/
	double wpBearing = findAngle(plane1.getCurrentLoc().latitude, 
		plane1.getCurrentLoc().longitude, wp.latitude, wp.longitude);
	/* Calculate cartesian current bearing of plane (currentBearing is stored as Cardinal)*/
	double currentBearingCardinal = plane1.getCurrentBearing();	
	double currentBearingCartesian = toCartesian(currentBearingCardinal);
	
	
	if (fabs(currentBearingCardinal) < 90.0)
	/* Figure out which side of the plane the waypoint is on*/		
		if ((wpBearing < currentBearingCartesian) && 
				(wpBearing > manipulateAngle(currentBearingCartesian - 180.0)))
			destOnRight = true;
		else destOnRight = false;
	else
		if ((wpBearing > currentBearingCartesian) && 
				(wpBearing < manipulateAngle(currentBearingCartesian - 180.0)))
			destOnRight = false;
		else destOnRight = true;
	/* Calculate the center of the circle of minimum turning radius on the side that the waypoint is on*/
	
	circleCenter = calculateLoopingCircleCenter(plane1, minTurningRadius, destOnRight);

	/* If destination is inside circle, must fly opposite direction before we can reach destination*/
	if (findDistance(circleCenter.latitude, circleCenter.longitude, wp.latitude, wp.longitude) < 
			minTurningRadius) {
		return calculateWaypoint(plane1, minTurningRadius, !destOnRight);
	}
	else {
		//we can simply pass the current waypoint because it's accessible
		//ROS_WARN("FINE: %f", findDistance(circleCenter.latitude, circleCenter.longitude, wp.latitude, wp.longitude));
		return wp;
	}
}

/* This function takes a plane, its turning radius, and the direction to turn 
and returns the center of the circle of its turning radius. */
AU_UAV_ROS::coordinate AU_UAV_ROS::calculateCircleCenter(PlaneObject &plane, double turnRadius, bool turnRight) {
	AU_UAV_ROS::coordinate circleCenter;
	circleCenter.altitude = plane.getCurrentLoc().altitude;
	double angle;
	if (turnRight) {
		angle = (toCartesian(plane.getCurrentBearing()) - 90) * PI/180.0; 
	}
	else {
		angle = (toCartesian(plane.getCurrentBearing()) + 90) * PI/180.0;
	}
	double xdiff = turnRadius*cos(angle);
	double ydiff = turnRadius*sin(angle);
	circleCenter.longitude = plane.getCurrentLoc().longitude + xdiff/DELTA_LON_TO_METERS;
	circleCenter.latitude = plane.getCurrentLoc().latitude + ydiff/DELTA_LAT_TO_METERS; 

	return circleCenter;
}

AU_UAV_ROS::coordinate AU_UAV_ROS::calculateLoopingCircleCenter(PlaneObject &plane, double turnRadius, bool turnRight) {
	AU_UAV_ROS::coordinate circleCenter;
	circleCenter.altitude = plane.getCurrentLoc().altitude;
	double angle;
	if (turnRight) {
		angle = (toCartesian(plane.getCurrentBearing()) - 90 - 22.5) * PI/180.0; 
	}
	else {
		angle = (toCartesian(plane.getCurrentBearing()) + 90 + 22.5) * PI/180.0;
	}
	double xdiff = turnRadius*cos(angle);
	double ydiff = turnRadius*sin(angle);
	circleCenter.longitude = plane.getCurrentLoc().longitude + xdiff/DELTA_LON_TO_METERS;
	circleCenter.latitude = plane.getCurrentLoc().latitude + ydiff/DELTA_LAT_TO_METERS; 

	return circleCenter;
}
