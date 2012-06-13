/*
Implementation of standardFuncs.h.  For information on how to use these functions, visit standardFuncs.h.  Comments in this file
are related to implementation, not usage.
*/

#include <cmath>
#include <stdlib.h>
#include "AU_UAV_ROS/standardFuncs.h"

/* Constants for converting latitude/longitude to meters */
#define DELTA_LAT_TO_METERS = 111200;
#define DELTA_LON_TO_METERS = 93670;

/*
Given a waypoint (latitude, longitude, and altitude) as well as the bearing and angular distance to travel,
calculateCoordinate will return the new location in the form of a waypoint.
*/
AU_UAV_ROS::waypoint calculateCoordinate(AU_UAV_ROS::waypoint currentPosition, double bearing, double distance){
	// Calculate final latitude and longitude; see movable-type.co.uk/scripts/latlong.html for more detail
	bearing *= DEGREES_TO_RADIANS; // convert angle of force to radians

	double lat1 = currentPosition.latitude*DEGREES_TO_RADIANS; // lat1 = current latitude in radians
	double dLat = distance*cos(bearing); // calculate change in latitude
	double lat2 = lat1 + dLat; // calculate final latitude
	double dPhi = log(tan(lat2/2+PI/4)/tan(lat1/2+PI/4));
	double q = (!(dPhi < 0.0001)) ? dLat/dPhi : cos(lat1);  // East-West line gives dPhi=0
	double dLon = distance*sin(bearing)/q; // calculate change in longitude
	
	// check for some daft bugger going past the pole, normalise latitude if so
	if (abs(lat2) > PI/2) 
		lat2 = lat2>0 ? PI-lat2 : -(PI-lat2);
	
	double lon2 = (currentPosition.longitude*DEGREES_TO_RADIANS+dLon) * RADIANS_TO_DEGREES; // calculate final latitude and convert to degrees

	//wrap around if necessary to ensure final longitude is on the interval [-180, 180]
	lon2 = manipulateAngle(lon2);

	lat2 *= RADIANS_TO_DEGREES; // convert final latitude to degrees

	AU_UAV_ROS::waypoint coordinate;
	coordinate.latitude = lat2;
	coordinate.longitude = lon2;
	coordinate.altitude = currentPosition.altitude;
	
	return coordinate;
}

/* Convert Cardinal direction to an angle in the Cartesian plane */
double toCartesian(double UAVBearing){
	UAVBearing = manipulateAngle(UAVBearing); /* get angle on the interval [-180, 180] */

	if (UAVBearing < 180 && UAVBearing >= 0) /* UAV bearing is in the first or fourth quadrant */
		return 90 - UAVBearing;
	else if (UAVBearing < 0 && UAVBearing >= -90) /* UAV bearing is in the second quadrant */
		return -1*UAVBearing + 90;
	else if (UAVBearing < -90 && UAVBearing > -180) /* UAV bearing is in the third quadrant */
		return -1*(UAVBearing + 180) - 90;
	else if (UAVBearing == 180 || UAVBearing == -180)
		return -90;
	else
		return -999; /* should never happen in current setup */
}

/* Convert angle in the Cartesian plane to a Cardinal direction */
double toCardinal(double angle){
	angle = manipulateAngle(angle); /* get angle on the interval [-180, 180] */

	if (angle <= 90 && angle >= -90) /* angle is in the first or fourth quadrant */
		return 90 - angle;
	else if (angle >= 90 && angle <= 180) /* angle is in the second quadrant */
		return -1*angle + 90;
	else if (angle <= -90 && angle >= -180) /* angle is in third quadrant */
		return -180 + -1*(90 + angle);
	else 
		return -999; /* should never happen in current setup */ 
}

/* Modify the angle so that it remains on the interval [-180, 180] */
double manipulateAngle(double angle){
	while (angle > 180){
		/* decrease angle by one 360 degree cycle */
		angle-=360;
	}

	while (angle < -180){
		/* increase angle by one 360 degree cycle cycle */
		angle+=360;
	}

	return angle;
}

/* 
Returns the distance between two points of latitude and longitude in meters.  The first two parameters
are the latitude and longitude of the starting point, and the last two parameters are the latitude and
longitude of the ending point.
*/

double findDistance(double lat1, double long1, double lat2, double long2){
	double latDiff = 0.0, lonDiff = 0.0;
	
	/* Get difference in radians */
	latDiff = (lat2 - lat1)*DELTA_LAT_TO_METERS;
	lonDiff = (long2 - long1)*DELTA_LON_TO_METERS;

	/* Return result in meters */
	return sqrt(pow(latDiff, 2) + pow(lonDiff, 2));
}

/* 
Returns the Cartesian angle between two points of latitude and longitude in degrees.  The starting point is given
by lat1 and long1 (the first two parameters), and the final point is given by lat2 and long2 (the final two parameters).
The value returned is on the interval [-180, 180].
*/
double findAngle(double lat1, double long1, double lat2, double long2){
	double latDiff = 0.0, lonDiff = 0.0;
	
	/* Get difference in radians */
	latDiff = (lat2 - lat1)*DELTA_LAT_TO_METERS;
	lonDiff = (long2 - long1)*DELTA_LON_TO_METERS;

	/* Return result in degrees */
	return atan2(latDiff/lonDiff*180/PI);
}

/* Returns the sign of the double*/
double findSign(double number){
	if (number > 0) return 1;
	if (number < 0) return -1;
	return 0;
}


/* Calculates the signed supplement of an angle*/
double calculateSupplement(double theta){
	return findSign(theta)*(PI-abs(theta));	
}


