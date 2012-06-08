/* Implementation of planeObject.h

*/

#include "ros/ros.h"
#include "AU_UAV_ROS/planeObject.h"
#include "AU_UAV_ROS/standardFuncs.h"
#include "AU_UAV_ROS/SimulatedPlane.h"

#include <math.h>
#include "AU_UAV_ROS/standardFuncs.h" /* for PI, EARTH_RADIUS in meters */

/* Explicit value constructor using TelemetryUpdate */
AU_UAV_ROS::PlaneObject::PlaneObject(double cRadius, const AU_UAV_ROS::TelemetryUpdate &msg) {
	this->id = msg.planeID;
	this->currentLoc.altitude = msg.currentAltitude;
	this->currentLoc.latitude = msg.currentLatitude;
	this->currentLoc.longitude = msg.currentLongitude;
	this->targetBearing = msg.targetBearing;
	this->currentBearing = 0.0;

	this->speed = msg.groundSpeed;
	this->destination.latitude = msg.destLatitude;
	this->destination.longitude = msg.destLongitude;
	this->destination.altitude = msg.destAltitude;
	this->lastUpdateTime = ros::Time::now().toSec();
	this->collisionRadius = cRadius;
}

/* Copy constructor */
AU_UAV_ROS::PlaneObject::PlaneObject(const AU_UAV_ROS::PlaneObject& plane) {
	this->id = plane.id;
	this->currentLoc.altitude = plane.currentloc.altitude;
	this->targetBearing = plane.targetBearing;
	this->currentBearing = plane.currentBearing;
	this->speed = plane.speed;
	this->destination = plane.destination;
	this->lastUpdateTime = plane.lastUpdateTime;
}

/* mutator functions to update member variables */
void AU_UAV_ROS::PlaneObject::setID(int id){
	this->id = id;
}

void AU_UAV_ROS::PlaneObject::setPreviousLoc(double lat, double lon, double alt) {
	this->previousLoc.latitude = lat;
	this->previousLoc.longitude = lon;
	this->previousLoc.altitude = alt;
}

void AU_UAV_ROS::PlaneObject::setCurrentLoc(double lat, double lon, double alt) {
	this->currentLoc.latitude = lat;
	this->currentLoc.longitude = lon;
	this->currentLoc.altitude = alt;
}

void AU_UAV_ROS::PlaneObject::setTargetBearing(double tBearing) {
	this->targetBearing = tBearing;
}

void AU_UAV_ROS::PlaneObject::setCurrentBearing(double cBearing) {
	this->currentBearing = cBearing;
}

void AU_UAV_ROS::PlaneObject::setSpeed(double speed) {
	this->speed = speed;
}

void AU_UAV_ROS::PlaneObject::setDestinaton(const AU_UAV_ROS::waypoint &destination) {
	this->destination = destination;
}

void AU_UAV_ROS::PlaneObject::updateTime(void) {
	this->lastUpdateTime = ros::Time::now().toSec();
}

void AU_UAV_ROS::PlaneObject::update(const AU_UAV_ROS::TelemetryUpdate &msg) {

	this->setTargetBearing(msg.targetBearing);

	//TODO Calculate current bearing

	//TODO setCurrentBearing

	this->setCurrentLoc(msg.currentLatitude, msg.currentLongitude, msg.currentAltitude);
	this->setSpeed(msg.groundSpeed);
	this->updateTime();
}

/* accessor functions */
int AU_UAV_ROS::PlaneObject::getID(void) const {
	return this->id;
}

AU_UAV_ROS::coordinate AU_UAV_ROS::getPreviousLoc(void) const {
	return this->previousLoc;
}

AU_UAV_ROS::coordinate AU_UAV_ROS::PlaneObject::getCurrentLoc(void) const {
	return this->currentLoc;
}

double AU_UAV_ROS::PlaneObject::getTargetBearing(void) const {
	return this->targetBearing;
}

double AU_UAV_ROS::PlaneObject::getCurrentBearing(void) const {
	return this->currentBearing;
}
	
double AU_UAV_ROS::PlaneObject::getSpeed(void) const {
	return this->speed;
}

double AU_UAV_ROS::PlaneObject::getLastUpdateTime(void) const {
	return this->lastUpdateTime;
}

AU_UAV_ROS::waypoint AU_UAV_ROS::PlaneObject::getDestination(void) const {
	return this->destination;
}

/* Find distance between this plane and another plane */
double AU_UAV_ROS::PlaneObject::findDistance(const AU_UAV_ROS::PlaneObject::PlaneObject& plane) {
	return this->findDistance(plane.currentLoc.latitude, plane.currentLoc.longitude);
}

/* Find distance between this plane and another plane's latitude/longitude */
double AU_UAV_ROS::findDistance(double lat, double lon) const {
	double latDiff = 0.0, lonDiff = 0.0;
	double squareHalfChord = 0.0, angularDistance = 0.0;

	/* Get difference in latitude and longitude in radians */
	latDiff = (this->currentLoc.latitude - lat) * PI / 180.0;
	lonDiff = (this->currentLoc.longitude - lon) * PI / 180.0;

	/* Haversine math: see http://www.movable-type.co.uk/scripts/latlong.html for more information */

	/* Find the square of half of the chord length between the two points */
	/* sin(lat difference)^2 + cos(lat1) * cos(lat2) * sin(lon difference)^2 */
	squareHalfChord = pow(sin(latDiff / 2), 2) + 
                    	  pow(sin(lonDiff / 2), 2) *
                    	  cos(this->currentLoc.latitude * PI / 180.0) *
                    	  cos(lat * PI / 180.0);

	/* Calulate the angular distance in radians */
	/* 2 * arctan(sqrt(squareHalfChord), sqrt(1 - squareHalfChord) */
	angularDistance = 2 * atan2(sqrt(squareHalfChord), sqrt(1 - squareHalfChord));

	/* Return result in meters */
	return angularDistance * EARTH_RADIUS;
}

/* Find angle between this plane and another plane */
double AU_UAV_ROS::PlaneObject::findAngle(const AU_UAV_ROS::PlaneObject::PlaneObject plane) const {
	return findAngle(plane.currentLoc.latitude, plane.currentLoc.longitude);
}

/* Find angle between this plane and another plane's latitude/longitude */
double AU_UAV_ROS::PlaneObject::findAngle(double lat, double lon) const {
	double lonDiff = 0.0, angle = 0.0;
	double x = 0.0, y = 0.0;
	double thisLat;

	/* Convert latitudes to radians */
	lat *= PI / 180.0;
	thisLat = this->currentLoc.latitude * PI /180.0;

	lonDiff = (lon - this->currentLoc.longitude) * PI / 180.0; /* convert difference in longitude to radians */

	/* Haversine math: see http://www.movable-type.co.uk/scripts/latlong.html for more information */
	y = sin(lonDiff)*cos(lat);
	x = cos(thisLat)*sin(lat)-sin(thisLat)*cos(lat)*cos(lonDiff);

	angle = atan2(y,x) * 180.0 / PI; /* convert final result to degrees */

	return angle;
}

AU_UAV_ROS::PlaneObject::PlaneObject& AU_UAV_ROS::PlaneObject::PlaneObject::operator=(const AU_UAV_ROS::PlaneObject::PlaneObject& plane) {
  this->latitude = plane.currentLoc.latitude;
  this->longitude = plane.currentLoc.longitude;
  this->collisionRadius = plane.collisionRadius;
  return *this;
}
