/*PlaneObject

*/

#ifndef PLANE_OBJECT_H
#define PLANE_OBJECT_H

#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/standardDefs.h"

namespace AU_UAV_ROS {

	class PlaneObject {

	public:
		PlaneObject(void);

		PlaneObject(int id, double cBearing, double speed, double cRadius, 
			    const AU_UAV_ROS::waypoint &currentPos, const AU_UAV_ROS::waypoint &dest);
		/*
		Explicit value constructor: Takes a collision radius and a telemetry update and creates a new PlaneObject.
		*/
		PlaneObject(double cRadius, const AU_UAV_ROS::TelemetryUpdate &msg);

		/* Copy constructor */
		PlaneObject(const PlaneObject& pobj);

		/*mutator functions*/
		void setID(int id);
		void setPreviousLoc(double lat, double lon, double alt);
		void setCurrentLoc(double lat, double lon, double alt);
		void setTargetBearing(double tBearing);		/* set bearing to destination */
		void setCurrentBearing(double cBearing); 	/* set current bearing in the air */
		void setSpeed(double speed);
		void setDestination(const AU_UAV_ROS::waypoint &destination);

		/*accessor functions*/
		void update(void);

		/* Update the plane's data members with the information contained within the telemetry update */
		void update(const AU_UAV_ROS::TelemetryUpdate &msg);

		/* Accessor functions: Allow the client to access the plane's id, altitude, bearing, spped, etc. */
		int getID(void) const;
		double getPreviousLoc(void) const;
		double getCurrentLoc(void) const;
		double getTargetBearing(void) const;		/* get target bearing */
		double getCurrentBearing(void) const;		/* get current bearing in the air */
		double getSpeed(void) const;
		double getLastUpdateTime(void) const;
		AU_UAV_ROS::waypoint getDestination(void) const;

		/* 
		Find distance / Cardinal angle between two planes. The calling plane gives the starting latitude and longitude, 
		and the object passed as a parameter gives the final latitude and longitude.
		*/
		double findDistance(const PlaneObject& pobj) const;
		double findAngle(const PlaneObject& pobj) const;

		/* Overloaded equality operator */
		PlaneObject& operator=(const PlaneObject& pobj);

		/* Find the distance between this plane object and another plane object or a latitude and longitude */
		double findDistance(double lat, double lon) const;
		double findDistance(const PlaneObject& planeObj) const;

		/* Find the Cardinal angle between this plane object and another plane object or a latitude and longitude */
		double findAngle(double lat, double lon) const;
		double findAngle(const PlaneObject& planeObj) const;

		/* Returns true if a plane object is within the cRadius meters of this plane object, false otherwise */
		bool isColliding(const PlaneObject& planeObj) const;

	private:
		/* Private data members */
		int id;
		double collisionRadius;
		double targetBearing;		/* get bearing to destination */
		double currentBearing;		/* get current bearing in the air */
		double speed;
		double lastUpdateTime;
		AU_UAV_ROS::coordinate previousLoc;	/*used to calculate currentBearing*/
		AU_UAV_ROS::coordinate currentLoc;
		AU_UAV_ROS::waypoint destination;
		
	};
};


#endif
