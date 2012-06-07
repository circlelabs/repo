/*
collisionAvoidance
This is where students will be able to program in a collision avoidance algorithm.  The telemetry callback
is already setup along with a dummy version of how the service request would work.
*/

//standard C++ headers
#include <sstream>
#include <stdlib.h>
#include <time.h>

//ROS headers
#include "ros/ros.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/GoToWaypoint.h"
#include "AU_UAV_ROS/RequestWaypointInfo.h"
#include "AU_UAV_ROS/standardDefs.h"


//ROS service client for calling a service from the coordinator
ros::ServiceClient client;
ros::ServiceClient goToWaypointClient;
ros::ServiceClient requestWaypointInfoClient;


//keeps count of the number of services requested
int count;

//this function is run everytime new telemetry information from any plane is recieved
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
	//TODO:Make this function do something useful, aka an avoidance algorithm
	ROS_INFO("Received update #[%lld]", msg->currentWaypointIndex);
	std::cout<<msg->currentLatitude;
	std::cout<<msg->currentLongitude;
	std::cout<<msg->currentAltitude;
	std::cout<<msg->destLatitude;
	std::cout<<msg->destLongitude;
	std::cout<<msg->destAltitude;
	std::cout<<msg->groundSpeed;
	std::cout<<msg->targetBearing;
	std::cout<<msg->currentWaypointIndex;
	std::cout<<msg->distanceToDestination;
	
	//this 'if' statement will be changed to run when collision avoidance service needs to be used
	if(rand() % 5 == 0)
	{
		//this will be replaced by students to do more than just send a string
		std::stringstream ss;
		ss << "Sending service request " << count++;

		//dummying up a service request for the REU students to see
		AU_UAV_ROS::GoToWaypoint srv;
		srv.request.planeID = msg->planeID;
		srv.request.latitude = 100;
		srv.request.longitude = 100;
		srv.request.altitude = 100;
		
		//these settings mean it is an avoidance maneuver waypoint AND to clear the avoidance queue
		srv.request.isAvoidanceManeuver = true;
		srv.request.isNewQueue = true;

		//check to make sure the client call worked (regardless of return values from service)
		if(client.call(srv))
		{
			ROS_INFO("Received response from service request %d", (count-1));
		}
		else
		{
			ROS_ERROR("Did not receive response");
		}
	}
}

int main(int argc, char **argv)
{
	//standard ROS startup
	ros::init(argc, argv, "collisionAvoidance");
	ros::NodeHandle n;
	
	//subscribe to telemetry outputs and create clients for the goToWaypoint and requestWaypointInfo services
	ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
	//goToWaypointClient = n.serviceClient<AU_UAV_ROS::GoToWaypoint>("go_to_waypoint");
	//requestWaypointInfoClient = n.serviceClient<AU_UAV_ROS::RequestWaypointInfo>("request_waypoint_info");
	
	//initialize counting
	count = 0;

	//needed for ROS to wait for callbacks
	ros::spin();	

	return 0;
}
