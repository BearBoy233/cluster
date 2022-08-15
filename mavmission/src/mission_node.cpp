#include <mission.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "uav_mission");

	mav_mission::Mav_Mission mission;

	ros::Duration ( 0.5 );

	mission.run();

	return 0;
}

