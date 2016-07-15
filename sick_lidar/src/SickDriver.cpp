#include "SickLDMRSSensor.h"
#include <ros/ros.h>
#include <QCoreApplication>


int main (int argc, char *argv[]) 
{
	QCoreApplication app(argc, argv);
	ros::init(argc, argv, "lidar"); 
//    sick_lidar::SickLDMRSSensor sensor("192.168.1.200", 12002);
    sick_lidar::SickLDMRSSensor sensor("192.168.1.113", 55056);
	return app.exec();
}



