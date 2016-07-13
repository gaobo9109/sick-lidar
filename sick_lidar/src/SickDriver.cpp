#include "SickLDMRSSensor.h"
#include <ros/ros.h>
#include <QCoreApplication>


int main (int argc, char *argv[]) 
{
	QCoreApplication app(argc, argv);
	ros::init(argc, argv, "lidar"); 
	pacpus::SickLDMRSSensor sensor("0.0.0.0", 12000);
	sensor.startActivity();
	return app.exec();
}



