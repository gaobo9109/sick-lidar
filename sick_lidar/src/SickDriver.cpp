#include "SickLDMRSSensor.h"
#include <ros/ros.h>
#include <QCoreApplication>


int main (int argc, char *argv[]) 
{
	ros::init(argc, argv, "lidar");
	ros::NodeHandle nh; 
	pacpus::SickLDMRSSensor sensor;
	sensor.startActivity();
	QCoreApplication app(argc, argv);
	return app.exec();
}



