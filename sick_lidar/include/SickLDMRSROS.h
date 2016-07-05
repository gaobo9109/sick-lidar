#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <SickLDMRSSensor.h>
#include <ros/ros.h>


namespace pacpus{

class SickLDMRSROS
{
	public:


	private:
		int queueSize = 1;
		ros::NodeHandle nh = getPrivateNodeHandle();
		ros::Publisher pcl = nh.advertise<sensor_msgs::PointCloud2>("/lidar/cloud",queueSize);
		ros::Publisher scan0 = nh.advertise<sensor_msg::LaserScan>("/lidar/scan0",queueSize);
		ros::Publisher scan1 = nh.advertise<sensor_msg::LaserScan>("/lidar/scan1",queueSize);
		ros::Publisher scan2 = nh.advertise<sensor_msg::LaserScan>("/lidar/scan2",queueSize);
		ros::Publisher scan3 = nh.advertise<sensor_msg::LaserScan>("/lidar/scan3",queueSize);

};

} // namespace pacpus
