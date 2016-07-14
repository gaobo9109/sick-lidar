#ifndef __SICKLDMRS_ROS_H__
#define __SICKLDMRS_ROS_H__


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace sick_lidar{

class MessageLDMRS;
struct ScanPoint;

class SickLDMRSROS
{
	public:
		SickLDMRSROS();
		~SickLDMRSROS();
		void makePointCloud(MessageLDMRS *message);

	private:

		ros::NodeHandle	nh;
		ros::Publisher pclPub;
		PointCloud cloud;
		float vSinLut[4];
		float vCosLut[4];

		void initLut();
		void polar2Cartesian(ScanPoint &ptPolar, pcl::PointXYZ &ptXYZ, float radPerTick);

};

} // namespace sick_lidar

#endif
