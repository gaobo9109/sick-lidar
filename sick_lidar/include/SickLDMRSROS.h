#ifndef __SICKLDMRS_ROS_H__
#define __SICKLDMRS_ROS_H__


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <map>
#include "SickLDMRSData.h"
#include "SickLDMRSSensor.h"

#define PI 3.14159265


typedef pcl::PointXYZ Point3D;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef sensor_msgs::LaserScan LaserScan;


namespace sick_lidar{

static const int TICK_FREQUENCY = 3200;
static const int TICK_INDEX = TICK_FREQUENCY/800;
static const float RAD_PER_TICK = 2*PI/11520;

//comparator
bool compareEcho(const ScanPoint &pt1, const ScanPoint &pt2);
bool compareLayer(const ScanPoint &pt1, const ScanPoint &pt2);
bool compareAngle(const ScanPoint& pt1,const ScanPoint& pt2);

class SickLDMRSROS
{
public:
    SickLDMRSROS();
    ~SickLDMRSROS();
   void processData(MessageScanData &data);

private:

    ros::NodeHandle	nh;
    ros::Publisher pclPub;
    ros::Publisher scanPub1;
    ros::Publisher scanPub2;
    ros::Publisher scanPub3;
    ros::Publisher scanPub4;

    PointCloud cloud;
    LaserScan scan[4];
    float vSinLut[4];
    float vCosLut[4];

    ros::Time currTime;
    ros::Time prevTime;
    ros::Time recvTimePrev;
    bool firstPacket;


    //initialization
    void initPublisher();
    void initLut();
    void initROSMessage();

//    void unpackData(MessageScanData& data);
    void setTimeStamp(ros::Time recvTime);
    void makePointCloud(MessageScanData &data);
    void makeScan(MessageScanData &data);

    //auxilliary functions
    int computeTickFreq(MessageScanData& data);
    void polar2Cartesian(ScanPoint &ptPolar, Point3D &ptXYZ);
};

} // namespace sick_lidar

#endif
