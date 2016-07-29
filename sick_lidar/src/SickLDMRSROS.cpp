#include "SickLDMRSROS.h"
#include <math.h>  
#include <QDebug>
#include <iostream>
#include <string>

namespace sick_lidar{

float elevAngle[4] = {-1.2, -0.4, 0.4, 1.2};

SickLDMRSROS::SickLDMRSROS(){
    firstPacket = true;
    initPublisher();
	initLut();
    initROSMessage();
}

SickLDMRSROS::~SickLDMRSROS(){}

void SickLDMRSROS::initPublisher(){
    pclPub = nh.advertise<PointCloud>("/lidar/cloud",1);
    scanPub1 = nh.advertise<LaserScan>("lidar/scan1",1);
    scanPub2 = nh.advertise<LaserScan>("lidar/scan2",1);
    scanPub3 = nh.advertise<LaserScan>("lidar/scan3",1);
    scanPub4 = nh.advertise<LaserScan>("lidar/scan4",1);
}

void SickLDMRSROS::initLut(){
    for (int i=0; i<4; i++){
		vSinLut[i] = sin(elevAngle[i]*PI/180);
		vCosLut[i] = cos(elevAngle[i]*PI/180);
	}
}


void SickLDMRSROS::initROSMessage(){
    //for point cloud message
    cloud.header.frame_id = "map";

    //for laser scan message
    std::string frameID = "laser_frame";

    for(int i=0; i<4; i++){
        scan[i].header.frame_id = frameID + boost::lexical_cast<std::string>(i);
        scan[i].range_min = 0.01;
        scan[i].range_max = 10000;
    }

}

void SickLDMRSROS::setTimeStamp(ros::Time recvTime){

    if(firstPacket){
        currTime = recvTime;
        prevTime = recvTime;
        firstPacket = false;
    } else{
        prevTime = currTime;
        currTime = recvTime;
    }
}


void SickLDMRSROS::polar2Cartesian(ScanPoint &ptPolar, Point3D &ptXYZ){
    float radAngle = RAD_PER_TICK * ptPolar.angle;
    float range = ptPolar.distance/100.0;    //lidar output range in cm
    uint8_t layer = ptPolar.layer;
	ptXYZ.x = vCosLut[layer] * cos(radAngle) * range;
	ptXYZ.y = vCosLut[layer] * sin(radAngle) * range;
	ptXYZ.z = vSinLut[layer] * range;
	
}

void SickLDMRSROS::makePointCloud(MessageScanData &data){
    cloud.header.stamp = currTime.toSec();
    uint16_t numPoints = data.header.numPoints;

    cloud.height = 1;
    cloud.width = numPoints;

    for(int i=0; i<numPoints; i++)
    {
        ScanPoint ptPolar = data.scanPoints[i];
        Point3D ptXYZ;
        polar2Cartesian(ptPolar, ptXYZ);
        cloud.points.push_back(ptXYZ);
    }

    pclPub.publish(cloud);
    cloud.points.clear();
}

void SickLDMRSROS::makeScan(MessageScanData &data){
    uint16_t numPoints = data.header.numPoints;
    std::stable_sort(data.scanPoints,data.scanPoints+numPoints, compareEcho);
    std::stable_sort(data.scanPoints,data.scanPoints+numPoints, compareAngle);
    std::stable_sort(data.scanPoints,data.scanPoints+numPoints, compareLayer);

    int pointsPerScan = (data.header.startAngle - data.header.endAngle)/TICK_INDEX;
    float scanTime = (currTime - prevTime).toSec();
    float angleMin = data.header.startAngle * RAD_PER_TICK;
    float angleMax = data.header.endAngle * RAD_PER_TICK;
    float angleIncrement = -TICK_INDEX * RAD_PER_TICK;
    float timeIncrement = (float)TICK_INDEX / TICK_FREQUENCY;

    int currLayer = 0;
    int pointIndex = 0;

    for(int i=0; i<4; i++){
        scan[i].header.stamp = currTime;
        scan[i].scan_time = scanTime;
        scan[i].angle_min = angleMin;
        scan[i].angle_max = angleMax;
        scan[i].angle_increment = angleIncrement;
        scan[i].time_increment = timeIncrement;
        scan[i].ranges.resize(pointsPerScan);

        //slot range info into correct location in the array
        while(currLayer == i && pointIndex < numPoints){
            int rangeIndex = (data.header.startAngle - data.scanPoints[pointIndex].angle)/TICK_INDEX;
            scan[i].ranges[rangeIndex] = data.scanPoints[pointIndex].distance/100.0;
            pointIndex++;
            currLayer = data.scanPoints[pointIndex].layer;
        }
    }

    scanPub1.publish(scan[0]);
    scanPub2.publish(scan[1]);
    scanPub3.publish(scan[2]);
    scanPub4.publish(scan[3]);
}

void SickLDMRSROS::processData(MessageScanData &data){
    setTimeStamp(data.time);
    makePointCloud(data);
    makeScan(data);
}

//int SickLDMRSROS::computeTickFreq(MessageScanData& data){
//    uint64_t deltaT = data.header.endNtpTime - data.header.startNtpTime;
//    uint32_t deltaTick = data.header.startAngle - data.header.endAngle;
//    return (deltaTick << 32)/deltaT;
//}

bool compareEcho(const ScanPoint& pt1, const ScanPoint& pt2){
    return pt1.echo < pt2.echo;
}

bool compareAngle(const ScanPoint& pt1, const ScanPoint& pt2){
    return pt1.angle > pt2.angle;
}

bool compareLayer(const ScanPoint& pt1, const ScanPoint& pt2){
    return pt1.layer < pt2.layer;
}

}  
