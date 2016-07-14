#include "SickLDMRSData.h"
#include "SickLDMRSSensor.h"
#include "SickLDMRSROS.h"
#include <math.h>  

#define PI 3.14159265


namespace sick_lidar{

float elevAngle[4] = {-1.2, -0.4, 0.4, 1.2};

SickLDMRSROS::SickLDMRSROS()
{
	pclPub = nh.advertise<PointCloud>("/lidar/cloud",1); 
	initLut();
}

SickLDMRSROS::~SickLDMRSROS()
{
	
}

void SickLDMRSROS::initLut(){
	for (int i=0; i<4; i++){
		vSinLut[i] = sin(elevAngle[i]*PI/180);
		vCosLut[i] = cos(elevAngle[i]*PI/180);
	}
}

void SickLDMRSROS::polar2Cartesian(ScanPoint &ptPolar, pcl::PointXYZ &ptXYZ, float radPerTick)
{
	float radAngle = radPerTick * ptPolar.angle;
	uint16_t range = ptPolar.distance;
	uint8_t layer = ptPolar.layerEcho >> 4;
	ptXYZ.x = vCosLut[layer] * cos(radAngle) * range;
	ptXYZ.y = vCosLut[layer] * sin(radAngle) * range;
	ptXYZ.z = vSinLut[layer] * range;
	
}

void SickLDMRSROS::makePointCloud(MessageLDMRS *message)
{
//	cloud.header.stamp = ros::Time::now().toNSec();
//	cloud.header.frame_id = "Point_cloud";
//	uint16_t numPoints = message->hScan.numPoints;
//	float radPerTick = 2*PI/message->hScan.ticksPerRot;

//	cloud.height = 1;
//	cloud.width = numPoints;

//	for(int i=0; i<numPoints; i++)
//	{
//		ScanPoint ptPolar = *(ScanPoint*)(message->body + i*sizeof(ScanPoint));
//		pcl::PointXYZ ptXYZ;
//		polar2Cartesian(ptPolar, ptXYZ, radPerTick);
//		cloud.points.push_back(ptXYZ);
//	}

//	pclPub.publish(cloud);
//	cloud.points.clear();
}


}
