/*********************************************************************
//  created:    2014/02/02 - 12:08
//  filename:   SickLDMRSSensor.cpp
//
//  author:     Cyril Fougeray
//              Copyright Heudiasyc UMR UTC/CNRS 6599
// 
//  version:    $Id: $
//
//  purpose:    The acquisition class of the SickLDMRS sensor
//
*********************************************************************/

#include "SickLDMRSSensor.h"
#include "SickSocket.h"
#include "SickLDMRSROS.h"
#include "SickLDMRSData.h"
#include "SickLDMRSCommand.h"
#include <QTcpSocket>
#include <string>
#include <QDebug>

using namespace std;


namespace sick_lidar {

SickLDMRSSensor::SickLDMRSSensor(QString ip, int port)
    : ipaddr_(ip),
      port_(port)
{   
    S_socket = new SickSocket(this);
    rosHandler = new SickLDMRSROS;
    connect(S_socket, SIGNAL(configuration()), this, SLOT(configure()) );
    pendingBytes.time = ros::Time::now();
    pendingBytes.previousData = false;
    initDevice();

}


SickLDMRSSensor::~SickLDMRSSensor()
{
    delete S_socket;
    delete rosHandler;
}


void SickLDMRSSensor::initDevice()
{
    S_socket->connectToServer(ipaddr_, port_);
}


void SickLDMRSSensor::closeDevice()
{

    S_socket->closeSocket();
}

void SickLDMRSSensor::startScanner()
{
    // Start measuring
    S_socket->sendToServer(SickLDMRSCommand::genSetCommand(COMMAND_START_MEASURE));
    ROS_DEBUG("start measuring.");
}

void SickLDMRSSensor::stopScanner()
{
    S_socket->sendToServer(SickLDMRSCommand::genSetCommand(COMMAND_STOP_MEASURE));
}


int32_t SickLDMRSSensor::findMagicWord(const char * message, const unsigned length)
{
    if (length < 4) {
        return -1;
    }

    int32_t i = 0;

    while(*((uint32_t*)(message+i)) != MAGICWORD){ // BigE
        if (i == length) {
            return -1;
        }
        ++i;

    }

    return i;

}


uint32_t SickLDMRSSensor::getMessageSize(const char * message, const unsigned length, const long magicWordIndex)
{

    // we need at least 12 bytes
    if (length < 12) {
        return 0;
    }
    return ((*(message+magicWordIndex+11))&0x000000FF)
            + ((*(message+magicWordIndex+10)<<8)&0x0000FF00)
            + ((*(message+magicWordIndex+9)<<16)&0x00FF0000)
            + ((*(message+magicWordIndex+8)<<24)&0xFF000000)
            + 24 ;
}

uint16_t SickLDMRSSensor::getMessageType(const char * message, const long magicWordIndex)
{
    return *(message+magicWordIndex);
}


bool SickLDMRSSensor::isMessageComplete(const unsigned length, const long size)
{
    if (size <= length) {
        return true;
    }
    return false;
}


void SickLDMRSSensor::fillScanHeader( MessageLDMRS &msg )
{
    // address = base + Data header size (24-byte long) + offset
//    msg.hScan.scanNumber = *((uint16_t*)(msg.body+24));
//    msg.hScan.scannerStatus = *((uint16_t*)(msg.body+24+2));
//    msg.hScan.phaseOffset = *((uint16_t*)(msg.body+24+4));
//    msg.hScan.startNtpTime = *((uint64_t*)(msg.body+24+6));
//    msg.hScan.endNtpTime = *((uint64_t*)(msg.body+24+14));
//    msg.hScan.ticksPerRot= *((uint16_t*)(msg.body+24+22)); // needed to compute angle (°)
//    msg.hScan.startAngle = *((int16_t*)(msg.body+24+24));
//    msg.hScan.endAngle = *((int16_t*)(msg.body+24+26));
//    msg.hScan.numPoints = *((uint16_t*)(msg.body+24+28));

//    msg.hScan.mountingYawAngle = *((int16_t*)(msg.body+24+30));
//    msg.hScan.mountingPitchAngle = *((int16_t*)(msg.body+24+32));
//    msg.hScan.mountingRollAngle = *((int16_t*)(msg.body+24+34));
//    msg.hScan.mountingX = *((int16_t*)(msg.body+24+36));
//    msg.hScan.mountingY = *((int16_t*)(msg.body+24+38));
//    msg.hScan.mountingZ = *((int16_t*)(msg.body+24+40));
}



void SickLDMRSSensor::storePendingBytes(ros::Time time)
{
    if (!pendingBytes.previousData)
    {
        pendingBytes.time = time;
        pendingBytes.previousData = true;
    }
}


void SickLDMRSSensor:: splitPacket(const char * packet, const int length, ros::Time time)
{ 
    int32_t index = 0;
    long msgSize = 0;
    bool msgComplete = false;
    uint16_t msgType = 0;

    pendingBytes.data.append(packet,length);

    while (pendingBytes.data.size() > 0)
    {
        index = findMagicWord(pendingBytes.data.c_str() , pendingBytes.data.size());

        if (index == -1)
        {
            storePendingBytes(time);
            break;
        }

        // we are looking for the size of the message (scan data + scan points)
        msgSize = getMessageSize(pendingBytes.data.c_str(), pendingBytes.data.size(), index);
        if (msgSize == 0)
        {
            storePendingBytes(time);
            break;
        }

        // we are verifying if the message is complete
        msgComplete = isMessageComplete( pendingBytes.data.size() , msgSize );
        if (msgComplete == false)
        {
            storePendingBytes(time);
            break;
        }

        MessageLDMRS msg;

        msgType = getMessageType(pendingBytes.data.c_str(),index);
        msg.msgType = msgType;
        memcpy(msg.body, pendingBytes.data.c_str() + index + 24, msgSize);

        if (pendingBytes.previousData)
        {
            // the timestamp is the one of the previous packet
            msg.time = pendingBytes.time;
            pendingBytes.previousData = false;
        }
        else
        {
            // the timestamp is the one of the actual received packet
            msg.time = time;
        }

        msgList.push_back(msg);
        pendingBytes.data.erase(0, msgSize);
    }
}


unsigned long SickLDMRSSensor::processMessage(MessageLDMRS &msg)
{ 
//    if (msg.msgType == SICKLDMRS_SCANDATA_TYPE) {
//        ROS_DEBUG("(Process Message) Scan Data Type!");
//        fillScanHeader(msg);

//        int index = 24 + 44; // data header + scan header

//        if(sizeof(ScanPoint) * msg.hScan.numPoints > BODY_MAX_SIZE){
//            ROS_FATAL("Size of the message is too long !");
//            return 0;
//        }

//        ScanPoint scanPoints[msg.hScan.numPoints];

//        // replace memory with structured data
//        for (int i = 0; i < msg.hScan.numPoints; ++i) {
//            scanPoints[i].layerEcho = *((uint8_t*)(msg.body + index));
//            scanPoints[i].flags = *((uint8_t*)(msg.body + index + 1));
//            scanPoints[i].angle = *((uint16_t*)(msg.body + index + 2));
//            scanPoints[i].distance = *((uint16_t*)(msg.body + index + 4));
//            scanPoints[i].echoPulseWidth = *((uint16_t*)(msg.body + index + 6));
//        }

//        memcpy(msg.body, scanPoints, sizeof(ScanPoint) * msg.hScan.numPoints);
//    }
//    else if (msg.hData.dataType == SICKLDMRS_OBJECTDATA_TYPE){
//        ROS_DEBUG("(Process Message) Object Data Type!");

//        // TODO
//    }
//    else {// irrelevant data type
//        // TODO
//    }


//    return msg.hData.dataType;
}




void SickLDMRSSensor::configure(){

}


void SickLDMRSSensor::customEvent(QEvent * e)
{   
    SickFrame * frame = ((SickFrameEvent*)e)->frame;

    // we try to find some messages in the current packet + the pending bytes of the previous incoming data
    splitPacket(frame->msg, frame->size, frame->time);

    // we delete the heap variable
    delete frame;

    // we test if we have some messages to decode
    while ( !msgList.empty() )
    {
        ROS_DEBUG("Message waiting");
        // get the first (the eldest) message and process it
        MessageLDMRS msgToProcess = msgList.front();
        unsigned long type = processMessage(msgToProcess);
        ROS_DEBUG("Message processed !");

        if (type == SICKLDMRS_SCANDATA_TYPE)
        {
            rosHandler->makePointCloud(&msgToProcess);
        }
        else if (type == SICKLDMRS_OBJECTDATA_TYPE)
        {
            // Handled via CAN bus ?
        }

        msgList.pop_front();
    }
}

} // namespace pacpus

