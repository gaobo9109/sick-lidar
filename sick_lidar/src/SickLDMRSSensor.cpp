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
#include "SickLDMRSCommand.h"
#include "SickLDMRSROS.h"
#include "DebugUtil.h"
#include <string>
#include <QDebug>
#include <iostream>
#include <bitset>

using namespace std;


namespace sick_lidar {

SickLDMRSSensor::SickLDMRSSensor(QString ip, int port)
    : ipaddr_(ip),
      port_(port)
{   
    S_socket = new SickSocket(this);
    rosHandler = new SickLDMRSROS;
    connect(S_socket, SIGNAL(configuration()), this, SLOT(configure()) );
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
    S_socket->connectToServer(ipaddr_,port_);

}


void SickLDMRSSensor::closeDevice()
{
    stopScanner();
    S_socket->closeSocket();
}

void SickLDMRSSensor::startScanner()
{
    // Start measuring
    S_socket->sendToServer(SickLDMRSCommand::genSetCommand(COMMAND_START_MEASURE));
//    S_socket->sendToServer(SickLDMRSCommand::genGetParamCommand(COMMAND_GET_PARAM,0x1102));
    ROS_DEBUG("start measuring.");
}

void SickLDMRSSensor::stopScanner()
{
    S_socket->sendToServer(SickLDMRSCommand::genSetCommand(COMMAND_STOP_MEASURE));
}


int32_t SickLDMRSSensor::findMagicWord(const char * message, const unsigned length)
{
    if (length < 4){
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
    return *((uint16_t*)(message+magicWordIndex+14));
}


bool SickLDMRSSensor::isMessageComplete(const unsigned length, const long size)
{
    if (size <= length) {
        return true;
    }
    return false;
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
            qDebug() << "cannot find magic word";
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
//        DebugUtil::printArray(pendingBytes.data.c_str()+index+24,msgSize-index-24);

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

void SickLDMRSSensor::fillScanHeader(MessageLDMRS &msg, MessageScanData &scan)
{
    scan.header.scanNumber = *((uint16_t*)(msg.body));
    scan.header.scannerStatus = *((uint16_t*)(msg.body+2));
    scan.header.phaseOffset = *((uint16_t*)(msg.body+4));
    scan.header.startNtpTime = *((uint64_t*)(msg.body+6));
    scan.header.endNtpTime = *((uint64_t*)(msg.body+14));
    scan.header.ticksPerRot= *((uint16_t*)(msg.body+22)); // needed to compute angle (°)
    scan.header.startAngle = *((int16_t*)(msg.body+24));
    scan.header.endAngle = *((int16_t*)(msg.body+26));
    scan.header.numPoints = *((uint16_t*)(msg.body+28));

//    scan.header.mountingYawAngle = *((int16_t*)(msg.body+30));
//    scan.header.mountingPitchAngle = *((int16_t*)(msg.body+32));
//    scan.header.mountingRollAngle = *((int16_t*)(msg.body+34));
//    scan.header.mountingX = *((int16_t*)(msg.body+36));
//    scan.header.mountingY = *((int16_t*)(msg.body+38));
//    scan.header.mountingZ = *((int16_t*)(msg.body+40));
//    std::cout << scan.header.numPoints << std::endl;
//    std::cout << scan.header.mountingY << std::endl;
//    std::cout << scan.header.mountingZ << std::endl;
//    std::cout << " " << std::endl;

}

void SickLDMRSSensor::handleScanMessage(MessageLDMRS &msg)
{
    MessageScanData scan;
    scan.time = msg.time;

    fillScanHeader(msg,scan);
    int offset = 44;
    int pointSize = 10;

    int numPoints = scan.header.numPoints;
    if(numPoints > POINTS_MAX_SIZE)
    {
        ROS_FATAL("Cannot allocate memory for %d points",numPoints);
        return;
    }

    int count = 0;
    for(int i=0; i<numPoints; i++){

        uint8_t layerEcho = *((uint8_t*)(msg.body + i*pointSize + offset));
        scan.scanPoints[i].layer = layerEcho >> 4;
        scan.scanPoints[i].echo = layerEcho & 0x0F;
        scan.scanPoints[i].flags = *((uint8_t*)(msg.body + i*pointSize + offset + 1));
        scan.scanPoints[i].angle = *((int16_t*)(msg.body + i*pointSize + offset + 2));
        scan.scanPoints[i].distance = *((uint16_t*)(msg.body + i*pointSize + offset + 4));
        scan.scanPoints[i].echoPulseWidth = *((uint16_t*)(msg.body + i*pointSize + offset + 6));

        if((scan.scanPoints[i].distance) < 10)
        {
            count++;
            std::cout << scan.scanPoints[i].layer << std::endl;
            std::cout << scan.scanPoints[i].echo << std::endl;
            std::cout << scan.scanPoints[i].distance << std::endl;
            std::cout << scan.scanPoints[i].angle << std::endl;
        }


    }
    std::cout << "count: " << count << " total: " << numPoints << std::endl;
    rosHandler->processData(scan);

}


void SickLDMRSSensor::processMessage(MessageLDMRS &msg)
{ 
    if (msg.msgType == SICKLDMRS_SCANDATA_TYPE) {
        handleScanMessage(msg);
    }
    else if (msg.msgType == SICKLDMRS_ERROR_TYPE){
        //TODO

    }
    else if(msg.msgType == SICKLDMRS_REPLY_TYPE){
        //TODO
        qDebug() << "reply data";
//        DebugUtil::printArray(msg.body,8);

    }

}




void SickLDMRSSensor::configure(){
    startScanner();
}


void SickLDMRSSensor::customEvent(QEvent * e)
{   
    SickFrame * frame = ((SickFrameEvent*)e)->frame;
    splitPacket(frame->msg, frame->size, frame->time);
    delete frame;

    while ( !msgList.empty() )
    {
        MessageLDMRS msgToProcess = msgList.front();
        processMessage(msgToProcess);
        msgList.pop_front();
    }
}

}

