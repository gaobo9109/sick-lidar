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
#include <ros/ros.h>
#include <iostream>
#include <QTcpSocket>
#include <string>

using namespace std;

// #define SICKLDMRS_SH_MEM

namespace pacpus {


static const long MagicWord = 0xAFFEC0C2; // Sick LDMRS


static const int SICKLDMRS_SCANDATA_TYPE    = 0x2202;
static const int SICKLDMRS_OBJECTDATA_TYPE  = 0x2221;


SickLDMRSSensor::SickLDMRSSensor()
    : ipaddr_("192.168.0.1"),
      port_(12002)
{
    S_socket = new SickSocket(this);
    connect(S_socket, SIGNAL(configuration()), this, SLOT(configure()) );
    pendingBytes.time = 0;
    pendingBytes.previousData = false;
}

SickLDMRSSensor::SickLDMRSSensor(QString ip, int port)
    : ipaddr_(ip),
      port_(port)
{   
    S_socket = new SickSocket(this);
    connect(S_socket, SIGNAL(configuration()), this, SLOT(configure()) );
    pendingBytes.time = 0;
    pendingBytes.previousData = false;

}


SickLDMRSSensor::~SickLDMRSSensor()
{
    delete S_socket;
}


void SickLDMRSSensor::startActivity()
{
    S_socket->connectToServer(ipaddr_, port_);
}


void SickLDMRSSensor::stopActivity()
{
    S_socket->sendToServer(QString((uint32_t)0x0021));
    S_socket->closeSocket();
}



uint32_t SickLDMRSSensor::findMagicWord(const char * message, const unsigned length)
{
    if (length < 4) {
        return -1;
    }

    unsigned long i = 0;
    while(*((uint32_t*)(message+i)) != 0xC2C0FEAF){ // BigE
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
    msg.hScan.scanNumber = *((uint16_t*)(msg.body+24));
    msg.hScan.scannerStatus = *((uint16_t*)(msg.body+24+2));
    msg.hScan.phaseOffset = *((uint16_t*)(msg.body+24+4));
    msg.hScan.startNtpTime = *((uint64_t*)(msg.body+24+6));
    msg.hScan.endNtpTime = *((uint64_t*)(msg.body+24+14));
    msg.hScan.ticksPerRot= *((uint16_t*)(msg.body+24+22)); // needed to compute angle (°)
    msg.hScan.startAngle = *((int16_t*)(msg.body+24+24));
    msg.hScan.endAngle = *((int16_t*)(msg.body+24+26));
    msg.hScan.numPoints = *((uint16_t*)(msg.body+24+28));

//    msg.hScan.mountingYawAngle = *((int16_t*)(msg.body+24+30));
//    msg.hScan.mountingPitchAngle = *((int16_t*)(msg.body+24+32));
//    msg.hScan.mountingRollAngle = *((int16_t*)(msg.body+24+34));
//    msg.hScan.mountingX = *((int16_t*)(msg.body+24+36));
//    msg.hScan.mountingY = *((int16_t*)(msg.body+24+38));
//    msg.hScan.mountingZ = *((int16_t*)(msg.body+24+40));
}


void SickLDMRSSensor::fillDataHeader(MessageLDMRS &msg)
{

    msg.hData.magicWord = ((*(msg.body+3))&0x000000FF) +
            ((*(msg.body+2)<<8)&0x0000FF00) +
            ((*(msg.body+1)<<16)&0x00FF0000)+
            ((*(msg.body)<<24)&0xFF000000);
    // TODO check if OK

    msg.hData.sizePreviousMessage = ((*(msg.body+7))&0x000000FF)+
            ((*(msg.body+6)<<8)&0x0000FF00)+
            ((*(msg.body+5)<<16)&0x00FF0000)+
            ((*(msg.body+4)<<24)&0xFF000000);

    msg.hData.sizeCurrentMessage =((*(msg.body+11))&0x000000FF)+
            ((*(msg.body+10)<<8)&0x0000FF00)+
            ((*(msg.body+9)<<16)&0x00FF0000)+
            ((*(msg.body+8)<<24)&0xFF000000);



    msg.hData.deviceId = *(msg.body+13);

    msg.hData.dataType =  ((*(msg.body+15))&0x000000FF)+((*(msg.body+14)<<8)&0x0000FF00);


    msg.hData.ntpTime = ((*(msg.body+15))&0x000000FF)+
            ((*(msg.body+14)<<8)&0x0000FF00)+
            ((*(msg.body+13)<<16)&0x00FF0000)+
            ((*(msg.body+12)<<24)&0xFF000000)+
            ((*(msg.body+11))&0x000000FF)+
            ((*(msg.body+10)<<8)&0x0000FF00)+
            ((*(msg.body+9)<<16)&0x00FF0000)+
            ((*(msg.body+8)<<24)&0xFF000000);
}


void SickLDMRSSensor::storePendingBytes(road_time_t time)
{
    if (!pendingBytes.previousData)
    {
        pendingBytes.time = time;
        pendingBytes.previousData = true;
    }
}


void SickLDMRSSensor:: splitPacket(const char * packet, const int length, road_time_t time)
{ 
    long index = 0;
    long msgSize = 0;
    bool msgComplete = false;

    // we are working on the previous not decoded data + the actual incoming packet
    pendingBytes.data.append(packet,length);


    while (pendingBytes.data.size() > 0)
    {
        // we are looking for the MagicWord
        index = findMagicWord(pendingBytes.data.c_str() , pendingBytes.data.size());
        if (index == -1)
        {
            storePendingBytes(time);
            // exit the while loop
            break;
        }


        // we are looking for the size of the message (scan data + scan points)
        msgSize = getMessageSize(pendingBytes.data.c_str(), pendingBytes.data.size(), index);
        if (msgSize == 0)
        {
            storePendingBytes(time);
            // exit the while loop
            break;
        }


        // we are verifying if the message is complete
        msgComplete = isMessageComplete( pendingBytes.data.size() , msgSize );
        if (msgComplete == false)
        {
            storePendingBytes(time);
            // exit the while loop
            break;
        }

        // we have a complete message available that we can add to the list
        MessageLDMRS msg;

        // we copy the bytes in the body message
//        char* messageData = (char*)malloc(msgSize);
//        if(messageData == NULL){
//            LOG_FATAL("(Packet reconstitution) Malloc FAILED. Packet lost.");
//            return;
//        }
        memcpy(msg.body, pendingBytes.data.c_str() + index, msgSize);

//        msg->body = messageData;

        // we set the timestamp of the message
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

        // we add the message to the list
        msgList.push_back(msg);
        // and we suppress the processed bytes of the pending data
        pendingBytes.data.erase(0, msgSize);
    }
}


unsigned long SickLDMRSSensor::processMessage(MessageLDMRS &msg)
{ 
    fillDataHeader(msg);
    if (msg.hData.dataType == SICKLDMRS_SCANDATA_TYPE) {
        ROS_DEBUG("(Process Message) Scan Data Type!");
        fillScanHeader(msg);

        int index = 24 + 44; // data header + scan header

        if(sizeof(ScanPoint) * msg.hScan.numPoints > BODY_MAX_SIZE){
            ROS_FATAL("Size of the message is too long !");
            return 0;
        }

        ScanPoint scanPoints[msg.hScan.numPoints];

        // replace memory with structured data
        for (int i = 0; i < msg.hScan.numPoints; ++i) {
            scanPoints[i].layerEcho = *((uint8_t*)(msg.body + index));
            scanPoints[i].flags = *((uint8_t*)(msg.body + index + 1));
            scanPoints[i].angle = *((uint16_t*)(msg.body + index + 2));
            scanPoints[i].distance = *((uint16_t*)(msg.body + index + 4));
            scanPoints[i].echoPulseWidth = *((uint16_t*)(msg.body + index + 6));
        }

        memcpy(msg.body, scanPoints, sizeof(ScanPoint) * msg.hScan.numPoints);
    }
    else if (msg.hData.dataType == SICKLDMRS_OBJECTDATA_TYPE){
        ROS_DEBUG("(Process Message) Object Data Type!");

        // TODO
    }
    else {// irrelevant data type
        // TODO
    }


    return msg.hData.dataType;
}




void SickLDMRSSensor::configure(){
    // Start measuring
    // S_socket->sendToServer(QString((uint32_t)0x0020));

    // ROS_DEBUG(this->name_ +" configured.");
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
//            setState(ComponentBase::MONITOR_OK);
            // write data on the disk
        }
        else if (type == SICKLDMRS_OBJECTDATA_TYPE)
        {
            // Handled via CAN bus ?
        }

        // removes the processed item of the list
        // free(msgList.front().body);
        msgList.pop_front();
    }
}

} // namespace pacpus
