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
#include "DebugUtil.h"
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
    pendingBytes.time = 0;
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
    S_socket->sendToServer(SickLDMRSCommand::genSetCommand(COMMAND_GET_PARAM));
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
    return *(message+magicWordIndex+14);
}


bool SickLDMRSSensor::isMessageComplete(const unsigned length, const long size)
{
    if (size <= length) {
        return true;
    }
    return false;
}


void SickLDMRSSensor::storePendingBytes(uint32_t time)
{
    if (!pendingBytes.previousData)
    {
        pendingBytes.time = time;
        pendingBytes.previousData = true;
    }
}


void SickLDMRSSensor:: splitPacket(const char * packet, const int length, uint32_t time)
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

        qDebug() << "message output: ";
        DebugUtil::printArray(pendingBytes.data.c_str(),pendingBytes.data.size());
        qDebug() << "string size" << pendingBytes.data.size();
        qDebug() << "message size" << msgSize;
        qDebug() << " ";
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

void SickLDMRSSensor::handleScanMessage(MessageLDMRS &msg)
{
    MessageScanData scan;
    scan.time = msg.time;

    memcpy(&scan.header, msg.body,sizeof(ScanHeader));
    int numPoints = scan.header.scanNumber;

    for(int i=0; i<numPoints; i++){
        ScanPoint point;
        memcpy(&point, msg.body+sizeof(ScanHeader)+i*sizeof(ScanPoint),sizeof(ScanPoint));
        scan.scanPoints.push_back(point);
    }

    rosHandler->publishData(scan);

}


void SickLDMRSSensor::processMessage(MessageLDMRS &msg)
{ 
    if (msg.msgType == SICKLDMRS_SCANDATA_TYPE) {
        handleScanMessage(msg);
        qDebug() << "scan data";
    }
    else if (msg.msgType == SICKLDMRS_ERROR_TYPE){
        //TODO

    }
    else if(msg.msgType == SICKLDMRS_REPLY_TYPE){
        //TODO
        qDebug() << "reply data";
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

