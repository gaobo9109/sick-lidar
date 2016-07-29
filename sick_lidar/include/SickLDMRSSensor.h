/*********************************************************************
//  created:    2014/02/02 - 10:48
//  filename:   SickLDMRSSensor.h
//
//  author:     Cyril Fougeray
//              Copyright Heudiasyc UMR UTC/CNRS 6599
// 
//  version:    $Id: $
//
//  purpose:    Definition of the SickLDMRSSensor class
*********************************************************************/

#ifndef SICKLDMRSSENSOR_H
#define SICKLDMRSSENSOR_H


#include "SickLDMRSData.h"
#include "SickSocket.h"
#include <string>
#include <ros/ros.h>


#define BODY_MAX_SIZE   20000
#define POINTS_MAX_SIZE 2000

class QEvent;


namespace sick_lidar {

class SickLDMRSROS;

struct MessagePacket {
    ros::Time time;
    std::string data;
    bool previousData;
};

class MessageLDMRS
{
public:

    MessageLDMRS(){}
    ~MessageLDMRS(){}

    uint16_t msgType;
    char body[BODY_MAX_SIZE];
    ros::Time time;

};

class MessageScanData
{
public:
    MessageScanData(){}
    ~MessageScanData(){}

    ScanHeader header;
    ScanPoint scanPoints[POINTS_MAX_SIZE];
    ros::Time time;

};


class SickLDMRSSensor : QObject
{
   Q_OBJECT

public:

    SickLDMRSSensor(QString ip, int port);
    ~SickLDMRSSensor();
    void closeDevice();
    void initDevice();
    void startScanner();
    void stopScanner();

public Q_SLOTS:  

    void customEvent(QEvent * e);
    void configure();
    
public:

    SickSocket * S_socket;
    SickLDMRSROS * rosHandler;

private:
    QString ipaddr_;
    int port_;
    std::list<MessageLDMRS> msgList;
    MessagePacket pendingBytes;

    void storePendingBytes(ros::Time time);
    void fillScanHeader(MessageLDMRS& msg, MessageScanData& scan);
    void splitPacket(const char * packet, const int length, ros::Time time);
    void handleScanMessage(MessageLDMRS &msg);
    void processMessage(MessageLDMRS & msg);
    int32_t findMagicWord(const char * message, const unsigned length);
    uint32_t getMessageSize(const char * message, const unsigned length, const long magicWordIndex);
    uint16_t getMessageType(const char * message, const long magicWordIndex);
    bool isMessageComplete(const unsigned length, const long size);

};

}

#endif // SICKLDMRSSENSOR_H
