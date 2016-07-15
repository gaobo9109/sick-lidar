/*********************************************************************
//  created:    2014/02/11 - 12:08
//  filename:   SickLDMRSData.h
//
//  author:     Cyril Fougeray
//              Copyright Heudiasyc UMR UTC/CNRS 6599
//
//  version:    $Id: $
//
//  purpose:    Structures to store Sick LDMRS data
//
*********************************************************************/

#ifndef __SICKLDMRS_DATA_H__
#define __SICKLDMRS_DATA_H__

#include "stdint.h"

namespace sick_lidar{

static const uint32_t MAGICWORD = 0xC2C0FEAF; // Big E

/*!
 * Sick LD-MRS Data Type, Big E
 */

static const uint16_t SICKLDMRS_SCANDATA_TYPE    = 0x0222;
static const uint16_t SICKLDMRS_OBJECTDATA_TYPE  = 0x2122;
static const uint16_t SICKLDMRS_COMMAND_TYPE = 0x1020;
static const uint16_t SICKLDMRS_ERROR_TYPE = 0x3020;
static const uint16_t SICKLDMRS_REPLY_TYPE = 0x2020;

/*!
 * Sick LD_MRS command ID, Little E
 */

static const uint16_t COMMAND_RESET = 0x0000;
static const uint16_t COMMAND_GET_STATUS = 0x0001;
static const uint16_t COMMAND_SAVE_CONF = 0x0004;
static const uint16_t COMMAND_SET_PARAM = 0x0010;
static const uint16_t COMMAND_GET_PARAM = 0x0011;
static const uint16_t COMMAND_RESET_PARAM = 0x001A;
static const uint16_t COMMAND_START_MEASURE = 0x0020;
static const uint16_t COMMAND_STOP_MEASURE = 0x0021;

/*!
 * Sick LD_MRS parameter index, Little E
 * TODO
 */


struct DataHeader {
    uint32_t magicWord;            //!< 0xAFFEC0C2 for the Sick LDMRS sensor (this value must be found in order to decode the message).
    uint32_t sizePreviousMessage;  //!< Size in bytes of the previous message.
    uint32_t sizeCurrentMessage;   //!< Size of the message content without the header (DataHeader).
    uint8_t reserved;
    uint8_t deviceId;              //!< Unused in data received directly from LD-MRS sensors.
    uint16_t dataType;             //!< Type of information carried into the message.
                                    ///< Types used are :
                                    ///<     - Points : 0x2202
                                    ///<     - Objects : 0x2221
    uint64_t ntpTime;              //!< Time of the sensor when the message is created
};


struct ScanHeader {
    uint16_t scanNumber;       //!< Number of the scan since the sensor started measuring.
    uint16_t scannerStatus;    //!< Status of the scanner
                                /**<
                                 * - 0x0007: reserved,
                                 * - 0x0008: set frequency reached,
                                 * - 0x0010: external sync signal detected,
                                 * - 0x0020: sync ok,
                                 * - 0x0040: sync master (instead of slave),
                                 * - 0xFF80: reserved
                                 */
    uint16_t phaseOffset;  ///<
    uint64_t startNtpTime; //!< NTP time first measurement
    uint64_t endNtpTime;   //!< NTP time last measurement
    uint16_t ticksPerRot;  //!< Angle ticks per rotation (used to compute the real angle of a point)
    int16_t startAngle;     //!< Angle of the first measured value
    int16_t endAngle;       //!< Angle of the last measured value
    uint16_t numPoints;    //!< Number of scanned points during this scan @see ScanPoint

    // mounting position; reference ?
    int16_t mountingYawAngle;
    int16_t mountingPitchAngle;
    int16_t mountingRollAngle;
    int16_t mountingX;
    int16_t mountingY;
    int16_t mountingZ;

    uint16_t reserved;

};


struct ScanPoint{
    uint8_t layerEcho;           //!< 4 LSB : Layer (scan layer of the point)
                                //!< 4 MSB : Echo
    uint8_t flags;
    uint16_t angle;            //!< Angle in number of ticks. You can easily compute the real angle :
                                //!< \f$ angle (degree) = \frac{angle (ticks)}{ScanHeader.ticksPerRot}\f$ @see ScanHeader

    uint16_t distance;         //!< Distance of the point from the sensor in centimeters.
    uint16_t echoPulseWidth;   //!< Width of echo pulse (cm)
    uint16_t reserved;
};


struct SetCommand{
    uint16_t commandID;
    uint16_t reserved;
};

struct SetParamCommand{
    uint16_t commandID;
    uint16_t reserved;
    uint16_t commandIndex;
    uint32_t param;
};

struct GetParamCommand{
    uint16_t commandID;
    uint16_t reserved;
    uint16_t commandIndex;
};



}

#endif
