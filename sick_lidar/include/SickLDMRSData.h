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

#include "cstdint.h"
#include "road_time.h"

// Export macro for SickLDMRS DLL for Windows only
#ifdef WIN32
#   ifdef SICKLDMRS_EXPORTS
        // make DLL
#       define SICKLDMRS_API __declspec(dllexport)
#   else
        // use DLL
#       define SICKLDMRS_API __declspec(dllimport)
#   endif
#else
    // On other platforms, simply ignore this
#   define SICKLDMRS_API
#endif

namespace pacpus{

/*!
 * \brief The DataHeader struct
 *
 * The DataHeader struct describes general information about the message used with.
 * On Sick LDMRS, DataHeader corresponds exactly to the very first data carried into the whole message.
 * See [Ethernet data protocol LD-MRS, page 4](docs/BAMessdatenProtokoll_LDMRSen_8014492_20110601.pdf).
 * > Warning : the data from the sensor is coded in Big Endian format.
 */
struct DataHeader {
    uint32_t magicWord;            //!< 0xAFFEC0C2 for the Sick LDMRS sensor (this value must be found in order to decode the message).
    uint32_t sizePreviousMessage;  //!< Size in bytes of the previous message.
    uint32_t sizeCurrentMessage;   //!< Size of the message content without the header (DataHeader).
    // u_int8_t reserved
    uint8_t deviceId;              //!< Unused in data received directly from LD-MRS sensors.
    uint16_t dataType;             //!< Type of information carried into the message.
                                    ///< Types used are :
                                    ///<     - Points : 0x2202
                                    ///<     - Objects : 0x2221
    uint64_t ntpTime;              //!< Time of the sensor when the message is created
};


/*!
 * \brief The ScanHeader struct
 *
 * General information about points measured.
 * Data type is 0x2202
 * @see DataHeader
 *
 * see Ethernet data protocol LD-MRS page 5
 */
struct SICKLDMRS_API ScanHeader {
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
//    int16_t mountingYawAngle;
//    int16_t mountingPitchAngle;
//    int16_t mountingRollAngle;
//    int16_t mountingX;
//    int16_t mountingY;
//    int16_t mountingZ;

    // u_int16_t reserved;

};


/*!
 * \brief The ScanPoint struct
 *
 * Used to describe a point.
 * Data type 0x2202 @see DataHeader
 */
struct SICKLDMRS_API ScanPoint{
    uint8_t layerEcho;           //!< 4 LSB : Layer (scan layer of the point)
                                //!< 4 MSB : Echo
    uint8_t flags;
    uint16_t angle;            //!< Angle in number of ticks. You can easily compute the real angle :
                                //!< \f$ angle (degree) = \frac{angle (ticks)}{ScanHeader.ticksPerRot}\f$ @see ScanHeader

    uint16_t distance;         //!< Distance of the point from the sensor in centimeters.
    uint16_t echoPulseWidth;   //!< Width of echo pulse (cm)
    // u_int16_t reserved;
};

/*!
 * \brief The ScanObject struct (not used)
 *
 * Used to describe an object.
 * Data type 0x2221 @see DataHeader
 */
struct SICKLDMRS_API ScanObject{
    // TODO
};





}

#endif
