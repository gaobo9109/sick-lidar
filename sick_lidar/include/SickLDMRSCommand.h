#ifndef __SICKLDMRS_COMMAND_H__
#define __SICKLDMRS_COMMAND_H__

#include "SickLDMRSData.h"
#include <QDebug>

namespace sick_lidar
{

class SickLDMRSCommand
{
public:
    static QByteArray genSetCommand(uint16_t commandID);
    static QByteArray genGetParamCommand(uint16_t commandID, uint16_t commandIndex);
    static QByteArray genSetParamCommand(uint16_t commandID,uint16_t commandIndex, uint32_t commandParam);

private:

    //represent length in big E
    static const uint32_t COMMAND_SET_LENGTH = 0x04000000;
    static const uint32_t COMMAND_SET_PARAM_LENGTH = 0x0A000000;
    static const uint32_t COMMAND_GET_PARAM_LENGTH = 0x06000000;

};

}

#endif
