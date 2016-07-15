#include "SickLDMRSCommand.h"
#include <algorithm>
#include <memory.h>
#include <cstring>
#include <QDebug>

namespace sick_lidar
{

    QByteArray SickLDMRSCommand::genSetCommand(uint16_t commandID)
    {
        DataHeader header = {MAGICWORD,0,COMMAND_SET_LENGTH,0,0,SICKLDMRS_COMMAND_TYPE,0};
        SetCommand cmd = {commandID,0};

        char buffer[sizeof(DataHeader)+sizeof(SetCommand)];
        std::memcpy(buffer, &header, sizeof(DataHeader));
        std::memcpy(buffer+sizeof(DataHeader), &cmd, sizeof(SetCommand));
        return QByteArray(buffer,sizeof(buffer));
    }

    QByteArray SickLDMRSCommand::genGetParamCommand(uint16_t commandID,uint16_t commandIndex)
    {
        DataHeader header = {MAGICWORD,0,COMMAND_GET_PARAM_LENGTH,0,0,SICKLDMRS_COMMAND_TYPE,0};
        GetParamCommand cmd = {commandID,0,commandIndex};

        char *buffer = new char[sizeof(DataHeader)+sizeof(GetParamCommand)];
        std::memcpy(buffer, &header, sizeof(DataHeader));
        std::memcpy(buffer+sizeof(DataHeader), &cmd, sizeof(GetParamCommand));
        return QByteArray(buffer,sizeof(buffer));
    }

    QByteArray SickLDMRSCommand::genSetParamCommand(uint16_t commandID,uint16_t commandIndex,uint32_t commandParam)
    {
        DataHeader header = {MAGICWORD,0,COMMAND_SET_PARAM_LENGTH,0,0,SICKLDMRS_COMMAND_TYPE,0};
        SetParamCommand cmd = {commandID,0,commandIndex,commandParam};

        char *buffer = new char[sizeof(DataHeader)+sizeof(SetParamCommand)];
        std::memcpy(buffer, &header, sizeof(DataHeader));
        std::memcpy(buffer+sizeof(DataHeader), &cmd, sizeof(SetParamCommand));
        return QByteArray(buffer,sizeof(buffer));
    }

}
