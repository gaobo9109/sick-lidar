#include "SickLDMRSCommand.h"
#include <algorithm>
#include <memory.h>
#include <cstring>

namespace sick_lidar
{
    void SickLDMRSCommand::genMessageHeader(int messageLength, DataHeader *header)
    {
        header->magicWord = MAGICWORD;
        header->sizePreviousMessage = 0x0;
        header->sizeCurrentMessage = messageLength;
        header->reserved = 0x0;
        header->deviceId = 0x0;
        header->dataType = SICKLDMRS_COMMAND_TYPE;
        header->ntpTime = 0x0;
    }

    char* SickLDMRSCommand::genSetCommand(uint16_t commandID)
    {
        DataHeader header;
        genMessageHeader(COMMAND_SET_LENGTH,&header);
        SetCommand cmd;
        cmd.commandID = commandID;
        cmd.reserved = 0x0;

        char *buffer = new char[sizeof(DataHeader)+sizeof(SetCommand)];
        std::memcpy(buffer, &header, sizeof(DataHeader));
        std::memcpy(buffer+sizeof(DataHeader), &cmd, sizeof(SetCommand));
        return buffer;
    }

    char* SickLDMRSCommand::genGetParamCommand(uint16_t commandID,uint16_t commandIndex)
    {
        DataHeader header;
        genMessageHeader(COMMAND_GET_PARAM_LENGTH,&header);
        GetParamCommand cmd;
        cmd.commandID = commandID;
        cmd.reserved = 0x0;
        cmd.commandIndex = commandIndex;

        char *buffer = new char[sizeof(DataHeader)+sizeof(GetParamCommand)];
        std::memcpy(buffer, &header, sizeof(DataHeader));
        std::memcpy(buffer+sizeof(DataHeader), &cmd, sizeof(GetParamCommand));
        return buffer;
    }

    char* SickLDMRSCommand::genSetParamCommand(uint16_t commandID,uint16_t commandIndex,uint32_t commandParam)
    {
        DataHeader header;
        genMessageHeader(COMMAND_SET_PARAM_LENGTH,&header);
        SetParamCommand cmd;
        cmd.commandID = commandID;
        cmd.reserved = 0x0;
        cmd.commandIndex = commandIndex;
        cmd.param = commandParam;

        char *buffer = new char[sizeof(DataHeader)+sizeof(SetParamCommand)];
        std::memcpy(buffer, &header, sizeof(DataHeader));
        std::memcpy(buffer+sizeof(DataHeader), &cmd, sizeof(SetParamCommand));
        return buffer;
    }

}
