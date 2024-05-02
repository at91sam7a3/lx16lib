#include "lx16driver.h"
#include <iostream>
#include <stdint.h>
#include <vector>
#include <algorithm>
#include <exception>

#define GET_LOW_BYTE(A) (uint8_t)((A))
//Macro function  get lower 8 bits of A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//Macro function  get higher 8 bits of A
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//Macro Function  put A as higher 8 bits   B as lower 8 bits   which amalgamated into 16 bits integer

#define LOBOT_SERVO_FRAME_HEADER         0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE      1
#define LOBOT_SERVO_MOVE_TIME_READ       2
#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LOBOT_SERVO_MOVE_TIME_WAIT_READ  8
#define LOBOT_SERVO_MOVE_START           11
#define LOBOT_SERVO_MOVE_STOP            12
#define LOBOT_SERVO_ID_WRITE             13
#define LOBOT_SERVO_ID_READ              14
#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST  17
#define LOBOT_SERVO_ANGLE_OFFSET_WRITE   18
#define LOBOT_SERVO_ANGLE_OFFSET_READ    19
#define LOBOT_SERVO_ANGLE_LIMIT_WRITE    20
#define LOBOT_SERVO_ANGLE_LIMIT_READ     21
#define LOBOT_SERVO_VIN_LIMIT_WRITE      22
#define LOBOT_SERVO_VIN_LIMIT_READ       23
#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ  25
#define LOBOT_SERVO_TEMP_READ            26
#define LOBOT_SERVO_VIN_READ             27
#define LOBOT_SERVO_POS_READ             28
#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE  29
#define LOBOT_SERVO_OR_MOTOR_MODE_READ   30
#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ  32
#define LOBOT_SERVO_LED_CTRL_WRITE       33
#define LOBOT_SERVO_LED_CTRL_READ        34
#define LOBOT_SERVO_LED_ERROR_WRITE      35
#define LOBOT_SERVO_LED_ERROR_READ       36

struct CommandData
{
    char commandId;
    char packetSize;
};

std::vector<CommandData> packetsInfo = {
    {LOBOT_SERVO_MOVE_TIME_WRITE, 7},
    {LOBOT_SERVO_MOVE_TIME_READ, 3},
    {LOBOT_SERVO_MOVE_TIME_WAIT_WRITE, 7},
    {LOBOT_SERVO_MOVE_TIME_WAIT_READ, 3},
    {LOBOT_SERVO_MOVE_START, 3},
    {LOBOT_SERVO_MOVE_STOP, 3},
    {LOBOT_SERVO_ID_WRITE, 4},
    {LOBOT_SERVO_ID_READ, 3},
    {LOBOT_SERVO_ANGLE_OFFSET_ADJUST, 4},
    {LOBOT_SERVO_ANGLE_OFFSET_WRITE, 3},
    {LOBOT_SERVO_ANGLE_OFFSET_READ, 3},
    {LOBOT_SERVO_ANGLE_LIMIT_WRITE, 7},
    {LOBOT_SERVO_ANGLE_LIMIT_READ, 3},
    {LOBOT_SERVO_VIN_LIMIT_WRITE, 7},
    {LOBOT_SERVO_VIN_LIMIT_READ, 3},
    {LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE, 4},
    {LOBOT_SERVO_TEMP_MAX_LIMIT_READ, 3},
    {LOBOT_SERVO_TEMP_READ, 3},
    {LOBOT_SERVO_VIN_READ, 3},
    {LOBOT_SERVO_POS_READ, 3},
    {LOBOT_SERVO_OR_MOTOR_MODE_WRITE, 7},
    {LOBOT_SERVO_OR_MOTOR_MODE_READ, 3},
    {LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, 4},
    {LOBOT_SERVO_LOAD_OR_UNLOAD_READ, 3},
    {LOBOT_SERVO_LED_CTRL_WRITE, 4},
    {LOBOT_SERVO_LED_CTRL_READ, 3},
    {LOBOT_SERVO_LED_ERROR_WRITE, 4},
    {LOBOT_SERVO_LED_ERROR_READ, 3},
};

lx16driver::lx16driver(const char *device, bool loopFix)
    : m_operational(false), m_loopbackFix(loopFix)
{
    int Ret = m_handle.Open(device, 115200);
    if (Ret != 1)
    {
        std::cout << "Opening serial port " << device << " at 115200 " << std::endl;
        std::cerr << "ERROR: Comport not availabe!" << std::endl;
        return;
    }
    m_operational = true;
    m_handle.FlushReceiver();
}

lx16driver::~lx16driver()
{
    m_handle.Close();
}

bool lx16driver::isOperational()
{
    return m_operational;
}

void lx16driver::RevriteId(int id)
{
    MakePacket(LOBOT_SERVO_ID_WRITE, 254);
    m_buf[5] = id; // new id
    set8bitParam(id, 0);
    sendPacket();
}

int lx16driver::ReadId(void)
{
    return 0;
}

void lx16driver::ServoMoveTimeWrite(int id, int position, int time)
{
    position = std::clamp(position, 0, 1000);

    MakePacket(LOBOT_SERVO_MOVE_TIME_WRITE, id);
    set16bitParam(position, 0);
    set16bitParam(time, 1);
    sendPacket();
}

int lx16driver::ServoPositionRead(int id)
{
    MakePacket(LOBOT_SERVO_POS_READ, id);
    sendPacket();
    return readAnswer16bit();
}

int lx16driver::ServoVoltageRead(int id)
{
    MakePacket(LOBOT_SERVO_VIN_READ, id);
    sendPacket();
    return readAnswer16bit();
}

int lx16driver::ServoAdjustAngleGet(int id)
{
    MakePacket(LOBOT_SERVO_ANGLE_OFFSET_READ, id);
    sendPacket();
    return readAnswer16bit();
}

void lx16driver::ServoAdjustAngleSave(int id)
{
    MakePacket(LOBOT_SERVO_ANGLE_OFFSET_WRITE, id);
    sendPacket();
}

void lx16driver::ServoAdjustAngleSet(int id, char angle)
{

    MakePacket(LOBOT_SERVO_ANGLE_OFFSET_ADJUST, id);
    set8bitParam(angle, 0);
    sendPacket();
}

char lx16driver::LobotCheckSum()
{
    char i;
    uint16_t temp = 0;
    for (i = 2; i < m_buf[3] + 2; i++) {
        temp += m_buf[i];
    }
    temp = ~temp;
    i = (char)temp;
    return i;
}

void lx16driver::MakePacket(char command,int servoId)
{  
    m_buf[0] = m_buf[1] = LOBOT_SERVO_FRAME_HEADER;
    m_buf[2] = servoId;
    m_buf[3] = GetPacketSize(command);
    m_buf[4] = LOBOT_SERVO_ANGLE_OFFSET_ADJUST;
}

char lx16driver::GetPacketSize(char command)
{
    int requestSize = -1;
    for (const CommandData &cd : packetsInfo)
    {
        if (cd.commandId == command)
        {
            requestSize = cd.packetSize;
            break;
        }
    }
    if (requestSize == -1)
        throw(std::runtime_error("invalid packet id"));
    return requestSize;
}

void lx16driver::sendPacket()
{
    m_handle.FlushReceiver();
    int size=m_buf[3] +2;
    m_buf[size-1] = LobotCheckSum();
    m_handle.Write(m_buf,size);
    if(m_loopbackFix)
    {
        m_handle.Read(m_buf,size,32);
    }
}

void lx16driver::set8bitParam(char data, int idx)
{
     m_buf[5+idx] = data;
}

void lx16driver::set16bitParam(int data, int idx)
{
    m_buf[5+idx*2] = GET_LOW_BYTE(data);
    m_buf[6+idx*2] = GET_HIGH_BYTE(data);
}

char lx16driver::readAnswer8bit()
{
    //TODO
    return 0;
}

int lx16driver::readAnswer16bit()
{
    m_handle.Read(m_RxBuf,16,10);

    if((m_RxBuf[0]!=LOBOT_SERVO_FRAME_HEADER) || (m_RxBuf[1]!=LOBOT_SERVO_FRAME_HEADER)){
        for(size_t i=1;i<5;++i)
        {
            if((m_RxBuf[i]==LOBOT_SERVO_FRAME_HEADER) && (m_RxBuf[i+1]==LOBOT_SERVO_FRAME_HEADER))
            {
                memcpy(&m_RxBuf[0],&m_RxBuf[i],16-i);
                break;
            }
        }
    }

    char crc = LobotCheckSum(); 
    if(m_RxBuf[3]!=m_buf[3] || m_RxBuf[4]!=m_buf[4] ) //compare command id
    {
        std::cerr<<"Comminication error!"<<std::endl;
        return 0;
    }
    if(crc != m_RxBuf[7])
    {
        std::cerr<<"CRC error"<<std::endl;
        return 0;
    }
    // Close the connection with the device
    return BYTE_TO_HW(m_RxBuf[6], m_RxBuf[5]);
}
