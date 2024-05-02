#ifndef LX16DRIVER_H
#define LX16DRIVER_H
#include "serialib.h"


class lx16driver
{
public:
    /*!
    lx16driver constructor
    deviceUrl - lile "/dev/ttyUSB0"
    loopbackFix - true is rx connectedToTx, 
                  false if cpecial board used
    */
    lx16driver(const char* deviceUrl, bool loopbackFix);
    ~lx16driver();
    //isOperationsl shows - was serial port opened successfully
    //has to be checked at start
    bool isOperational();
    void RevriteId(int id);
    void ServoMoveTimeWrite(int id, int angle, int moveTime);
    void ServoAdjustAngleSet(int id, char angle);
    void ServoAdjustAngleSave(int id);
    int ReadId(void);
    int ServoPositionRead(int id);
    int ServoVoltageRead(int id);
    int ServoAdjustAngleGet(int id);

private:
    char LobotCheckSum();
    void MakePacket(char command, int servoId);
    char GetPacketSize(char command);
    void sendPacket();
    void set8bitParam(char data, int idx);
    void set16bitParam(int data, int idx);
    char readAnswer8bit();
    int readAnswer16bit();

private:
    bool m_loopbackFix;
    serialib m_handle;
    bool m_operational;
    char m_buf[16];
    char m_RxBuf[100];
};

#endif // LX16DRIVER_H
