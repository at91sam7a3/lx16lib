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
    bool isOperational();
    void RevriteId(int id);
    void ServoMoveTimeWrite(int id, int angle, int moveTime);
    int ServoPositionRead(int id);
    int ServoVoltageRead(int id);
    int ServoAdjustAngleGet(int id);
    void ServoAdjustAngleSet(int id, char angle);
    void ServoAdjustAngleSave(int id);
private:
    char LobotCheckSum(char buf[]);
private:
    bool m_loopbackFix;
    serialib handle;
    bool operational;
};

#endif // LX16DRIVER_H
