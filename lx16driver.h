#pragma once
#include "serialib.h"

/*This class provides interface to work with Lewansoul LX-16A servos*/

class lx16driver
{
public:
    /*!
    lx16driver constructor
    deviceUrl - lile "/dev/ttyUSB0"
    loopbackFix - true is rx connected to Tx, 
                  false if cpecial board used
    */
    lx16driver(const char* deviceUrl, bool loopbackFix);
    ~lx16driver();
    //isOperationsl shows - was serial port opened successfully
    //has to be checked at start
    bool isOperational();
    /// @brief Set new ID to ALL CONNECTED SERVOS
    /// @param id - new ID to set
    void RevriteId(int id);
    void ServoMoveTimeWrite(int id, int angle, int moveTime);
    void ServoAdjustAngleSet(int id, char angle);
    ///
    void ServoAdjustAngleSave(int id);
    /// @brief Reads servo current angle
    /// @param id 
    /// @return servo angle in format 0-1000
    int ServoPositionRead(int id);
    /// @brief read voltage from servo
    /// @param id - servo id
    /// @return voltage as int in format xxxx , like 8132 for 8.132V
    int ServoVoltageRead(int id);
    int ServoAdjustAngleGet(int id);
    void SetAngleLimits(int id, int min, int max);
    char GetServoErrorStatus(int id);
    std::pair<int,int> GetAngleLimits(int id);
    void setDebugLogs(bool enable);
    
private:
    char LobotCheckSum(const char * buf);
    void MakePacket(const char command,const int servoId);
    char GetPacketSize(char command);
    void sendPacket();
    void set8bitParam(char data, int idx);
    void set16bitParam(int data, int idx);
    char readAnswer8bit();
    int readAnswerBase();
    int readAnswer();
    std::pair<int,int> readAnswerPair8bit();

private:
    bool m_loopbackFix;
    serialib m_handle;
    bool m_operational;
    char m_buf[16];
    char m_RxBuf[100];
    bool m_logsEnabled;
};

