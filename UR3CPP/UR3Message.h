/**
    05Robot - Computer Vision Module

    @author P.Kaczmarek,J.Tomczynski,T.Mankowski
    @version 1.0 2017
*/
#ifndef UR3MESSAGE_H
#define UR3MESSAGE_H

//movej; sppedc; force

#pragma once
#include <vector>
#include <inttypes.h>
#include <QVector>
#include <stdlib.h>
#include <string.h>

/// @cond NO_DOXY

#define bswap_16(value) \
((((value) & 0xff) << 8) | ((value) >> 8))

#define bswap_32(value) \
(((uint32_t)bswap_16((uint16_t)((value) & 0xffff)) << 16) | \
(uint32_t)bswap_16((uint16_t)((value) >> 16)))

#define bswap_64(value) \
(((uint64_t)bswap_32((uint32_t)((value) & 0xffffffff)) \
<< 32) | \
(uint64_t)bswap_32((uint32_t)((value) >> 32)))


enum RobotMode {
        ROBOT_MODE_DISCONNECTED = 0,
        ROBOT_MODE_CONFIRM_SAFETY = 1,
        ROBOT_MODE_BOOTING = 2,
        ROBOT_MODE_POWER_OFF = 3,
        ROBOT_MODE_POWER_ON = 4,
        ROBOT_MODE_IDLE = 5,
        ROBOT_MODE_BACKDRIVE = 6,
        ROBOT_MODE_RUNNING = 7,
        ROBOT_MODE_UPDATING_FIRMWARE = 8
};

enum ControlMode{
        CONTROL_MODE_POSITION = 0,
        CONTROL_MODE_TEACH = 1,
        CONTROL_MODE_FORCE = 2,
        CONTROL_MODE_TORQUE = 3

};


enum JointMode{
        JOINT_SHUTTING_DOWN_MODE = 236,
        JOINT_PART_D_CALIBRATION_MODE = 237,
        JOINT_BACKDRIVE_MODE = 238,
        JOINT_POWER_OFF_MODE = 239,
        JOINT_NOT_RESPONDING_MODE = 245,
        JOINT_MOTOR_INITIALISATION_MODE = 246,
        JOINT_BOOTING_MODE = 247,
        JOINT_PART_D_CALIBRATION_ERROR_MODE = 248,
        JOINT_BOOTLOADER_MODE = 249,
        JOINT_CALIBRATION_MODE = 250,
        JOINT_FAULT_MODE = 252,
        JOINT_RUNNING_MODE = 253,
        JOINT_IDLE_MODE = 255

};

enum ToolMode{
        TOOL_BOOTLOADER_MODE = 249,
        TOOL_RUNNING_MODE = 253,
        TOOL_IDLE_MODE= 255

};

enum MessageType
{
        ROBOT_STATE = 16, ROBOT_MESSAGE = 20, PROGRAM_STATE_MESSAGE = 25, RT_FRAME
};

enum PackageTypes
{
        ROBOT_MODE_DATA = 0,
        JOINT_DATA = 1,
        TOOL_DATA = 2,
        MASTERBOARD_DATA = 3,
        CARTESIAN_INFO = 4,
        KINEMATICS_INFO = 5,
        CONFIGURATION_DATA = 6,
        FORCE_MODE_DATA = 7,
        ADDITIONAL_INFO = 8,
        CALIBRATION_DATA = 9,
        SAFETY_DATA = 10
};

enum SafetyMode{
        SAFETY_MODE_FAULT = 9,
        SAFETY_MODE_VIOLATION = 8,
        SAFETY_MODE_ROBOT_EMERGENCY_STOP = 7,
        SAFETY_MODE_SYSTEM_EMERGENCY_STOP = 6,
        SAFETY_MODE_SAFEGUARD_STOP = 5,
        SAFETY_MODE_RECOVERY = 4,
        SAFETY_MODE_PROTECTIVE_STOP = 3,
        SAFETY_MODE_REDUCED = 2,
        SAFETY_MODE_NORMAL = 1

};

enum RobotMessageType {
        ROBOT_MESSAGE_TEXT = 0,
        ROBOT_MESSAGE_PROGRAM_LABEL = 1,
        PROGRAM_STATE_MESSAGE_VARIABLE_UPDATE = 2,
        ROBOT_MESSAGE_VERSION = 3,
        ROBOT_MESSAGE_SAFETY_MODE = 5,
        ROBOT_MESSAGE_ERROR_CODE = 6,
        ROBOT_MESSAGE_KEY = 7,
        ROBOT_MESSAGE_REQUEST_VALUE = 9,
        ROBOT_MESSAGE_RUNTIME_EXCEPTION = 10
};

class RobotModeData //value 0
{
private:
    uint64_t timestamp;

    bool isRobotConnected;
    bool isRealRobotEnabled;
    bool isRobotPowerOn;
    bool isEmergencyStopped;
    bool isProtectiveStopped;
    bool isProgramRunning;
    bool isProgramPaused;

    RobotMode robotMode;
    ControlMode controlMode;

    double speedFraction;
    double speedScaling;

public:
    RobotModeData();
    ~RobotModeData();

    uint64_t getTimestamp() const;
    void setTimestamp(const uint64_t &value);
    bool getIsRobotConnected() const;
    void setIsRobotConnected(bool value);
    bool getIsRealRobotEnabled() const;
    void setIsRealRobotEnabled(bool value);
    bool getIsRobotPowerOn() const;
    void setIsRobotPowerOn(bool value);
    bool getIsEmergencyStopped() const;
    void setIsEmergencyStopped(bool value);
    bool getIsProtectiveStopped() const;
    void setIsProtectiveStopped(bool value);
    bool getIsProgramRunning() const;
    void setIsProgramRunning(bool value);
    bool getIsProgramPaused() const;
    void setIsProgramPaused(bool value);
    RobotMode getRobotMode() const;
    void setRobotMode(const RobotMode &value);
    ControlMode getControlMode() const;
    void setControlMode(const ControlMode &value);
    double getSpeedFraction() const;
    void setSpeedFraction(double value);
    double getSpeedScaling() const;
    void setSpeedScaling(double value);
};


class JointData // value 1
{
private:
    double actualJointPosition;
    double targetJointPosition;
    double actualJointSpeed;
    float actualJointCurrent;
    float actualJointVoltage;
    float actualMotorTemprature;

    JointMode jointMode;

public:
    JointData();
    ~JointData();

    double getActualJointPosition() const;
    void setActualJointPosition(double value);
    double getTargetJointPosition() const;
    void setTargetJointPosition(double value);
    double getActualJointSpeed() const;
    void setActualJointSpeed(double value);
    float getActualJointCurrent() const;
    void setActualJointCurrent(float value);
    float getActualJointVoltage() const;
    void setActualJointVoltage(float value);
    float getActualMotorTemprature() const;
    void setActualMotorTemprature(float value);
    JointMode getJointMode() const;

    void setJointMode(const JointMode &value);
};


class ToolData //value 2
{
private:
    char analogInputRange2;
    char analogInputRange3;

    double analogInput2;
    double analogInput3;

    float toolVoltage48V;
    float toolCurrent;
    float toolTemperature;

    unsigned char toolOutputVoltage;

    ToolMode toolMode;
public:
    ToolData();
    ~ToolData();

    char getAnalogInputRange2() const;
    void setAnalogInputRange2(char value);
    char getAnalogInputRange3() const;
    void setAnalogInputRange3(char value);
    double getAnalogInput2() const;
    void setAnalogInput2(double value);
    double getAnalogInput3() const;
    void setAnalogInput3(double value);
    float getToolVoltage48V() const;
    void setToolVoltage48V(float value);
    float getToolCurrent() const;
    void setToolCurrent(float value);
    float getToolTemperature() const;
    void setToolTemperature(float value);
    unsigned char getToolOutputVoltage() const;
    void setToolOutputVoltage(unsigned char value);
    ToolMode getToolMode() const;
    void setToolMode(const ToolMode &value);
};


class MasterboardData //value 3
{
public:
    MasterboardData();
    ~MasterboardData();

    int getDigitalInputBits() const;
    void setDigitalInputBits(int value);

    int getDigitalOutputBits() const;
    void setDigitalOutputBits(int value);

    char getAnalogInputRange0() const;
    void setAnalogInputRange0(char value);

    char getAnalogInputRange1() const;
    void setAnalogInputRange1(char value);

    double getAnalogInput0() const;
    void setAnalogInput0(double value);

    double getAnalogInput1() const;
    void setAnalogInput1(double value);

    char getAnalogOutputDomain0() const;
    void setAnalogOutputDomain0(char value);

    char getAnalogOutputDomain1() const;
    void setAnalogOutputDomain1(char value);

    double getAnalogOutput0() const;
    void setAnalogOutput0(double value);

    double getAnalogOutput1() const;
    void setAnalogOutput1(double value);

    float getMasterBoardTemperature() const;
    void setMasterBoardTemperature(float value);

    float getRobotVoltage48V() const;
    void setRobotVoltage48V(float value);

    float getRobotCurrent() const;
    void setRobotCurrent(float value);

    float getMasterIOCurrent() const;
    void setMasterIOCurrent(float value);

    SafetyMode getSafetyMode() const;
    void setSafetyMode(const SafetyMode &value);

    char getEuromap67InterfaceInstalled() const;
    void setEuromap67InterfaceInstalled(char value);

    int getEuromapInputBits() const;
    void setEuromapInputBits(int value);

    int getEuromapOutputBits() const;
    void setEuromapOutputBits(int value);

    float getEuromapVoltage() const;
    void setEuromapVoltage(float value);

    float getEuromapCurrent() const;
    void setEuromapCurrent(float value);

    unsigned char getInReducedMode() const;
    void setInReducedMode(unsigned char value);

private:
    int digitalInputBits;
    int digitalOutputBits;
    char analogInputRange0;
    char analogInputRange1;
    double analogInput0;
    double analogInput1;
    char analogOutputDomain0;
    char analogOutputDomain1;
    double analogOutput0;
    double analogOutput1;
    float masterBoardTemperature;
    float robotVoltage48V;
    float robotCurrent;
    float masterIOCurrent;
    SafetyMode safetyMode;
    unsigned char InReducedMode;
    char euromap67InterfaceInstalled;
    int euromapInputBits;
    int euromapOutputBits;
    float euromapVoltage;
    float euromapCurrent;


};

class CartesianInfoData // value 4
{
private:
    double x,y,z;
    double rx, ry, rz;
    double tcpOffsetX, tcpOffsetY, tcpOffsetZ;
    double tcpOffsetRX, tcpOffsetRY, tcpOffsetRZ;

public:
    CartesianInfoData();
    ~CartesianInfoData();
    double getX() const;
    void setX(double value);
    double getY() const;
    void setY(double value);
    double getZ() const;
    void setZ(double value);
    double getRx() const;
    void setRx(double value);
    double getRy() const;
    void setRy(double value);
    double getRz() const;
    void setRz(double value);
    double getTcpOffsetX() const;
    void setTcpOffsetX(double value);
    double getTcpOffsetY() const;
    void setTcpOffsetY(double value);
    double getTcpOffsetZ() const;
    void setTcpOffsetZ(double value);
    double getTcpOffsetRZ() const;
    void setTcpOffsetRZ(double value);
    double getTcpOffsetRY() const;
    void setTcpOffsetRY(double value);
    double getTcpOffsetRX() const;
    void setTcpOffsetRX(double value);
};


class ConfigurationData //value 6
{
private:
    double jointMinLimit;
    double jointMaxLimit;
    double jointMaxSpeed;
    double jointMaxAcceleration;
    double vJointDefault;
    double aJointDefault;
    double vToolDefault;
    double aToolDefault;
    double eqRadius;
    double DHa;
    double DHd;
    double DHalpha;
    double DHtheta;
    int masterboardVersion;
    int controllerBoxType;
    int robotType;
    int robotSubType;

public:
    ConfigurationData();
    ~ConfigurationData();

    double getJointMinLimit() const;
    void setJointMinLimit(double value);
    double getJointMaxLimit() const;
    void setJointMaxLimit(double value);
    double getJointMaxSpeed() const;
    void setJointMaxSpeed(double value);
    double getJointMaxAcceleration() const;
    void setJointMaxAcceleration(double value);
    double getVJointDefault() const;
    void setVJointDefault(double value);
    double getAJointDefault() const;
    void setAJointDefault(double value);
    double getVToolDefault() const;
    void setVToolDefault(double value);
    double getAToolDefault() const;
    void setAToolDefault(double value);
    double getEqRadius() const;
    void setEqRadius(double value);
    double getDHa() const;
    void setDHa(double value);
    double getDHd() const;
    void setDHd(double value);
    double getDHalpha() const;
    void setDHalpha(double value);
    double getDHtheta() const;
    void setDHtheta(double value);
    int getMasterboardVersion() const;
    void setMasterboardVersion(int value);
    int getControllerBoxType() const;
    void setControllerBoxType(int value);
    int getRobotType() const;
    void setRobotType(int value);
    int getRobotSubType() const;
    void setRobotSubType(int value);
};


class ForceModeData // value 7
{
private:
    double actualForceValue;
    double fx,fy,fz;
    double tx, ty, tz;
    double RobotDexterity;

public:
    ForceModeData();
    ~ForceModeData();
    double getFX() const;
    void setFX(double value);
    double getFY() const;
    void setFY(double value);
    double getFZ() const;
    void setFZ(double value);
    double getRobotDexterity() const;
    void setRobotDexterity(double value);
    double getActualForceValue() const;
    void setActualForceValue(double value);
    double getTx() const;
    void setTx(double value);
    double getTy() const;
    void setTy(double value);
    double getTz() const;
    void setTz(double value);
};



class QTargetJointsPositions
{
public:
    double getKat1() const;
    void setKat1(double value);

    double getKat2() const;
    void setKat2(double value);

    double getKat3() const;
    void setKat3(double value);

    double getKat4() const;
    void setKat4(double value);

    double getKat5() const;
    void setKat5(double value);

    double getKat6() const;
    void setKat6(double value);

   // QVector<double> GetJointsPositions();


private:
    double kat1,kat2,kat3,kat4,kat5,kat6;

};

class QTargetJointsVelocities
{
public:

    double getPredkosc1() const;
    void setPredkosc1(double value);

    double getPredkosc2() const;
    void setPredkosc2(double value);

    double getPredkosc3() const;
    void setPredkosc3(double value);

    double getPredkosc4() const;
    void setPredkosc4(double value);

    double getPredkosc5() const;
    void setPredkosc5(double value);

    double getPredkosc6() const;
    void setPredkosc6(double value);

private:
    double predkosc1,predkosc2,predkosc3,predkosc4,predkosc5,predkosc6;
};

class QTargetJointsTorques
{
public:

    double getTorque1() const;
    void setTorque1(double value);

    double getTorque2() const;
    void setTorque2(double value);

    double getTorque3() const;
    void setTorque3(double value);

    double getTorque4() const;
    void setTorque4(double value);

    double getTorque5() const;
    void setTorque5(double value);

    double getTorque6() const;
    void setTorque6(double value);

private:
    double Torque1, Torque2, Torque3, Torque4, Torque5, Torque6;
};

class QActualJointsPositions
{
public:
    double getKat1() const;
    void setKat1(double value);

    double getKat2() const;
    void setKat2(double value);

    double getKat3() const;
    void setKat3(double value);

    double getKat4() const;
    void setKat4(double value);

    double getKat5() const;
    void setKat5(double value);

    double getKat6() const;
    void setKat6(double value);

private:
    double kat1, kat2, kat3, kat4, kat5, kat6;
};

class QActualJointsCurrents
{
public:
    double getCurrent1() const;
    void setCurrent1(double value);

    double getCurrent2() const;
    void setCurrent2(double value);

    double getCurrent3() const;
    void setCurrent3(double value);

    double getCurrent4() const;
    void setCurrent4(double value);

    double getCurrent5() const;
    void setCurrent5(double value);

    double getCurrent6() const;
    void setCurrent6(double value);

private:
    double current1, current2, current3, current4, current5, current6;
};

class QTargetCartesianCoordinatesTCP
{
public:
    double getX() const;
    void setX(double value);

    double getY() const;
    void setY(double value);

    double getZ() const;
    void setZ(double value);

    double getRx() const;
    void setRx(double value);

    double getRy() const;
    void setRy(double value);

    double getRz() const;
    void setRz(double value);

private:
    double x, y, z, Rx, Ry, Rz;
};
/// @endcond

/** \addtogroup UR
 *  @{
 */
/**
 * @brief Contains robot data fetched from data stream. To be used in UR3Intermediator
 */
class UR3Message
{
public:

    CartesianInfoData cartesianInfoData;
    ForceModeData forceModeData;
    MasterboardData masterboardData;
    ConfigurationData configurationData;
    QVector<JointData> jointsData;

    QVector <ForceModeData> forcesModeData;
    ToolData toolData;
    RobotModeData robotModeData;
public:

    UR3Message();
    ~UR3Message();

    CartesianInfoData getCartesianInfoData() const;

    void setCartesianInfoData(char *data, unsigned int offset);

    void setForceModeData(char *data, unsigned int offset);

    MasterboardData getMasterboardData() const;
    void setMasterboardData(char *data, int offset);
    ConfigurationData getConfigurationData() const;
    void setConfigurationData();//char *data, unsigned int offset);
    QVector<JointData> getJointsData() const;
    QVector<double> getActualJointAngles() const;
    QVector<double> getActualCartesian() const;
    QVector<double> getForceModeData() const;
    void setJointsData(char *data, int offset);

    ToolData getToolData() const;
    void setToolData(char *data, int offset);
    RobotModeData getRobotModeData() const;
    void setRobotModeData(char *data, int offset);

};

/** @}*/

#endif // UR3MESSAGE_H
