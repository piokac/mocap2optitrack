#ifndef UR3WORKSPACE_H
#define UR3WORKSPACE_H

#include <QObject>
#include <QMatrix4x4>
#include <QVector3D>
#include <cmath>
#include "UR3Intermediator.h"
#include "cxmlio.h"
#include "common.h"


/** \addtogroup UR
 *  @{
 */
/**
 * @brief struct representing definition of a physical bucket for object sorting
 */
struct BinDescriptor
{
    /**
     * @brief displayed in user interface
     */
    QString name;
    /**
     * @brief bin identification number
     */
    int     id;
    /**
     * @brief 6 element vector containing joint configuration describing bucket position
     */
    QVector<double> position;

    /**
     * @brief converts binDescriptor to QString for XML write
     * @return QString describing bin, returns empty QString on error
     */
    QString toString();
    /**
     * @brief parse XML entry
     * @param str XML entry
     */
    void fromString(QString str);
};

/**
 * @brief Wraper over UR3Intermediator class. Represents a robot working space. Enables robot connection and robot interface.
 * UR3Workspace contains robot calibration information and definition of environment, such as home position and binDescriptor etc.
 */
class UR3Workspace : public cXmlIO
{
    Q_OBJECT
public:
    /**
     * @brief default constructor. Sets initial workspace values.
     */
    UR3Workspace();
    ~UR3Workspace();

    /**
     * @brief UR3CalibWidget interface
     * @return description of all bins defined in UR3Workspace as BinDescriptor
     */
    QMap<int, BinDescriptor> getBinDescriptors() { return binDescriptors_; }
    /**
     * @brief UR3CalibWidget interface
     * @return 6 element vector defining array start position - [mm; rot. vct]
     */
    QVector<double> getArrayStartPosition() { return arrayStartPosition_; }
    /**
     * @brief UR3CalibWidget interface
     * @return 6 element vector defining array end position - [mm; rot. vct]
     */
    QVector<double> getArrayEndPosition() { return arrayEndPosition_; }
    /**
     * @brief UR3CalibWidget interface
     * @return returns 6 element vector defining robot starting position in joint configuration - [rad]
     */
    QVector<double> getHomePosition() { return homePosition_; }
    /**
     * @brief UR3CalibWidget interface
     * @return 6 element vector defining robot middle position between home and working space - [rad]
     */
    QVector<double> getBoardPosition() { return boardPosition_; }
    /**
     * @brief UR3CalibWidget interface
     * @return 6 element vector defining robot position over bins - [rad]
     */
    QVector<double> getOverBinPosition() { return overBinPosition_; }
    /**
     * @brief UR3CalibWidget interface
     * @return map of three six elements vectors describing robot celibration in relation to the camera - [mm; rot. vct]
     */
    QMap<int, QVector<double> > getCalibrationPoints() { return calibrationPoints_; }

    /**
     * @brief UR3CalibWidget interface
     * @return calibration mode
     */
    bool getEnableBinArrayMode() { return enableBinArrayMode_(); }
    /**
     * @brief UR3CalibWidget interface
     * @return number of bins for array calibration mode
     */
    int getBinArraySize() { return binArraySize_(); }

    /**
     * @brief sets one bin descriptor
     * @param key identification key of the bin written
     * @param cartesianData 6 element vector containing point X, Y, Z [mm] and orientation as rotation vector
     */
    void setBinPosition(int key, QVector<double> cartesianData);
    /**
     * @brief sets array starting position
     * @param cartesianData 6 element vector containing point X, Y, Z [mm] and orientation as rotation vector
     */
    void setArrayStartPosition(QVector<double> cartesianData);
    /**
     * @brief sets array end position
     * @param cartesianData 6 element vector containing point X, Y, Z [mm] and orientation as rotation vector
     */
    void setArrayEndPosition(QVector<double> cartesianData);
    /**
     * @brief sets robot starting position
     * @param jointData 6 element vector describing robot joint configuration [rad]
     */
    void setHomePosition(QVector<double> jointData);
    /**
     * @brief sets robot over bin position
     * @param jointData 6 element vector describing robot joint configuration [rad]
     */
    void setOverBinPosition(QVector<double> jointData);
    /**
     * @brief sets robot middle position between home and working space
     * @param jointData 6 element vector describing robot joint configuration [rad]
     */
    void setBoardPosition(QVector<double> jointData);
    /**
     * @brief sets one calibration point
     * @param key identification key of calibration point
     * @param cartesianData 6 element vector containing point X, Y, Z [mm] and orientation as rotation vector
     */
    void setCalibrationPoint(int key, QVector<double> cartesianData);

    /**
     * @brief recaltulates transformation matrix between camera and robot coordinate systems based on calibration points stored in UR3Workspace
     */
    void recalculateCalibration();
    /**
     * @brief recaltulates bin set for array calibration mode
     */
    void recalculateBinArray();

    /**
     * @brief sets robot Tool Center Point to calibration configuration as described in XML
     */
    void setCalibrationTCP();
    /**
     * @brief sets robot Tool Center Point to working configuration as described in XML
     */
    void setWorkingTCP();

    /**
     * @brief writes to XML file UR3Workspace meta properties and description of robot enviroment like bins, home position, working and calibration TCP, etc.
     * @param stream XML stream
     */
    virtual void writeXml(QXmlStreamWriter& stream);
    /**
     * @brief reads XML file UR3Workspace meta properties and description of robot enviroment like bins, home position, working and calibration TCP, etc.
     * @param stream XML stream
     */
    virtual void readXml(QXmlStreamReader& stream);

    /**
     * @brief Gets \ref robotPort property value
     */
    int getRobotPort() const { return UR3Intermediator_->Port; }
    /**
     * @brief Sets \ref robotPort property value
     */
    void setRobotPort(const int& port) { UR3Intermediator_->Port = port; }

    /**
     * @brief Gets \ref robotIP property value
     */
    QString getRobotIP() const { return UR3Intermediator_->IpAddress;}
    /**
     * @brief Sets \ref robotIP property value
     */
    void setRobotIP(const QString& IP) { UR3Intermediator_->IpAddress = IP; }

    /**
     * @brief Gets \ref ackServerPort property value
     */
    int getAckServerPort() const { return UR3Intermediator_->ackServerPort; }
    /**
     * @brief Sets \ref ackServerPort property value
     */
    void setAckServerPort(const int& port) { UR3Intermediator_->ackServerPort = port; }

public slots:
    /**
     * @brief opens a TCP/IP connection to robot using \ref robotIP and \ref robotPort properties
     */
    void connectToRobot();
    /**
     * @brief closes a TCP/IP connection from robot
     */
    void disconnectFromRobot();
    /**
     * @brief evokes a MoveJ command movement to home point set by \ref setHomePosition
     */
    void moveToWorkspaceHome();
    /**
     * @brief evokes a MoveJ command movement to point set by \ref setBoardPosition
     */
    void moveOverBoard();
    /**
     * @brief evokes a MoveJ command movement to point set by \ref setOverBinPosition
     */
    void moveOverBin();
    /**
     * @brief pickObject evokes a procedure of object picking Home -> OverBoard -> OverObject -> PickObjest -> OverObject -> OverBoard -> Home -> Bin -> Home
     * @param vector containing X, Y and Z [m] of object origin to pick in robot<->camera reference frame
     * @param object hight [m]
     * @param angle orientation of picked object - reorientates TCP according to this parameter
     * @param bin object destination bin (bucket)
     */
    void pickObject(QVector3D pointInMeters, float objectHeightInMeters, float angle, int bin);


signals:
    /**
     * @brief emitted when new configuration/position data from robot was received
     * @param data 6 element vector containing configuration or position
     * @param type determines message type: 'j' - joint [rad]; 'p' - pose [mm;rv]
     */
    void newPose(QVector<double> data, char type);
    /**
     * @brief emitted when new command has been send to a robot to execute
     */
    void commandStarted();
    /**
     * @brief emitted when acknowledgment of full command execution was received on port \ref ackServerPort
     */
    void commandExecuted();
    /**
     * @brief emitted after robot connection attempt
     * @param connected true if connection was successful
     */
    void connectedToRobot(bool connected);
    /**
     * @brief emitted when robot connection closed
     */
    void disconnectedFromRobot();
    /**
     * @brief emitted when robot is in emergency or protective stop
     */
    void robotEmergencyStopped();

private:
    UR3Intermediator* UR3Intermediator_;

    QMap<int, BinDescriptor> binDescriptors_;
    QVector<double> homePosition_;
    QVector<double> boardPosition_;
    QVector<double> overBinPosition_;
    QMap<int, QVector<double> > calibrationPoints_;

    QVector<double> arrayStartPosition_;
    QVector<double> arrayEndPosition_;

    QMatrix4x4 calibMatrix_;
    double pickArrayRotAngle_ = 0;

    QVector<double> calibrationTCP_;
    QVector<double> workingTCP_;

    QVector<double> getBoardPointInBaseCoordinates(QVector3D point, float angle);

    void setToolOutput(int output, bool state);

    /**
      * @brief Robot TCP/IP interface port. Default 30002
      */
    Q_PROPERTY(int robotPort READ getRobotPort WRITE setRobotPort DESIGNABLE true USER true)

    /**
      * @brief Robot TCP/IP interface IP
      */
    Q_PROPERTY(QString robotIP READ getRobotIP WRITE setRobotIP DESIGNABLE true USER true)

    /**
      * @brief Port for acknowledgment messages of command execution
      */
    Q_PROPERTY(int ackServerPort READ getAckServerPort WRITE setAckServerPort DESIGNABLE true USER true)

    /**
      * @brief enables array style calibration mode, default false
      */
    ADD_PROPERTY(enableBinArrayMode_, bool)
    /**
      * @brief defines bin count in array calibration mode, default 5
      */
    ADD_PROPERTY(binArraySize_, int)

    /**
      * @brief distance between robot TCP and final picking point (safety distance), default 0
      */
    ADD_PROPERTY(pickingObjectOffsetInMM_, int)
    /**
      * @brief additional offset for putting object into bin, default 5
      */
    ADD_PROPERTY(puttingObjectOffsetInMM_, int)
    /**
      * @brief  distance of final approach to picking point, when motion is switched to linear, default 10
      */
    ADD_PROPERTY(approachOffsetInMM_, int)

    /**
      * @brief robot joint acceleration, default 1
      */
    ADD_PROPERTY(jMoveAcceleration_, double)
    /**
      * @brief robot joint speed, default 1
      */
    ADD_PROPERTY(jMoveSpeed_, double)
    /**
      * @brief robot TCP linear acceleration, default 0.6
      */
    ADD_PROPERTY(lMoveToolAcceleration_, double)
    /**
      * @brief robot TCP linear speed, default 0.1
      */
    ADD_PROPERTY(lMoveToolSpeed_, double)


private slots:
    void newPoseFromRobot(QVector<double> data, char type);
    void robotStarted();
    void robotFinished();

    void onRobotDisconnected();
    void onRobotEmergencyStopped();
};
/** @}*/
#endif // UR3WORKSPACE_H

