#include "UR3Intermediator.h"
#include <QtGlobal>
#include <QtCore>
#include <QtEndian>
#include <QHostAddress>
#include <QDebug>
#include "UR3Message.h"
#include <QDateTime>

#define parseDouble( src_class, setter_suffix, data, offset){double val; memcpy(&val, &_data[offset], sizeof(val));val = bytesSwap(val);offset+=sizeof(val);src_class.set ## setter_suffix(val);}

static double RoundDouble(double val,int prec)
{
    auto precision = pow(10,prec);
    return round(val * precision) / precision;
}

char *strdup (const char *s)
{
    char* d = (char*)malloc(strlen (s) + 1);   // Space for length plus nul
    if (d == NULL) return NULL;          // No memory
    strcpy (d,s);                        // Copy the characters
    return d;                            // Return the new string
}

static double bytesSwap(double v)
{
    union {
        uint64_t i;
        double d;
    } conv;
    conv.d = v;
    conv.i = bswap_64(conv.i);
    return conv.d;

}

void UR3Intermediator::Execute(QString command)
{
    qDebug()<<"Execute: "<<command;

    _socket->write(command.toLatin1().data());
    _socket->waitForBytesWritten();

}


QString UR3Intermediator::MoveL(QVector<double> TargetPose, double toolAcceleration, double toolSpeed, double time, double blendRadius)
{
    QString command = "movel(p[" +
            QString::number(TargetPose[0]) + ", " +
            QString::number(TargetPose[1]) + ", " +
            QString::number(TargetPose[2]) + ", " +
            QString::number(TargetPose[3]) + ", " +
            QString::number(TargetPose[4]) + ", " +
            QString::number(TargetPose[5]) + "], " +
            "a=" + QString::number(toolAcceleration)+ ", " +
            "v=" + QString::number(toolSpeed)+ ", " +
            "t=" + QString::number(time)+", "+
            "r=" + QString::number(blendRadius)+
            ")\n";
    return command;
}

QString UR3Intermediator::MoveJ(QVector<double> Position, bool isBaseCS, double Acceleration, double Speed, double blendRadius )
{
    QString command;
    if(_connected )
    {
        command = "movej(";
        if(isBaseCS)
        {
            command+="p[";
        }
        else
        {
            command+="[";
        }
        command+=QString::number(Position[0]) + ", " +
            QString::number(Position[1]) + ", " +
            QString::number(Position[2]) + ", " +
            QString::number(Position[3]) + ", " +
            QString::number(Position[4]) + ", " +
            QString::number(Position[5]) + "], " +
            "a=" + QString::number(Acceleration)+ ", " +
            "v=" + QString::number(Speed);
        if(blendRadius != 0)
        {
            command += ", r=" + QString::number(blendRadius);
        }
        command += ")\n";
    }
    return command;
}

void UR3Intermediator::timerEvent(QTimerEvent */*event*/)
{
    if(_connected)
    {
        if(cmds.length()>0)
        {
            if(!_ackLock)
            {
                _ackLock = true;
                emit  CommandExecutionStart();
                Execute(cmds[0]);
                cmds.pop_front();
            }
        }
    }
}

UR3Intermediator::UR3Intermediator(QString ipAddress, int port, int ackServerPort): Port(port),IpAddress(ipAddress), ackServerPort(ackServerPort)
{
    this->_socket = new QTcpSocket();
    this->_lastJointPos.resize(6);
    this->_lastJointPos.fill(.0);
    this->_lastPolozenie.resize(6);
    this->_lastPolozenie.fill(.0);
    this->_lastForceValue.resize(6);
    this->_lastForceValue.fill(.0);

    _ackServer = NULL;

    connect(this->_socket,SIGNAL(readyRead()),this,SLOT(OnSocketNewBytesWritten()));
    connect(this->_socket,SIGNAL(disconnected()),this,SLOT(onSlotDisconnect()));
}

QString UR3Intermediator::setTCP(QVector<double> pose)
{
    QString command;
    if(_connected && pose.size() == 6)
    {
        command = "set_tcp(p[";

        command+=QString::number(pose[0]) + ", " +
                QString::number(pose[1]) + ", " +
                QString::number(pose[2]) + ", " +
                QString::number(pose[3]) + ", " +
                QString::number(pose[4]) + ", " +
                QString::number(pose[5]) + "]" + ")\n";
    }
    return command;
}


QString UR3Intermediator::robotSleep(float seconds)
{
    QString command = "sleep(" + QString::number(seconds) + ")\n";
    return command;
}

QString UR3Intermediator::setToolOutput(int output, bool state)
{
    QString command;
    if(output == 0 || output == 1)
    {
        command = "set_tool_digital_out(" + QString::number(output) + ",";
        if(state)
        {
            command += "True";
        }else
        {
            command += "False";
        }
        command += ")\n";
    }
    return command;
}

QString UR3Intermediator::setOutput(int output, bool state)
{
    QString command;
    if(output >= 0 && output <= 9)
    {
        command = "set_digital_out(" + QString::number(output) + ",";
        if(state)
        {
            command += "True";
        }else
        {
            command += "False";
        }
        command += ")\n";
    }
    return command;
}


void UR3Intermediator::GetRobotData()
{
    int size = 0;
    unsigned int offset = 0;
    // while(_DataFlow.size()<4)//dodana pętla
    // {
    if(mutex.tryLock())
    {
        _data = _DataFlow.data();
        memcpy(&size, &_data[offset], sizeof(size));
        size = bswap_32(size);
        if(size>2048)
        {
            _DataFlow.clear();
            qDebug()<<"frame corrupted. Size:"<<size;
            mutex.unlock();
            return;
        }
        if(_DataFlow.size()<size)
        {
            mutex.unlock();
            qDebug()<<"only part of frame received. Expected:"<<size<<", received"<<_DataFlow.size();
            return;
        }

        offset += sizeof(size);
        while(offset<size)
        {
            unsigned char Type;
            memcpy(&Type,&_data[offset],sizeof(Type));
            offset+=sizeof(Type);
            int messageType = Type;
            if(size==1060 && Port == 30003)
            {
                messageType = RT_FRAME;
            }

            switch(messageType)
            {
            case ROBOT_MESSAGE:
            {
                break;
            }
            case ROBOT_STATE:
            {
                GetRobotMessage(_data, offset, size);
                break;
            }
            case PROGRAM_STATE_MESSAGE:
            {
                break;
            }
            case RT_FRAME:
            {
                RealTime(offset);
                break;
            }
            default:
                qDebug()<<"Unknown command:"<<messageType<<" size:"<<size;
                offset = size;
                break;
            }
        }
        _DataFlow = _DataFlow.mid(size);

        mutex.unlock();

        _DataFlow.clear(); // TODO: NIE WIADOMO JAKIE DANE SA KASOWANE; ROZWIĄZANIE PROWIZORYCZNE
    }
    // }
}


bool UR3Intermediator::CheckIfRunning()
{
    _running =ActualRobotInfo.robotModeData.getIsProgramRunning();
    return _running;
}

void UR3Intermediator::GetRobotMessage(char *data, unsigned int &offset, int size)
{
    while(size>offset){
        int sizeOfPackage;
        memcpy(&sizeOfPackage, &data[offset], sizeof(sizeOfPackage));
        sizeOfPackage = bswap_32(sizeOfPackage);
        //sizeOfPackage = qFromBigEndian<int>(sizeOfPackage);
        offset+=sizeof(sizeOfPackage);

        unsigned char packageType;
        memcpy(&packageType, &data[offset], sizeof(packageType));
        offset+=sizeof(packageType);
        //  packageType = FORCE_MODE_DATA;
        switch(packageType)
        {
            case ROBOT_MODE_DATA:
                this->ActualRobotInfo.setRobotModeData(_data, offset);

                if(ActualRobotInfo.robotModeData.getIsEmergencyStopped() || ActualRobotInfo.robotModeData.getIsProtectiveStopped())
                {
                    cmds.clear();
                    _ackLock = false;
                    emit robotEmergancyStoped();
                }

                break;
            case JOINT_DATA:
                this->ActualRobotInfo.setJointsData(_data, offset);
                emit newPose(ActualRobotInfo.getActualJointAngles(), 'j');
                break;
            case TOOL_DATA:
                this->ActualRobotInfo.setToolData(_data,offset);
                break;
            case MASTERBOARD_DATA:
                this->ActualRobotInfo.setMasterboardData(_data, offset);
                break;
            case CARTESIAN_INFO:
                this->ActualRobotInfo.setCartesianInfoData(_data,offset);
                emit newPose(ActualRobotInfo.getActualCartesian(), 'p');
                break;
            case KINEMATICS_INFO:
                break;
            case CONFIGURATION_DATA:
                break;
            case FORCE_MODE_DATA:
                this->ActualRobotInfo.setForceModeData(_data, offset);
                emit newPose(ActualRobotInfo.getForceModeData(), 'f');
                break;
            case ADDITIONAL_INFO:
                break;
            case CALIBRATION_DATA:
                break;
            case SAFETY_DATA:
                break;
        }
        offset +=sizeOfPackage - 5; //-5 poniewaz wczesniej przesunalem o sizeofpackage i packagetype
    }
}

void UR3Intermediator::ReadDataFlow()
{
    if(_connected)
    {
        if(mutex.tryLock())
        {
            _DataFlow.push_back( this->_socket->readAll());
            //  timer.start();
            // qDebug()<<"new data"<<timerrr.elapsed();
            mutex.unlock();
        }
        else
        {
            qDebug()<<"mutex locked"<<timerrr.elapsed();
        }
    }
}

//void UR3Intermediator::RealTime(char *_data, unsigned int &offset, int size)
void UR3Intermediator::RealTime(unsigned int &offset)
{
    //   qDebug()<<"Ramka przychodzi";
    // QTargetJointsPositions TargetJointsPositions;
    //QTargetJointsVelocities TargetJointsVelocities;
    //QTargetJointsTorques TargetJointsTorques;
    //QActualJointsPositions ActualJointsPositions;

    QActualJointsCurrents ActualJointsCurrents;
    //QTargetCartesianCoordinatesTCP TargetCartesianCoordinatesTCP;
    //QRobotMode RobotMode;

    // if(timerrr.elapsed()!=8)
    int valT = timerrr.restart();
    offset--;
    //parseDouble(ActualJointsCurrents,Current1,data,offset);
    //qDebug()<<"timestamp:"<<ActualJointsCurrents.getCurrent1();
    offset += 8;

    //qtarget
    /*parseDouble(TargetJointsPositions,Kat1,data,offset);
        parseDouble(TargetJointsPositions,Kat2,data,offset);
        parseDouble(TargetJointsPositions,Kat3,data,offset);
        parseDouble(TargetJointsPositions,Kat4,data,offset);
        parseDouble(TargetJointsPositions,Kat5,data,offset);
        parseDouble(TargetJointsPositions,Kat6,data,offset);*/
    offset += 48;

    //qdtarget
    /*parseDouble(TargetJointsVelocities,Predkosc1,data,offset);
        parseDouble(TargetJointsVelocities,Predkosc2,data,offset);
        parseDouble(TargetJointsVelocities,Predkosc3,data,offset);
        parseDouble(TargetJointsVelocities,Predkosc4,data,offset);
        parseDouble(TargetJointsVelocities,Predkosc5,data,offset);
        parseDouble(TargetJointsVelocities,Predkosc6,data,offset);*/
    offset += 48;

    //qddtarget
    offset += 48;

    //itarget
    offset += 48;

    //mtarget
    /*parseDouble(TargetJointsTorques,Torque1,data,offset);
        parseDouble(TargetJointsTorques,Torque2,data,offset);
        parseDouble(TargetJointsTorques,Torque3,data,offset);
        parseDouble(TargetJointsTorques,Torque4,data,offset);
        parseDouble(TargetJointsTorques,Torque5,data,offset);
        parseDouble(TargetJointsTorques,Torque6,data,offset);*/
    offset += 48;

    //qactual
    /*parseDouble(ActualJointsPositions,Kat1,data,offset);
        parseDouble(ActualJointsPositions,Kat2,data,offset);
        parseDouble(ActualJointsPositions,Kat3,data,offset);
        parseDouble(ActualJointsPositions,Kat4,data,offset);
        parseDouble(ActualJointsPositions,Kat5,data,offset);
        parseDouble(ActualJointsPositions,Kat6,data,offset);*/
    offset += 48;

    //qdactual

    offset += 48;

    //iactual
    parseDouble(ActualJointsCurrents,Current1,data,offset);
    parseDouble(ActualJointsCurrents,Current2,data,offset);
    parseDouble(ActualJointsCurrents,Current3,data,offset);
    parseDouble(ActualJointsCurrents,Current4,data,offset);
    parseDouble(ActualJointsCurrents,Current5,data,offset);
    parseDouble(ActualJointsCurrents,Current6,data,offset);

    QVector<double> currents = QVector<double>({ActualJointsCurrents.getCurrent1(),ActualJointsCurrents.getCurrent2()
                                                ,ActualJointsCurrents.getCurrent3(),ActualJointsCurrents.getCurrent4(),
                                                ActualJointsCurrents.getCurrent5(),ActualJointsCurrents.getCurrent6()});

    emit newPose(currents, 'c');

    // offset += 48;

    //icontrol
    offset += 48;

    //toolvectoractual
    parseDouble(ActualRobotInfo.cartesianInfoData,X,data,offset);
    parseDouble(ActualRobotInfo.cartesianInfoData,Y,data,offset);
    parseDouble(ActualRobotInfo.cartesianInfoData,Z,data,offset);
    parseDouble(ActualRobotInfo.cartesianInfoData,Rx,data,offset);
    parseDouble(ActualRobotInfo.cartesianInfoData,Ry,data,offset);
    parseDouble(ActualRobotInfo.cartesianInfoData,Rz,data,offset);

    QVector<double> current = QVector<double>({ActualRobotInfo.cartesianInfoData.getX(),ActualRobotInfo.cartesianInfoData.getY()
                                               ,ActualRobotInfo.cartesianInfoData.getZ(),ActualRobotInfo.cartesianInfoData.getRx(),
                                               ActualRobotInfo.cartesianInfoData.getRy(),ActualRobotInfo.cartesianInfoData.getRz()});


    emit newPose(current, 'p');

    //   offset += 48;

    //toolspeedactual
    offset += 48;

    //tcpforce
    parseDouble(ActualRobotInfo.forceModeData,FX,data,offset);
    parseDouble(ActualRobotInfo.forceModeData,FY,data,offset);
    parseDouble(ActualRobotInfo.forceModeData,FZ,data,offset);
    parseDouble(ActualRobotInfo.forceModeData,Tx,data,offset);
    parseDouble(ActualRobotInfo.forceModeData,Ty,data,offset);
    parseDouble(ActualRobotInfo.forceModeData,Tz,data,offset);

    QVector<double> forces = QVector<double>({ActualRobotInfo.forceModeData.getFX(),ActualRobotInfo.forceModeData.getFY(),ActualRobotInfo.forceModeData.getFZ(),
                                              ActualRobotInfo.forceModeData.getTx(),ActualRobotInfo.forceModeData.getTy(),ActualRobotInfo.forceModeData.getTz()});

    emit newPose(forces, 'f');

    //    offset += 48;

    //toolvectortarget
    /* parseDouble(TargetCartesianCoordinatesTCP,X,data,offset);
        parseDouble(TargetCartesianCoordinatesTCP,Y,data,offset);
        parseDouble(TargetCartesianCoordinatesTCP,Z,data,offset);
        parseDouble(TargetCartesianCoordinatesTCP,Rx,data,offset);
        parseDouble(TargetCartesianCoordinatesTCP,Ry,data,offset);
        parseDouble(TargetCartesianCoordinatesTCP,Rz,data,offset);*/
    //    offset += 48;

    //tcpspeedtarget
    //    offset += 48;

    //digitalinputsbits
    //    offset += 8;

    //motortemperatures
    //    offset += 48;

    //controllertimer
    //    offset += 8;

    //testvalue
    //    offset += 8;

    //robotmode
    //parseDouble(RobotMode,Mode,data,offset);
    //    offset += 8;

    //jointmodes
    //    offset += 48;

    //safetymode
    //    offset += 8;
    //
    //    offset += 48;

    //toolaccelerometervalues
    //    offset += 24;

    //
    //    offset += 8;

    //speedscalling
    //    offset += 8;

    //linearmomentumnorm
    //    offset += 8;

    //
    //    offset += 8;

    //
    //    offset += 8;

    //vmain
    //    offset += 8;

    //vrobot
    //    offset += 8;

    //irobot
    //    offset += 8;

    //vactual

    /*qDebug()<<"kat1: "<<TargetJointsPositions.getKat1()*57;
        qDebug()<<"kat2: "<<TargetJointsPositions.getKat2()*57;
        qDebug()<<"kat3: "<<TargetJointsPositions.getKat3()*57;
        qDebug()<<"kat4: "<<TargetJointsPositions.getKat4()*57;
        qDebug()<<"kat5: "<<TargetJointsPositions.getKat5()*57;
        qDebug()<<"kat6: "<<TargetJointsPositions.getKat6()*57;*/

    /*   qDebug()<<"predkosc1: "<<TargetJointsVelocities.getPredkosc1();
        qDebug()<<"predkosc2: "<<TargetJointsVelocities.getPredkosc2();
        qDebug()<<"predkosc3: "<<TargetJointsVelocities.getPredkosc3();
        qDebug()<<"predkosc4: "<<TargetJointsVelocities.getPredkosc4();
        qDebug()<<"predkosc5: "<<TargetJointsVelocities.getPredkosc5();
        qDebug()<<"predkosc6: "<<TargetJointsVelocities.getPredkosc6();*/

    /* qDebug()<<"moment1: "<<TargetJointsTorques.getTorque1();
         qDebug()<<"moment2: "<<TargetJointsTorques.getTorque2();
         qDebug()<<"moment3: "<<TargetJointsTorques.getTorque3();
         qDebug()<<"moment4: "<<TargetJointsTorques.getTorque4();
         qDebug()<<"moment5: "<<TargetJointsTorques.getTorque5();
         qDebug()<<"moment6: "<<TargetJointsTorques.getTorque6();*/

    /* qDebug()<<"kat1: "<<ActualJointsPositions.getKat1();
         qDebug()<<"kat2: "<<ActualJointsPositions.getKat2();
         qDebug()<<"kat3: "<<ActualJointsPositions.getKat3();
         qDebug()<<"kat4: "<<ActualJointsPositions.getKat4();
         qDebug()<<"kat5: "<<ActualJointsPositions.getKat5();
         qDebug()<<"kat6: "<<ActualJointsPositions.getKat6();*/

    /*qDebug()<<"prad1: "<<ActualJointsCurrents.getCurrent1();
         qDebug()<<"prad2: "<<ActualJointsCurrents.getCurrent2();
         qDebug()<<"prad3: "<<ActualJointsCurrents.getCurrent3();
         qDebug()<<"prad4: "<<ActualJointsCurrents.getCurrent4();
         qDebug()<<"prad5: "<<ActualJointsCurrents.getCurrent5();
         qDebug()<<"prad6: "<<ActualJointsCurrents.getCurrent6();*/

    /*qDebug()<<"wspolrzedna x tcp: "<<ActualCartesianCoordinatesTCP.getX();
         qDebug()<<"wspolrzedna y tcp: "<<ActualCartesianCoordinatesTCP.getY();
         qDebug()<<"wspolrzedna z tcp: "<<ActualCartesianCoordinatesTCP.getZ();
         qDebug()<<"Orientacja Rx tcp: "<<ActualCartesianCoordinatesTCP.getRx();
         qDebug()<<"Orientacja Ry tcp: "<<ActualCartesianCoordinatesTCP.getRy();
         qDebug()<<"Orientacja Rz tcp: "<<ActualCartesianCoordinatesTCP.getRz();*/

    /*qDebug()<<"docelowa pozycja x TCP: "<<TargetCartesianCoordinatesTCP.getX();
         qDebug()<<"docelowa pozycja y TCP: "<<TargetCartesianCoordinatesTCP.getY();
         qDebug()<<"docelowa pozycja z TCP: "<<TargetCartesianCoordinatesTCP.getZ();
         qDebug()<<"docelowa orientacja Rx TCP: "<<TargetCartesianCoordinatesTCP.getRx();
         qDebug()<<"docelowa orientacja Ry TCP: "<<TargetCartesianCoordinatesTCP.getRy();
         qDebug()<<"docelowa orientacja Rz TCP: "<<TargetCartesianCoordinatesTCP.getRz();*/

    // qDebug()<<"tryb robota: "<<RobotMode.getMode();
    // qDebug()<<"Timer dane przychodzą: "<< valT << ", czas przetw.: " << timerrr.elapsed();
    offset = 1060;
}

bool UR3Intermediator::ConnectToRobot()
{
    if (_connected == false)
    {
        qDebug() << "UR IP:" << IpAddress << "Port: " << Port;
        _socket->connectToHost(IpAddress,Port);
        if(_socket->waitForConnected(1000))
        {
            qDebug() << "Port:" << ackServerPort;

            ackServerSetup();

            _connected = true;
            qDebug()<<"Connect suceeded";
            startTimer(250);
        }
        else
        {
            _connected = false;

            _socket->deleteLater();
            _socket = new QTcpSocket();

            connect(this->_socket,SIGNAL(readyRead()),this,SLOT(OnSocketNewBytesWritten()));
            connect(this->_socket,SIGNAL(disconnected()),this,SLOT(onSlotDisconnect()));

            qDebug()<<"Connect failed";
        }        
    }
    emit ConnectionAction(IpAddress,_connected);
    return _connected;
}

void UR3Intermediator::DisconnectFromRobot()
{
    if (_connected == true)
    {
        cmds.clear();
        _ackLock = false;
        _socket->disconnectFromHost();
    }
}

void UR3Intermediator::onSlotDisconnect()
{
    _connected = false;
    _socket->deleteLater();
    _socket = new QTcpSocket();

    connect(this->_socket,SIGNAL(readyRead()),this,SLOT(OnSocketNewBytesWritten()));
    connect(this->_socket,SIGNAL(disconnected()),this,SLOT(onSlotDisconnect()));

    emit DisconnectionAction();
}

void UR3Intermediator::addCommandToExecuteList(QString cmd)
{
    QString command;
    command += "def newCommand():\n";
    command += "socketOpen = socket_open(\"" + _ackServerIP + "\"," + QString::number(ackServerPort) + ",\"ackSocket\")\n";
    command += cmd;
    command += "socket_send_byte(10,\"ackSocket\")\n";
    command += "socket_close(\"ackSocket\")\n";
    command += "end\n";

    qDebug() << command;
    cmds.push_back(command);
    _running = true;
}

QString UR3Intermediator::getAckServerIp(QString UR3IP)
{
    QHostAddress robot_ip = QHostAddress(UR3IP);
    QList<QNetworkInterface> interfaces = QNetworkInterface::allInterfaces();
    QList<QString> local_addresses;
    for (auto &ifc : interfaces) {
        if (ifc.flags() & QNetworkInterface::IsUp) {
            QList<QNetworkAddressEntry> addresses = ifc.addressEntries();
            for (auto &addr : addresses) {
                if(robot_ip.isInSubnet(QHostAddress::parseSubnet(addr.ip().toString() + "/" + addr.netmask().toString()))) {
                    local_addresses.push_back(addr.ip().toString());
                }
            }
        }
    }

    qDebug() << local_addresses;

    if(local_addresses.size() == 1)
    {
        return local_addresses[0];
    }else
    {
        qDebug() << "WRONG HOST IP ADDRESS";
    }

    return "localhost";
}

void UR3Intermediator::ackServerSetup()
{
    if(_ackServer == NULL)
    {
        _ackServer = new QTcpServer(this);
        _ackServerIP = getAckServerIp(IpAddress);

        _ackServer->listen(QHostAddress::Any, ackServerPort);

        connect(_ackServer, SIGNAL(newConnection()), this, SLOT(ackServerNewConnection()));
    }
}

void UR3Intermediator::ackServerNewConnection()
{
    if(_ackServerSocket != NULL)
    {
        _ackServerSocket->disconnect();
    }
    _ackServerSocket = _ackServer->nextPendingConnection();
    connect(_ackServerSocket, SIGNAL(readyRead()), this, SLOT(ackServerNewMsg()));
}

void UR3Intermediator::ackServerNewMsg()
{
    QByteArray array = _ackServerSocket->read(_ackServerSocket->bytesAvailable());
    if(array[0] == (char)10)
    {
        _ackLock = false;
        qDebug() << "Command execution finished";
        emit CommandFinished();
    }
}

void UR3Intermediator::OnSocketNewBytesWritten()
{
    this->ReadDataFlow();
    GetRobotData();
}
