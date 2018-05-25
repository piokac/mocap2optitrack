#include "UR3Workspace.h"

QVector<double> rot2rv(QMatrix4x4 rot)
{
    float theta = acos((rot(0,0)+rot(1,1)+rot(2,2)-1)/2);
    float sth = sin(theta);
    float kx = (rot(2,1)-rot(1,2))/(2.0*sth);
    float ky = (rot(0,2)-rot(2,0))/(2.0*sth);
    float kz = (rot(1,0)-rot(0,1))/(2.0*sth);

    return {theta*kx, theta*ky, theta*kz};
}

UR3Workspace::UR3Workspace()
{
    m_enableBinArrayMode_ = false;
    m_binArraySize_ = 6;

    m_pickingObjectOffsetInMM_ = 0;
    m_puttingObjectOffsetInMM_ = 5;
    m_approachOffsetInMM_ = 10;

    m_jMoveAcceleration_ = 1.0;
    m_jMoveSpeed_ = 1.0;
    m_lMoveToolAcceleration_ = 0.6;
    m_lMoveToolSpeed_ = 0.1;

    UR3Intermediator_ = new UR3Intermediator();

    homePosition_ = {0, -1.5707, 0, -1.5707, 0, 0};
    boardPosition_ = {0, -1.5707, 0, -1.5707, 0, 0};
    overBinPosition_ = {0, -1.5707, 0, -1.5707, 0, 0};

    calibrationPoints_[0] = {0,0,0,0,0,0};
    calibrationPoints_[1] = {0,0,0,0,0,0};
    calibrationPoints_[2] = {0,0,0,0,0,0};

    arrayStartPosition_ = {0,0,0,0,0,0};
    arrayEndPosition_ = {0,0,0,0,0,0};

    binDescriptors_.clear();

    calibMatrix_.setToIdentity();

    workingTCP_ = {0,0,0.066,0,3.14159,0};
    calibrationTCP_ = {0.06,0,0.016,0,0,0};

    connect(UR3Intermediator_, SIGNAL(robotEmergancyStoped()), this, SLOT(onRobotEmergencyStopped()));
    connect(UR3Intermediator_, SIGNAL(DisconnectionAction()), this, SLOT(onRobotDisconnected()));
    connect(UR3Intermediator_, SIGNAL(CommandExecutionStart()), this, SLOT(robotStarted()));
    connect(UR3Intermediator_, SIGNAL(CommandFinished()), this, SLOT(robotFinished()));
    connect(UR3Intermediator_, SIGNAL(newPose(QVector<double>,char)), this, SLOT(newPoseFromRobot(QVector<double>,char)));
}

void UR3Workspace::setBinPosition(int key, QVector<double> cartesianData)
{
    if(cartesianData.size() == 6)
    {
        binDescriptors_[key].position = cartesianData;
    }
}

void UR3Workspace::setArrayStartPosition(QVector<double> cartesianData)
{
    if(cartesianData.size() == 6)
    {
        arrayStartPosition_ = cartesianData;
    }
}

void UR3Workspace::setArrayEndPosition(QVector<double> cartesianData)
{
    if(cartesianData.size() == 6)
    {
        arrayEndPosition_ = cartesianData;
    }
}

void UR3Workspace::setHomePosition(QVector<double> jointData)
{
    if(jointData.size() == 6)
    {
        homePosition_ = jointData;
    }
}

void UR3Workspace::setOverBinPosition(QVector<double> jointData)
{
    if(jointData.size() == 6)
    {
        overBinPosition_ = jointData;
    }
}

void UR3Workspace::setBoardPosition(QVector<double> jointData)
{
    if(jointData.size() == 6)
    {
        boardPosition_ = jointData;
    }
}

void UR3Workspace::setCalibrationPoint(int key, QVector<double> cartesianData)
{
    if(calibrationPoints_.contains(key) && cartesianData.size() == 6)
    {
        calibrationPoints_[key] = cartesianData;
    }
}

void UR3Workspace::setCalibrationTCP()
{
    UR3Intermediator_->addCommandToExecuteList(UR3Intermediator_->setTCP(calibrationTCP_));
}

void UR3Workspace::setWorkingTCP()
{
    UR3Intermediator_->addCommandToExecuteList(UR3Intermediator_->setTCP(workingTCP_));
}

void UR3Workspace::setToolOutput(int output, bool state)
{
    UR3Intermediator_->addCommandToExecuteList(UR3Intermediator_->setToolOutput(output, state));
}

void UR3Workspace::moveToWorkspaceHome()
{
    UR3Intermediator_->addCommandToExecuteList(UR3Intermediator_->MoveJ(homePosition_, false, m_jMoveAcceleration_, m_jMoveSpeed_));
}

void UR3Workspace::moveOverBoard()
{
    UR3Intermediator_->addCommandToExecuteList(UR3Intermediator_->MoveJ(boardPosition_, false, m_jMoveAcceleration_, m_jMoveSpeed_));
}

void UR3Workspace::moveOverBin()
{
    UR3Intermediator_->addCommandToExecuteList(UR3Intermediator_->MoveJ(overBinPosition_, false, m_jMoveAcceleration_, m_jMoveSpeed_));
}

void UR3Workspace::pickObject(QVector3D pointInMeters, float objectHeightInMeters, float angle, int bin)
{
    if(binDescriptors_.contains(bin))
    {
        QString command;

        command += UR3Intermediator_->setTCP(workingTCP_);

        command += UR3Intermediator_->MoveJ(homePosition_, false, m_jMoveAcceleration_, m_jMoveSpeed_);
        command += UR3Intermediator_->setToolOutput(0, false);
        command += UR3Intermediator_->setToolOutput(1, false);
        command += UR3Intermediator_->setOutput(0, false);
        command += UR3Intermediator_->setOutput(1, false);
        command += UR3Intermediator_->MoveJ(boardPosition_, false, m_jMoveAcceleration_, m_jMoveSpeed_, 0.05);

        pointInMeters[2] += objectHeightInMeters;
        pointInMeters[2] += m_pickingObjectOffsetInMM_/1000.0;

        QVector3D approachPoint = pointInMeters;
        approachPoint[2] += m_approachOffsetInMM_/1000.0;
        command += UR3Intermediator_->MoveJ(getBoardPointInBaseCoordinates(approachPoint, angle),true, m_jMoveAcceleration_, m_jMoveSpeed_);

        command += UR3Intermediator_->MoveL(getBoardPointInBaseCoordinates(pointInMeters, angle), m_lMoveToolAcceleration_, m_lMoveToolSpeed_);
        command += UR3Intermediator_->setToolOutput(0, true);
        command += UR3Intermediator_->setOutput(1, true);
        command += UR3Intermediator_->robotSleep(0.8);
        command += UR3Intermediator_->setOutput(0, true);
        command += UR3Intermediator_->setToolOutput(1, true);
        command += UR3Intermediator_->robotSleep(0.2);

        command += UR3Intermediator_->MoveL(getBoardPointInBaseCoordinates(approachPoint, angle), m_lMoveToolAcceleration_,  m_lMoveToolSpeed_);

        command += UR3Intermediator_->MoveJ(boardPosition_, false, m_jMoveAcceleration_, m_jMoveSpeed_, 0.05);
        command += UR3Intermediator_->MoveJ(homePosition_, false, m_jMoveAcceleration_, m_jMoveSpeed_, 0.1);

        QVector<double> binPoint = binDescriptors_[bin].position;
        binPoint[3] = 0;
        binPoint[4] = 0;
        binPoint[5] = 0;
        binPoint[2] += objectHeightInMeters;
        binPoint[2] += m_puttingObjectOffsetInMM_/1000.0;

        QVector<double> binApproachPoint = binPoint;
        binApproachPoint[2] += m_approachOffsetInMM_/1000.0;

        command += UR3Intermediator_->MoveJ(overBinPosition_, false, m_jMoveAcceleration_, m_jMoveSpeed_, 0.05);
        command += UR3Intermediator_->MoveJ(binApproachPoint, true, m_jMoveAcceleration_, m_jMoveSpeed_);
        command += UR3Intermediator_->MoveL(binPoint, m_lMoveToolAcceleration_,  m_lMoveToolSpeed_);
        command += UR3Intermediator_->setToolOutput(0, false);
        command += UR3Intermediator_->setToolOutput(1, false);
        command += UR3Intermediator_->setOutput(0, false);
        command += UR3Intermediator_->setOutput(1, false);
        command += UR3Intermediator_->robotSleep(1);

        command += UR3Intermediator_->MoveL(binApproachPoint, m_lMoveToolAcceleration_, m_lMoveToolSpeed_);
        command += UR3Intermediator_->MoveJ(overBinPosition_, false, m_jMoveAcceleration_, m_jMoveSpeed_, 0.05);
        command += UR3Intermediator_->MoveJ(homePosition_, false, m_jMoveAcceleration_, m_jMoveSpeed_);

        UR3Intermediator_->addCommandToExecuteList(command);
    }else
    {
        qDebug() << "UR3Workspace::pickObject:: No such bin:" << bin;
    }
}

void UR3Workspace::connectToRobot()
{
    if(UR3Intermediator_->ConnectToRobot())
    {
        emit connectedToRobot(true);
        setWorkingTCP();
    }
    else
    {
        emit connectedToRobot(false);
    }
}

void UR3Workspace::disconnectFromRobot()
{
    UR3Intermediator_->DisconnectFromRobot();
}

void UR3Workspace::onRobotDisconnected()
{
    emit disconnectedFromRobot();
}

void UR3Workspace::onRobotEmergencyStopped()
{
    emit robotEmergencyStopped();
}


void UR3Workspace::writeXml(QXmlStreamWriter &xml)
{
    xml.writeStartElement(sXmlKey());
    metaDataXmlWriter(xml,*this);

    xml.writeStartElement("Positions");
    writeCsvVector(xml,"Home",homePosition_);
    writeCsvVector(xml,"OverBoard",boardPosition_);
    writeCsvVector(xml,"OverBins",overBinPosition_);
    if(enableBinArrayMode_() == false)
    {
        xml.writeStartElement("Bins");
        for(int i=0; i<binDescriptors_.size(); i++)
        {
            xml.writeCharacters(binDescriptors_[i].toString() + "\n");
        }
        xml.writeEndElement();
    }else
    {
        writeCsvData(xml,"BinArray",{arrayStartPosition_, arrayEndPosition_});
    }
    xml.writeEndElement();

    xml.writeStartElement("Calibrations");
    writeCsvData(xml,"CalibrationPoints",{calibrationPoints_[0], calibrationPoints_[1], calibrationPoints_[2]});
    xml.writeEndElement();

    xml.writeStartElement("TCPs");
    writeCsvVector(xml,"WorkingTCP",workingTCP_);
    writeCsvVector(xml,"CalibrationTCP",calibrationTCP_);
    xml.writeEndElement();

    xml.writeEndElement();
}

void UR3Workspace::readXml(QXmlStreamReader& stream)
{
    while(!(stream.tokenType() == QXmlStreamReader::EndElement && stream.name() == sXmlKey()))
    {
        cXmlIO::readXml(stream);
        if(stream.tokenType() == QXmlStreamReader::StartElement)
        {
            if(stream.name()=="Home")
            {
                QVector<double> temp;
                readCsvVector(stream, temp);
                homePosition_ = temp;
            }

            if(stream.name()=="OverBoard")
            {
                QVector<double> temp;
                readCsvVector(stream, temp);
                boardPosition_ = temp;
            }

            if(stream.name()=="OverBins")
            {
                QVector<double> temp;
                readCsvVector(stream, temp);
                overBinPosition_ = temp;
            }

            if(stream.name()=="Bins")
            {
                binDescriptors_.clear();

                QString bins = stream.readElementText();
                QStringList list = bins.split('\n', QString::SkipEmptyParts);
                for(int i=0; i<list.size(); i++)
                {
                    BinDescriptor bin;
                    bin.fromString(list.at(i));
                    binDescriptors_[bin.id] = bin;
                }
            }

            if(stream.name()=="CalibrationPoints")
            {
                QVector<QVector<double> > points;
                readCsvData(stream, points);
                if(points.size() > 3)
                {
                    calibrationPoints_[0] = points[0];
                    calibrationPoints_[1] = points[1];
                    calibrationPoints_[2] = points[2];

                    recalculateCalibration();
                }
            }

            if(stream.name()=="BinArray")
            {
                QVector<QVector<double> > points;
                readCsvData(stream, points);
                if(points.size() > 2)
                {
                    arrayStartPosition_ = points[0];
                    arrayEndPosition_ = points[1];

                    recalculateBinArray();
                }
            }

            if(stream.name()=="WorkingTCP")
            {
                QVector<double> temp;
                readCsvVector(stream, temp);
                workingTCP_ = temp;
            }

            if(stream.name()=="CalibrationTCP")
            {
                QVector<double> temp;
                readCsvVector(stream, temp);
                calibrationTCP_ = temp;
            }
        }

        stream.readNext();
    }
}

QVector<double> UR3Workspace::getBoardPointInBaseCoordinates(QVector3D point, float angle)
{
    QVector4D boardPoint = QVector4D(point, 1.0);
    QVector4D fromBaseCordinates = calibMatrix_ * boardPoint;

    QVector<double> robotCartVector = {fromBaseCordinates[0], fromBaseCordinates[1], fromBaseCordinates[2]};

    angle -= pickArrayRotAngle_;

    QMatrix4x4 zRotation = QMatrix4x4(cos(angle), -sin(angle), 0, 0,
                                      sin(angle), cos(angle),  0, 0,
                                      0,          0,           1, 0,
                                      0,          0,           0, 1);
    QVector<double> RV = rot2rv(calibMatrix_ * zRotation);

    robotCartVector.append(RV);

    return robotCartVector;
}

void UR3Workspace::newPoseFromRobot(QVector<double> data, char type)
{
    emit newPose(data, type);
}

void UR3Workspace::robotStarted()
{
    emit commandStarted();
}

void UR3Workspace::robotFinished()
{
    emit commandExecuted();
}

UR3Workspace::~UR3Workspace()
{
    UR3Intermediator_->DisconnectFromRobot();
}

void UR3Workspace::recalculateCalibration()
{
    QVector<double> temp = calibrationPoints_[0];
    QVector3D P0 = QVector3D(temp[0], temp[1], temp[2]);
    temp = calibrationPoints_[1];
    QVector3D P1 = QVector3D(temp[0], temp[1], temp[2]);
    temp = calibrationPoints_[2];
    QVector3D P2 = QVector3D(temp[0], temp[1], temp[2]);

    QVector3D V1 = P1 - P0;
    QVector3D V2 = P2 - P0;

    V1.normalize();
    V2.normalize();

    QVector3D X = V1;
    QVector3D Z = QVector3D::crossProduct(V1, V2);
    QVector3D Y = QVector3D::crossProduct(Z, X);

    calibMatrix_ = QMatrix4x4(X[0], Y[0], Z[0], P0[0],
                             X[1], Y[1], Z[1], P0[1],
                             X[2], Y[2], Z[2], P0[2],
                             0   , 0   , 0   , 1);
}

void UR3Workspace::recalculateBinArray()
{
    QVector3D PStart = QVector3D(arrayStartPosition_[0], arrayStartPosition_[1], arrayStartPosition_[2]);
    QVector3D PEnd = QVector3D(arrayEndPosition_[0], arrayEndPosition_[1], arrayEndPosition_[2]);

    QVector3D dP = PEnd - PStart;
    QVector3D Pi;

    for(int i=0; i<m_binArraySize_; i++)
    {
        binDescriptors_[i+1].id = i+1;
        binDescriptors_[i+1].name = "Bin_" + QString::number(i+1);
        Pi = PStart + (i * dP) / (m_binArraySize_ - 1);

        binDescriptors_[i+1].position = {Pi[0], Pi[1], Pi[2], 0, 0, 0};
    }

    pickArrayRotAngle_ = atan2(dP[1], dP[0]);
}

QString BinDescriptor::toString()
{
    QString str = "";
    if(position.size() == 6)
    {
        str += QString::number(id) + ";" + name + ";";
        for(int i=0; i<6; i++)
        {
            if(i > 0)
            {
                str += ",";
            }
            str += QString::number(position[i]);
        }
    }
    return str;
}

void BinDescriptor::fromString(QString str)
{
    QStringList elements = str.split(";", QString::SkipEmptyParts);
    if(elements.size() == 3)
    {
        id = elements.at(0).toInt();
        name = elements.at(1);
        position.clear();
        QStringList pos = elements.at(2).split(",", QString::SkipEmptyParts);
        if(pos.size() == 6)
        {
            for(int i=0; i<6; i++)
            {
                position.push_back(pos.at(i).toDouble());
            }
        }
    }

}
