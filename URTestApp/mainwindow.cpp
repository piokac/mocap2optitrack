#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    UR3Intermediator_ = new UR3Intermediator("10.0.2.15",30002);


    connect(UR3Intermediator_, SIGNAL(robotEmergancyStoped()), this, SLOT(onRobotEmergancyStopped()));
    connect(UR3Intermediator_, SIGNAL(ConnectionAction(QString,bool)), this, SLOT(onRobotConnected()));
    connect(UR3Intermediator_, SIGNAL(DisconnectionAction()), this, SLOT(onRobotDisconnect()));
    connect(UR3Intermediator_, SIGNAL(CommandExecutionStart()), this, SLOT(robotStarted()));
    connect(UR3Intermediator_, SIGNAL(CommandFinished()), this, SLOT(robotFinished()));
    connect(UR3Intermediator_, SIGNAL(newPose(QVector<double>,char)), this, SLOT(newPoseFromRobot(QVector<double>,char)));

}

MainWindow::~MainWindow()
{
    delete ui;
}




void MainWindow::changeToRunninng()
{
    ui->r_state->setText("RUNNING");
}

void MainWindow::changeToIdle()
{
    ui->r_state->setText("IDLE");
}

void MainWindow::onRobotConnected()
{

    ui->r_state->setText("CONNECTED");
    ui->connect_r->setEnabled(false);
    ui->disconnect_r->setEnabled(true);
}

void MainWindow::onRobotDisconnect()
{
    ui->r_state->setText("DISCONNECTED");
    ui->connect_r->setEnabled(true);
    ui->disconnect_r->setEnabled(false);
}

void MainWindow::onRobotEmergancyStopped()
{
    ui->r_state->setText("EMERGENCY STOPPED");
}
void MainWindow::robotStarted()
{
    ui->r_state->setText("COMMAND STARTED");
}

void MainWindow::robotFinished()
{
    ui->r_state->setText("COMMAND FINISHED");
}


void MainWindow::newPoseFromRobot(QVector<double> data, char type)
{
    switch(type)
    {
    case 'j':{ //joint coordinates
        QString str = QString("Kąty w węzłach:q0:%1; q1:%2; q2:%3; q3:%4; q4:%5; q5:%6").arg(data[0],0,'g',2).arg(data[1],0,'g',2).arg(data[2],0,'g',2).arg(data[3],0,'g',2).arg(data[4],0,'g',2).arg(data[5],0,'g',2);
        ui->label_joints->setText(str);
    }
        break;
    case 'p':{ //TCP coordinates
        QString str = QString("Pozycja TCP: x:%1; y:%2; z:%3; Rx:%4;Ry:%5;Rz:%6").arg(data[0],0,'g',4).arg(data[1],0,'g',4).arg(data[2],0,'g',4).arg(data[3],0,'g',4).arg(data[4],0,'g',4).arg(data[5],0,'g',4);
        ui->label_pose->setText(str);
    }
        break;
    }
}




void MainWindow::on_pushButton_2_clicked()
{
    QVector<double> pos =   {0, -1.5707, 0, -1.5707, 0, 0};
    double jMoveAcceleration=1.0, jMoveSpeed=1.0;
    UR3Intermediator_->addCommandToExecuteList(UR3Intermediator_->MoveJ(pos, false,jMoveAcceleration, jMoveSpeed));

}



void MainWindow::on_connect_r_clicked()
{
    UR3Intermediator_->ConnectToRobot();
}

void MainWindow::on_disconnect_r_clicked()
{
    UR3Intermediator_->DisconnectFromRobot();
}
