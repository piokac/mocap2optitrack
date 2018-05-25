#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    UR3Intermediator_ = new UR3Intermediator("127.0.0.1",30002);


    connect(UR3Intermediator_, SIGNAL(robotEmergancyStoped()), this, SLOT(onRobotEmergancyStopped()));
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
    QString str = QString("x:%1; y:%2; z:%3").arg(data[0],data[1],data[2]);
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
