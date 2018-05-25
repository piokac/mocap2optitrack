#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "UR3Intermediator.h"
#include <QVector3D>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:



private slots:
    void changeToRunninng();
    void changeToIdle();

    void onRobotConnected();
    void onRobotDisconnect();

    void onRobotEmergancyStopped();

    void on_connect_r_clicked();
    void newPoseFromRobot(QVector<double> data, char type);

    void on_pushButton_2_clicked();

    void on_disconnect_r_clicked();

    void robotStarted();
    void robotFinished();
signals:



private:
    Ui::MainWindow *ui;

    UR3Intermediator* UR3Intermediator_;
};

#endif // MAINWINDOW_H
