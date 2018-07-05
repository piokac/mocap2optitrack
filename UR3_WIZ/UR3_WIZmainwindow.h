#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include<cCliento>
#include<UR3Intermediator.h>
#include<Text2UR3>

struct toOptitrac
{
    float x,y,z;
};
struct toText2UR3
{
    float x,y,z;
};
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void setIPoptitrac();
    void getIPoptitrac();
    void sendDatatoOptitrac(toOptitrac toO);//wysyła ramkę z danymi do kalibracji
    void getOptiCoordinates();//pobiera współrzędne markera i wyświetla
    void calibrateOptitrac();//wyswietli okno i pobierze dane potrzebne Optitracowi
    //////////////////////
    void setIPur3();
    void getIPur3();
    void sendDatatoText(toText2UR3 toText);//wysyła ramkę z danymi do kalibracji
    void calibrateText();//wyświetli okno lub okna z instrukcjami do kalibracji ramienia
    void getText(QString text);//wyświetli okno dialogowe i pobierze tekst
    /////////////////////
     void getTcpCoordinates();//pobiera wpółrzędne tcp i wyświetla

private:
    Ui::MainWindow *ui;
    cCliento* clientO;
    UR3Intermediator* UR3mediator;
    Text2UR3* text2UR3;
};

#endif // MAINWINDOW_H
