#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

struct frame
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
    void setIP_o();
    void getIP_o();

    void setIP_u();
    void getIP_u();

    frame getPos();// pobiera dane o aktualnej pozycji markerów
    frame kalibracja();//wywołane oknem dialogowym skaluje położenie optitracka na ur3
    frame countPos();//oblicza pozycję ur3 ograniczoną kalibracją
    void sendPos(frame dane_u);//przesyła wskazanie miejsca w którym ma znaleźć się ur3


private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
