#include "ur3wiz.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    UR3wiz w;
    w.show();

    return a.exec();
}
