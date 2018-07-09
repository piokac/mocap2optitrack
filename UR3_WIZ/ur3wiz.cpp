#include "ur3wiz.h"
#include "ui_ur3wiz.h"

UR3wiz::UR3wiz(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::UR3wiz)
{
    ui->setupUi(this);
}

UR3wiz::~UR3wiz()
{
    delete ui;
}
