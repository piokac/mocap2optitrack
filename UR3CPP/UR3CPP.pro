#-------------------------------------------------
#
# Project created by QtCreator 2017-05-26T16:04:29
#
#-------------------------------------------------

QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = UR3CPP
TEMPLATE = app
include (../settings/settings.pri)

SOURCES += main.cpp\
        mainwindow.cpp \
    UR3MessageOut.cpp \
    UR3Intermediator.cpp \
    UR3Message.cpp \
    connectdialog.cpp

HEADERS  += mainwindow.h \
    UR3MessageOut.h \
    UR3Intermediator.h \
    UR3Message.h \
    connectdialog.h

FORMS    += mainwindow.ui \
    connectdialog.ui
