#-------------------------------------------------
#
# Project created by QtCreator 2018-07-09T20:30:38
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = UR3_WIZ
TEMPLATE = app

include(../UR3CPP/UR3CPP.pri)

DEFINES += QT_DEPRECATED_WARNINGS



SOURCES += \
        main.cpp \
        ur3wiz.cpp \
    connectdialog.cpp

HEADERS += \
        ur3wiz.h \
    connectdialog.h

FORMS += \
        ur3wiz.ui \
    connectdialog.ui
