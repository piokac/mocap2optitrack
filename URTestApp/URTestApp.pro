#-------------------------------------------------
#
# Project created by QtCreator 2017-08-29T09:52:00
#
#-------------------------------------------------

QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = URTestApp
TEMPLATE = app


include(../UR3CPP/UR3CPP.pri)


DEFINES += QT_DEPRECATED_WARNINGS

SOURCES += \
        main.cpp \
        mainwindow.cpp

HEADERS += \
        mainwindow.h

FORMS += \
        mainwindow.ui
