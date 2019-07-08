QT += core
QT -= gui

TARGET = serial_commu
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    serial_commu.cpp

HEADERS += \
    serial_commu.h

