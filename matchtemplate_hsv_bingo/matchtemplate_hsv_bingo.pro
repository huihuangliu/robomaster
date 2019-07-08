QT += core
QT -= gui

TARGET = matchtemplate_hsv_bingo
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    matchplate.cpp

HEADERS += \
    matchplate.h
INCLUDEPATH += /usr/local/include \
/usr/local/include/opencv \
/usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_world.so
