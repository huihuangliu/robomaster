QT += core
QT -= gui

TARGET = box_detect_new
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp

INCLUDEPATH += /usr/local/include \
/usr/local/include/opencv \
/usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_world.so
