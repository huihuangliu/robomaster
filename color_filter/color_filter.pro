QT += core
QT -= gui

TARGET = color_filter
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += \
    test.cpp

INCLUDEPATH += /usr/local/include \
/usr/local/include/opencv \
/usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_world.so

