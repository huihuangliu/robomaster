QT += core
QT -= gui

TARGET = box_detect
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += \
    matchtemplate.cpp

INCLUDEPATH += /usr/local/include \
/usr/local/include/opencv \
/usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_world.so

DISTFILES += \
    template_42.jpg

