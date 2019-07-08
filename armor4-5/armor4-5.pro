QT += core
QT -= gui

TARGET = armor4-5
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    armor.cpp \
    serial_commu.cpp \
    Structure.cpp

HEADERS += \
    armor.h \
    Structure.h
INCLUDEPATH += /usr/local/include \
/usr/local/include/opencv \
/usr/local/include/opencv2
QMAKE_CXXFLAGS += -std=c++11
LIBS += /usr/local/lib/libopencv_world.so.3.3\
/usr/local/lib/libopencv_world.so\
/usr/local/lib/libopencv_world.so.3.3.0
