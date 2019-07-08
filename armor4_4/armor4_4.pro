QT += core
QT -= gui

TARGET = armor4_4
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    armor.cpp \
    Structure.cpp \
    serial_commu.cpp

HEADERS += \
    armor.h \
    Structure.h

QMAKE_CXXFLAGS += -std=c++11
INCLUDEPATH += /usr/local/include \
/usr/local/include/opencv \
/usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_world.so
