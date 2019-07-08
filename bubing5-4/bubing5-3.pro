QT += core
QT -= gui

TARGET = bubing5-3
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    armor.cpp \
    buff.cpp \
    serial_commu.cpp \
    Structure.cpp


INCLUDEPATH += /usr/local/include \
/usr/local/include/opencv \
/usr/local/include/opencv2
QMAKE_CXXFLAGS += -std=c++11
LIBS += /usr/local/lib/libopencv_world.so.3.3\
/usr/local/lib/libopencv_world.so\
/usr/local/lib/libopencv_world.so.3.3.0

LIBS += /home/allspark002/anaconda2/lib/libpython2.7.so

INCLUDEPATH += /home/allspark002/anaconda2/include/python2.7

HEADERS += \
    armor.cpp.autosave.mg1890 \
    armor.h \
    buff.h \
    serial_commu.hpp \
    Structure.h
