QT += core
QT -= gui

TARGET = version6
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    armor.cpp \
    serial_commu.cpp \
    Structure.cpp \
    buff.cpp

HEADERS += \
    armor.h \
    serial_commu.hpp \
    Structure.h \
    buff.h

QMAKE_CXXFLAGS += -std=c++11
LIBS += /usr/local/lib/libopencv_calib3d.so \
/usr/local/lib/libopencv_core.so \
/usr/local/lib/libopencv_dnn.so \
/usr/local/lib/libopencv_features2d.so \
/usr/local/lib/libopencv_flann.so \
/usr/local/lib/libopencv_highgui.so \
/usr/local/lib/libopencv_imgcodecs.so \
/usr/local/lib/libopencv_imgproc.so \
/usr/local/lib/libopencv_ml.so \
/usr/local/lib/libopencv_objdetect.so \
/usr/local/lib/libopencv_photo.so \
/usr/local/lib/libopencv_shape.so \
/usr/local/lib/libopencv_stitching.so \
/usr/local/lib/libopencv_superres.so \
/usr/local/lib/libopencv_video.so \
/usr/local/lib/libopencv_videoio.so \
/usr/local/lib/libopencv_videostab.so \
/usr/lib/x86_64-linux-gnu/libpthread.so \
/usr/lib/x86_64-linux-gnu/libstdc++.so.6

LIBS += -lX11 -pthread -lGL
LIBS += -L/usr/lib/x86_64-linux-gnu/libv4l/libv4l2convert.so
LIBS += -L/usr/lib/x86_64-linux-gnu/libv4l

LIBS += /home/imagegroup1/anaconda2/lib/libpython2.7.so

INCLUDEPATH += /home/imagegroup1/anaconda2/include/python2.7
