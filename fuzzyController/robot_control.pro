TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    astar.cpp \
    fuzzycontroller.cpp \
    lidarsensor.cpp \
    marbledetection.cpp \
    pathplanning.cpp \
    pose.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv4

unix: PKGCONFIG += fuzzylite

HEADERS += \
    astar.h \
    callBackFunctions.h \
    fuzzycontroller.h \
    lidarsensor.h \
    marbledetection.h \
    pathplanning.h \
    pose.h
