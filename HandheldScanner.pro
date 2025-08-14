QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++14 link_pkgconfig rviz
PKGCONFIG += roscpp sensor_msgs diagnostic_updater

contains(CONFIG, rviz) {
    message("RViz integration enabled")
    PKGCONFIG += rviz
    DEFINES += HH_ENABLE_RVIZ
    SOURCES += \
        rvizwidget.cpp
    HEADERS += \
        rvizwidget.h
} else {
    message("RViz integration disabled. Pass CONFIG+=rviz to enable.")
}


# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

ROS_WS = /home/kodifly/handheld_scanner_ws/devel
INCLUDEPATH += $${ROS_WS}/include

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    rosbagrecorder.cpp \
    scanner_controller.cpp
    mapvizwidget.cpp

HEADERS += \
    mainwindow.h \
    diagnostics_monitor.h \
    rosbagrecorder.h \
    scanner_controller.h \
    process_config.h \
    qstring_hash.h \
    config.h \
    result.h
    mapvizwidget.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
