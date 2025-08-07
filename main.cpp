#include "mainwindow.h"
#include "process_config.h"

#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    initializeProcesses();
    ros::init(argc, argv, "my_gui");
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
