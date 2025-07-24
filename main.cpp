#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    // any process that has called ros::init is a node
    // once that call succeeds, every publisher, subscriber or timer you create inside that process is part of the same node
    ros::init(argc, argv, "my_gui");
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
