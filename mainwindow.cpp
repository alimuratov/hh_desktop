#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(ui->startButton, &QPushButton::clicked, this, &MainWindow::startDrivers);
}

void MainWindow::startDrivers()
{
    qDebug() << "Drivers started successfully!";
    if (driverProcess) return;
    driverProcess = new QProcess(this);
    connect(driverProcess, &QProcess::readyReadStandardOutput, this, &MainWindow::readDriverOutput);
    connect(driverProcess, &QProcess::errorOccurred, this, &MainWindow::driverCrashed);
    connect(driverProcess, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, &MainWindow::driverCrashed); 
    driverProcess->start("/bin/bash", {"-c", "/path/to/launch_drivers.sh"});
    pollTimer.start(); 
}

void MainWindow::driverCrashed() {
    pollTimer.stop(); 
    ui->driverStatus->setText("OFF");
    QMessageBox::warning(this, "Driver stopped", "Camera/LiDAR has stopped running.");
    driverProcess->deleteLater();
    driverProcess = nullptr;
}

MainWindow::~MainWindow()
{
    delete ui;
}

