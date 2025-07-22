#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(ui->startDriversButton, &QPushButton::clicked, this, &MainWindow::startDrivers);
    pollTimer.setInterval(2500);
    connect(&pollTimer, &QTimer::timeout, this, &MainWindow::pollDrivers);
}

void MainWindow::readDriverOutput() {
    const QString text = driverProcess->readAllStandardOutput();
    if (text.contains("OK")) lastStatus="OK";
    else if (text.contains("FAIL")) lastStatus="FAIL";
    ui->driverStatus->setText(lastStatus);
    if (lastStatus=="FAIL") QMessageBox::warning(this, "Driver error", "One or more drivers failed.");
}

void MainWindow::pollDrivers() {
    if (!driverProcess) return;
    driverProcess->write("status\n");
}

void MainWindow::startDrivers()
{
    if (driverProcess) return;
    driverProcess = new QProcess(this);
    connect(driverProcess, &QProcess::readyReadStandardOutput, this, &MainWindow::readDriverOutput);
    connect(driverProcess, &QProcess::errorOccurred, this, &MainWindow::driverCrashed);
    connect(ui->stopDriversButton, &QPushButton::clicked, this, &MainWindow::stopDrivers);
    // connect(driverProcess, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, &MainWindow::driverCrashed);
    driverProcess->start("/bin/bash", {"-c", "/home/kodifly/setup_scripts/camera_setup.sh"});
    pollTimer.start(); 
}

void MainWindow::stopDrivers() {
    pollTimer.stop();
    ui->driverStatus->setText("SHUT DOWN");
    driverProcess->deleteLater();
    driverProcess = nullptr;
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

