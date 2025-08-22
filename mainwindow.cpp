#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <signal.h>
#include <errno.h>
#ifdef HH_ENABLE_RVIZ
#include <QVBoxLayout>
#endif

// anonymous namespace grants internal linkage to everything declared inside it, meaning that the constants exist only within this translation unit
// implications:
// 1. no other cpp file can collide with these names
// 2. constants are not exported
// 3. we avoid one-definition-rule headaches that arise if you move the constants to a header
// we can freely change their values here without forcing a full rebuild of every file that includes the header
// and there is zero risk of multiple definitions at link time

namespace
{
    constexpr char kPtp4lKey[] = "ptp4l";
    constexpr char kCameraKey[] = "camera";
    constexpr char kLidarKey[] = "lidar";
    constexpr char kGpsKey[] = "gpsrtk";
    constexpr char kWatchdogKey[] = "watchdog";
    constexpr char kSlamKey[] = "slam";
    constexpr char kRoscoreKey[] = "roscore";
    constexpr char kDynamicReconfigureKey[] = "dynamic_reconfigure";
    constexpr char kMapvizKey[] = "mapviz";
#ifdef HH_ENABLE_RVIZ
    constexpr char kRvizConfig[] = "/home/kodifly/hh_desktop/config/view.rviz";
#endif
}

void MainWindow::updateUiState()
{
    const bool canStartSlam = scanner_->canStart("slam");
    const bool canStartDynReconf = scanner_->canStart("dynamic_reconfigure");
    const bool canStartMapviz = scanner_->canStart("mapviz");

    const bool slamRunning = scanner_->isRunning("slam");
    ui->startSlamButton->setEnabled(canStartSlam && !slamRunning);
    ui->stopSlamButton->setEnabled(slamRunning);

    const bool dynRunning = scanner_->isRunning("dynamic_reconfigure");
    ui->startDynamicReconfigureButton->setEnabled(canStartDynReconf && !dynRunning);
    ui->stopDynamicReconfigureButton->setEnabled(dynRunning);

    const bool mapvizRunning = scanner_->isRunning("mapviz");
    ui->startMapvizButton->setEnabled(canStartMapviz && !mapvizRunning);
    ui->stopMapvizButton->setEnabled(mapvizRunning);
}

// -------------------------- Color Helper --------------------------
static QColor levelToColor(uint8_t level)
{
    using diagnostic_msgs::DiagnosticStatus;
    switch (level)
    {
    case DiagnosticStatus::OK:
        return QColor("#2ECC71"); // green
    case DiagnosticStatus::WARN:
        return QColor("#F1C40F"); // yellow
    case DiagnosticStatus::ERROR:
        return QColor("#E74C3C"); // red
    default:
        return QColor("#95A5A6"); // grey
    }
}

// -------------------------- --------------------------

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->prevPageButton->setEnabled(false);
    if (ui->stackedWidget->count() <= 1)
    {
        ui->nextPageButton->setEnabled(false);
    }

#ifdef HH_ENABLE_RVIZ
    rviz_widget_ = std::make_unique<RvizWidget>(QString::fromUtf8(kRvizConfig), this);
    auto rvizLayout = new QVBoxLayout(ui->rvizContainer);
    rvizLayout->setContentsMargins(0, 0, 0, 0);
    rvizLayout->addWidget(rviz_widget_.get());
#else
    ui->rvizContainer->setVisible(false);
#endif

    ros::NodeHandle nh;

    scanner_ = std::make_unique<ScannerController>(nh, this);

    // Wire UI and controller signals
    setupUiActions();
    setupControllerSignals();
    updateUiState();
}

void MainWindow::setupUiActions()
{
    connect(ui->nextPageButton, &QPushButton::clicked, this, &MainWindow::showNextPage);
    connect(ui->prevPageButton, &QPushButton::clicked, this, &MainWindow::showPrevPage);

    connect(ui->startDriversButton, &QPushButton::clicked, this, &MainWindow::startDrivers);
    connect(ui->stopDriversButton, &QPushButton::clicked, this, &MainWindow::stopDrivers);
    connect(ui->startSlamButton, &QPushButton::clicked, this, &MainWindow::startSlam);
    connect(ui->stopSlamButton, &QPushButton::clicked, this, &MainWindow::stopSlam);
    connect(ui->startDynamicReconfigureButton, &QPushButton::clicked, this, &MainWindow::startDynamicReconfigure);
    connect(ui->stopDynamicReconfigureButton, &QPushButton::clicked, this, &MainWindow::stopDynamicReconfigure);

    connect(ui->startMapvizButton, &QPushButton::clicked, this, &MainWindow::startMapviz);
    connect(ui->stopMapvizButton, &QPushButton::clicked, this, &MainWindow::stopMapviz);
    
    // Checkbox handlers use Qt's auto-connect via on_<objectName>_stateChanged

    connect(ui->startRecordingButton, &QPushButton::clicked, this, [this]
            {
        if (scanner_->isRecording()) {
            QMessageBox::warning(this, "Record Warning", tr("Stop recording before starting a new one."));
            return;
        } else if (recordTopics_.isEmpty()) {
            QMessageBox::warning(this, "Record Warning", tr("Select at least one topic."));
            return;
        }
        
        QStringList topics = recordTopics_.values();
        scanner_->startRecording("my_bag", topics); });

    connect(ui->stopRecordingButton, &QPushButton::clicked,
            scanner_.get(), &ScannerController::stopRecording);
}

void MainWindow::setupControllerSignals()
{
    connect(scanner_.get(), &ScannerController::driverStarted,
            this, &MainWindow::onDriverStarted);
    connect(scanner_.get(), &ScannerController::driverStopped,
            this, &MainWindow::onDriverStopped);
    connect(scanner_.get(), &ScannerController::driverCrashed,
            this, &MainWindow::onDriverCrashed);
    connect(scanner_.get(), &ScannerController::driverError,
            this, [this](const QString &key, const QString &error)
            { QMessageBox::warning(this, "Driver Error", QString("%1: %2").arg(key, error)); });
    connect(scanner_.get(), &ScannerController::driverOutput,
            this, &MainWindow::onDriverOutput);
    connect(scanner_.get(), &ScannerController::diagnosticsUpdated,
            this, &MainWindow::onDiagStatus);
    connect(scanner_.get(), &ScannerController::recordingStarted,
            this, &MainWindow::onRecordingStarted);
    connect(scanner_.get(), &ScannerController::recordingStopped,
            this, &MainWindow::onRecordingStopped);
    connect(scanner_.get(), &ScannerController::recordingError,
            this, [this](const QString &err)
            { QMessageBox::warning(this, "Record Warning", err); });
}

// -------------------------- Buttons --------------------------

void MainWindow::startDrivers()
{
    scanner_->startDrivers();
    ui->roscoreStatus->setText(tr("Starting..."));
    updateUiState();
}

void MainWindow::stopDrivers()
{
    scanner_->stopDrivers();
    updateUiState();
}

void MainWindow::startSlam()
{
    scanner_->startSlam();
    updateUiState();
}

void MainWindow::stopSlam()
{
    scanner_->stopSlam();
    updateUiState();
}

void MainWindow::startDynamicReconfigure()
{
    scanner_->startDynamicReconfigure();
    updateUiState();
}

void MainWindow::stopDynamicReconfigure()
{
    scanner_->stopDynamicReconfigure();
    updateUiState();
}

void MainWindow::startMapviz()
{
    scanner_->startMapviz();
    updateUiState();
}

void MainWindow::stopMapviz()
{
    scanner_->stopMapviz();
    updateUiState();
}

// -------------------------- Diagnostics --------------------------

void MainWindow::onDiagStatus(const QString &name, int level, const QString &msg, const QString &recordedFrequency)
{
    qDebug() << "Name: " << name << " Level: " << level << " Message: " << msg;

    // create a file and insert all of the parameters
    QFile file("diagnostics.txt");
    if (!file.open(QIODevice::Append | QIODevice::Text))
    {
        qWarning() << "Failed to open diagnostics file for writing.";
        return;
    }
    QTextStream out(&file);
    out << "Name: " << name << ", Level: " << level << ", Message: " << msg
        << ", Frequency: " << recordedFrequency << "\n";
    file.close();

    const QString sev = (level == 0)   ? "OK"
                        : (level == 1) ? "WARN"
                        : (level == 2) ? "ERROR"
                                       : "STALE";

    QLabel *targetLabel;
    if (name == "river_watchdog: camera_rate")
    {
        targetLabel = ui->cameraStatus;
    }
    else if (name == "river_watchdog: lidar_rate")
    {
        targetLabel = ui->lidarStatus;
    }
    else if (name == "river_watchdog: gps_rate")
    {
        targetLabel = ui->gpsStatus;
    }
    else if (name == "river_watchdog: Offset Accuracy")
    {
        // print a message to the console
        //qDebug() << "------------------------ AAAAA ------------------------ ";
        targetLabel = ui->syncStatus;
    }
    else
    {
        return;
    }

    targetLabel->setText(tr("%1: %2").arg(sev, msg));
    targetLabel->setStyleSheet(QStringLiteral("color:%1;")
                                   .arg(levelToColor(level).name()));
}

// -------------------------- Driver Slots --------------------------
void MainWindow::onDriverOutput(const QString & /*key*/, const QString &output)
{
    qDebug().noquote() << output;
}

void MainWindow::onDriverStarted(const QString &key)
{
    if (key == kRoscoreKey)
    {
        ui->roscoreStatus->setText(tr("Running"));
        ui->roscoreStatus->setStyleSheet("color:green;");
    }
    else if (key == kCameraKey)
    {
    }
    else if (key == kLidarKey)
    {
    }
    else if (key == kGpsKey)
    {
    }
    else if (key == kPtp4lKey)
    {
    }
    else if (key == kSlamKey)
    {
    }
    else if (key == kDynamicReconfigureKey)
    {
    }
    else if (key == kMapvizKey)
    {
    }
    updateUiState();
}

void MainWindow::onDriverStopped(const QString &key)
{
    if (key == kRoscoreKey)
    {
        ui->roscoreStatus->setText(tr("Roscore stopped."));
        ui->roscoreStatus->setStyleSheet("");
    }
    else if (key == kCameraKey)
    {
        ui->cameraStatus->setText(tr("Camera driver stopped."));
        ui->cameraStatus->setStyleSheet("");
    }
    else if (key == kLidarKey)
    {
        ui->lidarStatus->setText(tr("Lidar driver stopped."));
        ui->lidarStatus->setStyleSheet("");
    }
    else if (key == kGpsKey)
    {
        ui->gpsStatus->setText(tr("GPS driver stopped."));
        ui->gpsStatus->setStyleSheet("");
    }
    else if (key == kPtp4lKey)
    {
        // offsetRunning = false;
        ui->syncStatus->setText(tr("ptp stopped."));
        ui->syncStatus->setStyleSheet("");
    }
    else if (key == kSlamKey)
    {
    }
    else if (key == kDynamicReconfigureKey)
    {
    }
    else if (key == kMapvizKey)
    {
    }
    updateUiState();
}

void MainWindow::onDriverCrashed(const QString &key)
{
    QMessageBox::warning(this, "Process Failure", tr("%1 has stopped running.").arg(key));
    if (key == kRoscoreKey)
    {
        ui->roscoreStatus->setText(tr("Roscore crashed."));
        ui->roscoreStatus->setStyleSheet("color:red;");
    }
    else if (key == kCameraKey || key == kLidarKey)
    {
        ui->startSlamButton->setEnabled(false);
    }
}

// -------------------------- closeEvent --------------------------

void MainWindow::closeEvent(QCloseEvent *event)
{
    scanner_->stopDrivers();
    QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow() = default;

void MainWindow::on_cameraCheckbox_stateChanged(int checked)
{
    if (checked == 0)
    {
        recordTopics_.remove("/left_camera/image");
    }
    else if (checked == 2)
    {
        recordTopics_ << "/left_camera/image";
    }
}

void MainWindow::on_lidarCheckbox_stateChanged(int checked)
{
    if (checked == 0)
    {
        recordTopics_.remove("/livox/lidar");
    }
    else if (checked == 2)
    {
        recordTopics_ << "/livox/lidar";
    }
}

void MainWindow::onRecordingStarted()
{
    ui->recordStatus->setText("Recording...");
}

void MainWindow::onRecordingStopped()
{
    ui->recordStatus->setText("OFF");
}

void MainWindow::showNextPage()
{
    int index = ui->stackedWidget->currentIndex();
    if (index < ui->stackedWidget->count() - 1)
    {
        ui->stackedWidget->setCurrentIndex(index + 1);
    }
    ui->prevPageButton->setEnabled(true);
    ui->nextPageButton->setEnabled(ui->stackedWidget->currentIndex() < ui->stackedWidget->count() - 1);
}

void MainWindow::showPrevPage()
{
    int index = ui->stackedWidget->currentIndex();
    if (index > 0)
    {
        ui->stackedWidget->setCurrentIndex(index - 1);
    }
    ui->nextPageButton->setEnabled(true);
    ui->prevPageButton->setEnabled(ui->stackedWidget->currentIndex() > 0);
}
