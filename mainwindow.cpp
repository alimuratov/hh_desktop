#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "modern_style.h"
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

namespace {
    constexpr char kCameraKey[] = "camera";
    constexpr char kLidarKey[] = "lidar";
    constexpr char kWatchdogKey[]  = "watchdog";
    constexpr char kSlamKey[] = "slam";
    constexpr char kRoscoreKey[] = "roscore";
    constexpr char kDynamicReconfigureKey[] = "dynamic_reconfigure";
#ifdef HH_ENABLE_RVIZ
      constexpr char kRvizConfig[] = "/home/kodifly/hh_desktop/config/view.rviz";
#endif
  }

// -------------------------- Material Design Colors --------------------------
static QColor materialColor(const QString& state) {
    if (state == "success" || state == "ok") return QColor("#4CAF50");      // Material Green
    if (state == "warning" || state == "warn") return QColor("#FF9800");    // Material Orange  
    if (state == "error" || state == "crash") return QColor("#F44336");     // Material Red
    if (state == "info" || state == "starting") return QColor("#2196F3");   // Material Blue
    return QColor("#9E9E9E");  // Material Grey for default/stopped
}

static QColor levelToColor(uint8_t level) {
  using diagnostic_msgs::DiagnosticStatus;
  switch (level) {
    case DiagnosticStatus::OK:    return materialColor("success");
    case DiagnosticStatus::WARN:  return materialColor("warning");
    case DiagnosticStatus::ERROR: return materialColor("error");
    default:                      return materialColor("default");
  }
}

// Material Design status update with elevation and proper typography
static void updateStatusLabel(QLabel* label, const QString& text, const QColor& color) {
    label->setText(text);
    // Material Design Card with elevation 2
    label->setStyleSheet(QString(
        "QLabel { "
        "background-color: %1; "
        "color: #FFFFFF; "
        "border: none; "
        "border-radius: 8px; "
        "padding: 16px; "
        "font-weight: 500; "
        "font-size: 14px; "
        "letter-spacing: 0.25px; "
        "box-shadow: 0px 3px 1px -2px rgba(0,0,0,0.2), "
        "            0px 2px 2px 0px rgba(0,0,0,0.14), "
        "            0px 1px 5px 0px rgba(0,0,0,0.12); "
        "}"
    ).arg(color.name()));
}   

// -------------------------- --------------------------

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    // Apply Material Design styling
    // this->setStyleSheet(MaterialStyle::getStyleSheet());
    
    // Set window properties for a more modern look
    this->setWindowTitle("Handheld Scanner Control");
    this->setMinimumSize(900, 700);
    
    // Add tooltips for better UX
    ui->startDriversButton->setToolTip("Start all essential drivers (ROS Core, Camera, Lidar, Watchdog)");
    ui->stopDriversButton->setToolTip("Stop all running drivers");
    ui->startSlamButton->setToolTip("Start SLAM mapping (requires Camera and Lidar)");
    ui->stopSlamButton->setToolTip("Stop SLAM mapping");
    ui->startDynamicReconfigureButton->setToolTip("Open dynamic parameter configuration");
    ui->stopDynamicReconfigureButton->setToolTip("Close dynamic parameter configuration");
    ui->startRecordingButton->setToolTip("Start recording selected topics to rosbag");
    ui->stopRecordingButton->setToolTip("Stop current recording");

    connect(ui->nextPageButton, &QPushButton::clicked, this, &MainWindow::showNextPage);
    connect(ui->prevPageButton, &QPushButton::clicked, this, &MainWindow::showPrevPage);
    ui->prevPageButton->setEnabled(false);
    if (ui->stackedWidget->count() <= 1) {
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

    connect(ui->startDriversButton, &QPushButton::clicked, this, &MainWindow::startDrivers);
    connect(ui->stopDriversButton, &QPushButton::clicked, this, &MainWindow::stopDrivers);
    connect(ui->startSlamButton, &QPushButton::clicked, this, &MainWindow::startSlam);
    connect(ui->stopSlamButton, &QPushButton::clicked, this, &MainWindow::stopSlam);
    connect(ui->startDynamicReconfigureButton, &QPushButton::clicked, this, &MainWindow::startDynamicReconfigure);
    connect(ui->stopDynamicReconfigureButton, &QPushButton::clicked, this, &MainWindow::stopDynamicReconfigure);
    ui->startSlamButton->setEnabled(false);
    ui->stopSlamButton->setEnabled(false);
    ui->startDynamicReconfigureButton->setEnabled(false);
    ui->stopDynamicReconfigureButton->setEnabled(false);

    ros::NodeHandle nh;

    scanner_ = std::make_unique<ScannerController>(nh, this);

    // TODO: combine signal-slot connections into a single connect call
    connect(scanner_.get(), &ScannerController::driverStarted,
            this, &MainWindow::onDriverStarted);
    connect(scanner_.get(), &ScannerController::driverStopped,
            this, &MainWindow::onDriverStopped);
    connect(scanner_.get(), &ScannerController::driverCrashed,
            this, &MainWindow::onDriverCrashed);
    connect(scanner_.get(), &ScannerController::driverError,
            this, [this](const QString& key, const QString& error) {
        QMessageBox::warning(this, "Driver Error", QString("%1: %2").arg(key, error));
    });
    connect(scanner_.get(), &ScannerController::driverOutput,
            this, &MainWindow::onDriverOutput);
    connect(scanner_.get(), &ScannerController::diagnosticsUpdated,
            this, &MainWindow::onDiagStatus);
    connect(scanner_.get(), &ScannerController::recordingStarted,
            this, &MainWindow::onRecordingStarted);
    connect(scanner_.get(), &ScannerController::recordingStopped,
            this, &MainWindow::onRecordingStopped);
    connect(scanner_.get(), &ScannerController::recordingError,
            this, [this](const QString& err){ QMessageBox::warning(this, "Record Warning", err); });

    connect(ui->cameraCheckbox, &QCheckBox::stateChanged, this, &MainWindow::on_cameraCheckbox_stateChanged);
    connect(ui->lidarCheckbox, &QCheckBox::stateChanged, this, &MainWindow::on_lidarCheckbox_stateChanged);


    connect(ui->startRecordingButton, &QPushButton::clicked, this, [this] {
        if (scanner_->isRecording()) {
            QMessageBox::warning(this, "Record Warning", tr("Stop recording before starting a new one."));
            return;
        } else if (recordTopics_.isEmpty()) {
            QMessageBox::warning(this, "Record Warning", tr("Select at least one topic."));
            return;
        }
        
        QStringList topics = recordTopics_.values();
        scanner_->startRecording("my_bag", topics);
    });

    connect(ui->stopRecordingButton, &QPushButton::clicked,
            scanner_.get(), &ScannerController::stopRecording);
}

// -------------------------- Buttons --------------------------

void MainWindow::startDrivers()
{    
    scanner_->startDrivers();
    updateStatusLabel(ui->roscoreStatus, tr("Starting..."), materialColor("starting"));
    // only enable the SLAM button if both camera and lidar drivers are running
    // TODO 
    ui->startSlamButton->setEnabled(cameraRunning_ && lidarRunning_);
}

void MainWindow::stopDrivers() {
    scanner_->stopDrivers();
    ui->startSlamButton->setEnabled(false);
    ui->stopSlamButton->setEnabled(false);
}

void MainWindow::startSlam() {
    scanner_->startSlam();
    ui->startSlamButton->setEnabled(false);
    ui->stopSlamButton->setEnabled(true);
}

void MainWindow::stopSlam() {
    scanner_->stopSlam();
    ui->stopSlamButton->setEnabled(false);
    ui->startSlamButton->setEnabled(cameraRunning_ && lidarRunning_);
}

void MainWindow::startDynamicReconfigure() {
    scanner_->startDynamicReconfigure();
    ui->startDynamicReconfigureButton->setEnabled(false);
    ui->stopDynamicReconfigureButton->setEnabled(true);
}

void MainWindow::stopDynamicReconfigure() {
    scanner_->stopDynamicReconfigure();
    ui->stopDynamicReconfigureButton->setEnabled(false);
    ui->startDynamicReconfigureButton->setEnabled(cameraRunning_);
}

// -------------------------- Diagnostics --------------------------

void MainWindow::onDiagStatus(const QString &name, int level, const QString &msg, const QString &recordedFrequency) {
    qDebug() << "Name: " << name << " Level: " << level << " Message: " << msg;

    const QString sev = (level == 0) ? "OK"
                       : (level == 1) ? "WARN"
                       : (level == 2) ? "ERROR"
                       : "STALE";

    QLabel* targetLabel; 
    if (name == "river_watchdog: camera_rate") {
        targetLabel = ui->cameraStatus;
    } else if (name == "river_watchdog: lidar_rate") {
        targetLabel = ui->lidarStatus;
    } else {
        return; 
    }

    const QString statusText = tr("%1: %2").arg(sev, msg);
    updateStatusLabel(targetLabel, statusText, levelToColor(level));
}


// -------------------------- Driver Slots --------------------------
void MainWindow::onDriverOutput(const QString& /*key*/, const QString& output) {
    qDebug().noquote() << output;
}   

void MainWindow::onDriverStarted(const QString& key) {
    if (key == kRoscoreKey) {
        updateStatusLabel(ui->roscoreStatus, tr("Running"), materialColor("success"));
    } else if (key == kCameraKey) {
        cameraRunning_ = true;
        if (cameraRunning_ && lidarRunning_) ui->startSlamButton->setEnabled(true);
        ui->startDynamicReconfigureButton->setEnabled(true);
    } else if (key == kLidarKey) {
        lidarRunning_ = true;
        if (cameraRunning_ && lidarRunning_) ui->startSlamButton->setEnabled(true);
    } else if (key == kSlamKey) {
        ui->stopSlamButton->setEnabled(true);
    } else if (key == kDynamicReconfigureKey) {
        ui->startDynamicReconfigureButton->setEnabled(false);
        ui->stopDynamicReconfigureButton->setEnabled(true);
    }
}

void MainWindow::onDriverStopped(const QString& key) {
    if (key == kRoscoreKey) {
        updateStatusLabel(ui->roscoreStatus, tr("Stopped"), materialColor("default"));
    } else if (key == kCameraKey) {
        cameraRunning_ = false;
        updateStatusLabel(ui->cameraStatus, tr("Camera Stopped"), materialColor("default"));
        ui->startSlamButton->setEnabled(false);
        ui->startDynamicReconfigureButton->setEnabled(false);
    } else if (key == kLidarKey) {
        lidarRunning_ = false;
        updateStatusLabel(ui->lidarStatus, tr("Lidar Stopped"), materialColor("default"));
        ui->startSlamButton->setEnabled(false);
    } else if (key == kSlamKey) {
        ui->stopSlamButton->setEnabled(false);
        ui->startSlamButton->setEnabled(cameraRunning_ && lidarRunning_);
    } else if (key == kDynamicReconfigureKey) {
        ui->stopDynamicReconfigureButton->setEnabled(false);
        ui->startDynamicReconfigureButton->setEnabled(cameraRunning_);
    }
 }

 void MainWindow::onDriverCrashed(const QString& key) {
    QMessageBox::warning(this, "Process Failure", tr("%1 has stopped running.").arg(key));
    if (key == kRoscoreKey) {
        updateStatusLabel(ui->roscoreStatus, tr("CRASHED"), materialColor("error"));
    } else if (key == kCameraKey || key == kLidarKey) {
        ui->startSlamButton->setEnabled(false);
        if (key == kCameraKey) {
            updateStatusLabel(ui->cameraStatus, tr("CRASHED"), materialColor("error"));
        } else {
            updateStatusLabel(ui->lidarStatus, tr("CRASHED"), materialColor("error"));
        }
    }
 }
 
// -------------------------- closeEvent --------------------------

void MainWindow::closeEvent(QCloseEvent* event) {
    scanner_->stopDrivers();
    QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow() = default; 


void MainWindow::on_cameraCheckbox_stateChanged(int checked)
{
    if (checked == 0) {
        recordTopics_.remove("/left_camera/image");
    } else if (checked == 2) {
        recordTopics_ << "/left_camera/image";
    }
}

void MainWindow::on_lidarCheckbox_stateChanged(int checked)
{
    if (checked == 0) {
        recordTopics_.remove("/livox/lidar");
    } else if (checked == 2) {
        recordTopics_ << "/livox/lidar";
    }
}

void MainWindow::onRecordingStarted() {
    updateStatusLabel(ui->recordStatus, "Recording...", QColor("#DC004E")); // Material Pink
}

void MainWindow::onRecordingStopped() {
    updateStatusLabel(ui->recordStatus, "OFF", materialColor("default"));
}

void MainWindow::showNextPage() {
    int index = ui->stackedWidget->currentIndex();
    if (index < ui->stackedWidget->count() - 1) {
        ui->stackedWidget->setCurrentIndex(index + 1);
    }
    ui->prevPageButton->setEnabled(true);
    ui->nextPageButton->setEnabled(ui->stackedWidget->currentIndex() < ui->stackedWidget->count() - 1);
}

void MainWindow::showPrevPage() {
    int index = ui->stackedWidget->currentIndex();
    if (index > 0) {
        ui->stackedWidget->setCurrentIndex(index - 1);
    }
    ui->nextPageButton->setEnabled(true);
    ui->prevPageButton->setEnabled(ui->stackedWidget->currentIndex() > 0);
}
