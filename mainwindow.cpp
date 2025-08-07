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

namespace {
    constexpr char kCameraKey[] = "camera";
    constexpr char kLidarKey[] = "lidar";
    constexpr char kWatchdogKey[]  = "watchdog";
    constexpr char kSlamKey[] = "slam";
    constexpr char kRoscoreKey[] = "roscore";
#ifdef HH_ENABLE_RVIZ
      constexpr char kRvizConfig[] = "/home/kodifly/hh_desktop/config/view.rviz";
#endif
  }

// -------------------------- Color Helper --------------------------
static QColor levelToColor(uint8_t level) {
  using diagnostic_msgs::DiagnosticStatus;
  switch (level) {
    case DiagnosticStatus::OK:    return QColor("#2ECC71"); // green
    case DiagnosticStatus::WARN:  return QColor("#F1C40F"); // yellow
    case DiagnosticStatus::ERROR: return QColor("#E74C3C"); // red
    default:                      return QColor("#95A5A6"); // grey
  }
}   

// -------------------------- --------------------------

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

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
    ui->startSlamButton->setEnabled(false);
    ui->stopSlamButton->setEnabled(false);

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

// -------------------------- Buttons --------------------------

void MainWindow::startDrivers()
{    
    scanner_->startDrivers();
    ui->roscoreStatus->setText(tr("Starting..."));
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

    targetLabel->setText(tr("%1: %2").arg(sev, msg ));
    targetLabel->setStyleSheet(QStringLiteral("color:%1;")
                               .arg(levelToColor(level).name()));
  }


// -------------------------- Driver Slots --------------------------
void MainWindow::onDriverOutput(const QString& /*key*/, const QString& output) {
    qDebug().noquote() << output;
}   

void MainWindow::onDriverStarted(const QString& key) {
    if (key == kRoscoreKey) {
        ui->roscoreStatus->setText(tr("Running"));
        ui->roscoreStatus->setStyleSheet("color:green;");
    } else if (key == kCameraKey) {
        cameraRunning_ = true;
        if (cameraRunning_ && lidarRunning_) ui->startSlamButton->setEnabled(true);
    } else if (key == kLidarKey) {
        lidarRunning_ = true;
        if (cameraRunning_ && lidarRunning_) ui->startSlamButton->setEnabled(true);
    } else if (key == kSlamKey) {
        ui->stopSlamButton->setEnabled(true);
    }
}

void MainWindow::onDriverStopped(const QString& key) {
    if (key == kRoscoreKey) {
        ui->roscoreStatus->setText(tr("Roscore stopped."));
        ui->roscoreStatus->setStyleSheet("");
    } else if (key == kCameraKey) {
        cameraRunning_ = false;
        ui->cameraStatus->setText(tr("Camera driver stopped."));
        ui->cameraStatus->setStyleSheet("");
        ui->startSlamButton->setEnabled(false);
    } else if (key == kLidarKey) {
        lidarRunning_ = false;
        ui->lidarStatus->setText(tr("Lidar driver stopped."));
        ui->lidarStatus->setStyleSheet("");
        ui->startSlamButton->setEnabled(false);
    } else if (key == kSlamKey) {
        ui->stopSlamButton->setEnabled(false);
        ui->startSlamButton->setEnabled(cameraRunning_ && lidarRunning_);
    }
 }

 void MainWindow::onDriverCrashed(const QString& key) {
    QMessageBox::warning(this, "Process Failure", tr("%1 has stopped running.").arg(key));
    if (key == kRoscoreKey) {
        ui->roscoreStatus->setText(tr("Roscore crashed."));
        ui->roscoreStatus->setStyleSheet("color:red;");
    } else if (key == kCameraKey || key == kLidarKey) {
        ui->startSlamButton->setEnabled(false);
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
    ui->recordStatus->setText("Recording...");
}

void MainWindow::onRecordingStopped() {
    ui->recordStatus->setText("OFF");
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
