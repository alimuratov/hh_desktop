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
    constexpr char kCameraScript[] = "/home/kodifly/setup_scripts/camera_setup.sh";
    constexpr char kLidarScript[] = "/home/kodifly/setup_scripts/lidar_setup.sh";
      constexpr char kWatchdogExec[]   = "/home/kodifly/setup_scripts/watchdog_setup.sh";
      constexpr char kWatchdogKey[]  = "watchdog";
    constexpr char kSlamKey[] = "slam";
    constexpr char kSlamScript[] = "/home/kodifly/setup_scripts/start_slam.sh";
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

    rosTimer_ = new QTimer(this);
    connect(rosTimer_, &QTimer::timeout, [] { ros::spinOnce(); });
    rosTimer_->start(10);

    connect(ui->startDriversButton, &QPushButton::clicked, this, &MainWindow::startDrivers);
    connect(ui->stopDriversButton, &QPushButton::clicked, this, &MainWindow::stopDrivers);
    connect(ui->startSlamButton, &QPushButton::clicked, this, &MainWindow::startSlam);
    connect(ui->stopSlamButton, &QPushButton::clicked, this, &MainWindow::stopSlam);
    ui->startSlamButton->setEnabled(false);
    ui->stopSlamButton->setEnabled(false);

    ros::NodeHandle nh;
    // can sit idle without hurting anything, its subscriber just sleeps until /diagnostics starts flowing 
    diag_monitor_ = std::make_unique<DiagnosticsMonitor>(nh, this);
    connect(diag_monitor_.get(), &DiagnosticsMonitor::statusChanged, this, &MainWindow::onDiagStatus);

    connect(ui->cameraCheckbox, &QCheckBox::stateChanged, this, &MainWindow::on_cameraCheckbox_stateChanged);
    connect(ui->lidarCheckbox, &QCheckBox::stateChanged, this, &MainWindow::on_lidarCheckbox_stateChanged);
    recorder_ = std::make_unique<RosbagRecorder>(this);
    connect(ui->startRecordingButton, &QPushButton::clicked, this, [this] {
        if (recorder_->getIsRecording()) {
            QMessageBox::warning(this, "Record Warning", tr("Stop recording before starting a new one."));
            return;
        } else if (recordTopics_.isEmpty()) {
            QMessageBox::warning(this, "Record Warning", tr("Select at least one topic."));
            return;
        }
        recorder_->startRecording("my_bag", recordTopics_);
    });
    connect(ui->stopRecordingButton, &QPushButton::clicked,
            recorder_.get(), &RosbagRecorder::stopRecording);
    connect(recorder_.get(), &RosbagRecorder::recordingStarted,
            this, &MainWindow::onRecordingStarted);
    connect(recorder_.get(), &RosbagRecorder::recordingStopped,
            this, &MainWindow::onRecordingStopped);
}

// -------------------------- Buttons --------------------------

void MainWindow::startDrivers()
{
    startCamera();
    startLidar();
    startWatchdog();
    if (drivers_.count(kCameraKey) && drivers_.count(kLidarKey)) {
        ui->startSlamButton->setEnabled(true);
    }
}

void MainWindow::stopDrivers() {
    stopSlam();
    stopCamera();
    stopLidar();
    stopWatchdog();
    ui->startSlamButton->setEnabled(false);
    ui->stopSlamButton->setEnabled(false);
}

// -------------------------- Driver Utilities --------------------------

void MainWindow::startCamera() {
    // if the camera driver is already running
    if (drivers_.count(kCameraKey)) return;

    auto proc = createDriverProcess(kCameraScript, kCameraKey);
    if (!proc) return;
    drivers_.emplace(kCameraKey, std::move(proc)); 
}

void MainWindow::stopCamera() {
    shutdownProcess(kCameraKey); 
    ui->cameraStatus->setText(tr("Camera driver stopped."));
    ui->cameraStatus->setStyleSheet("");
} 

void MainWindow::startLidar() {
    if (drivers_.count(kLidarKey)) return;

    auto proc = createDriverProcess(kLidarScript, kLidarKey);
    if (!proc) return;
    drivers_.emplace(kLidarKey, std::move(proc)); 
}

void MainWindow::stopLidar() {
    shutdownProcess(kLidarKey);
    ui->lidarStatus->setText(tr("Lidar driver stopped."));
    ui->lidarStatus->setStyleSheet("");
}

void MainWindow::startWatchdog() {
    if (drivers_.count(kWatchdogKey)) return;
    auto proc = std::make_unique<QProcess>(this);
    proc->setProcessChannelMode(QProcess::MergedChannels);

    const QString program("/usr/bin/setsid");
    const QStringList args {QString::fromUtf8(kWatchdogExec)};

    connect(proc.get(), &QProcess::started, this, [p = proc.get()]{
        qDebug() << "watchdog PID=" << p->processId();
    });
    connect(proc.get(), &QProcess::readyReadStandardOutput, this, &MainWindow::readDriverOutput);
    connect(proc.get(), &QProcess::errorOccurred, this, &MainWindow::processCrashed);
    connect(proc.get(), qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, &MainWindow::processFinished);

    proc->start(program, args);
    if (!proc->waitForStarted()) {
        QMessageBox::critical(this, tr("Failed to start"), tr("watchdog node could not be launched"));
        return;
    }
    drivers_.emplace(kWatchdogKey, std::move(proc));
}

void MainWindow::stopWatchdog() {
    shutdownProcess(kWatchdogKey);
}

void MainWindow::startSlam() {
    if (drivers_.count(kSlamKey)) return;

    auto proc = createDriverProcess(kSlamScript, kSlamKey);
    if (!proc) return;
    drivers_.emplace(kSlamKey, std::move(proc));
    ui->startSlamButton->setEnabled(false);
    ui->stopSlamButton->setEnabled(true);
}

void MainWindow::stopSlam() {
    shutdownProcess(kSlamKey);
    ui->stopSlamButton->setEnabled(false);
    ui->startSlamButton->setEnabled(drivers_.count(kCameraKey) && drivers_.count(kLidarKey));
}

// -------------------------- Process Factory --------------------------

std::unique_ptr<QProcess> MainWindow::createDriverProcess(const QString& scriptPath, const QString& key) {
    auto proc = std::make_unique<QProcess>(this);
    // tells QProcess to merge the child's stdout and stderr streams into a single channel before the process starts
    // with the default SeparateChannels mode you would need two different ready-read signals or two reads to catch everything the script prints 
    proc->setProcessChannelMode(QProcess::MergedChannels); // stderr + stdout together

    // running the script through setsid makes it the leader of a new process group
    // every ROS node it spawns inherits the same PGID, which lets us alter kill the whole tree with killpg() 
    const QString program("/usr/bin/setsid");
    const QStringList args {"/bin/bash", "-c", QStringLiteral("exec %1").arg(scriptPath)};

    // readyReadStandardOutput() is a signal that merely tells us "new bytes have arrived"
    connect(proc.get(), &QProcess::started, this, [this, key, p = proc.get()]{
        qDebug() << key << " PID=" << p->processId();
    });
    connect(proc.get(), &QProcess::readyReadStandardOutput, this, &MainWindow::readDriverOutput);
    connect(proc.get(), &QProcess::errorOccurred, this, &MainWindow::processCrashed);
    connect(proc.get(), qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
            this, &MainWindow::processFinished); 

    proc->start(program, args);
    // blocks the calling thead until the child process has actually been forked 
    // it then eturns true if the QProcess state became Running, otherwise false 
    if (!proc->waitForStarted()) {
        QMessageBox::critical(this, tr("Failed to start"),
                              tr("%1 driver could not be launched").arg(key)); 
        return nullptr;
    }
    return proc; 
}

// -------------------------- Diagnostics --------------------------

void MainWindow::onDiagStatus(const QString &name, int level, const QString &msg, const QString &recordedFrequency) {
    qDebug() << "Name: " << name << " Level: " << level << " Message: " << msg;
    // severity levels
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

    // if (level >= 2) { // ERROR ‑ pop up.
    //  QMessageBox::critical(this, tr("Watchdog"),
    //                         tr("%1 driver reported %2\n%3").arg(name, sev, msg));
    // }
  }


// -------------------------- Generic Slots --------------------------

void MainWindow::readDriverOutput() {
    auto* senderProc = qobject_cast<QProcess*>(sender());
    if (!senderProc) return;
    const QString text = QString::fromUtf8(senderProc->readAllStandardOutput());
    qDebug().noquote() << text; 
}

void MainWindow::processCrashed() {
    // if you drop the asterisk here, the deduced type is still QProcess*, so the program works
    // but the declartion would look as if senderProc might be an object rather than a pointer 
    auto* senderProc = qobject_cast<QProcess*>(sender()); 
    if (!senderProc) return; 
    QString victimKey = findVictimKey(senderProc);
    if (!victimKey.isEmpty()) {
        handleProcessCrash(victimKey);
    }
}

// not a slot, but a private method that handles process crashes
void MainWindow::handleProcessCrash(const QString& crashedProc) {
    shutdownProcess(crashedProc);
    QMessageBox::warning(this, "Process Failure", tr("%1 has stopped running.").arg(crashedProc));
    if (crashedProc == kSlamKey) {
        ui->stopSlamButton->setEnabled(false);
        ui->startSlamButton->setEnabled(drivers_.count(kCameraKey) && drivers_.count(kLidarKey));
    } else if (crashedProc == kCameraKey || crashedProc == kLidarKey) {
        stopSlam();
        ui->startSlamButton->setEnabled(false);
    }
}

void MainWindow::processFinished(int exitCode, QProcess::ExitStatus exitStatus) {
    auto* senderProc = qobject_cast<QProcess*>(sender()); 
    if (!senderProc) return; 
    QString victimKey = findVictimKey(senderProc);
    if (victimKey.isEmpty()) {
        return; 
    }
    if (exitStatus == QProcess::CrashExit) {
        handleProcessCrash(victimKey);
        return; 
    }

    handleProcessCompletion(victimKey);
}

// not a slot as well 
void MainWindow::handleProcessCompletion(const QString& completedProc) {
    shutdownProcess(completedProc);
    QMessageBox::information(this, "Process Completion", tr("%1 has finished.").arg(completedProc));
    if (completedProc == kSlamKey) {
        ui->stopSlamButton->setEnabled(false);
        ui->startSlamButton->setEnabled(drivers_.count(kCameraKey) && drivers_.count(kLidarKey));
    } else if (completedProc == kCameraKey || completedProc == kLidarKey) {
        stopSlam();
        ui->startSlamButton->setEnabled(false);
    }
 }

QString MainWindow::findVictimKey(QProcess* proc) {
    for (const auto& pair : drivers_) {
        if (pair.second.get() == proc) {
            return pair.first; 
        }
    }
    return QString(); // not found
}

// -------------------------- Shutdown Helpers --------------------------

void MainWindow::shutdownProcess(const QString& key) {
    auto it = drivers_.find(key);
    if (it == drivers_.end()) return; 

    const qint64 pid = it->second->processId();

    if (!killProcessGroup(pid, SIGINT, 2000) && !killProcessGroup(pid, SIGTERM, 2000)) {
        killProcessGroup(pid, SIGKILL, 0);
    }

    drivers_.erase(it); 
}

bool MainWindow::killProcessGroup(qint64 pid, int sig, int waitMs) {
    if (pid <= 0) return true;

    ::kill(-pid, sig); // only sends a signal SIGTERM (doesn't wait for the processes in that group to act on it)
    if (waitMs == 0) {
        return false; 
    }

    const qint64 t0 = QDateTime::currentMSecsSinceEpoch();
    while (QDateTime::currentMSecsSinceEpoch() - t0 < waitMs) {
        // ::kill(pid, 0) is a POSIX "probe": it delivers no signal but reutrns 0 if the process still exists
        if (::kill(pid, 0) == -1 && errno == ESRCH) return true;
        QThread::msleep(50); // suspend the current thread
    }
    return false; 
}

// -------------------------- closeEvent --------------------------

void MainWindow::closeEvent(QCloseEvent* event) {
    for (auto it = drivers_.begin(); it != drivers_.end(); ) {
        const QString key = it->first;
        ++it;
        shutdownProcess(key);
    }
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
