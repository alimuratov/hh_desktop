#include "scanner_controller.h"

#include <QDateTime>
#include <QThread>
#include <QDebug>
#include <errno.h>
#include <signal.h>

namespace {
    constexpr char kCameraKey[]    = "camera";
    constexpr char kLidarKey[]     = "lidar";
    constexpr char kCameraScript[] = "/home/kodifly/setup_scripts/camera_setup.sh";
    constexpr char kLidarScript[]  = "/home/kodifly/setup_scripts/lidar_setup.sh";
    constexpr char kWatchdogExec[] = "/home/kodifly/setup_scripts/watchdog_setup.sh";
    constexpr char kWatchdogKey[]  = "watchdog";
    constexpr char kSlamKey[]      = "slam";
    constexpr char kSlamScript[]   = "/home/kodifly/setup_scripts/start_slam.sh";
    constexpr char kRoscoreKey[]   = "roscore";
    constexpr char kRoscoreExec[]  = "roscore";
}


ScannerController::ScannerController(ros::NodeHandle& nh, QObject* parent)
    : QObject(parent)
    , recorder(std::make_unique<RosbagRecorder>(this))
    , diagnosticsMonitor(std::make_unique<DiagnosticsMonitor>(nh, this))
    , rosTimer_(new QTimer(this))
{
    connect(rosTimer_, &QTimer::timeout, [] { ros::spinOnce(); });
    rosTimer_->start(10);
    // Connect RosbagRecorder signals
    connect(recorder.get(), &RosbagRecorder::recordingStarted,
            this, &ScannerController::recordingStarted);
    
    connect(recorder.get(), &RosbagRecorder::recordingStopped,
            this, &ScannerController::recordingStopped);
    
    connect(recorder.get(), &RosbagRecorder::recordingError,
            this, &ScannerController::recordingError);
    
    // Connect DiagnosticsMonitor signals
    connect(diagnosticsMonitor.get(), &DiagnosticsMonitor::statusChanged,
            this, &ScannerController::diagnosticsUpdated);
}

// -------------------------- Recording --------------------------

void ScannerController::startRecording(const QString& filename, const QStringList& topics) {
    if (recorder->isRecording()) {
        emit recordingError("Recording already in progress");
        return;
    }
    
    recorder->startRecording(filename, topics);
}

void ScannerController::stopRecording() {
    recorder->stopRecording();
}

bool ScannerController::isRecording() const {
    return recorder->isRecording();
}

// -------------------------- Driver control --------------------------

void ScannerController::startDrivers() {
    startProcess(kRoscoreKey, kRoscoreExec);
    
    // Wait for roscore to initialize
    QThread::msleep(1000);
    
    startProcess(kCameraKey, kCameraScript);
    startProcess(kLidarKey, kLidarScript);
    startProcess(kWatchdogKey, kWatchdogExec);
}

void ScannerController::stopDrivers() {
    stopProcess(kSlamKey);
    stopProcess(kWatchdogKey);
    stopProcess(kLidarKey);
    stopProcess(kCameraKey);
    stopProcess(kRoscoreKey);
}

void ScannerController::startSlam() {
    startProcess(kSlamKey, kSlamScript);
}

void ScannerController::stopSlam() { stopProcess(kSlamKey); }

// -------------------------- process helpers --------------------------

std::unique_ptr<QProcess> ScannerController::createDriverProcess(const QString& scriptPath,
                                                                const QString& key) {
    auto proc = std::make_unique<QProcess>(this);
    proc->setProcessChannelMode(QProcess::MergedChannels);

    const QString program("/usr/bin/setsid");
    const QStringList args {"/bin/bash", "-c", QStringLiteral("exec %1").arg(scriptPath)};

    connect(proc.get(), &QProcess::started, this, [this, key, p = proc.get()] {
        qDebug() << key << " PID=" << p->processId();
        emit driverStarted(key);
    });
    connect(proc.get(), &QProcess::readyReadStandardOutput,
            this, &ScannerController::readDriverOutput);
    connect(proc.get(), &QProcess::errorOccurred,
            this, &ScannerController::processCrashed);
    connect(proc.get(), qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
            this, &ScannerController::processFinished);

    proc->start(program, args);
    if (!proc->waitForStarted()) {
        emit driverCrashed(key);
        return nullptr;
    }
    return proc;
}

void ScannerController::startProcess(const QString& key, const QString& scriptPath) {
    if (drivers_.count(key)) return;
    
    // Special handling for SLAM - check prerequisites before starting
    if (key == kSlamKey) {
        if (!drivers_.count(kCameraKey) || !drivers_.count(kLidarKey)) {
            qDebug() << "Cannot start SLAM without camera and lidar";
            emit driverError(key, "Cannot start SLAM without camera and lidar running");
            return;
        }
    }
    
    auto proc = createDriverProcess(scriptPath, key);
    if (!proc) return;
    drivers_.emplace(key, std::move(proc));
}

void ScannerController::stopProcess(const QString& key) {
    shutdownProcess(key);
}

// -------------------------- generic slots --------------------------


void ScannerController::readDriverOutput() {
    auto* senderProc = qobject_cast<QProcess*>(sender());
    if (!senderProc) return;
    const QString text = QString::fromUtf8(senderProc->readAllStandardOutput());
    const QString key = findVictimKey(senderProc);
    emit driverOutput(key, text);
    qDebug().noquote() << text;
}

void ScannerController::processCrashed() {
    auto* senderProc = qobject_cast<QProcess*>(sender());
    if (!senderProc) return;
    QString victimKey = findVictimKey(senderProc);
    if (!victimKey.isEmpty()) {
        handleProcessCrash(victimKey);
    }
}

void ScannerController::processFinished(int exitCode, QProcess::ExitStatus exitStatus) {
    Q_UNUSED(exitCode);
    auto* senderProc = qobject_cast<QProcess*>(sender());
    if (!senderProc) return;
    QString victimKey = findVictimKey(senderProc);
    if (victimKey.isEmpty()) return;
    if (exitStatus == QProcess::CrashExit) {
        handleProcessCrash(victimKey);
        return;
    }
    handleProcessCompletion(victimKey);
}

// -------------------------- handlers --------------------------

void ScannerController::handleProcessCrash(const QString& crashedProc) {
    shutdownProcess(crashedProc);
    emit driverCrashed(crashedProc);
    if (crashedProc == kCameraKey || crashedProc == kLidarKey) {
        stopSlam();
    } else if (crashedProc == kSlamKey) {
        // nothing extra
    } else if (crashedProc == kRoscoreKey) {
        // nothing extra
    }
}

void ScannerController::handleProcessCompletion(const QString& completedProc) {
    shutdownProcess(completedProc);
    emit driverStopped(completedProc);
    if (completedProc == kCameraKey || completedProc == kLidarKey) {
        stopSlam();
    }
}

QString ScannerController::findVictimKey(QProcess* proc) {
    for (const auto& pair : drivers_) {
        if (pair.second.get() == proc) {
            return pair.first;
        }
    }
    return QString();
}

// -------------------------- shutdown helpers --------------------------

void ScannerController::shutdownProcess(const QString& key) {
    auto it = drivers_.find(key);
    if (it == drivers_.end()) return;

    const qint64 pid = it->second->processId();

    if (!killProcessGroup(pid, SIGINT, 2000) &&
        !killProcessGroup(pid, SIGTERM, 2000)) {
        killProcessGroup(pid, SIGKILL, 0);
    }

    drivers_.erase(it);
    emit driverStopped(key);
}

bool ScannerController::killProcessGroup(qint64 pid, int sig, int waitMs) {
    if (pid <= 0) return true;

    ::kill(-pid, sig);
    if (waitMs == 0) return false;

    const qint64 t0 = QDateTime::currentMSecsSinceEpoch();
    while (QDateTime::currentMSecsSinceEpoch() - t0 < waitMs) {
        if (::kill(pid, 0) == -1 && errno == ESRCH) return true;
        QThread::msleep(50);
    }
    return false;
}
