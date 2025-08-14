#include "scanner_controller.h"
#include "process_config.h"
#include <QDateTime>
#include <QThread>
#include <QDebug>
#include <errno.h>
#include <signal.h>


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
    startProcess("roscore");

    // Wait for roscore to initialize - this could also be moved to ProcessConfig
    QThread::msleep(1000);

    startProcess("camera");
    startProcess("lidar");
    startProcess("watchdog");

    // Start dynamic_reconfigure after camera is ready
    // The dependency is handled in process_config.h
    startProcess("dynamic_reconfigure");
}

void ScannerController::stopDrivers() {
    stopProcess("slam");
    stopProcess("dynamic_reconfigure");
    stopProcess("watchdog");
    stopProcess("lidar");
    stopProcess("camera");
    stopProcess("roscore");
}

void ScannerController::startSlam() {
    startProcess("slam");
}

void ScannerController::stopSlam() {
    stopProcess("slam");
}

void ScannerController::startDynamicReconfigure() {
    startProcess("dynamic_reconfigure");
}

void ScannerController::stopDynamicReconfigure() {
    stopProcess("dynamic_reconfigure");
}// -------------------------- process helpers --------------------------

std::unique_ptr<QProcess> ScannerController::createDriverProcess(const ProcessConfig& config) {
    auto proc = std::make_unique<QProcess>(this);
    proc->setProcessChannelMode(QProcess::MergedChannels);

    const QString program("/usr/bin/setsid");
    QStringList args;

    // Handle different execution modes based on arguments
    if (config.arguments.isEmpty()) {
        // For simple executables or scripts, use exec
        args = QStringList{"/bin/bash", "-c", QStringLiteral("exec %1").arg(config.executable)};
    } else {
        // For executables with arguments, construct proper command
        QString fullCommand = config.executable + " " + config.arguments.join(" ");
        args = QStringList{"/bin/bash", "-c", QStringLiteral("exec %1").arg(fullCommand)};
    }

    connect(proc.get(), &QProcess::started, this, [this, key = config.key, p = proc.get()] {
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
        emit driverCrashed(config.key);
        return nullptr;
    }
    return proc;
}

void ScannerController::startProcess(const QString& key) {
    if (drivers_.count(key)) return;

    const ProcessConfig* config = ProcessRegistry::instance().getProcess(key);
    if (!config) {
        emit driverError(key, QString("Unknown process: %1").arg(key));
        return;
    }

    std::unordered_map<QString, bool, QStringHash> runningProcesses;
    for (const auto& [k, _] : drivers_) {
        runningProcesses[k] = true;
    }

    if (!config->canStart(runningProcesses)) {
        emit driverError(key, QString("%1 prerequisites not met").arg(config->name));
        return;
    }

    auto proc = createDriverProcess(*config);
    if (!proc) {
        emit driverError(key, QString("Failed to start %1").arg(config->name));
        return;
    }

    drivers_.emplace(key, std::move(proc));

    // Handle startup delay if configured
    if (config->startupDelayMs > 0) {
        QThread::msleep(config->startupDelayMs);
    }
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

    // Handle dependencies - if camera or lidar crashes, stop SLAM
    if (crashedProc == "camera" || crashedProc == "lidar") {
        stopSlam();
    }

    // If critical process crashed (like roscore), could stop everything
    const ProcessConfig* config = ProcessRegistry::instance().getProcess(crashedProc);
    if (config && config->critical) {
        qDebug() << "Critical process" << crashedProc << "crashed, stopping all drivers";
        stopDrivers();
    }
}

void ScannerController::handleProcessCompletion(const QString& completedProc) {
    shutdownProcess(completedProc);
    emit driverStopped(completedProc);

    // Handle dependencies
    if (completedProc == "camera" || completedProc == "lidar") {
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
