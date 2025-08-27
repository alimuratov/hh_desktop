#include "scanner_controller.h"
#include "process_config.h"
#include <QDateTime>
#include <QThread>
#include <QDebug>
#include <QFile>
#include <QProcess>
#include <QTextStream>
#include <QRegularExpression>
#include <errno.h>
#include <signal.h>

ScannerController::ScannerController(ros::NodeHandle &nh, QObject *parent)
    : QObject(parent), recorder(std::make_unique<RosbagRecorder>(this)), diagnosticsMonitor(std::make_unique<DiagnosticsMonitor>(nh, this)), rosTimer_(new QTimer(this))
{
    connect(rosTimer_, &QTimer::timeout, []
            { ros::spinOnce(); });
    rosTimer_->start(10);
    // Periodic dump of drivers_ snapshot
    driversDumpTimer_ = new QTimer(this);
    driversDumpTimer_->setInterval(1000);
    connect(driversDumpTimer_, &QTimer::timeout, this, &ScannerController::dumpDriversSnapshot);
    driversDumpTimer_->start();
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

void ScannerController::startRecording(const QString &filename, const QStringList &topics)
{
    if (recorder->isRecording())
    {
        emit recordingError("Recording already in progress");
        return;
    }

    recorder->startRecording(filename, topics);
}

void ScannerController::stopRecording()
{
    recorder->stopRecording();
}

bool ScannerController::isRecording() const
{
    return recorder->isRecording();
}

// -------------------------- Driver control --------------------------

void ScannerController::startDrivers()
{
    if (forceExternalCleanup_)
    {
        qDebug() << "Force external cleanup enabled. Attempting to terminate externally running processes before startup.";
        cleanupAllExternals();
    }
    // Start or adopt roscore first
    if (adoptExternalRoscore_ && hasExternalRunning("roscore"))
    {
        qDebug() << "Adopting external roscore (will not start internal instance).";
    }
    else
    {
        startProcess("roscore");
        // Allow some time for master to come up
        QThread::msleep(1000);
    }

    startProcess("camera");
    startProcess("lidar");
    startProcess("gpsrtk");
    startProcess("ptp4l");
    startProcess("watchdog");
    startProcess("sync");
    // Start dynamic_reconfigure after camera is ready
    // The dependency is handled in process_config.h
    startProcess("dynamic_reconfigure");
}

void ScannerController::stopDrivers()
{
    stopProcess("slam");
    stopProcess("dynamic_reconfigure");
    stopProcess("watchdog");
    stopProcess("ptp4l");
    stopProcess("gpsrtk");
    stopProcess("lidar");
    stopProcess("camera");
    stopProcess("roscore");
}

void ScannerController::startSlam()
{
    startProcess("slam");
}

void ScannerController::stopSlam()
{
    stopProcess("slam");
}

void ScannerController::startDynamicReconfigure()
{
    startProcess("dynamic_reconfigure");
}

void ScannerController::stopDynamicReconfigure()
{
    stopProcess("dynamic_reconfigure");
}

void ScannerController::startMapviz()
{
    startProcess("mapviz");
}

void ScannerController::stopMapviz()
{
    stopProcess("mapviz");
}
// -------------------------- process helpers --------------------------

std::unique_ptr<QProcess> ScannerController::createDriverProcess(const ProcessConfig &config)
{
    auto proc = std::make_unique<QProcess>(this);
    proc->setProcessChannelMode(QProcess::MergedChannels);

    const QString program("/usr/bin/setsid");
    QStringList args;

    // Handle different execution modes based on arguments
    if (config.arguments.isEmpty())
    {
        // For simple executables or scripts, use exec
        args = QStringList{"/bin/bash", "-c", QStringLiteral("exec %1").arg(config.executable)};
    }
    else
    {
        // For executables with arguments, construct proper command
        QString fullCommand = config.executable + " " + config.arguments.join(" ");
        args = QStringList{"/bin/bash", "-c", QStringLiteral("exec %1").arg(fullCommand)};
    }

    connect(proc.get(), &QProcess::started, this, [this, key = config.key, p = proc.get()]
            {
        qDebug() << key << " PID=" << p->processId();
        emit driverStarted(key); });
    connect(proc.get(), &QProcess::readyReadStandardOutput,
            this, &ScannerController::readDriverOutput);
    connect(proc.get(), &QProcess::errorOccurred,
            this, &ScannerController::processCrashed);
    connect(proc.get(), qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
            this, &ScannerController::processFinished);

    proc->start(program, args);
    if (!proc->waitForStarted())
    {
        emit driverCrashed(config.key);
        return nullptr;
    }
    return proc;
}

void ScannerController::startProcess(const QString &key)
{
    if (drivers_.count(key))
        return;

    const ProcessConfig *config = ProcessRegistry::instance().getProcess(key);
    if (!config)
    {
        emit driverError(key, QString("Unknown process: %1").arg(key));
        return;
    }

    // External instance handling
    QList<qint64> initialExternalPids;
    if (hasExternalRunning(key))
    {
        initialExternalPids = findExternalPids(key); // snapshot before retries / adoption
        if (key == "roscore" && adoptExternalRoscore_)
        {
            qDebug() << "Adopting external" << key;
            return; // treat as running
        }
        // If cleanup just ran, allow a retry window to let processes exit
        bool cleared = false;
        for (int attempt = 0; attempt < kExternalRetryAttempts; ++attempt)
        {
            if (!hasExternalRunning(key))
            {
                cleared = true;
                break;
            }
            QThread::msleep(kExternalRetryDelayMs);
        }
        if (!cleared && hasExternalRunning(key))
        {
            // Collect detailed PID / PGID and command line info for the message box
            QList<qint64> finalPids = findExternalPids(key); // refresh after waits

            // Build sets / details
            auto buildDetails = [](const QList<qint64> &pids)
            {
                QStringList pidStrs;
                QSet<qint64> pgids;
                QStringList details;
                for (qint64 pid : pids)
                {
                    if (pid <= 0)
                        continue;
                    bool alive = (::kill(pid, 0) == 0);
                    pidStrs << QString::number(pid);
                    pid_t pg = getpgid(static_cast<pid_t>(pid));
                    if (pg > 0)
                        pgids.insert(pg);
                    QString infoCmd = QString("ps -o args= -p %1 2>/dev/null").arg(pid);
                    QProcess ps;
                    ps.start("/bin/sh", QStringList() << "-c" << infoCmd);
                    QString cmdLine;
                    if (ps.waitForFinished(300))
                        cmdLine = QString::fromUtf8(ps.readAllStandardOutput()).trimmed();
                    if (cmdLine.isEmpty())
                        cmdLine = "<unknown>";
                    const int kMaxCmd = 80;
                    if (cmdLine.size() > kMaxCmd)
                        cmdLine = cmdLine.left(kMaxCmd) + "...";
                    details << QString("%1:{alive=%2 pg=%3 cmd=%4}")
                                   .arg(pid)
                                   .arg(alive ? "1" : "0")
                                   .arg(pg > 0 ? QString::number(pg) : QStringLiteral("?"))
                                   .arg(cmdLine.replace('\n', ' '));
                }
                return std::tuple<QStringList, QSet<qint64>, QStringList>(pidStrs, pgids, details);
            };

            auto [initialPidStrs, initialPgids, initialDetails] = buildDetails(initialExternalPids);
            auto [finalPidStrs, finalPgids, finalDetails] = buildDetails(finalPids);

            QStringList initialPgidStrs;
            for (qint64 g : initialPgids)
                initialPgidStrs << QString::number(g);
            QStringList finalPgidStrs;
            for (qint64 g : finalPgids)
                finalPgidStrs << QString::number(g);

            emit driverError(key, QString(
                                      "%1 already running externally. INITIAL{PIDs=[%2] PGIDs=[%3] DETAILS={%4}} FINAL{PIDs=[%5] PGIDs=[%6] DETAILS={%7}} EXEC=%8")
                                      .arg(config->name,
                                           initialPidStrs.join(","),
                                           initialPgidStrs.join(","),
                                           initialDetails.join(" | "),
                                           finalPidStrs.join(","),
                                           finalPgidStrs.join(","),
                                           finalDetails.join(" | "),
                                           config->executable));
            return;
        }
    }

    // Verify prerequisites using unified logic (includes external processes)
    if (!canStart(key))
    {
        emit driverError(key, QString("%1 prerequisites not met").arg(config->name));
        return;
    }

    auto proc = createDriverProcess(*config);
    if (!proc)
    {
        emit driverError(key, QString("Failed to start %1").arg(config->name));
        return;
    }

    drivers_.emplace(key, std::move(proc));

    // Handle startup delay if configured
    if (config->startupDelayMs > 0)
    {
        QThread::msleep(config->startupDelayMs);
    }
}

void ScannerController::stopProcess(const QString &key)
{
    shutdownProcess(key);
}

// -------------------------- generic slots --------------------------

void ScannerController::readDriverOutput()
{
    auto *senderProc = qobject_cast<QProcess *>(sender());
    if (!senderProc)
        return;
    const QString text = QString::fromUtf8(senderProc->readAllStandardOutput());
    const QString key = findVictimKey(senderProc);
    emit driverOutput(key, text);
    // qDebug().noquote() << text;
}

void ScannerController::processCrashed()
{
    auto *senderProc = qobject_cast<QProcess *>(sender());
    if (!senderProc)
        return;
    QString victimKey = findVictimKey(senderProc);
    if (!victimKey.isEmpty())
    {
        handleProcessCrash(victimKey);
    }
}

void ScannerController::processFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    Q_UNUSED(exitCode);
    auto *senderProc = qobject_cast<QProcess *>(sender());
    if (!senderProc)
        return;
    QString victimKey = findVictimKey(senderProc);
    if (victimKey.isEmpty())
        return;
    if (exitStatus == QProcess::CrashExit)
    {
        handleProcessCrash(victimKey);
        return;
    }
    handleProcessCompletion(victimKey);
}

// -------------------------- handlers --------------------------

void ScannerController::handleProcessCrash(const QString &crashedProc)
{
    shutdownProcess(crashedProc);
    emit driverCrashed(crashedProc);

    // Handle dependencies - if camera or lidar crashes, stop SLAM
    if (crashedProc == "camera" || crashedProc == "lidar")
    {
        stopSlam();
    }

    // If critical process crashed (like roscore), could stop everything
    const ProcessConfig *config = ProcessRegistry::instance().getProcess(crashedProc);
    if (config && config->critical)
    {
        qDebug() << "Critical process" << crashedProc << "crashed, stopping all drivers";
        stopDrivers();
    }
}

void ScannerController::handleProcessCompletion(const QString &completedProc)
{
    shutdownProcess(completedProc);
    emit driverStopped(completedProc);

    // Handle dependencies
    if (completedProc == "camera" || completedProc == "lidar")
    {
        stopSlam();
    }
}

QString ScannerController::findVictimKey(QProcess *proc)
{
    for (const auto &pair : drivers_)
    {
        if (pair.second.get() == proc)
        {
            return pair.first;
        }
    }
    return QString();
}

// -------------------------- shutdown helpers --------------------------

void ScannerController::shutdownProcess(const QString &key)
{
    auto it = drivers_.find(key);
    if (it == drivers_.end())
        return;

    const qint64 pid = it->second->processId();

    if (!killProcessGroup(pid, SIGINT, 2000) &&
        !killProcessGroup(pid, SIGTERM, 2000))
    {
        killProcessGroup(pid, SIGKILL, 0);
    }

    drivers_.erase(it);
    emit driverStopped(key);
}

bool ScannerController::killProcessGroup(qint64 pid, int sig, int waitMs)
{
    if (pid <= 0)
        return true;

    ::kill(-pid, sig);
    if (waitMs == 0)
        return false;

    const qint64 t0 = QDateTime::currentMSecsSinceEpoch();
    while (QDateTime::currentMSecsSinceEpoch() - t0 < waitMs)
    {
        if (::kill(pid, 0) == -1 && errno == ESRCH)
            return true;
        QThread::msleep(50);
    }
    return false;
}

void ScannerController::dumpDriversSnapshot()
{
    const qint64 ts = QDateTime::currentMSecsSinceEpoch();
    const QString filename = QString("drivers_%1.txt").arg(ts);

    QFile f(filename);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        return;
    }
    QTextStream out(&f);
    out << "# drivers_ snapshot at " << QDateTime::fromMSecsSinceEpoch(ts).toString(Qt::ISODate) << "\n";
    for (const auto &pair : drivers_)
    {
        const QString &key = pair.first;
        QProcess *proc = pair.second.get();
        out << key << ": pid=" << proc->processId() << " state=" << proc->state() << "\n";
    }
    f.close();
}

// -------------------------- query helpers --------------------------

bool ScannerController::isRunning(const QString &key) const
{
    return drivers_.count(key) > 0 || hasExternalRunning(key);
}

bool ScannerController::canStart(const QString &key) const
{
    const ProcessConfig *config = ProcessRegistry::instance().getProcess(key);
    if (!config)
        return false;
    std::unordered_map<QString, bool, QStringHash> running;
    // include controller-managed processes
    for (const auto &pair : drivers_)
    {
        running[pair.first] = true;
    }
    // include externally-running processes
    for (const QString &k : ProcessRegistry::instance().getAllKeys())
    {
        if (running.count(k) == 0 && !findExternalPids(k).isEmpty())
        {
            running[k] = true;
        }
    }
    return config->canStart(running);
}

QList<qint64> ScannerController::findExternalPids(const QString &key) const
{
    QList<qint64> result;
    const ProcessConfig *config = ProcessRegistry::instance().getProcess(key);
    if (!config)
        return result;

    QSet<qint64> ownedPids;
    for (const auto &pair : drivers_)
    {
        if (pair.second)
            ownedPids.insert(pair.second->processId());
    }

    QProcess pgrep;
    // Collect broad candidates first
    QString baseExec = config->executable;
    QString candidateCmd = QString("pgrep -f %1").arg(baseExec);
    pgrep.start("/bin/sh", QStringList() << "-c" << candidateCmd);
    if (!pgrep.waitForFinished(1000))
        return result;
    const QList<QByteArray> lines = pgrep.readAllStandardOutput().split('\n');

    // Prepare stricter match regex: start-of-line optional exec path, we verify with ps output
    QString escaped = QRegularExpression::escape(baseExec);
    QRegularExpression strictRe(QStringLiteral("(^| )%1( |$)"), QRegularExpression::CaseInsensitiveOption);
    // (Using captured escaped inside each per-process ps inspection instead of constructing complex pattern here.)

    for (const QByteArray &line : lines)
    {
        bool ok = false;
        qint64 pid = QString::fromUtf8(line).trimmed().toLongLong(&ok);
        if (!ok || pid <= 0)
            continue;
        if (ownedPids.contains(pid))
            continue; // skip internally managed
        // Validate by inspecting command line
        QString infoCmd = QString("ps -o args= -p %1 2>/dev/null").arg(pid);
        QProcess ps;
        ps.start("/bin/sh", QStringList() << "-c" << infoCmd);
        if (!ps.waitForFinished(300))
            continue; // skip if can't read
        QString cmdLine = QString::fromUtf8(ps.readAllStandardOutput()).trimmed();
        if (cmdLine.isEmpty())
            continue;
        // Heuristic: require the executable token to appear at start or after a space boundary
        if (!cmdLine.startsWith(baseExec) && !cmdLine.contains(QString(" ") + baseExec))
            continue;
        result.append(pid);
    }
    return result;
}

bool ScannerController::hasExternalRunning(const QString &key) const
{
    return !findExternalPids(key).isEmpty();
}

void ScannerController::cleanupExternal(const QString &key)
{
    const QList<qint64> pids = findExternalPids(key);
    if (key == "roscore" && adoptExternalRoscore_)
    {
        if (!pids.isEmpty())
        {
            qDebug() << "Skipping kill of external roscore (adopt).";
        }
        return;
    }
    for (qint64 pid : pids)
    {
        // Attempt group signaling where possible
        pid_t pgid = getpgid(static_cast<pid_t>(pid));
        auto sendSig = [&](int sig)
        {
            if (pgid > 0)
            {
                ::kill(-pgid, sig); // negative to target process group
            }
            else
            {
                ::kill(pid, sig);
            }
        };
        sendSig(SIGINT);
        QThread::msleep(kSignalPhaseDelayMs);
        if (::kill(pid, 0) == 0)
        {
            sendSig(SIGTERM);
            QThread::msleep(kSignalPhaseDelayMs);
        }
        if (::kill(pid, 0) == 0)
        {
            sendSig(SIGKILL);
        }
    }
}

void ScannerController::cleanupAllExternals()
{
    const auto keys = ProcessRegistry::instance().getAllKeys();
    for (const QString &key : keys)
    {
        const QList<qint64> pids = findExternalPids(key);
        if (pids.isEmpty())
            continue;
        qDebug() << "Cleaning external" << key << "PIDs=" << pids;
        cleanupExternal(key);
    }
}
