#include "scanner_controller.h"
#include "process_config.h"
#include <QDateTime>
#include <QThread>
#include <QDebug>
#include <QFile>
#include <QProcess>
#include <QTextStream>
#include <QRegularExpression>
#include <QElapsedTimer>
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
    qDebug() << "[startup] Beginning driver startup sequence";
    // Define ordered startup (excluding roscore handled separately first)
    const QStringList orderedKeys = {"camera", "lidar", "gpsrtk", "ptp4l", "watchdog", "sync", "dynamic_reconfigure"};
    const int totalSteps = 1 + orderedKeys.size(); // roscore + others
    int step = 1;

    // Start or adopt roscore first
    if (adoptExternalRoscore_ && hasExternalRunning("roscore"))
    {
        qDebug() << QString("[startup] (%1/%2) roscore: adopting existing external instance").arg(step).arg(totalSteps);
    }
    else
    {
        qDebug() << QString("[startup] (%1/%2) roscore: launching internal instance").arg(step).arg(totalSteps);
        startProcess("roscore");
        // Allow some time for master to come up
        qDebug() << "[startup] waiting 1s for roscore to initialize";
        QThread::msleep(1000);
    }
    // Iterate remaining processes
    for (const QString &key : orderedKeys)
    {
        ++step;
        qDebug() << QString("[startup] (%1/%2) launching %3").arg(step).arg(totalSteps).arg(key);
        startProcess(key);
    }
    qDebug() << "[startup] Startup sequence queued. Processes will emit individual PID logs as they actually start.";
}

void ScannerController::stopDrivers()
{
    // Invalidate any pending scheduled restarts and clear tracking at the end.
    ++restartEpoch_;
    qDebug() << "Stopping all drivers; invalidate restart timers epoch=" << restartEpoch_;
    // Ordered shutdown list (reverse-ish dependency order; adjust as needed)
    QStringList shutdownOrder = {"slam", "dynamic_reconfigure", "watchdog", "ptp4l", "gpsrtk", "lidar", "camera", "roscore"};
    int total = shutdownOrder.size();
    int idx = 0;
    for (const QString &k : shutdownOrder)
    {
        ++idx;
        bool owned = drivers_.count(k) > 0;
        qDebug() << QString("[shutdown] (%1/%2) %3 %4")
                        .arg(idx)
                        .arg(total)
                        .arg(k)
                        .arg(owned ? "stopping" : "skip (not running / external)");
        if (owned)
            stopProcess(k);
    }
    qDebug() << "[shutdown] sequence complete";
    restartAttempts_.clear();
    startTimestamps_.clear();
    // Reset diagnostics cache so next startup re-emits initial statuses even if level unchanged
    resetDiagnosticsCache();
}

void ScannerController::resetDiagnosticsCache()
{
    if (diagnosticsMonitor)
        diagnosticsMonitor->resetCache();
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
            startTimestamps_[key] = QDateTime::currentMSecsSinceEpoch();
            emit driverStarted(key); });
    connect(proc.get(), &QProcess::readyReadStandardOutput, this, &ScannerController::readDriverOutput);
    connect(proc.get(), &QProcess::errorOccurred,
            this, [this, k = config.key](QProcess::ProcessError err)
            {
                if (k == "mapviz") {
                    qDebug() << "[mapviz] QProcess errorOccurred" << err;
                }
                processCrashed(); });

    connect(proc.get(), qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
            this, [this, k = config.key](int code, QProcess::ExitStatus st)
            {
                if (k == "mapviz") {
                    qDebug() << "[mapviz] processFinished exitCode=" << code << " exitStatus=" << (st==QProcess::NormalExit?"Normal":"Crash");
                }
                processFinished(code, st); });

    proc->start(program, args);

    if (!proc->waitForStarted())
    {
        if (config.key == "mapviz")
        {
            qDebug() << "[mapviz] waitForStarted failed";
        }
        emit driverCrashed(config.key);
        return nullptr;
    }

    if (config.key == "mapviz")
    {
        qDebug() << "[mapviz] createDriverProcess started PID=" << proc->processId() << " program=/usr/bin/setsid wrapped command args=" << args;
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

    if (key == "mapviz")
    {
        qDebug() << "[mapviz] startProcess invoked";
    }

    // External instance handling
    QList<qint64> initialExternalPids;
    if (hasExternalRunning(key))
    {
        if (key == "mapviz")
        {
            qDebug() << "[mapviz] detected external instance candidates before start" << findExternalPids(key);
        }
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
                // For each candidate external PID we gather a concise diagnostic snapshot:
                // 1) Verify PID is positive.
                // 2) alive flag via kill(pid,0) (1 means still exists).
                // 3) Query its process group (pgid) so we can reason about group kills later.
                // 4) Fetch trimmed command line (ps -o args=) to confirm it actually matches the
                //    executable we care about (this was already pre-filtered, but we show it for the user).
                // 5) Truncate overly long command lines for message box readability.
                // The resulting string list is used purely for rich error reporting when we refuse to
                // start because an external instance is still present; we are NOT killing here.
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
        if (key == "mapviz")
        {
            qDebug() << "[mapviz] canStart returned false";
        }
        emit driverError(key, QString("%1 prerequisites not met").arg(config->name));
        return;
    }
    else if (key == "mapviz")
    {
        qDebug() << "[mapviz] prerequisites satisfied, launching process";
    }

    auto proc = createDriverProcess(*config);
    if (!proc)
    {
        emit driverError(key, QString("Failed to start %1").arg(config->name));
        return;
    }

    drivers_.emplace(key, std::move(proc));
    if (key == "mapviz")
    {
        qDebug() << "[mapviz] process stored in drivers_ map";
    }

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
    // emit driverOutput(key, text);
    // qDebug().noquote() << text;
    if (key == "mapviz" && !text.trimmed().isEmpty())
    {
        // Use QString::SkipEmptyParts for broader Qt version compatibility (older Qt versions don't have Qt::SkipEmptyParts)
        QStringList lines = text.split('\n', QString::SkipEmptyParts);
        for (const QString &l : lines)
        {
            qDebug().noquote() << "[mapviz][stdout] " << l;
        }
    }
}

void ScannerController::processCrashed()
{
    auto *senderProc = qobject_cast<QProcess *>(sender());
    if (!senderProc)
        return;
    QString victimKey = findVictimKey(senderProc);
    if (!victimKey.isEmpty())
    {
        if (victimKey == "mapviz")
        {
            qDebug() << "[mapviz] processCrashed slot invoked";
        }
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
    {
        // This happens when we manually initiated shutdown: we removed the process from
        // drivers_ inside shutdownProcess() before the QProcess emitted finished().
        // In that case driverStopped was already emitted (source=shutdownProcess) and
        // we intentionally skip handleProcessCompletion to avoid double emissions.
        qDebug() << "[restart] finished signal for already-removed process (likely manual stop)";
        return;
    }
    if (exitStatus == QProcess::CrashExit)
    {
        if (victimKey == "mapviz")
        {
            qDebug() << "[mapviz] processFinished forwarded as crash";
        }
        handleProcessCrash(victimKey);
        return;
    }
    if (victimKey == "mapviz")
    {
        qDebug() << "[mapviz] processFinished normal exit";
    }
    handleProcessCompletion(victimKey);
}

// -------------------------- handlers --------------------------

void ScannerController::handleProcessCrash(const QString &crashedProc)
{
    if (crashedProc == "mapviz")
    {
        qDebug() << "[mapviz] handleProcessCrash";
    }
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

    // Auto-restart logic (only for non-critical processes)
    if (config && !config->critical && shouldAutoRestart(crashedProc))
    {
        int attempt = ++restartAttempts_[crashedProc];
        scheduleRestart(crashedProc, QStringLiteral("crash"), attempt);
    }
}

void ScannerController::handleProcessCompletion(const QString &completedProc)
{
    if (completedProc == "mapviz")
    {
        qDebug() << "[mapviz] handleProcessCompletion";
    }
    qint64 uptimeMs = -1;
    auto tsIt = startTimestamps_.find(completedProc);
    if (tsIt != startTimestamps_.end())
    {
        uptimeMs = QDateTime::currentMSecsSinceEpoch() - tsIt->second;
    }
    shutdownProcess(completedProc);
    // NOTE: driverStopped already emitted inside shutdownProcess(). We previously
    // emitted it again here (causing duplicate signals on natural exit). That
    // duplication confused restart / UI logic, so it has been removed. Leaving
    // this comment for traceability.
    qDebug() << "[restart] completion path after shutdownProcess for" << completedProc << "uptimeMs=" << uptimeMs;

    // Handle dependencies
    // if (completedProc == "camera" || completedProc == "lidar")
    //{
    //    stopSlam();
    //}

    // Decide if we should restart (short-lived normal exit treated as failure)
    const ProcessConfig *config = ProcessRegistry::instance().getProcess(completedProc);
    if (config && !config->critical && shouldAutoRestart(completedProc))
    {
        // If process exited very quickly (< stable uptime), attempt restart
        if (uptimeMs >= 0 && uptimeMs < kStableUptimeMs)
        {
            int attempt = ++restartAttempts_[completedProc];
            qDebug() << "[restart] early-exit normal completion scheduling restart" << completedProc << "attempt" << attempt << "uptimeMs=" << uptimeMs << "<" << kStableUptimeMs;
            scheduleRestart(completedProc, QStringLiteral("early-exit (%1 ms)").arg(uptimeMs), attempt);
            return;
        }
        else
        {
            qDebug() << "[restart] completion considered stable (no restart)" << completedProc << "uptimeMs=" << uptimeMs << ">=" << kStableUptimeMs;
        }
    }
    else
    {
        qDebug() << "[restart] auto-restart not applicable after completion" << completedProc << "critical=" << (config ? config->critical : -1);
    }

    // Stable completion (clear attempts tracking)
    restartAttempts_.erase(completedProc);
    startTimestamps_.erase(completedProc);
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
    if (key == "mapviz")
    {
        qDebug() << "[mapviz] shutdownProcess begin";
    }

    const qint64 pid = it->second->processId();
    QElapsedTimer timer;
    timer.start();

    auto sendWithLog = [&](int sigName, const char *sigLabel, int waitMs) -> bool
    {
        qDebug() << "[shutdown]" << key << "send" << sigLabel << "pid=" << pid << "pgid=-" << pid << "waitMs=" << waitMs;
        bool done = killProcessGroup(pid, sigName, waitMs);
        if (done)
            qDebug() << "[shutdown]" << key << sigLabel << "success (terminated)";
        else
            qDebug() << "[shutdown]" << key << sigLabel << "timeout";
        return done;
    };

    if (!sendWithLog(SIGINT, "SIGINT", 2000) &&
        !sendWithLog(SIGTERM, "SIGTERM", 2000))
    {
        qDebug() << "[shutdown]" << key << "escalating to SIGKILL";
        killProcessGroup(pid, SIGKILL, 0);
    }

    drivers_.erase(it);

    qDebug() << "[shutdown]" << key << ": emit driverStopped (source=shutdownProcess)";
    emit driverStopped(key);

    if (key == "mapviz")
    {
        qDebug() << "[mapviz] shutdownProcess end";
    }
    qDebug() << "[shutdown]" << key << "removed; elapsedMs=" << timer.elapsed();
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

bool ScannerController::shouldAutoRestart(const QString &key) const
{
    // Policy: don't auto-restart roscore (leave adoption/cleanup logic), otherwise allow for all known processes including mapviz.
    if (key == "roscore")
    {
        qDebug() << "[restart] shouldAutoRestart(" << key << ")=false (roscore policy)";
        return false;
    }
    // Respect max attempts
    auto it = restartAttempts_.find(key);
    if (it != restartAttempts_.end() && it->second >= kMaxRestartAttempts)
    {
        qDebug() << "[restart] shouldAutoRestart(" << key << ")=false (max attempts reached) attempts=" << it->second;
        return false;
    }
    qDebug() << "[restart] shouldAutoRestart(" << key << ")=true";
    return true;
}

void ScannerController::scheduleRestart(const QString &key, const QString &reason, int attempt)
{
    if (attempt > kMaxRestartAttempts)
        return;                                // safety guard
    int delay = kBaseRestartDelayMs * attempt; // simple linear backoff
    qDebug() << "[restart] scheduling restart for" << key << "attempt" << attempt << "in" << delay << "ms reason=" << reason;
    quint64 epochAtSchedule = restartEpoch_;
    QTimer::singleShot(delay, this, [this, key, epochAtSchedule]()
                       {
        if (epochAtSchedule != restartEpoch_) {
            qDebug() << "[restart] epoch changed; aborting scheduled restart for" << key;
            return; // stopDrivers() called meanwhile
        }

        // Ensure not already running (might have been started manually in the meantime)
        if (drivers_.count(key))
            return;

        if (!canStart(key))
        {
            qDebug() << "[restart] prerequisites still not met for" << key << "- deferring attempt";
            // Requeue with small delay without increasing attempt count (prereq wait)
            QTimer::singleShot(500, this, [this, key, epochAtSchedule]() {
                if (epochAtSchedule != restartEpoch_) {
                    qDebug() << "[restart] epoch changed; aborting deferred restart for" << key;
                    return;
                }
                scheduleRestart(key, QStringLiteral("waiting-prereqs"), restartAttempts_[key]);
            });
            return;
        }

        startProcess(key); });
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
    bool ok = config->canStart(running);
    if (key == "mapviz")
    {
        QStringList runningList;
        for (auto &kv : running)
            runningList << kv.first;
        // qDebug() << "[mapviz] canStart check: running set=" << runningList << " result=" << ok;
    }
    return ok;
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
    {
        if (verboseExternalDetect_)
        {
            qDebug() << "[ext-detect] timeout waiting for pgrep for" << key << "cmd=" << candidateCmd;
        }
        return result;
    }
    const QList<QByteArray> lines = pgrep.readAllStandardOutput().split('\n');
    if (verboseExternalDetect_)
    {
        qDebug() << "[ext-detect] raw pgrep candidates for" << key << ":" << lines;
    }

    // Prepare stricter match regex: start-of-line optional exec path, we verify with ps output
    QString escaped = QRegularExpression::escape(baseExec);
    QRegularExpression strictRe(QStringLiteral("(^| )%1( |$)"), QRegularExpression::CaseInsensitiveOption);
    // (Using captured escaped inside each per-process ps inspection instead of constructing complex pattern here.)

    for (const QByteArray &line : lines)
    {
        bool ok = false;
        qint64 pid = QString::fromUtf8(line).trimmed().toLongLong(&ok);
        if (!ok || pid <= 0)
        {
            if (verboseExternalDetect_)
                qDebug() << "[ext-detect] skip non-numeric token" << line;
            continue;
        }
        if (ownedPids.contains(pid))
        {
            if (verboseExternalDetect_)
                qDebug() << "[ext-detect] skip owned pid" << pid;
            continue; // skip internally managed
        }
        QString infoCmd = QString("ps -o args= -p %1 2>/dev/null").arg(pid);
        QProcess ps;
        ps.start("/bin/sh", QStringList() << "-c" << infoCmd);
        if (!ps.waitForFinished(300))
        {
            if (verboseExternalDetect_)
                qDebug() << "[ext-detect] ps timeout pid" << pid;
            continue;
        }
        QString cmdLine = QString::fromUtf8(ps.readAllStandardOutput()).trimmed();
        if (cmdLine.isEmpty())
        {
            if (verboseExternalDetect_)
                qDebug() << "[ext-detect] empty cmdline pid" << pid;
            continue;
        }
        bool match = cmdLine.startsWith(baseExec) || cmdLine.contains(QString(" ") + baseExec);
        if (!match)
        {
            if (verboseExternalDetect_)
                qDebug() << "[ext-detect] reject pid" << pid << "cmdLine=" << cmdLine;
            continue;
        }
        if (verboseExternalDetect_)
            qDebug() << "[ext-detect] accept pid" << pid << "cmdLine=" << cmdLine;
        result.append(pid);
    }
    if (verboseExternalDetect_)
    {
        qDebug() << "[ext-detect] final accepted pids for" << key << "=>" << result;
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
