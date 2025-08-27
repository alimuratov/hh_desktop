#pragma once
#include <QObject>
#include <QProcess>
#include <QTimer>
#include <QString>
#include <memory>
#include <unordered_map>
#include <QStringList>
#include "rosbagrecorder.h"
#include "diagnostics_monitor.h"
#include "qstring_hash.h"

// Forward declaration
struct ProcessConfig;

class ScannerController : public QObject
{
    Q_OBJECT
public:
    explicit ScannerController(ros::NodeHandle &nh, QObject *parent = nullptr);

    // -------------------------- Recording --------------------------
    void startRecording(const QString &filename, const QStringList &topics);
    void stopRecording();
    bool isRecording() const;

    // -------------------------- Driver control --------------------------
    void startDrivers();
    void stopDrivers();
    void startSlam();
    void stopSlam();
    void startDynamicReconfigure();
    void stopDynamicReconfigure();

    // -------------------------- Mapviz control --------------------------
    void startMapviz();
    void stopMapviz();

    // -------------------------- Query state/prerequisites --------------------------
    bool isRunning(const QString &key) const;
    bool canStart(const QString &key) const;
    bool hasExternalRunning(const QString &key) const;
    void cleanupExternal(const QString &key);
    void cleanupAllExternals();

    // Configuration: force cleanup of externally running processes before starting drivers
    void setForceExternalCleanup(bool enable) { forceExternalCleanup_ = enable; }
    bool forceExternalCleanup() const { return forceExternalCleanup_; }
    void setAdoptExternalRoscore(bool enable) { adoptExternalRoscore_ = enable; }
    bool adoptExternalRoscore() const { return adoptExternalRoscore_; }
    void setExternalDetectionVerbose(bool enable) { verboseExternalDetect_ = enable; }
    bool externalDetectionVerbose() const { return verboseExternalDetect_; }
    void resetDiagnosticsCache();

signals:
    void recordingStarted();
    void recordingStopped();
    void recordingError(const QString &error);
    void diagnosticsUpdated(const QString &taskName, int level,
                            const QString &message, const QString &frequency);

    void driverStarted(const QString &key);
    void driverStopped(const QString &key);
    void driverCrashed(const QString &key);
    void driverError(const QString &key, const QString &errorMessage);
    void driverOutput(const QString &key, const QString &output);

private:
    // recording and diagnostics
    std::unique_ptr<RosbagRecorder> recorder;
    std::unique_ptr<DiagnosticsMonitor> diagnosticsMonitor;

    // process management
    using DriverMap = std::unordered_map<QString, std::unique_ptr<QProcess>, QStringHash>;
    DriverMap drivers_;
    QTimer *rosTimer_;
    QTimer *driversDumpTimer_ = nullptr;

    // Generic process control
    void startProcess(const QString &key);
    void stopProcess(const QString &key);

    // helpers
    std::unique_ptr<QProcess> createDriverProcess(const ProcessConfig &config);
    void shutdownProcess(const QString &key);
    bool killProcessGroup(qint64 pid, int sig, int waitMs);
    QString findVictimKey(QProcess *proc);
    QList<qint64> findExternalPids(const QString &key) const;

    void handleProcessCrash(const QString &crashedProc);
    void handleProcessCompletion(const QString &completedProc);

    // config flags
    bool forceExternalCleanup_ = true;   // if true, startDrivers() will attempt to stop externally running matching processes first
    bool adoptExternalRoscore_ = true;   // treat an already-running roscore as acceptable and 'adopt' it instead of erroring
    bool verboseExternalDetect_ = false; // emit detailed logs for findExternalPids

    // tuning constants
    static constexpr int kExternalRetryAttempts = 10; // total attempts after cleanup to verify external gone
    static constexpr int kExternalRetryDelayMs = 100; // delay between attempts
    static constexpr int kSignalPhaseDelayMs = 500;   // delay between escalating signals in cleanupExternal
    // auto-restart tuning
    static constexpr int kMaxRestartAttempts = 5;   // maximum restart attempts per process
    static constexpr int kStableUptimeMs = 1000;    // uptime threshold (ms) after which process considered stable (no more restarts)
    static constexpr int kBaseRestartDelayMs = 500; // base delay before restarting (will be multiplied by attempt index for simple backoff)

    // restart tracking
    std::unordered_map<QString, int, QStringHash> restartAttempts_;    // how many times we've tried restarting
    std::unordered_map<QString, qint64, QStringHash> startTimestamps_; // ms timestamp when process last started
    quint64 restartEpoch_ = 0;                                         // increments whenever stopDrivers() is invoked to invalidate pending restart timers

    void scheduleRestart(const QString &key, const QString &reason, int attempt);
    bool shouldAutoRestart(const QString &key) const; // policy gate

private slots:
    void readDriverOutput();
    void processCrashed();
    void processFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void dumpDriversSnapshot();
};