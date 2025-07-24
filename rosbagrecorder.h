#ifndef ROSBAGRECORDER_H
#define ROSBAGRECORDER_H

#include <QObject>
#include <QStringList>
#include <QProcess>
#include <memory>
#include <csignal>

class RosbagRecorder : public QObject
{
    Q_OBJECT
public:
    // declare a constructor as explicit to prevent unintended implicit type conversions.
    explicit RosbagRecorder(QObject *parent = nullptr);
    ~RosbagRecorder();
    bool getIsRecording() { return isRecording; }

public slots:
    void startRecording(const QString &bagName, const QStringList &topics);
    void stopRecording();

signals:
    void recordingStarted();
    void recordingStopped();
    void processOutput(const QString &output);

private slots:
    void onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void onReadyReadStandardOutput();

private:
    bool killProcessGroup(qint64 pid, int sig, int waitMs);
    std::unique_ptr<QProcess> rosbagProc_;
    bool isRecording = false;
};

#endif // ROSBAGRECORDER_H
