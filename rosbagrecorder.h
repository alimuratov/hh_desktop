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

public slots:
    void startRecording(const QStringList &topics);
    void stopRecording();

signals:
    void recordingStarted();
    void recordingStopped();
    void processOutput(const QString &output);

private slots:
    void onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void onReadyReadStandardOutput();

private:
    std::unique_ptr<QProcess> *rosbagProc;
};

#endif // ROSBAGRECORDER_H
