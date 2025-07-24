#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QDebug>
#include <QProcess>
#include <QTimer>
#include <QString>
#include <QMessageBox>
#include <memory>
#include <QDateTime>
#include <QThread>
#include <csignal>
#include <unordered_map>
#include <QString>
#include <QtGlobal> 
#include <QLabel>
#include "diagnostics_monitor.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

// Every translation unit that includes mainwindow.h needs the QString hash functor's definition, so we place it here
// One-definition-rule is still not broken because a class/struct definition in a header is allowed to appear in multiple TUs as long as it is identical
struct QStringHash {
    std::size_t operator()(const QString &s) const noexcept { return qHash(s); } 
};

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
protected:
    void closeEvent(QCloseEvent* event) override; 
private slots:
    // UI buttons
    void startDrivers();
    void stopDrivers();

    // generic helpers
    void readDriverOutput();
    void processCrashed(); 
    void processFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void onDiagStatus(const QString&, int, const QString&, const QString&);
private:
    std::unique_ptr<QProcess> createDriverProcess(const QString& scriptPath,
                                                  const QString& key); 
    void shutdownProcess(const QString& key); 
    bool killProcessGroup(qint64 pid, int sig, int waitMs); // qint64 is a qt's aliws for int64
    void startCamera();
    void stopCamera();
    void startLidar();
    void stopLidar(); 
    void startWatchdog();
    void stopWatchdog();
    void handleProcessCrash(const QString& crashedProc);
    void handleProcessCompletion(const QString& completedProc); 
    QString findVictimKey(QProcess* proc);

    Ui::MainWindow *ui;
    std::unordered_map<QString, std::unique_ptr<QProcess>, QStringHash> drivers_; 
    std::unique_ptr<DiagnosticsMonitor> diag_monitor_;
    QTimer* rosTimer_;
};
#endif // MAINWINDOW_H
