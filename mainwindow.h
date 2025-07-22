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

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

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
    void driverCrashed(); 
private:
    std::unique_ptr<QProcess> createDriverProcess(const QString& scriptPath,
                                                  const QString& key); 
    void shutdownDriver(const QString& key); 
    bool killProcessGroup(qint64 pid, int sig, int waitMs); // qint64 is a qt's aliws for int64
    void startCamera();
    void stopCamera();
    void startLidar();
    void stopLidar(); 

    Ui::MainWindow *ui;
    std::unordered_map<QString, std::unique_ptr<QProcess>> drivers_; 
};
#endif // MAINWINDOW_H
