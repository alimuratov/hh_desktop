#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QDebug>
#include <QString>
#include <QMessageBox>
#include <memory>
#include <QDateTime>
#include <QThread>
#include <csignal>
#include <unordered_map>
#include <QString>
#include <QStringList>
#include <QtGlobal> 
#include <QLabel>
#include <QSet>
#include <QCloseEvent>
#include "scanner_controller.h"
#include "diagnostics_monitor.h"
#include "rosbagrecorder.h"
#ifdef HH_ENABLE_RVIZ
#include "rvizwidget.h"
#endif

#include "mapvizwidget.h"

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
    void startSlam();
    void stopSlam();
    void startDynamicReconfigure();
    void stopDynamicReconfigure();

    //driver feedback
    void onDriverStarted(const QString& key);
    void onDriverStopped(const QString& key);
    void onDriverCrashed(const QString& key);
    void onDriverOutput(const QString& key, const QString& output);

    void onDiagStatus(const QString&, int, const QString&, const QString&);

    void onRecordingStarted();
    void onRecordingStopped();
    void on_cameraCheckbox_stateChanged(int checked);
    void on_lidarCheckbox_stateChanged(int checked);
    void on_gpsCheckbox_stateChanged(int checked);
    void showNextPage();
    void showPrevPage();
private:
    Ui::MainWindow *ui;
#ifdef HH_ENABLE_RVIZ
      std::unique_ptr<RvizWidget> rviz_widget_;
#endif
    std::unique_ptr<MapvizWidget> mapviz_widget_;
    std::unique_ptr<ScannerController> scanner_;
    QSet<QString> recordTopics_;
    bool cameraRunning_ = false;
    bool lidarRunning_ = false;
    bool gpsRunning_ = false;
};
#endif // MAINWINDOW_H
