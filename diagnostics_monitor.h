#pragma once
#include <QObject>
#include <QString>
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <unordered_map>
#include <string>
#include "config.h"

// inheriting from QObject allows us to pass a parent pointer in the constructor
// ensuring that the monitor is destroyed automatically when the window closes
class DiagnosticsMonitor : public QObject {
  Q_OBJECT // gives the class its own signal - slot mechanism
public:
  // The ctor takes an already‑initialised ROS node‑handle and begins the
  // subscription.  The object lives entirely in the Qt (GUI) thread: callbacks
  // are delivered there because we use ros::AsyncSpinner(1) elsewhere.
  explicit DiagnosticsMonitor(ros::NodeHandle &nh, QObject *parent = nullptr)
      : QObject(parent) {
    sub_ = nh.subscribe("/diagnostics", 10,
                        &DiagnosticsMonitor::diagCb, this);
  }

signals:
  // Emitted whenever the level of a watched task changes (0 = OK, 1 = WARN, 2 = ERROR, 3 = STALE). 
  void statusChanged(QString taskName, int level, QString message, QString recordedFrequency);

private:
  void diagCb(const diagnostic_msgs::DiagnosticArrayConstPtr &msg) {
    for (const auto &status : msg->status) {
      const std::string &name = status.name;
      if (name != "river_watchdog: camera_rate" &&
          name != "river_watchdog: lidar_rate" &&
          name != "river_watchdog: Offset Accuracy") {
        continue; // we only care about these three tasks
      }
      int level = status.level;
      auto it = last_level_.find(name); 
      std::string frequency; 
      for (const auto& kv : status.values)
      {
          if (kv.key == "Actual frequency (Hz)")
          {
              frequency = kv.value; 
              break;
          }
      }
      if (it == last_level_.end() || it->second != level) {
        last_level_[name] = level;
        emit statusChanged(QString::fromStdString(name), level,
                           QString::fromStdString(status.message),
                           QString::fromStdString(frequency));
      }
    }
  }

  /*void diagCb(const diagnostic_msgs::DiagnosticArrayConstPtr &msg) {
    // Get the list of tasks we should monitor from the central config file.
    const QStringList monitored_tasks = Config::getMonitoredTasks();

    for (const auto &status : msg->status) {
      // Convert the incoming status name to a QString for comparison
      const QString taskName = QString::fromStdString(status.name);

      // Check if this task is in our list from the config file.
      // If the list from the config file does NOT contain this taskName, skip it.
      if (!monitored_tasks.contains(taskName)) {
        continue;
      }

      // --- The rest of the function is the same as before ---
      // If we get here, it's a name we care about!
      int level = status.level;
      auto it = last_level_.find(status.name); // Use status.name (std::string) here
      std::string frequency; 
      for (const auto& kv : status.values)
      {
          if (kv.key == "Actual frequency (Hz)")
          {
              frequency = kv.value; 
              break;
          }
      }
      if (it == last_level_.end() || it->second != level) {
        last_level_[status.name] = level;
        emit statusChanged(taskName, level,
                           QString::fromStdString(status.message),
                           QString::fromStdString(frequency));
      }
    }
  }*/


  ros::Subscriber sub_;
  std::unordered_map<std::string, int> last_level_;
};
