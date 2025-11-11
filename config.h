#pragma once
#include <QString>
#include <QStringList>

namespace Config {
    // Recording settings
    constexpr int RECORDING_DURATION_SEC = 600;  // 10 minutes
    const QString DEFAULT_BAG_PREFIX = "scan";
    
    inline QStringList getRecordingTopics() {
        return {
            "/river_msgs/camera/color/CameraInfo",
            "/river_msgs/camera/depth/CameraInfo",
            "/river_msgs/camera/color/image",
            "/river_msgs/camera/depth/image",
            "/velodyne_points",
            "/imu/data",
            "/tf",
            "/tf_static",
            "/average_offset",
            "/gps/fix"
        };
    }
    
    // Diagnostics settings
    inline QStringList getMonitoredTasks() {
        return {
            "river_watchdog: camera_rate",
            "river_watchdog: lidar_rate",
            "river_watchdog: Offset Accuracy",
            "river_watchdog: gps_rate"
        };
    }
    
    // UI settings
    constexpr int COUNTDOWN_UPDATE_INTERVAL_MS = 1000;
    
    // Diagnostic levels
    enum DiagnosticLevel {
        OK = 0,
        WARN = 1,
        ERROR = 2,
        STALE = 3
    };
}