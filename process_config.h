#pragma once
#include <QString>
#include <QStringList>
#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>
#include "qstring_hash.h"

// Process definition structure
struct ProcessConfig {
    QString key;
    QString name;
    QString executable;
    QStringList arguments;
    // Check if the process can start based on currently running processes
    std::function<bool(const std::unordered_map<QString, bool, QStringHash>&)> canStart;
    int startupDelayMs = 0;
    bool critical = false;  // If true, crash will stop all processes
};

// Process registry singleton
class ProcessRegistry {
public:
    static ProcessRegistry& instance() {
        static ProcessRegistry instance;
        return instance;
    }
    
    void registerProcess(const ProcessConfig& config) {
        processes_[config.key] = config;
    }
    
    const ProcessConfig* getProcess(const QString& key) const {
        auto it = processes_.find(key);
        return it != processes_.end() ? &it->second : nullptr;
    }
    
    std::vector<QString> getAllKeys() const {
        std::vector<QString> keys;
        for (const auto& [key, _] : processes_) {
            keys.push_back(key);
        }
        return keys;
    }
    
private:
    ProcessRegistry() = default;
    ProcessRegistry(const ProcessRegistry&) = delete;
    ProcessRegistry& operator=(const ProcessRegistry&) = delete;
    std::unordered_map<QString, ProcessConfig, QStringHash> processes_;
};

// Initialize all process definitions
inline void initializeProcesses() {
    auto& registry = ProcessRegistry::instance();

    // Roscore
    registry.registerProcess({
        "roscore",
        "ROS Core",
        "/usr/bin/roscore",
        {},
        [](const auto&) { return true; },  // Always can start
        1000,
        true
    });

    // Camera
    registry.registerProcess({
        "camera",
        "Camera Driver",
        "/home/kodifly/setup_scripts/camera_setup.sh",
        {},
        [](const auto& running) {
            return running.count("roscore") > 0;
        },
        0,
        false
    });

    // Lidar
    registry.registerProcess({
        "lidar",
        "Lidar Driver",
        "/home/kodifly/setup_scripts/lidar_setup.sh",
        {},
        [](const auto& running) {
            return running.count("roscore") > 0;
        },
        0,
        false
    });

    // PTP
    registry.registerProcess({
        "ptp4l",
        "PTP4L",
        "/home/kodifly/setup_scripts/ptp4l.sh",
        {},
        [](const auto& running) {
            return running.count("camera") > 0 && running.count("lidar") > 0;
        },
        0,
        false
    });

    // Sync Status
    registry.registerProcess({
        "sync",
        "SYNC",
        "/home/kodifly/setup_scripts/offset_setup.sh",
        {},
        [](const auto& running) {
            return running.count("camera") > 0 && running.count("lidar") > 0;
        },
        0,
        false
    });

    // SLAM
    registry.registerProcess({
        "slam",
        "SLAM",
        "/home/kodifly/setup_scripts/start_slam.sh",
        {},
        [](const auto& running) {
            return running.count("camera") > 0 && running.count("lidar") > 0;
        },
        0,
        false
    });

    // Watchdog
    registry.registerProcess({
        "watchdog",
        "Watchdog",
        "/home/kodifly/setup_scripts/watchdog_setup.sh",
        {},
        [](const auto& running) {
            return running.count("roscore") > 0;
        },
        0,
        false
    });

    // Dynamic Reconfigure
    registry.registerProcess({
        "dynamic_reconfigure",
        "Dynamic Reconfigure",
        "/home/kodifly/setup_scripts/dynamic_reconfigure_setup.sh",
        {},
        [](const auto& running) {
            return running.count("camera") > 0;
        },
        500,  // Small delay to ensure camera is ready
        false
    });
}
