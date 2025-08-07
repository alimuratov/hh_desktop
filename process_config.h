#pragma once
#include <QString>
#include <QStringList>
#include <functional>
#include <memory>
#include <unordered_map>

// Process definition structure
struct ProcessConfig {
    QString key;
    QString name;
    QString executable;
    QStringList arguments;
    // Check if the process can start based on currently running processes
    std::function<bool(const std::unordered_map<QString, bool>&)> canStart;
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
    std::unordered_map<QString, ProcessConfig> processes_;
};

// Initialize all process definitions
inline void initializeProcesses() {
    auto& registry = ProcessRegistry::instance();
    
    // Roscore
    registry.registerProcess({
        .key = "roscore",
        .name = "ROS Core",
        .executable = "/usr/bin/roscore",
        .arguments = {},
        .canStart = [](const auto&) { return true; },  // Always can start
        .startupDelayMs = 1000,
        .critical = true
    });
    
    // Camera
    registry.registerProcess({
        .key = "camera",
        .name = "Camera Driver",
        .executable = "/home/kodifly/setup_scripts/camera_setup.sh",
        .arguments = {},
        .canStart = [](const auto& running) { 
            return running.count("roscore") > 0; 
        },
        .critical = false
    });
    
    // Lidar
    registry.registerProcess({
        .key = "lidar",
        .name = "Lidar Driver",
        .executable = "/home/kodifly/setup_scripts/lidar_setup.sh",
        .arguments = {},
        .canStart = [](const auto& running) { 
            return running.count("roscore") > 0; 
        },
        .critical = false
    });
    
    // SLAM
    registry.registerProcess({
        .key = "slam",
        .name = "SLAM",
        .executable = "/home/kodifly/setup_scripts/start_slam.sh",
        .arguments = {},
        .canStart = [](const auto& running) { 
            return running.count("camera") > 0 && running.count("lidar") > 0; 
        },
        .critical = false
    });
    
    // Watchdog
    registry.registerProcess({
        .key = "watchdog",
        .name = "Watchdog",
        .executable = "/home/kodifly/setup_scripts/watchdog_setup.sh",
        .arguments = {},
        .canStart = [](const auto& running) { 
            return running.count("roscore") > 0; 
        },
        .critical = false
    });
}