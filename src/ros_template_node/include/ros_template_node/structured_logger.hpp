#pragma once

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <nlohmann/json.hpp>
#include <memory>
#include <string>
#include <chrono>

namespace ros_template_node {

class StructuredLogger {
public:
    enum class Level {
        DEBUG = 0,
        INFO = 1,
        WARN = 2,
        ERROR = 3,
        FATAL = 4
    };

    enum class Component {
        STARTUP,
        TIMER,
        SUBSCRIBER,
        HEALTH,
        SHUTDOWN
    };

    enum class EventType {
        PUBLISH,
        RECEIVE,
        HEALTH_CHECK,
        INITIALIZATION,
        PARAMETER_UPDATE,
        ERROR_OCCURRED
    };

    explicit StructuredLogger(const std::string& node_name);
    ~StructuredLogger() = default;

    void log(Level level, Component component, EventType event_type, 
             const std::string& message, const nlohmann::json& context = {});

    void debug(Component component, EventType event_type, 
               const std::string& message, const nlohmann::json& context = {});
    void info(Component component, EventType event_type, 
              const std::string& message, const nlohmann::json& context = {});
    void warn(Component component, EventType event_type, 
              const std::string& message, const nlohmann::json& context = {});
    void error(Component component, EventType event_type, 
               const std::string& message, const nlohmann::json& context = {});

private:
    std::string node_name_;
    std::shared_ptr<spdlog::logger> logger_;
    std::string correlation_id_;

    std::string level_to_string(Level level);
    std::string component_to_string(Component component);
    std::string event_type_to_string(EventType event_type);
    std::string generate_correlation_id();
    std::string get_iso_timestamp();
};

} // namespace ros_template_node