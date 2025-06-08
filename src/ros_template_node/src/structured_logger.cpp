#include "ros_template_node/structured_logger.hpp"
#include <iomanip>
#include <sstream>
#include <random>

namespace ros_template_node {

StructuredLogger::StructuredLogger(const std::string& node_name) 
    : node_name_(node_name), correlation_id_(generate_correlation_id()) {
    
    // Create console logger with JSON format
    logger_ = spdlog::stdout_color_mt(node_name + "_structured");
    logger_->set_pattern("%v");  // Only print the message (our JSON)
    logger_->set_level(spdlog::level::debug);
}

void StructuredLogger::log(Level level, Component component, EventType event_type,
                          const std::string& message, const nlohmann::json& context) {
    nlohmann::json log_entry;
    
    log_entry["timestamp"] = get_iso_timestamp();
    log_entry["level"] = level_to_string(level);
    log_entry["node_name"] = node_name_;
    log_entry["component"] = component_to_string(component);
    log_entry["event_type"] = event_type_to_string(event_type);
    log_entry["message"] = message;
    log_entry["context"] = context;
    log_entry["correlation_id"] = correlation_id_;

    // Convert to string and log
    std::string json_str = log_entry.dump();
    
    switch (level) {
        case Level::DEBUG:
            logger_->debug(json_str);
            break;
        case Level::INFO:
            logger_->info(json_str);
            break;
        case Level::WARN:
            logger_->warn(json_str);
            break;
        case Level::ERROR:
            logger_->error(json_str);
            break;
        case Level::FATAL:
            logger_->critical(json_str);
            break;
    }
}

void StructuredLogger::debug(Component component, EventType event_type,
                            const std::string& message, const nlohmann::json& context) {
    log(Level::DEBUG, component, event_type, message, context);
}

void StructuredLogger::info(Component component, EventType event_type,
                           const std::string& message, const nlohmann::json& context) {
    log(Level::INFO, component, event_type, message, context);
}

void StructuredLogger::warn(Component component, EventType event_type,
                           const std::string& message, const nlohmann::json& context) {
    log(Level::WARN, component, event_type, message, context);
}

void StructuredLogger::error(Component component, EventType event_type,
                            const std::string& message, const nlohmann::json& context) {
    log(Level::ERROR, component, event_type, message, context);
}

std::string StructuredLogger::level_to_string(Level level) {
    switch (level) {
        case Level::DEBUG: return "DEBUG";
        case Level::INFO: return "INFO";
        case Level::WARN: return "WARN";
        case Level::ERROR: return "ERROR";
        case Level::FATAL: return "FATAL";
        default: return "UNKNOWN";
    }
}

std::string StructuredLogger::component_to_string(Component component) {
    switch (component) {
        case Component::STARTUP: return "startup";
        case Component::TIMER: return "timer";
        case Component::SUBSCRIBER: return "subscriber";
        case Component::HEALTH: return "health";
        case Component::SHUTDOWN: return "shutdown";
        default: return "unknown";
    }
}

std::string StructuredLogger::event_type_to_string(EventType event_type) {
    switch (event_type) {
        case EventType::PUBLISH: return "publish";
        case EventType::RECEIVE: return "receive";
        case EventType::HEALTH_CHECK: return "health_check";
        case EventType::INITIALIZATION: return "initialization";
        case EventType::PARAMETER_UPDATE: return "parameter_update";
        case EventType::ERROR_OCCURRED: return "error";
        default: return "unknown";
    }
}

std::string StructuredLogger::generate_correlation_id() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 15);
    
    std::stringstream ss;
    for (int i = 0; i < 8; ++i) {
        ss << std::hex << dis(gen);
    }
    return ss.str();
}

std::string StructuredLogger::get_iso_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::gmtime(&time_t), "%Y-%m-%dT%H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count() << 'Z';
    return ss.str();
}

} // namespace ros_template_node