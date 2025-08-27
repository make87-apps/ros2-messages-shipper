#include <memory>
#include <string>
#include <cstdlib>
#include <algorithm>
#include <regex>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <openssl/sha.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <nlohmann/json.hpp>
#include <rerun.hpp>

struct TopicConfig {
    std::string topic_name;
    std::string message_type;
};

struct RerunConfig {
    std::string vpn_ip;
    std::string vpn_port;
    std::string system_id;
    std::string application_name;
};

std::string deterministic_uuid_from_string(const std::string& s) {
    // Generate SHA256 hash
    unsigned char hash[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char*>(s.c_str()), s.length(), hash);

    // Take first 16 bytes for UUID
    unsigned char uuid_bytes[16];
    std::memcpy(uuid_bytes, hash, 16);

    // Set version 4 and variant bits
    uuid_bytes[6] = (uuid_bytes[6] & 0x0F) | 0x40; // Version 4
    uuid_bytes[8] = (uuid_bytes[8] & 0x3F) | 0x80; // Variant RFC 4122

    // Format as UUID string
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (int i = 0; i < 16; i++) {
        if (i == 4 || i == 6 || i == 8 || i == 10) ss << "-";
        ss << std::setw(2) << static_cast<int>(uuid_bytes[i]);
    }

    return ss.str();
}

RerunConfig get_rerun_config() {
    const char* config_env = std::getenv("MAKE87_CONFIG");
    const std::string default_ip = "127.0.0.1";
    const std::string default_port = "9876";
    const std::string default_system_id = "make87_ros2_shipper";
    const std::string default_application_name = "default_app";

    if (!config_env) {
        RCLCPP_INFO(rclcpp::get_logger("make87_listener"), "MAKE87_CONFIG not found, using defaults");
        return {default_ip, default_port, default_system_id, default_application_name};
    }

    try {
        auto config = nlohmann::json::parse(config_env);

        std::string vpn_ip = default_ip;
        std::string vpn_port = default_port;
        std::string system_id = default_system_id;
        std::string application_name = default_application_name;

        // Extract from application_info
        if (config.contains("application_info")) {
            if (config["application_info"].contains("system_id")) {
                system_id = config["application_info"]["system_id"];
            }
            if (config["application_info"].contains("application_name")) {
                application_name = config["application_info"]["application_name"];
            }
        }

        // Navigate to interfaces.rerun-grpc client service config
        if (config.contains("interfaces") &&
            config["interfaces"].contains("rerun-grpc") &&
            config["interfaces"]["rerun-grpc"].contains("clients") &&
            config["interfaces"]["rerun-grpc"]["clients"].contains("rerun-grpc-client")) {

            auto client_config = config["interfaces"]["rerun-grpc"]["clients"]["rerun-grpc-client"];

            if (client_config.contains("vpn_ip")) {
                vpn_ip = client_config["vpn_ip"];
            }

            if (client_config.contains("vpn_port")) {
                vpn_port = std::to_string(static_cast<int>(client_config["vpn_port"]));
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("make87_listener"), "Using rerun config - ip: %s, port: %s, system_id: %s, application_name: %s",
                    vpn_ip.c_str(), vpn_port.c_str(), system_id.c_str(), application_name.c_str());
        return {vpn_ip, vpn_port, system_id, application_name};

    } catch (const nlohmann::json::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("make87_listener"), "Error parsing MAKE87_CONFIG JSON: %s, using defaults", e.what());
        return {default_ip, default_port, default_system_id, default_application_name};
    }
}

struct HeaderInfo {
    std::string entity_path;
    double timestamp_secs;
};

HeaderInfo process_header(const std_msgs::msg::Header& header) {
    // Convert ROS timestamp to seconds since epoch
    double timestamp_secs = header.stamp.sec + (header.stamp.nanosec * 1e-9);

    // Create entity path from frame_id
    std::string entity_path = header.frame_id.empty() ? "/default" : "/" + header.frame_id;

    // Ensure entity path starts with /
    if (!entity_path.empty() && entity_path[0] != '/') {
        entity_path = "/" + entity_path;
    }

    return {entity_path, timestamp_secs};
}

std::string parse_message_type_from_topic_key(const std::string& topic_key) {
    // Parse format like "ros2-jazzy-talker-52551107110183/ros-chatter/ros2-std_msgs-msg-string"
    // or "ros2-jazzy-talker-52551107110183/ros-image/ros2-sensor_msgs-msg-compressedimage"
    // Extract the last part after the final slash, then parse the message type
    std::regex pattern(R"(.*/ros2-([^-]+(?:_[^-]+)*)-([^-]+)-([^-]+)$)");
    std::smatch matches;

    if (std::regex_match(topic_key, matches, pattern)) {
        if (matches.size() == 4) {
            // matches[1] = package (e.g., "std_msgs" or "sensor_msgs")
            // matches[2] = "msg"
            // matches[3] = type (e.g., "string" or "compressedimage")
            std::string package = matches[1].str();
            std::string msg_part = matches[2].str();
            std::string type_name = matches[3].str();

            // Handle special case mappings
            if (package == "sensor_msgs" && type_name == "compressedimage") {
                type_name = "CompressedImage";
            } else if (package == "sensor_msgs" && type_name == "image") {
                type_name = "Image";
            } else if (package == "std_msgs" && type_name == "string") {
                type_name = "String";
            }

            std::string message_type = package + "/" + msg_part + "/" + type_name;
            RCLCPP_INFO(rclcpp::get_logger("make87_listener"), "Parsed message type from topic_key: %s -> %s",
                        topic_key.c_str(), message_type.c_str());
            return message_type;
        }
    }

    // Default fallback
    RCLCPP_WARN(rclcpp::get_logger("make87_listener"), "Could not parse message type from topic_key: %s, using default std_msgs/msg/String", topic_key.c_str());
    return "std_msgs/msg/String";
}

TopicConfig get_topic_config() {
    const char* config_env = std::getenv("MAKE87_CONFIG");
    const std::string default_topic = "chatter";
    const std::string default_type = "std_msgs/msg/String";

    if (!config_env) {
        RCLCPP_INFO(rclcpp::get_logger("make87_listener"), "MAKE87_CONFIG not found, using default topic: %s with type: %s", default_topic.c_str(), default_type.c_str());
        return {"make87_" + default_topic, default_type};
    }

    try {
        auto config = nlohmann::json::parse(config_env);

        // Navigate to interfaces.ros.subscribers.any_message.topic_key
        if (config.contains("interfaces") &&
            config["interfaces"].contains("ros") &&
            config["interfaces"]["ros"].contains("subscribers") &&
            config["interfaces"]["ros"]["subscribers"].contains("any_message") &&
            config["interfaces"]["ros"]["subscribers"]["any_message"].contains("topic_key")) {

            std::string topic_key = config["interfaces"]["ros"]["subscribers"]["any_message"]["topic_key"];

            // Add make87_ prefix and sanitize hyphens to underscores
            std::string sanitized_topic = "make87_" + topic_key;
            std::replace(sanitized_topic.begin(), sanitized_topic.end(), '-', '_');

            // Parse message type from topic_key
            std::string message_type = parse_message_type_from_topic_key(topic_key);

            RCLCPP_INFO(rclcpp::get_logger("make87_listener"), "Extracted topic_key from MAKE87_CONFIG: %s, sanitized: %s, message_type: %s",
                        topic_key.c_str(), sanitized_topic.c_str(), message_type.c_str());
            return {sanitized_topic, message_type};
        } else {
            RCLCPP_WARN(rclcpp::get_logger("make87_listener"), "Could not find chatter subscriber topic_key in MAKE87_CONFIG, using default: %s with type: %s", default_topic.c_str(), default_type.c_str());
            return {"make87_" + default_topic, default_type};
        }
    } catch (const nlohmann::json::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("make87_listener"), "Error parsing MAKE87_CONFIG JSON: %s, using default topic: %s with type: %s", e.what(), default_topic.c_str(), default_type.c_str());
        return {"make87_" + default_topic, default_type};
    }
}

class Listener : public rclcpp::Node {
public:
  Listener() : rclcpp::Node("make87_listener") {
    // Initialize rerun recording stream following Rust implementation
    rerun_config_ = get_rerun_config();

    // Create deterministic recording ID from system_id
    std::string recording_id = deterministic_uuid_from_string(rerun_config_.system_id);

    RCLCPP_INFO(this->get_logger(), "Using recording id: %s", recording_id.c_str());

    // Create recording stream with system_id and deterministic recording_id
    rec_ = std::make_shared<rerun::RecordingStream>(rerun_config_.system_id, recording_id);
    rec_->set_global();

    // Format connection URL: rerun+http://ip:port/proxy
    std::string connection_url = "rerun+http://" + rerun_config_.vpn_ip + ":" + rerun_config_.vpn_port + "/proxy";

    // Connect with options (using default flush timeout)
    auto connect_result = rec_->connect_grpc(connection_url, 2.0);
    if (connect_result.is_err()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to rerun server at: %s", connection_url.c_str());
      throw std::runtime_error("Failed to connect to rerun server");
    }

    RCLCPP_INFO(this->get_logger(), "Connected to rerun server at: %s with recording_id: %s",
                connection_url.c_str(), recording_id.c_str());

    // Setup ROS2 subscriber
    TopicConfig config = get_topic_config();
    RCLCPP_INFO(this->get_logger(), "Message type detected: %s", config.message_type.c_str());

    if (config.message_type == "std_msgs/msg/String") {
      setup_string_subscriber(config.topic_name);
    } else if (config.message_type == "sensor_msgs/msg/CompressedImage") {
      setup_compressed_image_subscriber(config.topic_name);
    } else if (config.message_type == "sensor_msgs/msg/Image") {
      setup_image_subscriber(config.topic_name);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unsupported message type: %s", config.message_type.c_str());
      throw std::runtime_error("Unsupported message type: " + config.message_type);
    }

    RCLCPP_INFO(this->get_logger(), "make87_listener started; listening to '%s' with type '%s'",
                config.topic_name.c_str(), config.message_type.c_str());
  }

private:
  void setup_string_subscriber(const std::string& topic_name) {
    string_sub_ = this->create_subscription<std_msgs::msg::String>(
      topic_name, 10, [this, topic_name](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Received std_msgs/msg/String: %s", msg->data.c_str());

        // Log to rerun using message package nesting as entity path with application name suffix
        std::string entity_path = "/std_msgs/msg/String/" + rerun_config_.application_name;
        rec_->log(entity_path, rerun::TextDocument(msg->data));
      });
  }

  void setup_compressed_image_subscriber(const std::string& topic_name) {
    image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      topic_name, 10, [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Received sensor_msgs/msg/CompressedImage: format=%s, size=%zu bytes",
                    msg->format.c_str(), msg->data.size());

        // Process header if present
        HeaderInfo header_info = process_header(msg->header);

        // Set timestamp in rerun as seconds since epoch
        rec_->set_time_timestamp_secs_since_epoch("header_time", header_info.timestamp_secs);

        // Determine media type from format
        std::string media_type = "image/jpeg"; // Default
        if (msg->format.find("png") != std::string::npos) {
          media_type = "image/png";
        }

        // Log to rerun using message package nesting with frame_id suffix
        std::string entity_path = "/sensor_msgs/msg/CompressedImage";
        if (!msg->header.frame_id.empty() && msg->header.frame_id != "/") {
          entity_path += "/" + msg->header.frame_id;
        }

        auto encoded_image = rerun::EncodedImage::from_bytes(msg->data).with_media_type(rerun::MediaType(media_type));
        rec_->log(entity_path, encoded_image);

        RCLCPP_DEBUG(this->get_logger(), "Logged image to entity: %s with timestamp: %.3f",
                     entity_path.c_str(), header_info.timestamp_secs);
      });
  }

  void setup_image_subscriber(const std::string& topic_name) {
    raw_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_name, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Received sensor_msgs/msg/Image: encoding=%s, width=%d, height=%d",
                    msg->encoding.c_str(), msg->width, msg->height);

        // Only process RGB8 encoding
        if (msg->encoding != "rgb8") {
          RCLCPP_WARN(this->get_logger(), "Unsupported encoding: %s. Only rgb8 is supported.", msg->encoding.c_str());
          return;
        }

        // Process header if present
        HeaderInfo header_info = process_header(msg->header);

        // Set timestamp in rerun as seconds since epoch
        rec_->set_time_timestamp_secs_since_epoch("header_time", header_info.timestamp_secs);

        // Log to rerun using message package nesting with frame_id suffix
        std::string entity_path = "/sensor_msgs/msg/Image";
        if (!msg->header.frame_id.empty() && msg->header.frame_id != "/") {
          entity_path += "/" + msg->header.frame_id;
        }

        // Create RGB image from raw data
        auto image = rerun::Image::from_rgb24(msg->data, {msg->width, msg->height});
        rec_->log(entity_path, image);

        RCLCPP_DEBUG(this->get_logger(), "Logged RGB8 image to entity: %s with timestamp: %.3f, size: %dx%d",
                     entity_path.c_str(), header_info.timestamp_secs, msg->width, msg->height);
      });
  }

  std::shared_ptr<rerun::RecordingStream> rec_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_sub_;
  RerunConfig rerun_config_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}
