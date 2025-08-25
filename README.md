# ROS2 Jazzy Listener

A minimal ROS2 C++ subscriber application that demonstrates basic message subscription functionality with make87 platform integration. This application listens for and logs `std_msgs/msg/String` messages from a configurable ROS2 topic.

## What This Application Demonstrates

This simple listener application showcases several key concepts and technologies:

### ROS2 Fundamentals
- **Subscriber Node**: Creates a ROS2 node that subscribes to `std_msgs/msg/String` messages
- **Message Callback Handling**: Uses callback functions to process received messages
- **Topic Communication**: Demonstrates basic ROS2 publish/subscribe communication pattern
- **Node Lifecycle**: Shows proper ROS2 node initialization and shutdown procedures

### make87 Platform Integration
- **Dynamic Topic Configuration**: Reads topic names from the `MAKE87_CONFIG` environment variable
- **JSON Configuration Parsing**: Uses nlohmann/json library to parse platform configuration
- **Topic Name Sanitization**: Automatically converts topic names to ROS2-compatible format
- **Subscriber Configuration**: Reads subscriber configuration from the platform's interface specification

### Containerization & Deployment
- **Multi-Stage Docker Build**: Optimized Dockerfile with separate build and runtime stages
- **Zenoh Networking**: Configured for Zenoh-based ROS2 communication middleware
- **Production Ready**: Includes proper entrypoint scripts and environment configuration
- **Development Support**: Includes development container configuration with SSH access

## Key Features

- 📡 **Configurable Subscription**: Topic name determined by make87 platform configuration
- 🔄 **Robust Configuration**: Automatic fallback to default topic if configuration is missing
- 🐳 **Container Ready**: Fully containerized with Docker support
- 🌐 **Network Optimized**: Uses Zenoh middleware for efficient communication
- 🛠️ **Development Friendly**: Includes development container with git integration
- 📝 **Comprehensive Logging**: Detailed logging for debugging and monitoring
- 🎯 **Reliable Message Handling**: Queue depth of 10 for message buffering

### Topic Name Resolution

1. **With Configuration**: If `MAKE87_CONFIG` is provided and contains a valid subscriber configuration, the application will:
   - Navigate to `interfaces.ros.subscribers.chatter.topic_key` in the JSON configuration
   - Extract the topic key
   - Replace dashes with underscores for ROS2 compatibility
   - Prefix with "make87_" (e.g., "my-topic" becomes "make87_my_topic")

2. **Default Fallback**: If no configuration is provided or parsing fails, the application uses "make87_chatter" as the default topic

## Message Format

The application subscribes to `std_msgs/msg/String` messages and logs the received content. Example message format:

```
data: "Hello, World! 🤖"
```

When a message is received, it will be logged as:
```
[INFO] [make87_listener]: Received: Hello, World! 🤖
```

## Networking

This application is configured to use:
- **ROS Domain ID**: 87 (default)
- **Middleware**: Zenoh (`rmw_zenoh_cpp`)
- **Default Port**: 7447 for Zenoh communication
- **Queue Depth**: 10 messages for reliable subscription

## Development Environment

The development container includes:
- SSH server for remote development
- Git integration with automatic repository management

## Technical Details

- **Language**: C++17
- **ROS2 Distribution**: Jazzy
- **Build System**: CMake with ament
- **Dependencies**: rclcpp, std_msgs, nlohmann/json
- **Container Base**: ros:jazzy-ros-core
- **Node Name**: make87_listener
- **QoS Profile**: Default (queue depth: 10)

## Use Cases

This application serves as:
- **Learning Tool**: Introduction to ROS2 subscription concepts
- **Integration Example**: Template for make87 platform integration
- **Testing Component**: Simple subscriber for testing ROS2 communication setups
- **Microservice Building Block**: Foundation for larger distributed robotics systems
- **Data Consumer**: Receives and processes string messages from publisher nodes

## Companion Applications

This listener application is designed to work with:
- **ROS2 Jazzy Talker**: The corresponding publisher application that sends messages
- **Any ROS2 Publisher**: That publishes `std_msgs/msg/String` messages to the configured topic

## Error Handling

The application includes robust error handling for:
- Missing or malformed `MAKE87_CONFIG` environment variable
- JSON parsing errors
- Missing configuration keys
- Topic name sanitization

All errors are logged with appropriate severity levels (INFO, WARN, ERROR) for easy debugging.

## License

Apache-2.0 - See LICENSE file for details.

## Maintainer

make87 <nisse@make87.com>
