---
name: ROS2 Bag Utility
description: ROS2 bag recording and analysis utilities with Clean Architecture (Python & C++)
---

# ROS2 Bag Utility Skill

This skill provides guides for programmatic data recording (rosbag2) and replay mechanism adhering to Clean Architecture.

## Domain Layer

Defines the interface for recording and playback control.

```python
# domain/interfaces/data_recorder.py
class IDataRecorder(ABC):
    @abstractmethod
    def start_recording(self, config: RecordingConfig) -> bool:
        pass

    @abstractmethod
    def stop_recording(self) -> None:
        pass
```

## Infrastructure Layer (Python)

See previous Python example using `rosbag2_py`.

## Infrastructure Layer (C++)

Using `rosbag2_cpp` library.

```cpp
// infrastructure/ros2/bag/bag_recorder.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

namespace infrastructure::ros2::bag {

class BagRecorder {
public:
    explicit BagRecorder(rclcpp::Node::SharedPtr node) : node_(node) {}

    bool start_recording(const std::string& bag_name, const std::vector<std::string>& topics) {
        try {
            writer_ = std::make_unique<rosbag2_cpp::Writer>();

            rosbag2_storage::StorageOptions storage_options;
            storage_options.uri = bag_name;
            storage_options.storage_id = "sqlite3";

            rosbag2_cpp::ConverterOptions converter_options;
            converter_options.input_serialization_format = "cdr";
            converter_options.output_serialization_format = "cdr";

            writer_->open(storage_options, converter_options);

            // Subscribe to topics and write
            for (const auto& topic : topics) {
                // Determine topic type dynamically or use GenericSubscribe (Humble+)
                create_recorder_subscription(topic);
            }

            is_recording_ = true;
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open bag: %s", e.what());
            return false;
        }
    }

    void stop_recording() {
        writer_.reset(); // Closes the bag
        subs_.clear();
        is_recording_ = false;
    }

    template<typename T>
    void write_message(const std::string& topic, const T& msg) {
        if (is_recording_ && writer_) {
            auto timestamp = node_->get_clock()->now();
            writer_->write(msg, topic, timestamp);
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
    bool is_recording_ = false;

    void create_recorder_subscription(const std::string& topic) {
        // Implementation for generic subscription...
    }
};

} // namespace
```

### Bag Reader Helper (C++)

```cpp
// infrastructure/ros2/bag/bag_reader.hpp
#include <rosbag2_cpp/reader.hpp>

class BagReader {
public:
    explicit BagReader(const std::string& bag_path) {
        reader_ = std::make_unique<rosbag2_cpp::Reader>();
        reader_->open(bag_path);
    }

    void read_all() {
        while (reader_->has_next()) {
            auto bag_message = reader_->read_next();
            // Deserialization logic...
        }
    }

private:
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
};
```

## Best Practices

1. **Storage ID**: `sqlite3` is default, `mcap` is recommended for performance.
2. **Serialization**: Use `cdr`.
3. **Filtering**: Record only necessary topics to save disk space.
4. **Splitting**: Configure bag splitting for long-duration recordings.
