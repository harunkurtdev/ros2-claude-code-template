---
description: ROS2 Node Development Standards
---

# ROS2 Node Standards

This rule file defines how to create and configure ROS2 nodes.

## Node Base Class

### Python Example

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class BaseRobotNode(Node):
    """Base class for all robot nodes following Clean Architecture."""

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

        # Declare common parameters
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('use_sim_time', False)

        # Get parameters
        self._update_rate = self.get_parameter('update_rate').value

        # Setup logging
        self.get_logger().info(f'{node_name} initialized')

    def create_rate_timer(self, callback):
        """Create timer based on update_rate parameter."""
        period = 1.0 / self._update_rate
        return self.create_timer(period, callback)

    def safe_destroy(self):
        """Safely destroy the node."""
        self.get_logger().info('Shutting down...')
        self.destroy_node()
```

### C++ Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <string>

class BaseRobotNode : public rclcpp::Node {
public:
    BaseRobotNode(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node(node_name, options) {

        // Declare common parameters
        this->declare_parameter("update_rate", 10.0);
        this->declare_parameter("use_sim_time", false);

        update_rate_ = this->get_parameter("update_rate").as_double();

        RCLCPP_INFO(this->get_logger(), "%s initialized", node_name.c_str());
    }

protected:
    rclcpp::TimerBase::SharedPtr create_rate_timer(std::function<void()> callback) {
        auto period = std::chrono::duration<double>(1.0 / update_rate_);
        return this->create_wall_timer(period, callback);
    }

    double update_rate_;
};
```

## Publisher Pattern

### Python Example

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class PublisherMixin:
    """Mixin for standardized publisher creation."""

    def create_reliable_publisher(self, msg_type, topic_name, queue_size=10):
        """Create a reliable publisher for critical data."""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=queue_size
        )
        return self.create_publisher(msg_type, topic_name, qos)

    def create_sensor_publisher(self, msg_type, topic_name, queue_size=10):
        """Create a best-effort publisher for sensor data."""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=queue_size
        )
        return self.create_publisher(msg_type, topic_name, qos)
```

### C++ Example

```cpp
// In your node class or mixin
template<typename MessageT>
typename rclcpp::Publisher<MessageT>::SharedPtr create_reliable_publisher(
    const std::string& topic_name, size_t queue_size = 10) {

    rclcpp::QoS qos(queue_size);
    qos.reliable();
    qos.transient_local();

    return this->create_publisher<MessageT>(topic_name, qos);
}

template<typename MessageT>
typename rclcpp::Publisher<MessageT>::SharedPtr create_sensor_publisher(
    const std::string& topic_name, size_t queue_size = 10) {

    rclcpp::QoS qos(queue_size);
    qos.best_effort();
    qos.durability_volatile();

    return this->create_publisher<MessageT>(topic_name, qos);
}
```

## Subscriber Pattern

### Python Example

```python
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

class SubscriberMixin:
    """Mixin for standardized subscriber creation."""

    def create_sensor_subscriber(self, msg_type, topic_name, callback):
        """Create subscriber for sensor data (best-effort)."""
        return self.create_subscription(
            msg_type,
            topic_name,
            callback,
            qos_profile_sensor_data
        )

    def create_reliable_subscriber(self, msg_type, topic_name, callback):
        """Create subscriber for reliable data."""
        return self.create_subscription(
            msg_type,
            topic_name,
            callback,
            qos_profile_system_default
        )
```

## Service Pattern

### Python Example

```python
from rclpy.service import Service
from example_interfaces.srv import SetBool

class ServiceNode(BaseRobotNode):
    """Example node with service."""

    def __init__(self):
        super().__init__('service_node')

        # Create service
        self._srv = self.create_service(
            SetBool,
            'enable_feature',
            self._enable_callback
        )

    def _enable_callback(self, request, response):
        """Service callback."""
        try:
            # Business logic here
            self._feature_enabled = request.data
            response.success = True
            response.message = f"Feature {'enabled' if request.data else 'disabled'}"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
```

### C++ Example

```cpp
#include <example_interfaces/srv/set_bool.hpp>

class ServiceNode : public BaseRobotNode {
public:
    ServiceNode() : BaseRobotNode("service_node") {
        using namespace std::placeholders;
        srv_ = this->create_service<example_interfaces::srv::SetBool>(
            "enable_feature",
            std::bind(&ServiceNode::enable_callback, this, _1, _2));
    }

private:
    void enable_callback(
        const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
        std::shared_ptr<example_interfaces::srv::SetBool::Response> response) {

        try {
            // Business Logic
            response->success = true;
            response->message = request->data ? "Feature enabled" : "Feature disabled";
        } catch (const std::exception& e) {
            response->success = false;
            response->message = e.what();
        }
    }
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr srv_;
};
```

## Action Server Pattern

### Python Example

See `ros2_nodes.md` for full Python example.

### C++ Example

```cpp
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_tutorials_interfaces/action/fibonacci.hpp>

class ActionServerNode : public BaseRobotNode {
public:
    using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

    ActionServerNode() : BaseRobotNode("action_server") {
        action_server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",
            std::bind(&ActionServerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServerNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionServerNode::handle_accepted, this, std::placeholders::_1)
        );
    }
    // ... callback implementations ...
private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};
```

## Lifecycle Node Pattern

### Python Example

See `ros2_nodes.md` for full Python example.

### C++ Example

```cpp
#include <rclcpp_lifecycle/lifecycle_node.hpp>

class ManagedRobotNode : public rclcpp_lifecycle::LifecycleNode {
public:
    ManagedRobotNode(const std::string& node_name) : LifecycleNode(node_name) {}

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State&) {
        RCLCPP_INFO(get_logger(), "Configuring...");
        // publisher_ = this->create_publisher...
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    // Implement on_activate, on_deactivate, on_cleanup...
};
```
