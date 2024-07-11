#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "px4_msgs/msg/obstacle_distance.hpp"

class LaserScanToObstacleDistanceNode : public rclcpp::Node
{
public:
    LaserScanToObstacleDistanceNode()
        : Node("laserscan_to_obstacle_distance_node")
    {
        // auto qos = rclcpp::QoS(10);  // History depth is 10
        // qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        // publisher_ = this->create_publisher<px4_msgs::msg::ObstacleDistance>("/fmu/in/obstacle_distance", qos);

        // QoS profile, PX4 specific
	    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        publisher_ = this->create_publisher<px4_msgs::msg::ObstacleDistance>("/fmu/in/obstacle_distance", 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&LaserScanToObstacleDistanceNode::listener_callback, this, std::placeholders::_1));
    }

private:
    void listener_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto obstacle_distance = laserscan_to_obstacle_distance(msg);
        publisher_->publish(obstacle_distance);
    }
    
    px4_msgs::msg::ObstacleDistance laserscan_to_obstacle_distance(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        px4_msgs::msg::ObstacleDistance obstacle_distance;
        obstacle_distance.timestamp = scan->header.stamp.sec * 1e6 + scan->header.stamp.nanosec / 1e3;  // Convert to microseconds
        obstacle_distance.frame = 12;     
	    obstacle_distance.sensor_type = 0;
        // obstacle_distance.increment = scan->angle_increment;
        obstacle_distance.increment = 5.0;
        obstacle_distance.min_distance = scan->range_min * 1e2;  // Convert to centimeters
        obstacle_distance.max_distance = scan->range_max * 1e2;  // Convert to centimeters
        obstacle_distance.angle_offset = 0.0;  // Assuming the lidar is mounted in the front
        // obstacle_distance.distances[0] = std::min(std::max(static_cast<int>(scan->ranges[1] * 1e2), static_cast<int>(obstacle_distance.min_distance)), static_cast<int>(obstacle_distance.max_distance));
        for (size_t i = 0; i < scan->ranges.size() && i < obstacle_distance.distances.size(); ++i)
        {
            obstacle_distance.distances[i] = static_cast<int>(scan->ranges[i*5] * 1e2);
            if (obstacle_distance.distances[i]==0)
            {
                obstacle_distance.distances[i] = static_cast<int>(obstacle_distance.max_distance);
            }
            // else
            // {
            //     obstacle_distance.distances[i] = static_cast<int>(scan->ranges[i*5] * 1e2);
            // }
            // obstacle_distance.distances[i] = std::min(std::max(static_cast<int>(scan->ranges[i*5] * 1e2), static_cast<int>(obstacle_distance.min_distance)), static_cast<int>(obstacle_distance.max_distance));  // Convert to centimeters
        }
        // obstacle_distance.distances[41] = std::min(std::max(static_cast<int>(scan->ranges[200] * 1e2), static_cast<int>(obstacle_distance.min_distance)), static_cast<int>(obstacle_distance.max_distance));
        // obstacle_distance.distances[42] = std::min(std::max(static_cast<int>(scan->ranges[205] * 1e2), static_cast<int>(obstacle_distance.min_distance)), static_cast<int>(obstacle_distance.max_distance));

        // Print lengths after the loop for clarity
        std::cout << "Length of obstacle_distance.distances: " << obstacle_distance.distances.size() << std::endl;
        std::cout << "Length of scan->ranges: " << scan->ranges.size() << std::endl;
        return obstacle_distance;
    }

    rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanToObstacleDistanceNode>());
    rclcpp::shutdown();
    return 0;
}

