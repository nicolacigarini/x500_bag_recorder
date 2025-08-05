#ifndef X500_BAG_RECORDER_NODE
#define X500_BAG_RECORDER_NODE
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/rc_channels.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "ctime"

class x500_bag_recorder_node : public rclcpp::Node {
    struct Flags {
        int saveToBag;
        int reduceOutliers;
    };

    struct Topics {
        std::string lidarTopic;
        std::string imuLidarTopic;
        std::string rcChannelsTopic;
        std::string vehicleOdometryTopic;
    };

    public:
        x500_bag_recorder_node();
        void loadParams();
        void setupConnections();
        void lidarCallback(std::shared_ptr<rclcpp::SerializedMessage>);
        void vehicleOdomCallback(std::shared_ptr<rclcpp::SerializedMessage>);
        void imuLidarCallback(std::shared_ptr<rclcpp::SerializedMessage>);
        void rcCallback(const px4_msgs::msg::RcChannels &msg);

    private:
        Flags _flags;
        Topics _topics;
        std::string _namespace;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _lidarSubscriber;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _lidarImuSubscriber;
        rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr _rcSubscriber;
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _vehicleOdomSubscriber;
        std::unique_ptr<rosbag2_cpp::Writer> _bagWriter;
};


#endif