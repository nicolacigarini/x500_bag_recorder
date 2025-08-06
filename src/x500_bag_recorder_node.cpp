#include "../include/x500_bag_recorder/x500_bag_recorder_node.hpp"

x500_bag_recorder_node::x500_bag_recorder_node(): Node("x500_bag_recorder_node",
                                                        rclcpp::NodeOptions()
                                                    .allow_undeclared_parameters(true)
                                                    .automatically_declare_parameters_from_overrides(true))
{
    loadParams();
    setupConnections();
    _flags.saveToBag = -1;
    _flags.reduceOutliers = -1;
    _flags.allowRestart = true;
    startRecording();
}

void x500_bag_recorder_node::startRecording() {
    RCLCPP_INFO(this->get_logger(), "before opening");

    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d-%H%M%S", &tm);
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = this->get_parameter("file_path").as_string() + "/bag_" + std::string(buffer);
    storage_options.storage_id = "sqlite3";
    _bagWriter.open(storage_options);   
    RCLCPP_INFO(this->get_logger(), "after opening");
    std::cout << "Local time: " << std::put_time(&tm, "%c %Z") << '\n';


}

void x500_bag_recorder_node::stopRecording() {
    _bagWriter.close();
    
}
void x500_bag_recorder_node::loadParams() {
    _topics.lidarTopic = this->get_parameter("lidar_subscriber_topic").as_string();
    _topics.imuLidarTopic = this->get_parameter("imu_lidar_subscriber_topic").as_string();
    _topics.vehicleOdometryTopic = this->get_parameter("vehicle_odom_subscriber_topic").as_string();
    _topics.timesyncTopic = this->get_parameter("timesync_status_topic").as_string();
    _topics.rcChannelsTopic = this->get_parameter("rc_channels_topic").as_string();
    _namespace = this->get_namespace();
}

void x500_bag_recorder_node::setupConnections() {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);



    _lidarSubscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
         _topics.lidarTopic, 10, std::bind(&x500_bag_recorder_node::lidarCallback, this, std::placeholders::_1)); 
    _lidarImuSubscriber = this->create_subscription<sensor_msgs::msg::Imu>(
         _topics.imuLidarTopic, 10, std::bind(&x500_bag_recorder_node::imuLidarCallback, this, std::placeholders::_1)); 

    _rcSubscriber = this->create_subscription<px4_msgs::msg::RcChannels>(
        _topics.rcChannelsTopic, qos, std::bind(&x500_bag_recorder_node::rcCallback, this, std::placeholders::_1));   
    _vehicleOdomSubscriber = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        _topics.vehicleOdometryTopic, qos, std::bind(&x500_bag_recorder_node::vehicleOdomCallback, this, std::placeholders::_1));   
    _timesyncSubscriber = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
        _topics.timesyncTopic, qos, std::bind(&x500_bag_recorder_node::timesyncCallback, this, std::placeholders::_1));   
}

void x500_bag_recorder_node::lidarCallback(std::shared_ptr<rclcpp::SerializedMessage> msg){
    rclcpp::Time time_stamp = this->now();
    if(_flags.saveToBag == 1)
        _bagWriter.write(msg, _namespace + "/" + _topics.lidarTopic, "sensor_msgs/msg/PointCloud2", time_stamp);
}

void x500_bag_recorder_node::imuLidarCallback(std::shared_ptr<rclcpp::SerializedMessage> msg){
    rclcpp::Time time_stamp = this->now();
    if(_flags.saveToBag == 1)
        _bagWriter.write(msg, _namespace + "/" + _topics.imuLidarTopic, "sensor_msgs/msg/Imu", time_stamp);
}

void x500_bag_recorder_node::vehicleOdomCallback(std::shared_ptr<rclcpp::SerializedMessage> msg){
    rclcpp::Time time_stamp = this->now();
    if(_flags.saveToBag == 1)
        _bagWriter.write(msg, _namespace + "/" + _topics.vehicleOdometryTopic, "px4_msgs/msg/VehicleOdometry", time_stamp);
}

void x500_bag_recorder_node::timesyncCallback(std::shared_ptr<rclcpp::SerializedMessage> msg){
    rclcpp::Time time_stamp = this->now();
    if(_flags.saveToBag == 1)
        _bagWriter.write(msg, _namespace + "/" + _topics.timesyncTopic, "px4_msgs/msg/TimesyncStatus", time_stamp);
}

void x500_bag_recorder_node::rcCallback(const px4_msgs::msg::RcChannels &msg){
    _flags.saveToBag = msg.channels[6];
    _flags.reduceOutliers = msg.channels[7];

    if(msg.channels[11]==1 && _flags.allowRestart){
            RCLCPP_INFO(this->get_logger(), "inside if");
        _flags.allowRestart=false;
        stopRecording();
        startRecording();
    }
    if(msg.channels[11]==-1)
        _flags.allowRestart=true;
}


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<x500_bag_recorder_node>());
    rclcpp::shutdown();
    return 0;
}