#ifndef _BRIDGE_HPP_
#define _BRIDGE_HPP_
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <cmath>
#include <random>
#include <regex>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include <gz/msgs/odometry.pb.h>
#include <gz/transport/Node.hh>
#include "uwb_interfaces/srv/uwb_measure.hpp"
#include "uwb_interfaces/srv/get_position.hpp"
#include "uwb_interfaces/msg/uwb_distance.hpp"
#include "uwb_interfaces/msg/gazebo_position.hpp"
#include "tinyxml2.hpp"
class UWBRosBridge : public rclcpp::Node
{
public:
    UWBRosBridge();

private:
    void config();
    void load_config();
    void gz_odometry_topic_callback(const gz::msgs::Odometry &_msg);
    void measure_handle_service(const std::shared_ptr<uwb_interfaces::srv::UWBMeasure::Request> request,
                                std::shared_ptr<uwb_interfaces::srv::UWBMeasure::Response> response);
    void gazebo_position_callback(uwb_interfaces::msg::GazeboPosition::SharedPtr msg);
    int get_id(std::string label_name);
    rclcpp::Subscription<uwb_interfaces::msg::GazeboPosition>::SharedPtr gazeboPositionSubscription_;
    rclcpp::Publisher<uwb_interfaces::msg::GazeboPosition>::SharedPtr gazeboPositionPublisher_;
    std::unordered_map<int, geometry_msgs::msg::Point> anchorPoseMap;
    gz::transport::Node subscribeNode;
    std::string labelName;
    rclcpp::Service<uwb_interfaces::srv::UWBMeasure>::SharedPtr measure_service_;
    rclcpp::Service<uwb_interfaces::srv::GetPosition>::SharedPtr get_position_service_;
    geometry_msgs::msg::Point label_pose;
    std::default_random_engine tandomGenerator;

    std::unordered_map<int, geometry_msgs::msg::Point> label_pose_map;
};
#endif
