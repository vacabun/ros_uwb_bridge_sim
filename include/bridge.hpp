#ifndef _BRIDGE_HPP_
#define _BRIDGE_HPP_

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
    void get_position_handle_service(const std::shared_ptr<uwb_interfaces::srv::GetPosition::Request> request,
                                     std::shared_ptr<uwb_interfaces::srv::GetPosition::Response> response);
    geometry_msgs::msg::Point callServiceGetPosition(int id);
    void get_position_handle_response(rclcpp::Client<uwb_interfaces::srv::GetPosition>::SharedFuture future);
    void change_get_position_client_service_name(const std::string &service_name);
    std::unordered_map<int, geometry_msgs::msg::Point> anchorPoseMap;
    gz::transport::Node subscribeNode;
    std::string labelName;
    rclcpp::Service<uwb_interfaces::srv::UWBMeasure>::SharedPtr measure_service_;
    rclcpp::Service<uwb_interfaces::srv::GetPosition>::SharedPtr get_position_service_;
    geometry_msgs::msg::Point label_pose;
    std::default_random_engine tandomGenerator;
    bool service_call_completed;
    geometry_msgs::msg::Point service_get_position;
    rclcpp::Client<uwb_interfaces::srv::GetPosition>::SharedPtr get_position_client_;
    
};
#endif
