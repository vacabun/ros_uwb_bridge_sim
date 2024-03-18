#include "rclcpp/rclcpp.hpp"
#include <string>
#include <cmath>
#include <random>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include <gz/msgs/odometry.pb.h>
#include <gz/transport/Node.hh>
#include "uwb_interfaces/srv/uwb_measure.hpp"
#include "uwb_interfaces/srv/get_position.hpp"
#include "uwb_interfaces/msg/uwb_distance.hpp"
#include "tinyxml2.hpp"
#include "bridge.hpp"

int main(int argc, const char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UWBRosBridge>());
    rclcpp::shutdown();

    return 0;
}

UWBRosBridge::UWBRosBridge() : Node("uwb_ros_bridge")
{
    config();
    load_config();

}
void UWBRosBridge::config()
{
    this->declare_parameter("label_name", "x500_0");

    labelName =
        this->get_parameter("label_name").get_parameter_value().get<std::string>();
    RCLCPP_INFO(this->get_logger(), "label_name: %s", labelName.c_str());
    std::string subscribeTopic = "/model/" + labelName + "/odometry";

    RCLCPP_INFO(this->get_logger(), "subscribe topic : %s", subscribeTopic.c_str());
    if (!subscribeNode.Subscribe(subscribeTopic, &UWBRosBridge::gz_odometry_topic_callback, this))
    {
        RCLCPP_ERROR(this->get_logger(), "Error subscribing to topic [%s].", subscribeTopic.c_str());
    }

    measure_service_ = this->create_service<uwb_interfaces::srv::UWBMeasure>("/" + labelName + "/uwb_bridge", std::bind(&UWBRosBridge::measure_handle_service, this, std::placeholders::_1, std::placeholders::_2));
    get_position_service_ = this->create_service<uwb_interfaces::srv::GetPosition>("/" + labelName + "/gz_position", std::bind(&UWBRosBridge::get_position_handle_service, this, std::placeholders::_1, std::placeholders::_2));
}

void UWBRosBridge::gz_odometry_topic_callback(const gz::msgs::Odometry &_msg)
{
    label_pose.x = _msg.pose().position().x();
    label_pose.y = _msg.pose().position().y();
    label_pose.z = _msg.pose().position().z();
    // RCLCPP_INFO(this->get_logger(), "position: %f %f %f", x, y, z);
}

void UWBRosBridge::load_config()
{
    std::string packageShareDirectory = ament_index_cpp::get_package_share_directory("uwb_ros_bridge_sim");

    std::string anchorConfigFilePath = packageShareDirectory + "/config/anchor.xml";

    tinyxml2::XMLDocument anchorDoc;

    if (anchorDoc.LoadFile(anchorConfigFilePath.c_str()) != 0)
    {
        RCLCPP_INFO(this->get_logger(), "load anchor config file failed.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "load anchor config file successed.");
    }

    tinyxml2::XMLElement *anchor = anchorDoc.RootElement()->FirstChildElement("anchor");

    while (anchor)
    {
        int id = atoi(anchor->FirstAttribute()->Value());

        tinyxml2::XMLElement *attr = anchor->FirstChildElement();

        geometry_msgs::msg::Point p;

        p.set__x(std::stod(attr->GetText()));
        attr = attr->NextSiblingElement();
        p.set__y(std::stod(attr->GetText()));
        attr = attr->NextSiblingElement();
        p.set__z(std::stod(attr->GetText()));

        RCLCPP_INFO(this->get_logger(), "load anchor id: %d position:%f %f %f", id, p.x, p.y, p.z);
        anchorPoseMap[id] = p;
        anchor = anchor->NextSiblingElement();
    }
}

void UWBRosBridge::measure_handle_service(const std::shared_ptr<uwb_interfaces::srv::UWBMeasure::Request> request,
                                          std::shared_ptr<uwb_interfaces::srv::UWBMeasure::Response> response)
{   
    int srcAddr = request->src_address;
    int destAddr = request->dest_address;
    auto it = anchorPoseMap.find(destAddr);
    // RCLCPP_INFO(this->get_logger(), "destAddr: %d", destAddr);
    double distance = 0;
    if (it != anchorPoseMap.end())
    {
        double anchor_x = anchorPoseMap[destAddr].x;
        double anchor_y = anchorPoseMap[destAddr].y;
        double anchor_z = anchorPoseMap[destAddr].z;

        distance = sqrt(pow(label_pose.x - anchor_x, 2) + pow(label_pose.y - anchor_y, 2) + pow(label_pose.z - anchor_z, 2));
        std::normal_distribution<double> distribution_normal(0., 0.1);
        distance = distance + distribution_normal(tandomGenerator);
        // RCLCPP_INFO(this->get_logger(), "destAddr: %d, distance: %lf", destAddr, distance);
    }
    else
    {
        geometry_msgs::msg::Point p = callServiceGetPosition(destAddr);

        distance = sqrt(pow(label_pose.x - p.x, 2) + pow(label_pose.y - p.y, 2) + pow(label_pose.z - p.z, 2));
        // RCLCPP_INFO(this->get_logger(), "destAddr: %d, distance: %f", destAddr, distance);
    }
    uwb_interfaces::msg::UWBDistance uwb_distance;
    uwb_distance.src = srcAddr;
    uwb_distance.dest = destAddr;
    uwb_distance.distance = distance;
    response->uwb_distance = uwb_distance;

}

void UWBRosBridge::get_position_handle_service(const std::shared_ptr<uwb_interfaces::srv::GetPosition::Request> request,
                                               std::shared_ptr<uwb_interfaces::srv::GetPosition::Response> response)
{
    response->position = label_pose;
}
void UWBRosBridge::change_get_position_client_service_name(const std::string &service_name)
{
    // 销毁当前的服务客户端
    get_position_client_.reset();
    // 根据新的服务名称创建一个新的服务客户端
    get_position_client_ = this->create_client<uwb_interfaces::srv::GetPosition>(service_name);
}

geometry_msgs::msg::Point UWBRosBridge::callServiceGetPosition(int id)
{

    auto request = std::make_shared<uwb_interfaces::srv::GetPosition::Request>();
    geometry_msgs::msg::Point res;
    this->service_call_completed = false;
    change_get_position_client_service_name("/x500_" + std::to_string(id) + "/gz_position");
    auto future = get_position_client_->async_send_request(request, std::bind(&UWBRosBridge::get_position_handle_response, this, std::placeholders::_1));

    int timeout = 0;
    while (!this->service_call_completed)
    {
        if (timeout++ >= 30)
        {
            break;
        }
        usleep(1000);
    }
    if (timeout >= 1000)
    {
        RCLCPP_ERROR(this->get_logger(), "no response, timeout.");
    }
    else
    {
        res = this->service_get_position;
    }
    return res;
}
void UWBRosBridge::get_position_handle_response(rclcpp::Client<uwb_interfaces::srv::GetPosition>::SharedFuture future)
{
    auto response = future.get();

    this->service_get_position = response->position;
    this->service_call_completed = true;
}