#include "bridge.hpp"
using namespace std::chrono_literals;
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
    labelName = this->get_parameter("label_name").get_parameter_value().get<std::string>();
    RCLCPP_INFO(this->get_logger(), "label_name: %s", labelName.c_str());

    std::string subscribeTopic = "/model/" + labelName + "/odometry";
    RCLCPP_INFO(this->get_logger(), "subscribe topic : %s", subscribeTopic.c_str());
    if (!subscribeNode.Subscribe(subscribeTopic, &UWBRosBridge::gz_odometry_topic_callback, this))
    {
        RCLCPP_ERROR(this->get_logger(), "Error subscribing to topic [%s].", subscribeTopic.c_str());
    }

    measure_service_ = this->create_service<uwb_interfaces::srv::UWBMeasure>("/" + labelName + "/uwb_bridge", std::bind(&UWBRosBridge::measure_handle_service, this, std::placeholders::_1, std::placeholders::_2));

    gazeboPositionPublisher_ = this->create_publisher<uwb_interfaces::msg::GazeboPosition>("/gazebo_position", 10);
    gazeboPositionSubscription_ = this->create_subscription<uwb_interfaces::msg::GazeboPosition>("/gazebo_position", 10, std::bind(&UWBRosBridge::gazebo_position_callback, this, std::placeholders::_1));
}

void UWBRosBridge::gz_odometry_topic_callback(const gz::msgs::Odometry &_msg)
{
    label_pose.x = _msg.pose().position().x();
    label_pose.y = _msg.pose().position().y();
    label_pose.z = _msg.pose().position().z();
    // RCLCPP_INFO(this->get_logger(), "position: %f %f %f", x, y, z);

    uwb_interfaces::msg::GazeboPosition msg;
    msg.address = get_id(labelName);
    msg.position = label_pose;
    gazeboPositionPublisher_->publish(msg);
}
int UWBRosBridge::get_id(std::string label_name)
{
    std::regex expression("x500_([0-9]+)");
    std::smatch match;

    if (std::regex_search(label_name, match, expression) && match.size() > 1)
    {
        return std::stoi(match.str(1));
    }
    return -1;
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

    if (srcAddr == destAddr)
    {
        uwb_interfaces::msg::UWBDistance uwb_distance;
        uwb_distance.src = srcAddr;
        uwb_distance.dest = destAddr;
        uwb_distance.distance = 0;
        response->uwb_distance = uwb_distance;
        return;
    }
    auto it = anchorPoseMap.find(destAddr);

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
        geometry_msgs::msg::Point p = label_pose_map[destAddr];

        distance = sqrt(pow(label_pose.x - p.x, 2) + pow(label_pose.y - p.y, 2) + pow(label_pose.z - p.z, 2));
        // RCLCPP_INFO(this->get_logger(), "label_pose: x: %lf, y: %lf, z: %lf", label_pose.x, label_pose.y, label_pose.z);
        // RCLCPP_INFO(this->get_logger(), "pose: x: %lf, y: %lf, z: %lf", p.x, p.y, p.z);
        // RCLCPP_INFO(this->get_logger(), "destAddr: %d, distance: %f", destAddr, distance);
    }
    // RCLCPP_INFO(this->get_logger(), "src: %d, dest: %d, distance: %f", srcAddr, destAddr, distance);
    uwb_interfaces::msg::UWBDistance uwb_distance;
    uwb_distance.src = srcAddr;
    uwb_distance.dest = destAddr;
    uwb_distance.distance = distance;
    response->uwb_distance = uwb_distance;
}

void UWBRosBridge::gazebo_position_callback(uwb_interfaces::msg::GazeboPosition::SharedPtr msg)
{
    int address = msg->address;
    label_pose_map[address] = msg->position;
}
