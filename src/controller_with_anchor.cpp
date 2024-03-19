#include "rclcpp/rclcpp.hpp"
#include "uwb_interfaces/msg/uwb_control.hpp"
#include "uwb_interfaces/msg/uwb_data.hpp"
#include "uwb_interfaces/msg/uwb_distance.hpp"
#include "std_msgs/msg/string.hpp"
#include "uwb_interfaces/srv/uwb_measure.hpp"
#include "controller_with_anchor.hpp"
#include <string>
#include <regex>
#include <thread>
#include <future>
using namespace std::chrono_literals;
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UWBController>());
    rclcpp::shutdown();

    return 0;
}

UWBController::UWBController() : Node("uwb_controller")
{
    RCLCPP_INFO(this->get_logger(), "Node %s has been started.", this->get_name());
    config();

    client_ = this->create_client<uwb_interfaces::srv::UWBMeasure>(brigde_service_str);
    controllerPublisher_ = this->create_publisher<uwb_interfaces::msg::UWBControl>("uwb_control_topic", 10);
    controllerSubscription_ = this->create_subscription<uwb_interfaces::msg::UWBControl>("uwb_control_topic", 10, std::bind(&UWBController::control_callback, this, std::placeholders::_1));

    liveTimer_ = this->create_wall_timer(1s, std::bind(&UWBController::liveTimerCallback, this));
    livePublisher_ = this->create_publisher<std_msgs::msg::String>("uwb_control_live_topic", 10);
    liveSubscription_ = this->create_subscription<std_msgs::msg::String>("uwb_control_live_topic", 10, std::bind(&UWBController::live_callback, this, std::placeholders::_1));

    locatePublisher_ = this->create_publisher<uwb_interfaces::msg::UWBData>(locatePublishTopic, 10);

    controllerNodesList.push_back(nodeName);

    if (nodeName == "/uwb_controller_1")
    {
        uwb_interfaces::msg::UWBControl msg;
        msg.assigned_node = nodeName;
        control_handle(msg);
    }
}

void UWBController::liveTimerCallback()
{
    std_msgs::msg::String message;
    message.data = nodeName;
    livePublisher_->publish(message);
}

void UWBController::live_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if (find(controllerNodesList.begin(), controllerNodesList.end(), msg->data) == controllerNodesList.end())
    {
        controllerNodesList.push_back(msg->data);
        sort(controllerNodesList.begin(), controllerNodesList.end());
    }
}

void UWBController::config()
{
    this->declare_parameter("anchor_list", "11,12,13,14");

    this->declare_parameter("brigde_service", "/x500_1/uwb_bridge");

    brigde_service_str = this->get_parameter("brigde_service").as_string();

    RCLCPP_INFO(this->get_logger(), "brigde_service: %s", brigde_service_str.c_str());

    anchorsStr = this->get_parameter("anchor_list").as_string();

    anchorList = getAnchorList();

    nodeName = "/" + std::string(this->get_name());

    std::regex expression("/uwb_controller_([0-9]+)");
    std::smatch match;
    if (std::regex_search(nodeName, match, expression) && match.size() > 1)
    {
        id_str = match.str(1);
        id = std::stoi(id_str);
    }
    locatePublishTopic = "/uwbData/px4_" + id_str;

    for (int i = 0; i < anchorList.size(); i++)
    {
        RCLCPP_INFO(this->get_logger(), "Anchor %d has been added.", anchorList[i]);
    }
}

std::vector<int> UWBController::getAnchorList()
{
    std::vector<int> result;
    std::istringstream ss(anchorsStr);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        result.push_back(std::stoi(token));
    }
    return result;
}

void UWBController::control_callback(const uwb_interfaces::msg::UWBControl::SharedPtr msg)
{
    control_handle(*msg);
}
void UWBController::control_handle(const uwb_interfaces::msg::UWBControl msg)
{
    if (msg.assigned_node == nodeName)
    {
        RCLCPP_INFO(this->get_logger(), "Node %s is performing its task.", nodeName.c_str());

        // 获取已完成任务的节点列表
        completedNodes = msg.completed_nodes;
        // 将自己加入已完成节点列表
        completedNodes.push_back(nodeName);
        // 选择下一个节点
        std::string nextNode = selectNextNode();
        // 打包消息
        control_message.all_nodes = controllerNodesList;
        control_message.completed_nodes = completedNodes;
        control_message.assigned_node = nextNode;
        RCLCPP_INFO(this->get_logger(), "Next node is %s", nextNode.c_str());
        // 执行任务
        performTask();
    }
}
int64_t UWBController::get_time_ms()
{
    rclcpp::Time current_time = this->now();
    return current_time.nanoseconds() / 1000000;
}

void UWBController::performTask()
{
    callServiceMeasureDistance(0);
}

double UWBController::callServiceMeasureDistance(int index)
{
    int src = id;
    int dest = anchorList[index];

    auto request = std::make_shared<uwb_interfaces::srv::UWBMeasure::Request>();
    request->src_address = src;
    request->dest_address = dest;

    RCLCPP_INFO(this->get_logger(), "Node %s is calling service. src: %d, dest: %d", nodeName.c_str(), src, dest);
    while (!client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    auto future = client_->async_send_request(request, std::bind(&UWBController::handle_response, this, std::placeholders::_1));

    return 0.0;
}

void UWBController::handle_response(rclcpp::Client<uwb_interfaces::srv::UWBMeasure>::SharedFuture future)
{
    auto response = future.get();

    uwb_interfaces::msg::UWBDistance uwb_distance;
    uwb_distance = response->uwb_distance;

    RCLCPP_INFO(this->get_logger(), "dest: %ld, distance: %lf", uwb_distance.dest, uwb_distance.distance);
    uwb_data.distances.push_back(uwb_distance);

    int anchor_id = uwb_distance.dest;
    
    int index = -1;

    for (int i = 0; i < anchorList.size(); i++)
    {
        if (anchorList[i] == anchor_id)
        {
            index = i;
            break;
        }
    }

    RCLCPP_INFO(this->get_logger(), "index: %d", index);
    if (index == anchorList.size() - 1)
    {
        locatePublisher_->publish(uwb_data);
        RCLCPP_INFO(this->get_logger(), "Node %s has completed its task.", nodeName.c_str());
        controllerPublisher_->publish(control_message);
        uwb_data.distances.clear();
    }
    else
    {
        callServiceMeasureDistance(index + 1);
    }
}

std::string UWBController::selectNextNode()
{
    for (int i = 0; i < controllerNodesList.size(); i++)
    {
        if (std::find(completedNodes.begin(), completedNodes.end(), controllerNodesList[i]) == completedNodes.end())
        {
            RCLCPP_INFO(this->get_logger(), "Next node is %s", controllerNodesList[i].c_str());
            return controllerNodesList[i];
        }
    }
    completedNodes.clear();

    return controllerNodesList[0];
}
