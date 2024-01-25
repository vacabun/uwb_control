#include "rclcpp/rclcpp.hpp"
#include "uwb_interfaces/msg/uwb_control.hpp"
#include "uwb_interfaces/msg/uwb_data.hpp"
#include "uwb_interfaces/msg/uwb_distance.hpp"
#include "std_msgs/msg/string.hpp"
#include "uwb_interfaces/srv/uwb_measure.hpp"
#include "controller.hpp"
#include <string>
#include <regex>
using namespace std::chrono_literals;
int main(int argc, const char *argv[])
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

    rclcpp::SensorDataQoS qos;
    qos.keep_last(1);

    controllerPublisher_ = this->create_publisher<uwb_interfaces::msg::UWBControl>("uwb_control_topic", qos);
    controllerSubscription_ = this->create_subscription<uwb_interfaces::msg::UWBControl>(
        "uwb_control_topic", qos, std::bind(&UWBController::control_callback, this, std::placeholders::_1));

    liveTimer_ = this->create_wall_timer(1s, std::bind(&UWBController::liveTimerCallback, this));
    livePublisher_ = this->create_publisher<std_msgs::msg::String>("uwb_control_live_topic", qos);
    liveSubscription_ = this->create_subscription<std_msgs::msg::String>(
        "uwb_control_live_topic", qos, std::bind(&UWBController::live_callback, this, std::placeholders::_1));

    client_ = this->create_client<uwb_interfaces::srv::UWBMeasure>("/uwb_control");

    locatePublisher_ = this->create_publisher<uwb_interfaces::msg::UWBData>(locatePublishTopic, qos);

    controllerNodesList.push_back(nodeName);

    if (nodeName == "/uwb_controller_1")
    {
        // 执行任务
        performTask();
        // 将自己加入已完成节点列表
        completedNodes.push_back(nodeName);
        // 选择下一个节点
        std::string nextNode = selectNextNode();

        // 发布消息
        uwb_interfaces::msg::UWBControl message;
        message.completed_nodes = completedNodes;
        message.assigned_node = nextNode;
        controllerPublisher_->publish(message);
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

    if (msg->assigned_node == nodeName)
    {
        // 执行任务
        performTask();
        // 获取已完成任务的节点列表
        completedNodes = msg->completed_nodes;
        // 将自己加入已完成节点列表
        completedNodes.push_back(nodeName);
        // 选择下一个节点
        std::string nextNode = selectNextNode();

        // 发布消息
        uwb_interfaces::msg::UWBControl message;
        message.completed_nodes = completedNodes;
        message.assigned_node = nextNode;
        controllerPublisher_->publish(message);
    }
}

int64_t UWBController::get_time_ms()
{
    rclcpp::Time current_time = this->now();
    return current_time.nanoseconds() / 1000000;
}

void UWBController::performTask()
{
    // 实现节点的任务逻辑
    sleep(1);
    uwb_interfaces::msg::UWBData msg;
    // msg.label_name = labelName;
    for (int i = 0; i < anchorList.size(); i++)
    {
        int64_t anchor_id = anchorList[i]; // anchor id

        uwb_interfaces::msg::UWBDistance distance;
        distance.src = id;
        distance.dest = anchor_id;
        distance.distance = callServiceMeasureDistance(anchor_id);
        msg.distances.push_back(distance);
    }
    locatePublisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Node %s has completed its task.", nodeName.c_str());
}

int64_t UWBController::callServiceMeasureDistance(int anchorId)
{
    int64_t distance = 0;
    auto request = std::make_shared<uwb_interfaces::srv::UWBMeasure::Request>();
    request->cmd = 1;
    request->src = 2;
    request->dest = anchorId;

    service_call_completed = false;

    auto future = client_->async_send_request(request, std::bind(&UWBController::handle_response, this, std::placeholders::_1));

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
        distance = this->measureDistance;
    }

    return distance;
}
void UWBController::handle_response(rclcpp::Client<uwb_interfaces::srv::UWBMeasure>::SharedFuture future)
{
    auto response = future.get();
    // 在这里处理响应
    // 例如，打印响应消息
    this->measureDistance = response->distance;
    this->service_call_completed = true;
}

std::string UWBController::selectNextNode()
{
    // 选择下一个节点
    for (int i = 0; i < controllerNodesList.size(); i++)
    {
        if (std::find(completedNodes.begin(), completedNodes.end(), controllerNodesList[i]) == completedNodes.end())
        {
            return controllerNodesList[i];
        }
    }
    completedNodes.clear();
    return controllerNodesList[0];
}
