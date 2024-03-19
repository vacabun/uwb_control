#ifndef _CONTROLLER_HPP_
#define _CONTROLLER_HPP_

#include <string>
#include <regex>
#include <thread>
#include <future>
#include "rclcpp/rclcpp.hpp"
#include "uwb_interfaces/msg/uwb_control.hpp"
#include "uwb_interfaces/msg/uwb_data.hpp"
#include "uwb_interfaces/msg/uwb_distance.hpp"
#include "std_msgs/msg/string.hpp"
#include "uwb_interfaces/srv/uwb_measure.hpp"
#include "uwb_interfaces/msg/uwb_distance_matrix.hpp"

class UWBController : public rclcpp::Node
{
public:
    UWBController();

private:
    void config();
    void control_callback(const uwb_interfaces::msg::UWBControl::SharedPtr msg);
    void live_callback(const std_msgs::msg::String::SharedPtr msg);
    void liveTimerCallback();
    void performTask();
    std::string selectNextNode();
    double callServiceMeasureDistance(int index);
    int64_t get_time_ms();
    void handle_response(rclcpp::Client<uwb_interfaces::srv::UWBMeasure>::SharedFuture future);
    void control_handle(const uwb_interfaces::msg::UWBControl msg);
    int get_id(std::string nodeName);
    int get_controllerNodesList_index(int id);
    std::vector<std::vector<double>> distance_map_2_distance_matrix(std::map<std::pair<int, int>, double> map, std::set<int> &unique_nodes);
    std_msgs::msg::Float64MultiArray distance_matrix_2_ros_multi_array(std::vector<std::vector<double>> matrix);


    rclcpp::Publisher<uwb_interfaces::msg::UWBControl>::SharedPtr controllerPublisher_;
    rclcpp::Subscription<uwb_interfaces::msg::UWBControl>::SharedPtr controllerSubscription_;
    rclcpp::TimerBase::SharedPtr liveTimer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr livePublisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr liveSubscription_;
    rclcpp::Client<uwb_interfaces::srv::UWBMeasure>::SharedPtr client_;
    rclcpp::Publisher<uwb_interfaces::msg::UWBDistanceMatrix>::SharedPtr locatePublisher_;
    std::string nodeName;
    std::vector<std::string> controllerNodesList;
    std::vector<std::string> completedNodes;
    std::string brigde_service_str;
    std::string id_str;
    int64_t id;
    std::string locatePublishTopic;
    uwb_interfaces::msg::UWBData uwb_data;
    uwb_interfaces::msg::UWBControl control_message;
    std::map<std::pair<int, int>, double> distance_map;
};
#endif
