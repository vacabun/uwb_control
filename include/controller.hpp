#ifndef _CONTROLLER_HPP_
#define _CONTROLLER_HPP_

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
    std::vector<int> getAnchorList();
    int64_t callServiceMeasureDistance(int anchorId);
    int64_t get_time_ms();
    void handle_response(rclcpp::Client<uwb_interfaces::srv::UWBMeasure>::SharedFuture future);

    rclcpp::Publisher<uwb_interfaces::msg::UWBControl>::SharedPtr controllerPublisher_;
    rclcpp::Subscription<uwb_interfaces::msg::UWBControl>::SharedPtr controllerSubscription_;
    rclcpp::TimerBase::SharedPtr liveTimer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr livePublisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr liveSubscription_;
    rclcpp::Client<uwb_interfaces::srv::UWBMeasure>::SharedPtr client_;
    rclcpp::Publisher<uwb_interfaces::msg::UWBData>::SharedPtr locatePublisher_;
    std::string nodeName;
    std::vector<std::string> controllerNodesList;
    std::vector<std::string> completedNodes;
    std::vector<int> anchorList;
    std::string anchorsStr;

    bool service_call_completed;
    int64_t measureDistance;
    std::string id_str;
    int64_t id;
    std::string locatePublishTopic;
};
#endif
