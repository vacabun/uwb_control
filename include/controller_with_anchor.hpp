#ifndef _CONTROLLER_WITH_ANCHOR_HPP_
#define _CONTROLLER_WITH_ANCHOR_HPP_

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
    double callServiceMeasureDistance(int index);
    int64_t get_time_ms();
    void handle_response(rclcpp::Client<uwb_interfaces::srv::UWBMeasure>::SharedFuture future);
    void control_handle(const uwb_interfaces::msg::UWBControl msg);
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
    std::string brigde_service_str;
    std::string id_str;
    int64_t id;
    std::string locatePublishTopic;
    uwb_interfaces::msg::UWBData uwb_data;
    uwb_interfaces::msg::UWBControl control_message;
};
#endif
