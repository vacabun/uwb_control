#include "rclcpp/rclcpp.hpp"
#include "controller.hpp"

int main(int argc, const char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UWBController>());
    rclcpp::shutdown();

    return 0;
}

UWBController::UWBController() : Node("uwb_concroller")
{
    config();

}

void UWBController::config()
{
    this->declare_parameter("anchor_list", "11,12,13,14");
}
