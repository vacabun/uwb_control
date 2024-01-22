#ifndef _CONTROLLER_HPP_
#define _CONTROLLER_HPP_



class UWBController : public rclcpp::Node
{
public:
    UWBController();

private:
    void config();
};
#endif
