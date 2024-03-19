#include "controller.hpp"

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

    locatePublisher_ = this->create_publisher<uwb_interfaces::msg::UWBDistanceMatrix>(locatePublishTopic, 10);

    controllerNodesList.push_back(nodeName);

    if (nodeName == "/uwb_controller_1")
    {
        uwb_interfaces::msg::UWBControl msg;
        msg.assigned_node = nodeName;
        control_handle(msg);
    }
}

std::vector<std::vector<double>> UWBController::distance_map_2_distance_matrix(std::map<std::pair<int, int>, double> map, std::set<int> &unique_nodes)
{
    for (const auto &entry : map)
    {
        unique_nodes.insert(entry.first.first);
        unique_nodes.insert(entry.first.second);
    }

    std::map<int, int> node_index_map;
    int index = 0;
    for (const auto &node : unique_nodes)
    {
        node_index_map[node] = index++;
    }

    int num_nodes = unique_nodes.size();
    std::vector<std::vector<double>> distance_matrix(num_nodes, std::vector<double>(num_nodes, 0.0));
    for (const auto &entry : map)
    {
        int i = node_index_map[entry.first.first];
        int j = node_index_map[entry.first.second];
        distance_matrix[i][j] = entry.second;
    }
    // std::ostringstream node_oss;
    // RCLCPP_INFO(this->get_logger(), "Distance Matrix:");
    // for (const auto &node : unique_nodes)
    // {
    //     node_oss << node << "\t";
    // }
    // RCLCPP_INFO(this->get_logger(), "%s", node_oss.str().c_str());
    // for (const auto &row : distance_matrix)
    // {
    //     std::ostringstream oss;
    //     for (const auto &value : row)
    //     {
    //         oss << std::fixed << std::setprecision(2) << value << "\t";
    //     }
    //     RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    // }
    return distance_matrix;
}
std_msgs::msg::Float64MultiArray UWBController::distance_matrix_2_ros_multi_array(std::vector<std::vector<double>> matrix)
{
    auto msg = std_msgs::msg::Float64MultiArray();
    auto num_nodes = matrix.size();

    msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg.layout.dim[0].label = "rows";
    msg.layout.dim[0].size = num_nodes;
    msg.layout.dim[0].stride = num_nodes;
    msg.layout.dim[1].label = "cols";
    msg.layout.dim[1].size = num_nodes;
    msg.layout.dim[1].stride = num_nodes;
    msg.layout.data_offset = 0;

    for (const auto &row : matrix)
    {
        msg.data.insert(msg.data.end(), row.begin(), row.end());
    }
    return msg;
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

    this->declare_parameter("brigde_service", "/x500_1/uwb_bridge");
    brigde_service_str = this->get_parameter("brigde_service").as_string();
    RCLCPP_INFO(this->get_logger(), "brigde_service: %s", brigde_service_str.c_str());

    nodeName = "/" + std::string(this->get_name());
    id = get_id(nodeName);
    id_str = std::to_string(id);
    locatePublishTopic = "/uwbData/px4_" + id_str;
}

int UWBController::get_id(std::string nodeName)
{
    std::regex expression("/uwb_controller_([0-9]+)");
    std::smatch match;
    if (std::regex_search(nodeName, match, expression) && match.size() > 1)
    {
        return std::stoi(match.str(1));
    }
    return -1;
}

void UWBController::control_callback(const uwb_interfaces::msg::UWBControl::SharedPtr msg)
{
    control_handle(*msg);
}
void UWBController::control_handle(const uwb_interfaces::msg::UWBControl msg)
{
    if (msg.assigned_node == nodeName)
    {
        // RCLCPP_INFO(this->get_logger(), "Node %s is performing its task.", nodeName.c_str());

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
        // RCLCPP_INFO(this->get_logger(), "Next node is %s", nextNode.c_str());
        // 执行任务
        performTask();
    }
    int num_nodes = msg.node_num;
    std::vector<std::vector<double>> distances(num_nodes, std::vector<double>(num_nodes));
    for (int i = 0; i < num_nodes; ++i)
    {
        for (int j = 0; j < num_nodes; ++j)
        {
            distances[i][j] = msg.distance_mult_array.data[i * num_nodes + j];
            if (distances[i][j] != 0)
            {
                // RCLCPP_INFO(this->get_logger(), "distances(%d)(%d): %f", msg.address_list[i], msg.address_list[j], distances[i][j]);
                distance_map[std::make_pair(msg.address_list[i], msg.address_list[j])] = distances[i][j];
                distance_map[std::make_pair(msg.address_list[j], msg.address_list[i])] = distances[i][j];
            }
        }
    }
    // RCLCPP_INFO(this->get_logger(), "Distance Map:");
    // for (const auto &entry : distance_map)
    // {
    //     const auto &key = entry.first;
    //     const auto &value = entry.second;
    //     RCLCPP_INFO(this->get_logger(), "(%d, %d): %.2f", key.first, key.second, value);
    // }
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
    int dest = get_id(controllerNodesList[index]);
    auto request = std::make_shared<uwb_interfaces::srv::UWBMeasure::Request>();
    request->src_address = src;
    request->dest_address = dest;

    // RCLCPP_INFO(this->get_logger(), "Node %s is calling service. src: %d, dest: %d", nodeName.c_str(), src, dest);
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
int UWBController::get_controllerNodesList_index(int id)
{
    int index = -1;

    for (int i = 0; i < controllerNodesList.size(); i++)
    {
        if (get_id(controllerNodesList[i]) == id)
        {
            index = i;
            break;
        }
    }
    return index;
}
void UWBController::handle_response(rclcpp::Client<uwb_interfaces::srv::UWBMeasure>::SharedFuture future)
{
    auto response = future.get();

    uwb_interfaces::msg::UWBDistance uwb_distance;
    uwb_distance = response->uwb_distance;

    // RCLCPP_INFO(this->get_logger(), "src: %ld, dest: %ld, distance: %.2lf", uwb_distance.src, uwb_distance.dest, uwb_distance.distance);
    // uwb_data.distances.push_back(uwb_distance);
    distance_map[std::make_pair(uwb_distance.src, uwb_distance.dest)] = uwb_distance.distance;
    distance_map[std::make_pair(uwb_distance.dest, uwb_distance.src)] = uwb_distance.distance;

    int index = get_controllerNodesList_index(uwb_distance.dest);

    // RCLCPP_INFO(this->get_logger(), "index: %d", index);
    if (index >= controllerNodesList.size() - 1)
    {
        // RCLCPP_INFO(this->get_logger(), "Node %s has completed its task.", nodeName.c_str());

        std::set<int> unique_nodes;
        std::vector<std::vector<double>> distance_matrix = distance_map_2_distance_matrix(distance_map, unique_nodes);

        control_message.address_list.clear();
        for (auto node : unique_nodes)
        {
            control_message.address_list.push_back(node);
        }
        control_message.node_num = unique_nodes.size();
        control_message.distance_mult_array = distance_matrix_2_ros_multi_array(distance_matrix);

        uwb_interfaces::msg::UWBDistanceMatrix msg;
        msg.address_list = control_message.address_list;
        msg.node_num = control_message.node_num;
        msg.distance_mult_array = control_message.distance_mult_array;

        locatePublisher_->publish(msg);
        controllerPublisher_->publish(control_message);

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
            // RCLCPP_INFO(this->get_logger(), "Next node is %s", controllerNodesList[i].c_str());
            return controllerNodesList[i];
        }
    }
    completedNodes.clear();

    return controllerNodesList[0];
}
