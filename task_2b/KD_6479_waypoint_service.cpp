#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "waypoint_navigation/srv/get_waypoints.hpp"

class WaypointServiceNode : public rclcpp::Node
{
public:
    WaypointServiceNode() : Node("waypoints_service")
    {
        // Define the 4 waypoints in exact sequence: wp1 to wp4
        waypoints_ = {
            {-2.41, -2.41, 13.00},
            { 2.41,  1.20, 13.00},
            { 1.20, -2.41, 13.00},
            {-1.20,  1.20, 13.00}
        };

        marker_ids_ = {"wp1", "wp2", "wp3", "wp4"};

        service_ = this->create_service<waypoint_navigation::srv::GetWaypoints>(
            "waypoints",
            std::bind(&WaypointServiceNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Waypoint service node ready on /waypoints");
    }

private:
    std::vector<std::vector<double>> waypoints_;
    std::vector<std::string> marker_ids_;
    rclcpp::Service<waypoint_navigation::srv::GetWaypoints>::SharedPtr service_;

    void handle_service(
        const std::shared_ptr<waypoint_navigation::srv::GetWaypoints::Request> request,
        std::shared_ptr<waypoint_navigation::srv::GetWaypoints::Response> response)
    {
        if (request->get_waypoints) {
            response->waypoints.poses.resize(waypoints_.size());
            for (size_t i = 0; i < waypoints_.size(); ++i) {
                response->waypoints.poses[i].position.x = waypoints_[i][0];
                response->waypoints.poses[i].position.y = waypoints_[i][1];
                response->waypoints.poses[i].position.z = waypoints_[i][2];
                response->waypoints.poses[i].orientation.w = 1.0;
            }
            response->marker_ids = marker_ids_;
            RCLCPP_INFO(this->get_logger(), "Incoming request for Waypoints - sent 4 waypoints");
        } else {
            RCLCPP_INFO(this->get_logger(), "Request rejected");
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
