#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "waypoint_navigation/action/nav_to_waypoint.hpp"
#include "waypoint_navigation/srv/get_waypoints.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class WayPointClient : public rclcpp::Node
{
public:
    using NavToWaypoint = waypoint_navigation::action::NavToWaypoint;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavToWaypoint>;

    WayPointClient() : Node("waypoint_client"), goal_index_(0)
    {
        action_client_ = rclcpp_action::create_client<NavToWaypoint>(
            this, "waypoint_navigation");

        service_client_ = this->create_client<waypoint_navigation::srv::GetWaypoints>("waypoints");

        while (!service_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
    }

    void receive_goals()
    {
        auto request = std::make_shared<waypoint_navigation::srv::GetWaypoints::Request>();
        request->get_waypoints = true;

        auto future = service_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Waypoints received by the action client");
            for (const auto & pose : response->waypoints.poses) {
                goals_.push_back({pose.position.x, pose.position.y, pose.position.z});
                RCLCPP_INFO(this->get_logger(), "Waypoints: [%.2f, %.2f, %.2f]",
                    pose.position.x, pose.position.y, pose.position.z);
            }

            if (!goals_.empty()) {
                send_goal(goals_[0]);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service get_waypoints");
        }
    }

    void send_goal(const std::vector<double> & waypoint)
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        while (!action_client_->wait_for_action_server(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for action server. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Action server not available, waiting again...");
        }

        auto goal_msg = NavToWaypoint::Goal();
        goal_msg.waypoint.position.x = waypoint[0];
        goal_msg.waypoint.position.y = waypoint[1];
        goal_msg.waypoint.position.z = waypoint[2];
        goal_msg.waypoint.orientation.w = 1.0;
        goal_msg.hover = true;

        RCLCPP_INFO(this->get_logger(), "Sending goal [%zu/%zu]: [%.2f, %.2f, %.2f]",
            goal_index_ + 1, goals_.size(), waypoint[0], waypoint[1], waypoint[2]);

        auto send_goal_options = rclcpp_action::Client<NavToWaypoint>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&WayPointClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&WayPointClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&WayPointClient::result_callback, this, _1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    std::vector<std::vector<double>> goals_;
    size_t goal_index_;
    rclcpp_action::Client<NavToWaypoint>::SharedPtr action_client_;
    rclcpp::Client<waypoint_navigation::srv::GetWaypoints>::SharedPtr service_client_;

    void goal_response_callback(const GoalHandleNav::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal rejected :(");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted :)");
        }
    }

    void feedback_callback(
        GoalHandleNav::SharedPtr,
        const std::shared_ptr<const NavToWaypoint::Feedback> feedback)
    {
        double x = feedback->current_waypoint.pose.position.x;
        double y = feedback->current_waypoint.pose.position.y;
        double z = feedback->current_waypoint.pose.position.z;
        int t = feedback->current_waypoint.header.stamp.sec;
        RCLCPP_INFO(this->get_logger(), "Received feedback! The current whycon position is: %.2f, %.2f, %.2f", x, y, z);
        RCLCPP_INFO(this->get_logger(), "Max time inside sphere: %d", t);
    }

    void result_callback(const GoalHandleNav::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Result: %d", result.result->hov_time);
            goal_index_++;
            if (goal_index_ < goals_.size()) {
                send_goal(goals_[goal_index_]);
            } else {
                RCLCPP_INFO(this->get_logger(), "All waypoints have been reached successfully");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Goal failed or was aborted");
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto waypoint_client = std::make_shared<WayPointClient>();
    waypoint_client->receive_goals();
    rclcpp::spin(waypoint_client);
    rclcpp::shutdown();
    return 0;
}
