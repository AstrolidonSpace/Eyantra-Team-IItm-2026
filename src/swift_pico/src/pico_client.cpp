#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <vector>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "waypoint_navigation/action/nav_to_waypoint.hpp"
#include "waypoint_navigation/srv/get_waypoints.hpp"
#include "waypoint_navigation/srv/replan_route.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

struct RouteGoal
{
    geometry_msgs::msg::Pose pose;
    std::string marker_id;
    bool hover;
};

enum class MissionPhase
{
    INITIALIZING,
    CRITICAL,
    SWITCHING,
    STABLE,
    COMPLETED
};

class WayPointClient : public rclcpp::Node
{
public:
    using NavToWaypoint = waypoint_navigation::action::NavToWaypoint;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavToWaypoint>;

    WayPointClient()
        : Node("waypoint_client"),
          phase_(MissionPhase::INITIALIZING),
          goal_index_(0),
          clock_started_(false),
          switch_triggered_(false),
          critical_saves_count_(0),
          total_critical_survivors_(3)
    {
        action_client_ = rclcpp_action::create_client<NavToWaypoint>(
            this, "waypoint_navigation");

        waypoints_service_client_ = this->create_client<waypoint_navigation::srv::GetWaypoints>("waypoints");
        replan_service_client_ = this->create_client<waypoint_navigation::srv::ReplanRoute>("replan_waypoints");

        this->declare_parameter<double>("critical_window", 55.0);

        // Timer to monitor the 55s critical window at 20 Hz
        clock_timer_ = this->create_wall_timer(
            50ms, std::bind(&WayPointClient::clock_monitor_callback, this));

        RCLCPP_INFO(this->get_logger(), "WayPointClient initialized for Task 3A");
    }

    void start_mission()
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for /waypoints service...");
        while (!waypoints_service_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for /waypoints service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "/waypoints service not available yet, waiting...");
        }

        auto request = std::make_shared<waypoint_navigation::srv::GetWaypoints::Request>();
        request->get_waypoints = true;

        auto future = waypoints_service_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Received %zu critical route waypoints from service",
                        response->waypoints.poses.size());

            critical_goals_.clear();
            for (size_t i = 0; i < response->waypoints.poses.size(); ++i) {
                RouteGoal g;
                g.pose = response->waypoints.poses[i];
                g.marker_id = response->marker_ids[i];
                // orientation.z: 1.0 = hover/save, 0.0 = pass-through
                g.hover = (g.pose.orientation.z > 0.5f);
                critical_goals_.push_back(g);
                RCLCPP_INFO(this->get_logger(), "  Critical Goal [%zu]: %s [%.2f, %.2f, %.2f] hover=%s",
                            i + 1, g.marker_id.c_str(), g.pose.position.x, g.pose.position.y,
                            g.pose.position.z, g.hover ? "true" : "false");
            }

            // Wait for action server before sending first goal
            RCLCPP_INFO(this->get_logger(), "Waiting for waypoint_navigation action server...");
            while (!action_client_->wait_for_action_server(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for action server. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Action server not available yet, waiting...");
            }

            if (!critical_goals_.empty()) {
                current_goals_ = &critical_goals_;
                goal_index_ = 0;
                send_current_goal();
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call /waypoints service.");
        }
    }

private:
    MissionPhase phase_;
    std::vector<RouteGoal> critical_goals_;
    std::vector<RouteGoal> stable_goals_;
    std::vector<RouteGoal>* current_goals_;
    size_t goal_index_;

    rclcpp::Time takeoff_time_;
    std::atomic<bool> clock_started_;
    std::atomic<bool> switch_triggered_;
    size_t critical_saves_count_;
    const size_t total_critical_survivors_;

    geometry_msgs::msg::Point last_feedback_position_;
    GoalHandleNav::SharedPtr active_goal_handle_;

    rclcpp_action::Client<NavToWaypoint>::SharedPtr action_client_;
    rclcpp::Client<waypoint_navigation::srv::GetWaypoints>::SharedPtr waypoints_service_client_;
    rclcpp::Client<waypoint_navigation::srv::ReplanRoute>::SharedPtr replan_service_client_;
    rclcpp::TimerBase::SharedPtr clock_timer_;

    void clock_monitor_callback()
    {
        if (phase_ == MissionPhase::CRITICAL && clock_started_ && !switch_triggered_) {
            double elapsed = (this->now() - takeoff_time_).seconds();
            double window = this->get_parameter("critical_window").as_double();
            if (elapsed >= (window - 0.2)) {
                RCLCPP_WARN(this->get_logger(), "CRITICAL WINDOW EXPIRED (t=%.2fs >= %.1fs)! Triggering the Switch...", elapsed, window);
                trigger_switch();
            }
        }
    }

    void trigger_switch()
    {
        if (switch_triggered_) return;
        switch_triggered_ = true;
        phase_ = MissionPhase::SWITCHING;

        RCLCPP_INFO(this->get_logger(), "*****************************************");
        RCLCPP_INFO(this->get_logger(), "***             THE SWITCH            ***");
        RCLCPP_INFO(this->get_logger(), "*****************************************");

        if (active_goal_handle_) {
            RCLCPP_INFO(this->get_logger(), "Canceling active in-flight goal for Switch handoff...");
            action_client_->async_cancel_goal(active_goal_handle_);
        } else {
            call_replan_service();
        }
    }

    void call_replan_service()
    {
        RCLCPP_INFO(this->get_logger(), "Calling /replan_waypoints from drone position: [%.2f, %.2f, %.2f]",
                    last_feedback_position_.x, last_feedback_position_.y, last_feedback_position_.z);

        while (!replan_service_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for /replan_waypoints service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "/replan_waypoints service not available yet, waiting...");
        }

        auto request = std::make_shared<waypoint_navigation::srv::ReplanRoute::Request>();
        request->position = last_feedback_position_;

        replan_service_client_->async_send_request(
            request,
            [this](rclcpp::Client<waypoint_navigation::srv::ReplanRoute>::SharedFuture future_res) {
                auto res = future_res.get();
                RCLCPP_INFO(this->get_logger(), "Received %zu stable route waypoints from replan service ending at F6",
                            res->waypoints.poses.size());

                stable_goals_.clear();
                for (size_t i = 0; i < res->waypoints.poses.size(); ++i) {
                    RouteGoal g;
                    g.pose = res->waypoints.poses[i];
                    g.marker_id = res->marker_ids[i];
                    g.hover = res->hover[i];
                    stable_goals_.push_back(g);
                    RCLCPP_INFO(this->get_logger(), "  Stable Goal [%zu]: %s [%.2f, %.2f, %.2f] hover=%s",
                                i + 1, g.marker_id.c_str(), g.pose.position.x, g.pose.position.y,
                                g.pose.position.z, g.hover ? "true" : "false");
                }

                phase_ = MissionPhase::STABLE;
                current_goals_ = &stable_goals_;
                goal_index_ = 0;

                if (!stable_goals_.empty()) {
                    send_current_goal();
                } else {
                    RCLCPP_WARN(this->get_logger(), "Stable route empty!");
                }
            });
    }

    void send_current_goal()
    {
        if (!current_goals_ || goal_index_ >= current_goals_->size()) {
            return;
        }

        const auto & g = (*current_goals_)[goal_index_];

        auto goal_msg = NavToWaypoint::Goal();
        goal_msg.waypoint = g.pose;
        goal_msg.hover = g.hover;

        RCLCPP_INFO(this->get_logger(), "Sending Goal [%zu/%zu] (%s): [%.2f, %.2f, %.2f] hover=%s",
                    goal_index_ + 1, current_goals_->size(), g.marker_id.c_str(),
                    g.pose.position.x, g.pose.position.y, g.pose.position.z,
                    g.hover ? "true" : "false");

        auto send_goal_options = rclcpp_action::Client<NavToWaypoint>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&WayPointClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&WayPointClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&WayPointClient::result_callback, this, _1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleNav::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
        } else {
            active_goal_handle_ = goal_handle;
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server.");
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

        last_feedback_position_.x = x;
        last_feedback_position_.y = y;
        last_feedback_position_.z = z;

        // Start the critical window clock upon lift-off (when drone starts ascending from ground Z >= 14.8)
        if (!clock_started_) {
            takeoff_time_ = this->now();
            clock_started_ = true;
            phase_ = MissionPhase::CRITICAL;
            RCLCPP_INFO(this->get_logger(), ">>> TAKEOFF DETECTED! Critical window clock started at t=0.00s (Limit: 55.0s) <<<");
        }

        double elapsed = clock_started_ ? (this->now() - takeoff_time_).seconds() : 0.0;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Telemetry [t=%.1fs] Pose: [%.2f, %.2f, %.2f] Hold: %ds", elapsed, x, y, z, t);
    }

    void result_callback(const GoalHandleNav::WrappedResult & result)
    {
        active_goal_handle_.reset();

        if (result.code == rclcpp_action::ResultCode::CANCELED) {
            RCLCPP_INFO(this->get_logger(), "Goal canceled cleanly by server.");
            if (phase_ == MissionPhase::SWITCHING) {
                call_replan_service();
            }
            return;
        }

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Goal reached successfully. Hover time: %d s", result.result->hov_time);

            if (phase_ == MissionPhase::CRITICAL) {
                const auto & g = (*current_goals_)[goal_index_];
                if (g.hover) {
                    critical_saves_count_++;
                    RCLCPP_INFO(this->get_logger(), "Critical Survivor Saved! (%zu/%zu saved)",
                                critical_saves_count_, total_critical_survivors_);
                }

                // If all 3 critical survivors are saved, trigger Switch immediately without waiting for 55s
                if (critical_saves_count_ >= total_critical_survivors_) {
                    RCLCPP_INFO(this->get_logger(), "All 3 Critical Survivors Saved! Triggering early Switch to maximize score...");
                    trigger_switch();
                    return;
                }

                goal_index_++;
                if (goal_index_ < current_goals_->size()) {
                    send_current_goal();
                } else {
                    // Critical route completed, trigger Switch
                    trigger_switch();
                }
            }
            else if (phase_ == MissionPhase::STABLE) {
                const auto & g = (*current_goals_)[goal_index_];
                if (g.hover && g.marker_id == "F6") {
                    phase_ = MissionPhase::COMPLETED;
                    double total_time = (this->now() - takeoff_time_).seconds();
                    RCLCPP_INFO(this->get_logger(), "=========================================================");
                    RCLCPP_INFO(this->get_logger(), "MISSION FULLY COMPLETED in %.2f seconds (Cap: 150.0s)!", total_time);
                    RCLCPP_INFO(this->get_logger(), "All survivors saved and drone safely landed at F6!");
                    RCLCPP_INFO(this->get_logger(), "=========================================================");
                    return;
                }

                goal_index_++;
                if (goal_index_ < current_goals_->size()) {
                    send_current_goal();
                } else {
                    phase_ = MissionPhase::COMPLETED;
                    RCLCPP_INFO(this->get_logger(), "All stable goals completed.");
                }
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Goal failed or was aborted.");
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto waypoint_client = std::make_shared<WayPointClient>();

    waypoint_client->start_mission();
    rclcpp::spin(waypoint_client);

    rclcpp::shutdown();
    return 0;
}
