#include <chrono>
#include <thread>
#include <functional>
#include <memory>
#include <algorithm>
#include <cmath>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "whycode_interfaces/msg/marker_array.hpp"
#include "swift_msgs/msg/swift_msgs.hpp"
#include "error_msg/msg/error.hpp"
#include "std_msgs/msg/header.hpp"
#include "waypoint_navigation/action/nav_to_waypoint.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

struct CMD
{
    int rc_roll;
    int rc_pitch;
    int rc_yaw;
    int rc_throttle;
    int rc_aux4;
};

class WayPointServer : public rclcpp::Node
{
public:
    using NavToWaypoint = waypoint_navigation::action::NavToWaypoint;
    using GoalHandleNav = rclcpp_action::ServerGoalHandle<NavToWaypoint>;

    WayPointServer() : Node("waypoint_server")
    {
        drone_position[0] = 0.0f;
        drone_position[1] = 0.0f;
        drone_position[2] = 13.0f;

        prev_drone_position[0] = 0.0f;
        prev_drone_position[1] = 0.0f;
        prev_drone_position[2] = 13.0f;

        actual_vel_x = 0.0f;
        actual_vel_y = 0.0f;
        actual_vel_z = 0.0f;

        // Default setpoint to arena center F6 (Task 2C starting position)
        setpoint[0] = 0.01f;
        setpoint[1] = 0.01f;
        setpoint[2] = 13.0f;

        current_dtime = 0.0;
        first_marker_received = false;

        cmd.rc_roll = 1500;
        cmd.rc_pitch = 1500;
        cmd.rc_yaw = 1500;
        cmd.rc_throttle = 1500;
        cmd.rc_aux4 = 1500;

        // Roll (0) and Pitch (1) cascaded gains
        Kp_pos[0] = 66.0f;  Kp_pos[1] = 66.0f;
        Ki_pos[0] = 0.016f; Ki_pos[1] = 0.016f;
        Kd_pos[0] = 90.0f;  Kd_pos[1] = 90.0f;
        pos_error_sum[0] = 0.0f; pos_error_sum[1] = 0.0f;
        prev_error[0] = 0.0f;    prev_error[1] = 0.0f;

        Kp_vel[0] = 25.0f; Kp_vel[1] = 25.0f;
        Ki_vel[0] = 0.0f;  Ki_vel[0] = 0.0f;
        Kd_vel[0] = 0.0f;  Kd_vel[0] = 0.0f;
        vel_error_sum[0] = 0.0f; vel_error_sum[1] = 0.0f;
        prev_vel_error[0] = 0.0f; prev_vel_error[1] = 0.0f;

        // Altitude cascaded gains
        Kp_pos_z = 3.0f;
        Ki_pos_z = 0.045f;
        Kd_pos_z = 2.2f;
        pos_z_error_sum = 0.0f;
        prev_error[2] = 0.0f;

        Kp_vel_z = 22.0f;
        Ki_vel_z = 2.0f;
        Kd_vel_z = 0.0f;
        vel_z_error_sum = 0.0f;
        prev_vel_z_error = 0.0f;

        max_horizontal_vel = 1.5f;
        max_vertical_vel = 5.0f;
        windup_guard = 40.0f;

        for (int i = 0; i < 3; i++) {
            min_values[i] = 1000;
            max_values[i] = 2000;
        }

        sample_time = 33ms;
        dt = std::chrono::duration<float>(sample_time).count();

        // Callback groups
        pid_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Publishers
        command_pub_ = this->create_publisher<swift_msgs::msg::SwiftMsgs>("/drone_command", 10);
        pos_error_pub_ = this->create_publisher<error_msg::msg::Error>("/pos_error", 10);
        heartbeat_pub_ = this->create_publisher<std_msgs::msg::Header>("/controller_heartbeat", 10);

        // Subscriber to markers
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = pid_callback_group_;
        whycode_sub_ = this->create_subscription<whycode_interfaces::msg::MarkerArray>(
            "/whycode_node/markers", 1,
            std::bind(&WayPointServer::whycode_callback, this, _1),
            sub_opt);

        // Action Server
        action_server_ = rclcpp_action::create_server<NavToWaypoint>(
            this,
            "waypoint_navigation",
            std::bind(&WayPointServer::handle_goal, this, _1, _2),
            std::bind(&WayPointServer::handle_cancel, this, _1),
            std::bind(&WayPointServer::handle_accepted, this, _1));

        publish_heartbeat();
        heartbeat_timer_ = this->create_wall_timer(
            100ms, std::bind(&WayPointServer::publish_heartbeat, this), pid_callback_group_);

        std::this_thread::sleep_for(500ms);
        arm();

        pid_timer_ = this->create_wall_timer(
            sample_time, std::bind(&WayPointServer::pid, this), pid_callback_group_);

        RCLCPP_INFO(this->get_logger(), "WayPointServer C++ node initialized and armed");
    }

private:
    float drone_position[3];
    float prev_drone_position[3];
    float actual_vel_x;
    float actual_vel_y;
    float actual_vel_z;
    float setpoint[3];
    double current_dtime;
    bool first_marker_received;

    CMD cmd;
    std::chrono::milliseconds sample_time;
    float dt;

    // Roll (0) and Pitch (1) cascaded PID parameters
    float Kp_pos[2];
    float Ki_pos[2];
    float Kd_pos[2];
    float pos_error_sum[2];
    float prev_error[3];
    float max_horizontal_vel;

    float Kp_vel[2];
    float Ki_vel[2];
    float Kd_vel[2];
    float vel_error_sum[2];
    float prev_vel_error[2];

    // Altitude cascaded PID parameters
    float Kp_pos_z;
    float Ki_pos_z;
    float Kd_pos_z;
    float pos_z_error_sum;
    float max_vertical_vel;

    float Kp_vel_z;
    float Ki_vel_z;
    float Kd_vel_z;
    float vel_z_error_sum;
    float prev_vel_z_error;

    float windup_guard;
    int min_values[3];
    int max_values[3];

    std::mutex mutex_;

    rclcpp::CallbackGroup::SharedPtr pid_callback_group_;

    rclcpp::Publisher<swift_msgs::msg::SwiftMsgs>::SharedPtr command_pub_;
    rclcpp::Publisher<error_msg::msg::Error>::SharedPtr pos_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr heartbeat_pub_;

    rclcpp::Subscription<whycode_interfaces::msg::MarkerArray>::SharedPtr whycode_sub_;
    rclcpp_action::Server<NavToWaypoint>::SharedPtr action_server_;

    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr pid_timer_;

    void publish_heartbeat()
    {
        auto hb = std_msgs::msg::Header();
        hb.stamp = this->get_clock()->now();
        hb.frame_id = "pico_controller_active";
        heartbeat_pub_->publish(hb);
    }

    void disarm()
    {
        auto msg = swift_msgs::msg::SwiftMsgs();
        msg.rc_roll = 1000;
        msg.rc_pitch = 1000;
        msg.rc_yaw = 1000;
        msg.rc_throttle = 1000;
        msg.rc_aux4 = 1000;
        command_pub_->publish(msg);
    }

    void arm()
    {
        disarm();
        auto msg = swift_msgs::msg::SwiftMsgs();
        msg.rc_roll = 1500;
        msg.rc_pitch = 1500;
        msg.rc_yaw = 1500;
        msg.rc_throttle = 1500;
        msg.rc_aux4 = 2000;
        command_pub_->publish(msg);
    }

    void whycode_callback(const whycode_interfaces::msg::MarkerArray & msg)
    {
        if (!msg.markers.empty()) {
            std::lock_guard<std::mutex> lock(mutex_);
            drone_position[0] = msg.markers[0].position.position.x;
            drone_position[1] = msg.markers[0].position.position.y;
            drone_position[2] = msg.markers[0].position.position.z;

            if (!first_marker_received) {
                prev_drone_position[0] = drone_position[0];
                prev_drone_position[1] = drone_position[1];
                prev_drone_position[2] = drone_position[2];
                first_marker_received = true;
            }

            current_dtime = static_cast<double>(msg.header.stamp.sec) +
                            static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
        }
    }

    void pid()
    {
        if (!first_marker_received) {
            return;
        }

        float cur_pos[3];
        float cur_sp[3];
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cur_pos[0] = drone_position[0];
            cur_pos[1] = drone_position[1];
            cur_pos[2] = drone_position[2];
            cur_sp[0] = setpoint[0];
            cur_sp[1] = setpoint[1];
            cur_sp[2] = setpoint[2];
        }

        auto cmd_msg = swift_msgs::msg::SwiftMsgs();
        auto error_msg = error_msg::msg::Error();

        // Calculate position error in marker coordinates
        float error[3];
        error[0] = cur_sp[1] - cur_pos[1]; // roll axis (WhyCode Y)
        error[1] = cur_sp[0] - cur_pos[0]; // pitch axis (WhyCode X)
        error[2] = cur_sp[2] - cur_pos[2]; // throttle axis (WhyCode Z)

        error_msg.roll_error = error[0];
        error_msg.pitch_error = error[1];
        error_msg.throttle_error = error[2];
        pos_error_pub_->publish(error_msg);

        // Compute velocities and EMA filter
        float raw_vel_x = (cur_pos[0] - prev_drone_position[0]) / dt;
        float raw_vel_y = (cur_pos[1] - prev_drone_position[1]) / dt;
        prev_drone_position[0] = cur_pos[0];
        prev_drone_position[1] = cur_pos[1];

        actual_vel_x = 0.35f * raw_vel_x + 0.65f * actual_vel_x;
        actual_vel_y = 0.35f * raw_vel_y + 0.65f * actual_vel_y;

        float actual_vel_lat[2] = { actual_vel_y, actual_vel_x };

        float out[2] = {0.0f, 0.0f};
        for (int i = 0; i < 2; i++)
        {
            // Conditional integration anti-windup: only integrate near setpoint
            if (std::abs(error[i]) < 0.6f) {
                pos_error_sum[i] += error[i] * dt;
                pos_error_sum[i] = std::clamp(pos_error_sum[i], -windup_guard, windup_guard);
            } else {
                pos_error_sum[i] = 0.0f;
            }

            float pos_derivative = (error[i] - prev_error[i]) / dt;
            prev_error[i] = error[i];

            float desired_vel = (Kp_pos[i] * error[i]) + 
                                (Ki_pos[i] * pos_error_sum[i]) + 
                                (Kd_pos[i] * pos_derivative);

            desired_vel = std::clamp(desired_vel, -max_horizontal_vel, max_horizontal_vel);

            float vel_error = desired_vel - actual_vel_lat[i];
            vel_error_sum[i] += vel_error * dt;
            vel_error_sum[i] = std::clamp(vel_error_sum[i], -windup_guard, windup_guard);

            float vel_derivative = (vel_error - prev_vel_error[i]) / dt;
            prev_vel_error[i] = vel_error;

            out[i] = (Kp_vel[i] * vel_error) + 
                     (Ki_vel[i] * vel_error_sum[i]) + 
                     (Kd_vel[i] * vel_derivative);
        }

        // Altitude cascaded control
        float raw_vel_z = (cur_pos[2] - prev_drone_position[2]) / dt;
        prev_drone_position[2] = cur_pos[2];

        actual_vel_z = 0.25f * raw_vel_z + 0.75f * actual_vel_z;

        if (std::abs(error[2]) < 1.0f) {
            pos_z_error_sum += error[2] * dt;
            pos_z_error_sum = std::clamp(pos_z_error_sum, -windup_guard, windup_guard);
        } else {
            pos_z_error_sum = 0.0f;
        }

        float pos_z_derivative = (error[2] - prev_error[2]) / dt;
        prev_error[2] = error[2];

        float desired_vel_z = (Kp_pos_z * error[2]) + 
                              (Ki_pos_z * pos_z_error_sum) + 
                              (Kd_pos_z * pos_z_derivative);

        desired_vel_z = std::clamp(desired_vel_z, -max_vertical_vel, max_vertical_vel);

        float vel_z_error = desired_vel_z - actual_vel_z;
        vel_z_error_sum += vel_z_error * dt;
        vel_z_error_sum = std::clamp(vel_z_error_sum, -windup_guard, windup_guard);

        float vel_z_derivative = (vel_z_error - prev_vel_z_error) / dt;
        prev_vel_z_error = vel_z_error;

        float out_throttle = (Kp_vel_z * vel_z_error) + 
                             (Ki_vel_z * vel_z_error_sum) + 
                             (Kd_vel_z * vel_z_derivative);

        int target_roll     = static_cast<int>(1500.0f + out[0]);
        int target_pitch    = static_cast<int>(1500.0f + out[1]);
        int target_throttle = static_cast<int>(1500.0f - out_throttle);

        cmd_msg.rc_roll     = std::clamp(target_roll, min_values[0], max_values[0]);
        cmd_msg.rc_pitch    = std::clamp(target_pitch, min_values[1], max_values[1]);
        cmd_msg.rc_throttle = std::clamp(target_throttle, min_values[2], max_values[2]);
        cmd_msg.rc_yaw      = 1500; // Hold yaw_rate = 0.0 rigidly in simulator
        cmd_msg.rc_aux4     = 2000;

        command_pub_->publish(cmd_msg);
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavToWaypoint::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        // Execute in a separate detached worker thread
        std::thread{std::bind(&WayPointServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal...");
        const auto goal = goal_handle->get_goal();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            setpoint[0] = static_cast<float>(goal->waypoint.position.x);
            setpoint[1] = static_cast<float>(goal->waypoint.position.y);
            setpoint[2] = static_cast<float>(goal->waypoint.position.z);

            // Reset loop integrals and errors on new waypoint transition
            pos_error_sum[0] = 0.0f; pos_error_sum[1] = 0.0f;
            vel_error_sum[0] = 0.0f; vel_error_sum[1] = 0.0f;
            prev_error[0] = setpoint[1] - drone_position[1];
            prev_error[1] = setpoint[0] - drone_position[0];
            prev_vel_error[0] = 0.0f; prev_vel_error[1] = 0.0f;

            pos_z_error_sum = 0.0f;
            vel_z_error_sum = 0.0f;
            prev_error[2] = setpoint[2] - drone_position[2];
            prev_vel_z_error = 0.0f;
        }

        RCLCPP_INFO(this->get_logger(), "New Waypoint Set: [%.2f, %.2f, %.2f]",
            goal->waypoint.position.x, goal->waypoint.position.y, goal->waypoint.position.z);

        auto feedback = std::make_shared<NavToWaypoint::Feedback>();
        auto result = std::make_shared<NavToWaypoint::Result>();

        double start_time;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            start_time = current_dtime;
        }

        double point_in_sphere_start_time = -1.0;
        double time_inside_sphere = 0.0;
        double max_time_inside_sphere = 0.0;

        rclcpp::Rate loop_rate(20);

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            float cur_x, cur_y, cur_z;
            float target_x, target_y, target_z;
            double now_dtime;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                cur_x = drone_position[0];
                cur_y = drone_position[1];
                cur_z = drone_position[2];
                target_x = setpoint[0];
                target_y = setpoint[1];
                target_z = setpoint[2];
                now_dtime = current_dtime;
            }

            feedback->current_waypoint.pose.position.x = cur_x;
            feedback->current_waypoint.pose.position.y = cur_y;
            feedback->current_waypoint.pose.position.z = cur_z;
            feedback->current_waypoint.header.stamp.sec = static_cast<int>(time_inside_sphere);

            goal_handle->publish_feedback(feedback);

            // Hold within ±0.4 whycode units on all 3 axes
            bool inside = (std::abs(cur_x - target_x) <= 0.4f) &&
                          (std::abs(cur_y - target_y) <= 0.4f) &&
                          (std::abs(cur_z - target_z) <= 0.4f);

            if (!inside) {
                if (point_in_sphere_start_time >= 0.0) {
                    RCLCPP_INFO(this->get_logger(), "Drone drifted out of tolerance");
                    point_in_sphere_start_time = -1.0;
                    time_inside_sphere = 0.0;
                }
            } else {
                if (point_in_sphere_start_time < 0.0) {
                    point_in_sphere_start_time = now_dtime;
                    RCLCPP_INFO(this->get_logger(), "Drone inside tolerance for 1st time");
                } else {
                    time_inside_sphere = now_dtime - point_in_sphere_start_time;
                }
            }

            if (time_inside_sphere > max_time_inside_sphere) {
                max_time_inside_sphere = time_inside_sphere;
            }

            if (time_inside_sphere >= 3.0) {
                RCLCPP_INFO(this->get_logger(), "Hovered inside tolerance for 3 seconds! Waypoint reached.");
                break;
            }

            loop_rate.sleep();
        }

        result->hov_time = static_cast<int32_t>(std::round(current_dtime - start_time));
        goal_handle->succeed(result);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto waypoint_server = std::make_shared<WayPointServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(waypoint_server);

    try {
        executor.spin();
    } catch (const std::exception & e) {
        RCLCPP_INFO(waypoint_server->get_logger(), "Shutting down waypoint server.\n");
    }

    rclcpp::shutdown();
    return 0;
}
