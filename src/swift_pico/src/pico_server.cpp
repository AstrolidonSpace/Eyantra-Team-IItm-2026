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
        drone_position[2] = 15.0f; // Ground level before takeoff

        prev_drone_position[0] = 0.0f;
        prev_drone_position[1] = 0.0f;
        prev_drone_position[2] = 15.0f;

        actual_vel_x = 0.0f;
        actual_vel_y = 0.0f;
        actual_vel_z = 0.0f;

        // Default setpoint to arena center F6
        setpoint[0] = 0.0067f;
        setpoint[1] = 0.0068f;
        setpoint[2] = 13.0f;

        current_dtime = 0.0;
        last_marker_received_time = 0.0;
        first_marker_received = false;
        is_disarmed_ = false;

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
        Ki_vel[0] = 0.0f;  Ki_vel[1] = 0.0f;
        Kd_vel[0] = 0.0f;  Kd_vel[1] = 0.0f;
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

        max_horizontal_vel = 1.7f;
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

        // Action Server (with cancel support enabled)
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

        RCLCPP_INFO(this->get_logger(), "WayPointServer C++ node initialized and armed for Task 3A");
    }

private:
    float drone_position[3] = {0.0f, 0.0f, 15.0f};
    float prev_drone_position[3] = {0.0f, 0.0f, 15.0f};
    float actual_vel_x = 0.0f;
    float actual_vel_y = 0.0f;
    float actual_vel_z = 0.0f;
    float setpoint[3] = {0.0f, 0.0f, 13.0f};
    double current_dtime = 0.0;
    double last_marker_received_time = 0.0;
    bool first_marker_received = false;
    bool is_disarmed_ = false;

    CMD cmd{};
    std::chrono::milliseconds sample_time{33ms};
    float dt{0.033f};

    // Roll (0) and Pitch (1) cascaded PID parameters
    float Kp_pos[2] = {66.0f, 66.0f};
    float Ki_pos[2] = {0.016f, 0.016f};
    float Kd_pos[2] = {90.0f, 90.0f};
    float pos_error_sum[2] = {0.0f, 0.0f};
    float prev_error[3] = {0.0f, 0.0f, 0.0f};
    float max_horizontal_vel = 2.4f;

    float Kp_vel[2] = {25.0f, 25.0f};
    float Ki_vel[2] = {0.0f, 0.0f};
    float Kd_vel[2] = {0.0f, 0.0f};
    float vel_error_sum[2] = {0.0f, 0.0f};
    float prev_vel_error[2] = {0.0f, 0.0f};

    // Altitude cascaded PID parameters
    float Kp_pos_z = 3.0f;
    float Ki_pos_z = 0.045f;
    float Kd_pos_z = 2.2f;
    float pos_z_error_sum = 0.0f;
    float max_vertical_vel = 5.0f;

    float Kp_vel_z = 22.0f;
    float Ki_vel_z = 2.0f;
    float Kd_vel_z = 0.0f;
    float vel_z_error_sum = 0.0f;
    float prev_vel_z_error = 0.0f;

    float windup_guard = 40.0f;
    int min_values[3] = {1000, 1000, 1000};
    int max_values[3] = {2000, 2000, 2000};

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
        is_disarmed_ = true;
        auto msg = swift_msgs::msg::SwiftMsgs();
        msg.rc_roll = 1500;
        msg.rc_pitch = 1500;
        msg.rc_yaw = 1500;
        msg.rc_throttle = 1000;
        msg.rc_aux4 = 1000;
        command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Drone DISARMED on ground.");
    }

    void arm()
    {
        is_disarmed_ = false;
        auto msg = swift_msgs::msg::SwiftMsgs();
        msg.rc_roll = 1500;
        msg.rc_pitch = 1500;
        msg.rc_yaw = 1500;
        msg.rc_throttle = 1500;
        msg.rc_aux4 = 2000;
        command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Drone ARMED.");
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
            last_marker_received_time = current_dtime;
        }
    }

    void pid()
    {
        if (!first_marker_received || is_disarmed_) {
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

        float raw_desired_vel[2];
        for (int i = 0; i < 2; i++)
        {
            pos_error_sum[i] += error[i] * dt;
            pos_error_sum[i] = std::clamp(pos_error_sum[i], -windup_guard, windup_guard);

            float pos_derivative = (error[i] - prev_error[i]) / dt;
            prev_error[i] = error[i];

            raw_desired_vel[i] = (Kp_pos[i] * error[i]) + 
                                 (Ki_pos[i] * pos_error_sum[i]) + 
                                 (Kd_pos[i] * pos_derivative);
        }

        float horiz_speed = std::hypot(raw_desired_vel[0], raw_desired_vel[1]);
        float desired_vel[2];
        if (horiz_speed > max_horizontal_vel) {
            float scale = max_horizontal_vel / horiz_speed;
            desired_vel[0] = raw_desired_vel[0] * scale;
            desired_vel[1] = raw_desired_vel[1] * scale;
        } else {
            desired_vel[0] = raw_desired_vel[0];
            desired_vel[1] = raw_desired_vel[1];
        }

        float out[2] = {0.0f, 0.0f};
        for (int i = 0; i < 2; i++)
        {
            float vel_error = desired_vel[i] - actual_vel_lat[i];
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
        cmd_msg.rc_yaw      = 1500;
        cmd_msg.rc_aux4     = 2000;

        command_pub_->publish(cmd_msg);
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavToWaypoint::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(this->get_logger(), "Received goal request: [%.2f, %.2f, %.2f] hover=%s",
                    goal->waypoint.position.x, goal->waypoint.position.y, goal->waypoint.position.z,
                    goal->hover ? "true" : "false");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received request to cancel active goal. Accepting cancel.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        std::thread{std::bind(&WayPointServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        if (is_disarmed_) {
            arm();
            std::this_thread::sleep_for(100ms);
        }

        const auto goal = goal_handle->get_goal();
        float target_x = static_cast<float>(goal->waypoint.position.x);
        float target_y = static_cast<float>(goal->waypoint.position.y);
        bool should_hover = goal->hover;

        bool is_f6 = (std::hypot(target_x, target_y) < 0.25f);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            setpoint[0] = target_x;
            setpoint[1] = target_y;
            setpoint[2] = 13.0f; // Cruise altitude

            // Update prev_error to prevent derivative kick from setpoint step change
            prev_error[0] = setpoint[1] - drone_position[1];
            prev_error[1] = setpoint[0] - drone_position[0];
            prev_error[2] = setpoint[2] - drone_position[2];
        }

        RCLCPP_INFO(this->get_logger(), "Executing Goal -> Target: [%.2f, %.2f, 13.00], hover=%s, is_f6=%s",
                    target_x, target_y, should_hover ? "true" : "false", is_f6 ? "true" : "false");

        auto feedback = std::make_shared<NavToWaypoint::Feedback>();
        auto result = std::make_shared<NavToWaypoint::Result>();

        rclcpp::Rate loop_rate(20); // 50 ms loop
        const double loop_dt = 0.05;

        // =========================================================================
        // CASE 1: Intermediate navigation waypoint (!should_hover)
        // Fly past at Z = 13 without stopping!
        // =========================================================================
        if (!should_hover) {
            while (rclcpp::ok()) {
                if (goal_handle->is_canceling()) {
                    float cur_x, cur_y;
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        cur_x = drone_position[0];
                        cur_y = drone_position[1];
                        setpoint[0] = cur_x;
                        setpoint[1] = cur_y;
                        setpoint[2] = 13.0f;
                    }
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Pass-through goal canceled mid-flight. Holding at [%.2f, %.2f, 13.00]",
                                cur_x, cur_y);
                    return;
                }

                float cur_x, cur_y, cur_z;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    cur_x = drone_position[0];
                    cur_y = drone_position[1];
                    cur_z = drone_position[2];
                }

                feedback->current_waypoint.pose.position.x = cur_x;
                feedback->current_waypoint.pose.position.y = cur_y;
                feedback->current_waypoint.pose.position.z = cur_z;
                feedback->current_waypoint.header.stamp.sec = 0;
                goal_handle->publish_feedback(feedback);

                float d_xy = std::hypot(cur_x - target_x, cur_y - target_y);
                float d_z = std::abs(cur_z - 13.0f);
                if (d_xy <= 0.55f && d_z <= 0.55f) {
                    RCLCPP_INFO(this->get_logger(), "Passed navigation waypoint [%.2f, %.2f] (d=%.2fm) without stopping.", target_x, target_y, d_xy);
                    result->hov_time = 0;
                    goal_handle->succeed(result);
                    return;
                }
                loop_rate.sleep();
            }
        }

        // =========================================================================
        // CASE 2: Home at F6 (Hover at Z=13 for 1s, then descend and land!)
        // =========================================================================
        if (is_f6) {
            RCLCPP_INFO(this->get_logger(), "Arrived at F6: Initiating 1s hover at Z=13, then landing...");
            double f6_hover_time = 0.0;

            while (rclcpp::ok()) {
                if (goal_handle->is_canceling()) {
                    float cur_x, cur_y;
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        cur_x = drone_position[0];
                        cur_y = drone_position[1];
                        setpoint[0] = cur_x;
                        setpoint[1] = cur_y;
                        setpoint[2] = 13.0f;
                    }
                    goal_handle->canceled(result);
                    return;
                }

                float cur_x, cur_y, cur_z;
                double now_time;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    cur_x = drone_position[0];
                    cur_y = drone_position[1];
                    cur_z = drone_position[2];
                    now_time = current_dtime;
                }

                feedback->current_waypoint.pose.position.x = cur_x;
                feedback->current_waypoint.pose.position.y = cur_y;
                feedback->current_waypoint.pose.position.z = cur_z;
                feedback->current_waypoint.header.stamp.sec = static_cast<int>(f6_hover_time);
                goal_handle->publish_feedback(feedback);

                float d_sphere = std::sqrt(
                    (cur_x - target_x) * (cur_x - target_x) +
                    (cur_y - target_y) * (cur_y - target_y) +
                    (cur_z - 13.0f) * (cur_z - 13.0f)
                );

                bool marker_valid = (now_time - last_marker_received_time < 0.3);
                if (d_sphere <= 0.40f && marker_valid) {
                    f6_hover_time += loop_dt;
                } else {
                    f6_hover_time = 0.0;
                }

                if (f6_hover_time >= 1.0) {
                    RCLCPP_INFO(this->get_logger(), "F6 Hover (1s at Z=13) complete! Descending to land...");
                    break;
                }
                loop_rate.sleep();
            }

            // Descend slowly from Z=13.0 to ground Z=15.2
            float landing_sp_z = 13.0f;
            while (rclcpp::ok()) {
                landing_sp_z = std::min(15.2f, landing_sp_z + 0.06f); // Ramp altitude down
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    setpoint[2] = landing_sp_z;
                }

                float cur_z;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    cur_z = drone_position[2];
                }

                feedback->current_waypoint.pose.position.z = cur_z;
                goal_handle->publish_feedback(feedback);

                if (cur_z >= 14.85f && landing_sp_z >= 15.1f) {
                    RCLCPP_INFO(this->get_logger(), "Drone touched ground at Z=%.2f. Settling...", cur_z);
                    std::this_thread::sleep_for(500ms);
                    disarm();
                    result->hov_time = 1;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "LANDED AT F6 SUCCESSFULLY! Mission Goal Complete.");
                    return;
                }
                loop_rate.sleep();
            }
        }

        // =========================================================================
        // CASE 3: Survivor Saving Maneuver (Visit 1s @ Z=13, Save 2s @ Z=14)
        // =========================================================================
        RCLCPP_INFO(this->get_logger(), "Starting Saving Maneuver: Stage 1 (Visit @ Z=13, 1s)...");

        // Stage 1: Visit (Z = 13, 1 second inside sphere r = 0.4)
        double visit_hold_time = 0.0;
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                float cur_x, cur_y;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    cur_x = drone_position[0];
                    cur_y = drone_position[1];
                    setpoint[0] = cur_x;
                    setpoint[1] = cur_y;
                    setpoint[2] = 13.0f;
                }
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled during Stage 1 Visit. Holding at [%.2f, %.2f, 13.00]",
                            cur_x, cur_y);
                return;
            }

            float cur_x, cur_y, cur_z;
            double now_time;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                cur_x = drone_position[0];
                cur_y = drone_position[1];
                cur_z = drone_position[2];
                now_time = current_dtime;
            }

            feedback->current_waypoint.pose.position.x = cur_x;
            feedback->current_waypoint.pose.position.y = cur_y;
            feedback->current_waypoint.pose.position.z = cur_z;
            feedback->current_waypoint.header.stamp.sec = static_cast<int>(visit_hold_time);
            goal_handle->publish_feedback(feedback);

            float d_sphere = std::sqrt(
                (cur_x - target_x) * (cur_x - target_x) +
                (cur_y - target_y) * (cur_y - target_y) +
                (cur_z - 13.0f) * (cur_z - 13.0f)
            );

            bool marker_valid = (now_time - last_marker_received_time < 0.3);
            if (d_sphere <= 0.40f && marker_valid) {
                visit_hold_time += loop_dt;
            } else {
                if (visit_hold_time > 0.0) {
                    RCLCPP_INFO(this->get_logger(), "Drifted out of Visit sphere (d=%.2f) or lost marker. Resetting hold.", d_sphere);
                }
                visit_hold_time = 0.0;
            }

            if (visit_hold_time >= 1.0) {
                RCLCPP_INFO(this->get_logger(), "Stage 1 (Visit @ Z=13) COMPLETE! Descending to Z=14 for Stage 2 (Save)...");
                break;
            }
            loop_rate.sleep();
        }

        // Transition to Stage 2: Save (descend to Z = 14, hold 2 seconds)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            setpoint[2] = 14.0f;
            prev_error[2] = setpoint[2] - drone_position[2];
        }

        double save_hold_time = 0.0;
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                float cur_x, cur_y;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    cur_x = drone_position[0];
                    cur_y = drone_position[1];
                    setpoint[0] = cur_x;
                    setpoint[1] = cur_y;
                    setpoint[2] = 13.0f;
                }
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled during Stage 2 Save. Holding at [%.2f, %.2f, 13.00]",
                            cur_x, cur_y);
                return;
            }

            float cur_x, cur_y, cur_z;
            double now_time;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                cur_x = drone_position[0];
                cur_y = drone_position[1];
                cur_z = drone_position[2];
                now_time = current_dtime;
            }

            feedback->current_waypoint.pose.position.x = cur_x;
            feedback->current_waypoint.pose.position.y = cur_y;
            feedback->current_waypoint.pose.position.z = cur_z;
            feedback->current_waypoint.header.stamp.sec = static_cast<int>(1.0 + save_hold_time);
            goal_handle->publish_feedback(feedback);

            float d_sphere = std::sqrt(
                (cur_x - target_x) * (cur_x - target_x) +
                (cur_y - target_y) * (cur_y - target_y) +
                (cur_z - 14.0f) * (cur_z - 14.0f)
            );

            bool marker_valid = (now_time - last_marker_received_time < 0.3);
            if (d_sphere <= 0.40f && marker_valid) {
                save_hold_time += loop_dt;
            } else {
                if (save_hold_time > 0.0) {
                    RCLCPP_INFO(this->get_logger(), "Drifted out of Save sphere (d=%.2f) or lost marker. Resetting hold.", d_sphere);
                }
                save_hold_time = 0.0;
            }

            if (save_hold_time >= 2.0) {
                RCLCPP_INFO(this->get_logger(), "Stage 2 (Save @ Z=14) COMPLETE! Survivor SAVED!");
                break;
            }
            loop_rate.sleep();
        }

        // Restore cruise altitude Z=13 and ascend back before completing goal
        {
            std::lock_guard<std::mutex> lock(mutex_);
            setpoint[2] = 13.0f;
            prev_error[2] = setpoint[2] - drone_position[2];
        }

        while (rclcpp::ok()) {
            float cur_z;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                cur_z = drone_position[2];
            }
            if (cur_z <= 13.45f) {
                break;
            }
            loop_rate.sleep();
        }

        result->hov_time = 3;
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
