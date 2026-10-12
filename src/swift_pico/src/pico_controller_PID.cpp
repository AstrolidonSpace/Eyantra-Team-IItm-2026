/*This cpp file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given dummy.
This node publishes and subscribes the following topics:

        PUBLICATIONS            SUBSCRIPTIONS
        /drone_command          /whycode_node/markers
        /pos_error              /throttle_pid
                                /pitch_pid
                                /roll_pid
*/

#include <chrono>
#include <thread>
#include <functional>
#include <memory>
#include <algorithm>

#include "whycode_interfaces/msg/marker_array.hpp"
#include "swift_msgs/msg/swift_msgs.hpp"
#include "controller_msg/msg/pid_tune.hpp"
#include "error_msg/msg/error.hpp"
#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

struct CMD
{
    int rc_roll;
    int rc_pitch;
    int rc_yaw;
    int rc_throttle;
    int rc_aux4;
};

struct ERROR
{
    float roll_error;
    float pitch_error;
    float throttle_error;
    float yaw_error;
};

class Swift_Pico : public rclcpp::Node
{
public:
    Swift_Pico() : Node("pico_controller")
    {
        drone_position[0] = 0.0f;
        drone_position[1] = 0.0f;
        drone_position[2] = 0.0f;

        prev_drone_position[0] = 0.0f;
        prev_drone_position[1] = 0.0f;
        prev_drone_position[2] = 0.0f;
        actual_vel_x = 0.0f;
        actual_vel_y = 0.0f;
        actual_vel_z = 0.0f;

        // Target setpoint [x, y, z] for Task 2A
        setpoint[0] = 1.5f;
        setpoint[1] = -1.5f;
        setpoint[2] = 13.0f;

        cmd.rc_roll = 1500;
        cmd.rc_pitch = 1500;
        cmd.rc_yaw = 1500;
        cmd.rc_throttle = 1500;
        cmd.rc_aux4 = 1500;

        // Roll and Pitch position loop gains (outer loop)
        Kp_pos[0] = 66.0f;  Kp_pos[1] = 66.0f;
        Ki_pos[0] = 0.016f; Ki_pos[1] = 0.016f;
        Kd_pos[0] = 90.0f;  Kd_pos[1] = 90.0f;
        pos_error_sum[0] = 0.0f; pos_error_sum[1] = 0.0f;

        // Roll and Pitch velocity loop gains (inner loop)
        Kp_vel[0] = 25.0f; Kp_vel[1] = 25.0f;
        Ki_vel[0] = 0.0f;  Ki_vel[1] = 0.0f;
        Kd_vel[0] = 0.0f;  Kd_vel[1] = 0.0f;
        vel_error_sum[0] = 0.0f; vel_error_sum[1] = 0.0f;
        prev_vel_error[0] = 0.0f; prev_vel_error[1] = 0.0f;

        // Default gain arrays
        Kp[0] = 66.0f;  Kp[1] = 66.0f;  Kp[2] = 3.0f;
        Ki[0] = 0.016f; Ki[1] = 0.016f; Ki[2] = 0.045f;
        Kd[0] = 90.0f;  Kd[1] = 90.0f;  Kd[2] = 2.2f;

        // Altitude outer loop gains (position to vertical velocity)
        Kp_pos_z = 3.0f;
        Ki_pos_z = 0.045f;
        Kd_pos_z = 2.2f;
        pos_z_error_sum = 0.0f;

        // Altitude inner loop gains (vertical velocity to throttle)
        Kp_vel_z = 22.0f;
        Ki_vel_z = 2.0f;
        Kd_vel_z = 0.0f;
        vel_z_error_sum = 0.0f;
        prev_vel_z_error = 0.0f;

        for (int i = 0; i < 3; i++) {
            error[i] = 0.0f;
            prev_error[i] = 0.0f;
            error_sum[i] = 0.0f;
            max_values[i] = 2000;
            min_values[i] = 1000;
        }

        sample_time = 33ms;
        dt = std::chrono::duration<float>(sample_time).count();

        // Publishers
        command_pub = this->create_publisher<swift_msgs::msg::SwiftMsgs>("/drone_command", 10);
        pos_error_pub = this->create_publisher<error_msg::msg::Error>("/pos_error", 10);
        heartbeat_pub = this->create_publisher<std_msgs::msg::Header>("/controller_heartbeat", 10);

        // Subscribers
        whycode_sub = this->create_subscription<whycode_interfaces::msg::MarkerArray>(
            "/whycode_node/markers", 1, std::bind(&Swift_Pico::whycode_callback, this, _1));
        
        // Tuning GUI subscribers
        throttle_pid_sub = this->create_subscription<controller_msg::msg::PIDTune>(
            "/throttle_pid", 1, std::bind(&Swift_Pico::altitude_set_pid, this, _1));
        pitch_pid_sub = this->create_subscription<controller_msg::msg::PIDTune>(
            "/pitch_pid", 1, std::bind(&Swift_Pico::pitch_set_pid, this, _1));
        roll_pid_sub = this->create_subscription<controller_msg::msg::PIDTune>(
            "/roll_pid", 1, std::bind(&Swift_Pico::roll_set_pid, this, _1));

        publish_heartbeat();
        heartbeat_timer = this->create_wall_timer(100ms, std::bind(&Swift_Pico::publish_heartbeat, this));

        std::this_thread::sleep_for(500ms);
        arm();

        pid_timer = this->create_wall_timer(sample_time, std::bind(&Swift_Pico::pid, this));
    }

private:
    float drone_position[3];
    float prev_drone_position[3];
    float actual_vel_x;
    float actual_vel_y;
    float actual_vel_z;
    float setpoint[3];
    CMD cmd;

    std::chrono::milliseconds sample_time;
    float dt;

    float Kp[3];
    float Ki[3];
    float Kd[3];

    // Roll (index 0) and Pitch (index 1) cascaded PID parameters
    // Position loop (outer)
    float Kp_pos[2];
    float Ki_pos[2];
    float Kd_pos[2];
    float pos_error_sum[2];
    const float max_horizontal_vel = 2.0f;

    // Velocity loop (inner)
    float Kp_vel[2];
    float Ki_vel[2];
    float Kd_vel[2];
    float vel_error_sum[2];
    float prev_vel_error[2];

    // Altitude cascaded PID parameters
    // Position loop (outer)
    float Kp_pos_z;
    float Ki_pos_z;
    float Kd_pos_z;
    float pos_z_error_sum;
    const float max_vertical_vel = 5.0f;

    // Velocity loop (inner)
    float Kp_vel_z;
    float Ki_vel_z;
    float Kd_vel_z;
    float vel_z_error_sum;
    float prev_vel_z_error;

    float error[3];
    float prev_error[3];
    float error_sum[3];

    int min_values[3];
    int max_values[3];
    const float windup_guard = 40.0f;

    rclcpp::Publisher<swift_msgs::msg::SwiftMsgs>::SharedPtr command_pub;
    rclcpp::Publisher<error_msg::msg::Error>::SharedPtr pos_error_pub;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr heartbeat_pub;

    rclcpp::Subscription<whycode_interfaces::msg::MarkerArray>::SharedPtr whycode_sub;
    rclcpp::Subscription<controller_msg::msg::PIDTune>::SharedPtr throttle_pid_sub;
    rclcpp::Subscription<controller_msg::msg::PIDTune>::SharedPtr pitch_pid_sub;
    rclcpp::Subscription<controller_msg::msg::PIDTune>::SharedPtr roll_pid_sub;

    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    rclcpp::TimerBase::SharedPtr pid_timer;

    void publish_heartbeat()
    {
        auto hb = std_msgs::msg::Header();
        hb.stamp = this->get_clock()->now();
        hb.frame_id = "pico_controller_active";
        heartbeat_pub->publish(hb);
    }

    void disarm()
    {
        auto msg = swift_msgs::msg::SwiftMsgs();
        msg.rc_roll = 1000;
        msg.rc_pitch = 1000;
        msg.rc_yaw = 1000;
        msg.rc_throttle = 1000;
        msg.rc_aux4 = 1000;
        command_pub->publish(msg);
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
        command_pub->publish(msg);
    }

    void whycode_callback(const whycode_interfaces::msg::MarkerArray & msg)
    {
        if (!msg.markers.empty()) {
            drone_position[0] = msg.markers[0].position.position.x;
            drone_position[1] = msg.markers[0].position.position.y;
            drone_position[2] = msg.markers[0].position.position.z;
        }
    }

    // Callbacks to update PID gains from tuning topics
    void altitude_set_pid(const controller_msg::msg::PIDTune & alt)
    {
        Kp_pos_z = alt.kp;
        Ki_pos_z = alt.ki;
        Kd_pos_z = alt.kd;
    }

    void pitch_set_pid(const controller_msg::msg::PIDTune & pitch)
    {
        Kp_pos[1] = pitch.kp;
        Ki_pos[1] = pitch.ki;
        Kd_pos[1] = pitch.kd;
        Kp[1] = pitch.kp;
        Ki[1] = pitch.ki;
        Kd[1] = pitch.kd;
    }

    void roll_set_pid(const controller_msg::msg::PIDTune & roll)
    {
        Kp_pos[0] = roll.kp;
        Ki_pos[0] = roll.ki;
        Kd_pos[0] = roll.kd;
        Kp[0] = roll.kp;
        Ki[0] = roll.ki;
        Kd[0] = roll.kd;
    }

    void pid()
    {
        auto cmd_msg = swift_msgs::msg::SwiftMsgs();
        auto error_msg = error_msg::msg::Error();

        // Calculate position error in marker coordinates
        error[0] = setpoint[1] - drone_position[1]; // roll axis (Y)
        error[1] = setpoint[0] - drone_position[0]; // pitch axis (X)
        error[2] = setpoint[2] - drone_position[2]; // throttle axis (Z)

        // Publish error for logging and evaluation
        error_msg.roll_error = error[0];
        error_msg.pitch_error = error[1];
        error_msg.throttle_error = error[2];
        pos_error_pub->publish(error_msg);

        // Compute velocities and filter out sensor noise
        float raw_vel_x = (drone_position[0] - prev_drone_position[0]) / dt;
        float raw_vel_y = (drone_position[1] - prev_drone_position[1]) / dt;
        prev_drone_position[0] = drone_position[0];
        prev_drone_position[1] = drone_position[1];

        // Low-pass filter for lateral velocity estimates
        actual_vel_x = 0.35f * raw_vel_x + 0.65f * actual_vel_x;
        actual_vel_y = 0.35f * raw_vel_y + 0.65f * actual_vel_y;

        float actual_vel_lat[2] = { actual_vel_y, actual_vel_x }; // 0: Roll (Y), 1: Pitch (X)

        float out[2] = {0.0f, 0.0f};
        for (int i = 0; i < 2; i++)
        {
            // Outer loop: position error to desired velocity
            pos_error_sum[i] += error[i] * dt;
            pos_error_sum[i] = std::clamp(pos_error_sum[i], -windup_guard, windup_guard);

            float pos_derivative = (error[i] - prev_error[i]) / dt;
            prev_error[i] = error[i];

            float desired_vel = (Kp_pos[i] * error[i]) + 
                                (Ki_pos[i] * pos_error_sum[i]) + 
                                (Kd_pos[i] * pos_derivative);

            desired_vel = std::clamp(desired_vel, -max_horizontal_vel, max_horizontal_vel);

            // Inner loop: velocity error to control command
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
        float raw_vel_z = (drone_position[2] - prev_drone_position[2]) / dt;
        prev_drone_position[2] = drone_position[2];

        // Low-pass filter for vertical velocity
        actual_vel_z = 0.25f * raw_vel_z + 0.75f * actual_vel_z;

        // Outer loop: height error to desired vertical velocity
        pos_z_error_sum += error[2] * dt;
        pos_z_error_sum = std::clamp(pos_z_error_sum, -windup_guard, windup_guard);

        float pos_z_derivative = (error[2] - prev_error[2]) / dt;
        prev_error[2] = error[2];

        float desired_vel_z = (Kp_pos_z * error[2]) + 
                              (Ki_pos_z * pos_z_error_sum) + 
                              (Kd_pos_z * pos_z_derivative);

        desired_vel_z = std::clamp(desired_vel_z, -max_vertical_vel, max_vertical_vel);

        // Inner loop: velocity error to throttle command
        float vel_z_error = desired_vel_z - actual_vel_z;
        vel_z_error_sum += vel_z_error * dt;
        vel_z_error_sum = std::clamp(vel_z_error_sum, -windup_guard, windup_guard);

        float vel_z_derivative = (vel_z_error - prev_vel_z_error) / dt;
        prev_vel_z_error = vel_z_error;

        float out_throttle = (Kp_vel_z * vel_z_error) + 
                             (Ki_vel_z * vel_z_error_sum) + 
                             (Kd_vel_z * vel_z_derivative);

        // Map controller output to RC command range (center 1500)
        int target_roll     = static_cast<int>(1500.0f + out[0]);
        int target_pitch    = static_cast<int>(1500.0f + out[1]);
        int target_throttle = static_cast<int>(1500.0f - out_throttle);

        // Clamp commands within safe boundaries
        cmd_msg.rc_roll     = std::clamp(target_roll, min_values[0], max_values[0]);
        cmd_msg.rc_pitch    = std::clamp(target_pitch, min_values[1], max_values[1]);
        cmd_msg.rc_throttle = std::clamp(target_throttle, min_values[2], max_values[2]);
        cmd_msg.rc_yaw      = 1500;
        cmd_msg.rc_aux4     = 2000;

        command_pub->publish(cmd_msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Swift_Pico>());
    rclcpp::shutdown();
    return 0;
}
