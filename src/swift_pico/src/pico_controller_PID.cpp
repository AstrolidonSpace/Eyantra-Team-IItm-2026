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

        // Target setpoint [x, y, z] from Task 2A
        setpoint[0] = 1.5f;
        setpoint[1] = -1.5f;
        setpoint[2] = 13.0f;

        cmd.rc_roll = 1500;
        cmd.rc_pitch = 1500;
        cmd.rc_yaw = 1500;
        cmd.rc_throttle = 1500;
        cmd.rc_aux4 = 1500;

        // Roll (0) and Pitch (1) standard PID gains
        Kp[0] = 0.0f;  Kp[1] = 0.0f;
        Ki[0] = 0.0f;  Ki[1] = 0.0f;
        Kd[0] = 0.0f;  Kd[1] = 0.0f;

        // [CHANGED] Outer Loop (Position PID -> Desired Velocity) tuned by GUI (/throttle_pid)
        Kp_pos_z = 0.0f;
        Ki_pos_z = 0.0f;
        Kd_pos_z = 0.0f;
        pos_z_error_sum = 0.0f;

        // [CHANGED] Inner Loop (Velocity PID -> rc_throttle) hardcoded fixed baseline
        Kp_vel_z = 15.0f;
        Ki_vel_z = 0.0f;
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
    float setpoint[3];
    CMD cmd;

    std::chrono::milliseconds sample_time;
    float dt;

    float Kp[3];
    float Ki[3];
    float Kd[3];

    // [CHANGED] Outer Loop parameters for Throttle (Z)
    float Kp_pos_z;
    float Ki_pos_z;
    float Kd_pos_z;
    float pos_z_error_sum;
    const float max_vertical_vel = 3.0f; // Velocity clamp in marker-units/sec

    // [CHANGED] Inner Loop parameters for Throttle (Z)
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

    // [CHANGED] Connect GUI sliders directly to the OUTER position loop gains
    void altitude_set_pid(const controller_msg::msg::PIDTune & alt)
    {
        Kp_pos_z = alt.kp;
        Ki_pos_z = alt.ki;
        Kd_pos_z = alt.kd;
    }

    void pitch_set_pid(const controller_msg::msg::PIDTune & pitch)
    {
        Kp[1] = pitch.kp;
        Ki[1] = pitch.ki;
        Kd[1] = pitch.kd;
    }

    void roll_set_pid(const controller_msg::msg::PIDTune & roll)
    {
        Kp[0] = roll.kp;
        Ki[0] = roll.ki;
        Kd[0] = roll.kd;
    }

    void pid()
    {
        auto cmd_msg = swift_msgs::msg::SwiftMsgs();
        auto error_msg = error_msg::msg::Error();

        // 1. Position Errors (WhyCode Units)
        error[0] = setpoint[1] - drone_position[1]; // roll_error (Y)
        error[1] = setpoint[0] - drone_position[0]; // pitch_error (X)
        error[2] = setpoint[2] - drone_position[2]; // throttle_error (Z)

        // 2. Publish /pos_error for GUI and rosbag
        error_msg.roll_error = error[0];
        error_msg.pitch_error = error[1];
        error_msg.throttle_error = error[2];
        pos_error_pub->publish(error_msg);

        // 3. Roll (Y) and Pitch (X) Position PID
        float out[2] = {0.0f, 0.0f};
        for (int i = 0; i < 2; i++)
        {
            error_sum[i] += error[i];
            error_sum[i] = std::clamp(error_sum[i], -windup_guard, windup_guard);

            float derivative = error[i] - prev_error[i];
            out[i] = (Kp[i] * error[i]) + (Ki[i] * error_sum[i]) + (Kd[i] * derivative);
            prev_error[i] = error[i];
        }

        // ================= CASCADED THROTTLE (Z) =================
        // A. Measure actual vertical velocity (units/sec)
        float actual_vel_z = (drone_position[2] - prev_drone_position[2]) / dt;
        prev_drone_position[2] = drone_position[2];

        // B. OUTER LOOP (Controlled via GUI /throttle_pid): Height Error -> Target Velocity
        pos_z_error_sum += error[2] * dt;
        pos_z_error_sum = std::clamp(pos_z_error_sum, -windup_guard, windup_guard);

        float pos_z_derivative = (error[2] - prev_error[2]) / dt;
        prev_error[2] = error[2];

        float desired_vel_z = (Kp_pos_z * error[2]) + 
                              (Ki_pos_z * pos_z_error_sum) + 
                              (Kd_pos_z * pos_z_derivative);

        // Clamp desired velocity to safe limits
        desired_vel_z = std::clamp(desired_vel_z, -max_vertical_vel, max_vertical_vel);

        // C. INNER LOOP (Fixed baseline): Velocity Error -> Throttle PWM
        float vel_z_error = desired_vel_z - actual_vel_z;
        vel_z_error_sum += vel_z_error * dt;
        vel_z_error_sum = std::clamp(vel_z_error_sum, -windup_guard, windup_guard);

        float vel_z_derivative = (vel_z_error - prev_vel_z_error) / dt;
        prev_vel_z_error = vel_z_error;

        float out_throttle = (Kp_vel_z * vel_z_error) + 
                             (Ki_vel_z * vel_z_error_sum) + 
                             (Kd_vel_z * vel_z_derivative);
        // ========================================================

        // 4. Map to Neutral 1500 Midpoint
        int target_roll     = static_cast<int>(1500.0f + out[0]);
        int target_pitch    = static_cast<int>(1500.0f + out[1]);
        int target_throttle = static_cast<int>(1500.0f - out_throttle);

        // 5. Output Clamping & Arm Flag
        cmd_msg.rc_roll     = std::clamp(target_roll, min_values[0], max_values[0]);
        cmd_msg.rc_pitch    = std::clamp(target_pitch, min_values[1], max_values[1]);
        cmd_msg.rc_throttle = std::clamp(target_throttle, min_values[2], max_values[2]);
        cmd_msg.rc_yaw      = 1500;
        cmd_msg.rc_aux4     = 2000; // Keep drone armed

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
