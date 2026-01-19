#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <algorithm>
#include <string>

using std::placeholders::_1;

class WallFollower : public rclcpp::Node
{
public:
    WallFollower()
    : Node("wall_follower_debug"),
      prev_error_(0.0f),
      error_sum_(0.0f),
      prev_dist_(0.7f)
    {
        declare_parameter("target_distance", 0.7);
        declare_parameter("forward_speed",  1.1);
        declare_parameter("gain_p",          0.6);
        declare_parameter("gain_d",          0.8);
        declare_parameter("gain_i",          0.05);
        declare_parameter("control_mode",    "robust"); // single / multi / robust
        declare_parameter("gain_angle", 0.8);        // 角度用ゲイン
        declare_parameter("angle_diff_thresh", 0.4); // 凹み判定 [m]

        get_parameter("target_distance", target_distance_);
        get_parameter("forward_speed",  forward_speed_);
        get_parameter("gain_p",          kp_);
        get_parameter("gain_d",          kd_);
        get_parameter("gain_i",          ki_);
        get_parameter("control_mode",    control_mode_);
        get_parameter("gain_angle",   ka_);
        get_parameter("angle_diff_thresh", angle_diff_thresh_);

        sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallFollower::scanCallback, this, _1));

        pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        RCLCPP_INFO(get_logger(),
            "Wall follower started (LEFT wall, mode=%s)",
            control_mode_.c_str());
    }

private:
    // ===============================
    // scan callback
    // ===============================
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        if (control_mode_ == "single") {
            singlePointControl(scan);
        } else if (control_mode_ == "multi") {
            multiPointControl(scan);
        } else if (control_mode_ == "robust") {
            robustAngleControl(scan);
        }
    }

    // ===============================
    // 単一点制御（90°）
    // ===============================
    void singlePointControl(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        float angle = M_PI / 2.0; // 左90°
        int idx = angleToIndex(scan, angle);
        if (idx < 0) return;

        float raw_dist = scan->ranges[idx];
        if (!std::isfinite(raw_dist)) return;

        // 平滑化
        float dist = 0.7f * prev_dist_ + 0.3f * raw_dist;
        prev_dist_ = dist;

        float error = dist - target_distance_;

        pidControl(error);
        debugPrint("single", dist, error);
    }

    // ===============================
    // 複数点制御（90° + 45°）
    // ===============================
    void multiPointControl(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        int idx_s = angleToIndex(scan, M_PI / 2.0); // 90°
        int idx_f = angleToIndex(scan, M_PI / 4.0); // 45°
        if (idx_s < 0 || idx_f < 0) return;

        float d_s = scan->ranges[idx_s];
        float d_f = scan->ranges[idx_f];
        if (!std::isfinite(d_s) || !std::isfinite(d_f)) return;

        float alpha = std::atan2(
            d_f * std::cos(M_PI / 4.0) - d_s,
            d_f * std::sin(M_PI / 4.0)
        );

        float error = d_s - target_distance_;
        float combined_error = error + 0.8f * alpha;

        pidControl(combined_error);
        debugPrint("multi", d_s, combined_error);
    }

    // ===============================
    // robust 制御（複数点 + median）
    // ===============================
    void robustAngleControl(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // 基本角度
        const float ang_s = M_PI / 2.0;   // 90°
        const float ang_f = M_PI / 4.0;   // 45°
        const float ang_b = 3.0 * M_PI / 4.0; // 135°

        int idx_s = angleToIndex(scan, ang_s);
        int idx_f = angleToIndex(scan, ang_f);
        int idx_b = angleToIndex(scan, ang_b);
        if (idx_s < 0 || idx_f < 0 || idx_b < 0) return;

        float d_s = scan->ranges[idx_s];
        float d_f = scan->ranges[idx_f];
        float d_b = scan->ranges[idx_b];

        if (!std::isfinite(d_s)) return;

        // ---- 角度推定（multi と同じ式）----
        auto calc_alpha = [](float ds, float df, float theta) {
            return std::atan2(
                df * std::cos(theta) - ds,
                df * std::sin(theta)
            );
        };

        float alpha_sf = std::isfinite(d_f)
            ? calc_alpha(d_s, d_f, M_PI / 4.0)
            : 0.0f;

        float alpha_sb = std::isfinite(d_b)
            ? calc_alpha(d_s, d_b, M_PI / 4.0)
            : 0.0f;

        // ---- 信頼性判定 ----
        bool sf_ok = std::isfinite(d_f) && std::abs(alpha_sf) < angle_diff_thresh_;
        bool sb_ok = std::isfinite(d_b) && std::abs(alpha_sb) < angle_diff_thresh_;

        float alpha;
        if (sf_ok) {
            alpha = alpha_sf;          // 通常は multi と同じ
        } else if (sb_ok) {
            alpha = alpha_sb;          // 凹み回避
        } else {
            alpha = 0.0f;              // 壁が信用できない → 直進
        }

        // ---- 距離誤差（multiと同じ）----
        float error = d_s - target_distance_;
        float combined_error = error + ka_ * alpha;

        pidControl(combined_error);

        debugPrint("robust", d_s, combined_error);
    }




    // ===============================
    // PID 制御
    // ===============================
    void pidControl(float error)
    {
        float d_error = error - prev_error_;
        prev_error_ = error;

        error_sum_ += error;
        error_sum_ = std::clamp(error_sum_, -0.5f, 0.5f);

        float omega =
            kp_ * error +
            ki_ * error_sum_ -
            kd_ * d_error;

        omega = std::clamp(omega, -0.6f, 0.6f);
        float v = forward_speed_ * std::exp(-std::abs(omega));

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = v;
        cmd.angular.z = omega;
        pub_cmd_->publish(cmd);
    }

    // ===============================
    // デバッグ出力
    // ===============================
    void debugPrint(const char* mode, float dist, float error)
    {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "[%s] mode=%s dist=%.3f err=%.3f",
            "left", mode, dist, error
        );
    }

    // ===============================
    int angleToIndex(const sensor_msgs::msg::LaserScan::SharedPtr scan, float angle)
    {
        if (angle < scan->angle_min || angle > scan->angle_max)
            return -1;

        int idx = (angle - scan->angle_min) / scan->angle_increment;
        return std::clamp(idx, 0, (int)scan->ranges.size() - 1);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;

    float target_distance_;
    float forward_speed_;
    float kp_, kd_, ki_;

    float ka_;  
    float angle_diff_thresh_; 

    float prev_error_;
    float error_sum_;
    float prev_dist_;

    std::string control_mode_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollower>());
    rclcpp::shutdown();
    return 0;
}
