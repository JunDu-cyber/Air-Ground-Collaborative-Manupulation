#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <Eigen/Dense>

class StuckRecovery {
public:
    StuckRecovery(ros::NodeHandle& nh) {
        nh.param("stuck_vel_threshold", vel_thresh_, 0.12);
        nh.param("stuck_duration", stuck_duration_, 2.5);
        nh.param("recovery_distance", recovery_dist_, 1.2);
        nh.param("recovery_height_gain", recovery_height_, 0.5);
        nh.param("check_rate", check_rate_, 10.0);
        nh.param("recovery_cooldown", recovery_cooldown_, 8.0);

        odom_sub_ = nh.subscribe("/mavros/local_position/odom", 10,
                                 &StuckRecovery::odomCallback, this);
        // Subscribe to the elevated goal actually used by the planner
        goal_sub_ = nh.subscribe("/goal_elevated", 10,
                                 &StuckRecovery::goalCallback, this);
        recovery_goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
            "/goal_elevated", 1);
        stuck_flag_pub_ = nh.advertise<std_msgs::Bool>("/planning/is_stuck", 1);

        timer_ = nh.createTimer(ros::Duration(1.0 / check_rate_),
                                &StuckRecovery::checkStuck, this);

        is_stuck_ = false;
        in_recovery_ = false;
        goal_active_ = false;
        low_vel_start_time_ = ros::Time(0);
        last_recovery_time_ = ros::Time(0);
        last_goal_time_ = ros::Time(0);
        has_odom_ = false;

        ROS_INFO("[StuckRecovery] Started. vel_thresh=%.2f, duration=%.1fs, cooldown=%.1fs",
                 vel_thresh_, stuck_duration_, recovery_cooldown_);
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_pos_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                        msg->pose.pose.position.y,
                                        msg->pose.pose.position.z);
        current_vel_ = Eigen::Vector3d(msg->twist.twist.linear.x,
                                        msg->twist.twist.linear.y,
                                        msg->twist.twist.linear.z);
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        current_yaw_ = atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
        has_odom_ = true;
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        Eigen::Vector3d goal_pos(msg->pose.position.x,
                                  msg->pose.position.y,
                                  msg->pose.position.z);
        // Ignore our own recovery goals (published to the same topic)
        if (in_recovery_) return;

        goal_active_ = true;
        last_goal_time_ = ros::Time::now();
        goal_pos_ = goal_pos;
        ROS_INFO("[StuckRecovery] Goal received: (%.2f, %.2f, %.2f)",
                 goal_pos(0), goal_pos(1), goal_pos(2));
    }

    void checkStuck(const ros::TimerEvent&) {
        if (!has_odom_) return;

        double vel_norm = current_vel_.norm();
        std_msgs::Bool flag;

        // If drone is moving, reset stuck timer and check goal proximity
        if (vel_norm >= vel_thresh_) {
            low_vel_start_time_ = ros::Time(0);
        }

        // Check if goal reached (drone near goal while goal is active)
        if (goal_active_) {
            double dist_to_goal = (current_pos_ - goal_pos_).norm();
            double goal_age = (ros::Time::now() - last_goal_time_).toSec();
            // Goal reached OR goal timed out (stale goal)
            if (dist_to_goal < 1.5 || goal_age > 30.0) {
                if (dist_to_goal < 1.5) {
                    ROS_INFO("[StuckRecovery] Goal reached (dist=%.2f).", dist_to_goal);
                } else {
                    ROS_INFO("[StuckRecovery] Goal timed out (age=%.1fs).", goal_age);
                }
                goal_active_ = false;
                low_vel_start_time_ = ros::Time(0);
            }
        }

        // Only detect stuck if there's an active goal (user told drone to go somewhere)
        if (!goal_active_) {
            flag.data = false;
            stuck_flag_pub_.publish(flag);
            return;
        }

        // Check recovery cooldown
        double cooldown_elapsed = (ros::Time::now() - last_recovery_time_).toSec();
        bool in_cooldown = (last_recovery_time_ != ros::Time(0) &&
                            cooldown_elapsed < recovery_cooldown_);

        // Wait a bit after goal received before checking (drone needs time to accelerate)
        double goal_age = (ros::Time::now() - last_goal_time_).toSec();
        if (goal_age < 2.0) {
            flag.data = false;
            stuck_flag_pub_.publish(flag);
            return;
        }

        if (vel_norm < vel_thresh_ && !in_cooldown && !in_recovery_) {
            if (low_vel_start_time_ == ros::Time(0)) {
                low_vel_start_time_ = ros::Time::now();
            }
            double duration = (ros::Time::now() - low_vel_start_time_).toSec();

            if (duration > stuck_duration_) {
                ROS_WARN("[StuckRecovery] STUCK DETECTED! vel=%.3f, duration=%.1fs",
                         vel_norm, duration);
                is_stuck_ = true;
                in_recovery_ = true;
                executeRecovery();
            }
        }

        flag.data = is_stuck_;
        stuck_flag_pub_.publish(flag);
    }

    void executeRecovery() {
        Eigen::Vector3d backward_dir(-cos(current_yaw_), -sin(current_yaw_), 0.0);
        Eigen::Vector3d recovery_dir = backward_dir * recovery_dist_;
        recovery_dir(2) = recovery_height_;
        Eigen::Vector3d recovery_target = current_pos_ + recovery_dir;

        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "world";
        goal.pose.position.x = recovery_target(0);
        goal.pose.position.y = recovery_target(1);
        goal.pose.position.z = recovery_target(2);
        goal.pose.orientation.w = 1.0;

        recovery_goal_pub_.publish(goal);
        ROS_WARN("[StuckRecovery] Recovery goal: (%.2f, %.2f, %.2f)",
                 recovery_target(0), recovery_target(1), recovery_target(2));

        // Update goal tracking so recovery goal becomes the active goal
        goal_active_ = true;
        last_goal_time_ = ros::Time::now();
        goal_pos_ = recovery_target;
        last_recovery_time_ = ros::Time::now();
        recovery_start_pos_ = current_pos_;

        ros::Duration(5.0).sleep();
        in_recovery_ = false;
        is_stuck_ = false;
        low_vel_start_time_ = ros::Time::now();
    }

    ros::Subscriber odom_sub_, goal_sub_;
    ros::Publisher recovery_goal_pub_, stuck_flag_pub_;
    ros::Timer timer_;

    Eigen::Vector3d current_pos_, current_vel_, goal_pos_, recovery_start_pos_;
    double current_yaw_;
    double vel_thresh_, stuck_duration_, recovery_dist_, recovery_height_;
    double check_rate_, recovery_cooldown_;
    ros::Time low_vel_start_time_, last_recovery_time_, last_goal_time_;
    bool is_stuck_, in_recovery_, goal_active_, has_odom_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "stuck_recovery_node");
    ros::NodeHandle nh("~");
    StuckRecovery node(nh);
    ros::spin();
    return 0;
}
