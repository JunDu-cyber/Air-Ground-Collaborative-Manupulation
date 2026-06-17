#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class DepthPreprocessor {
public:
    DepthPreprocessor(ros::NodeHandle& nh) {
        nh.param("dilate_kernel_size", kernel_size_, 5);
        nh.param("depth_max", depth_max_, 8.0);
        nh.param("depth_min", depth_min_, 0.2);
        nh.param("output_width", output_width_, 640);
        nh.param("output_height", output_height_, 360);

        std::string depth_in_topic;
        nh.param<std::string>("depth_in_topic", depth_in_topic, "/camera/depth/image_raw");
        sub_ = nh.subscribe(depth_in_topic, 1,
                            &DepthPreprocessor::callback, this);
        std::string depth_out_topic;
        nh.param<std::string>("depth_out_topic", depth_out_topic, "/depth/image_dilated");
        pub_ = nh.advertise<sensor_msgs::Image>(depth_out_topic, 1);

        ROS_INFO("[DepthPreprocess] Started. kernel_size=%d, depth_range=[%.2f, %.2f], output=%dx%d",
                 kernel_size_, depth_min_, depth_max_, output_width_, output_height_);
    }

private:
    void callback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat depth = cv_ptr->image;

        // Filter invalid depth pixels
        if (depth.type() == CV_32FC1) {
            cv::Mat mask_invalid = (depth != depth) | (depth <= depth_min_) | (depth > depth_max_);
            depth.setTo((float)depth_max_, mask_invalid);
        } else if (depth.type() == CV_16UC1) {
            uint16_t max_val = (uint16_t)(depth_max_ * 1000);
            uint16_t min_val = (uint16_t)(depth_min_ * 1000);
            cv::Mat mask_invalid = (depth == 0) | (depth < min_val) | (depth > max_val);
            depth.setTo(max_val, mask_invalid);
        }

        // Erode depth image to expand obstacles (obstacles are dark = small depth)
        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Size(kernel_size_, kernel_size_));
        cv::Mat depth_eroded;
        cv::erode(depth, depth_eroded, kernel);

        // Ensure output is always 16UC1 (mm), which ego-planner grid_map expects.
        if (depth_eroded.type() == CV_32FC1) {
            depth_eroded.convertTo(depth_eroded, CV_16UC1, 1000.0);
        }

        if (output_width_ > 0 && output_height_ > 0
            && (depth_eroded.cols != output_width_ || depth_eroded.rows != output_height_)) {
            cv::resize(depth_eroded, depth_eroded, cv::Size(output_width_, output_height_),
                       0.0, 0.0, cv::INTER_NEAREST);
        }

        cv_ptr->image = depth_eroded;
        cv_ptr->header = msg->header;
        cv_ptr->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        pub_.publish(cv_ptr->toImageMsg());
    }

    ros::Subscriber sub_;
    ros::Publisher pub_;
    int kernel_size_;
    int output_width_, output_height_;
    double depth_max_, depth_min_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_preprocess_node");
    ros::NodeHandle nh("~");
    DepthPreprocessor node(nh);
    ros::spin();
    return 0;
}
