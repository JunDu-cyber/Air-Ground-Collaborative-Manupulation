#include <cassert>
#include <cmath>
#include <chrono>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

#include "mobile_manipulator/Detection.h"
#include "mobile_manipulator/DetectObjects.h"
#include "TRT_InferenceEngine/TensorRT_InferenceEngine.h"

using namespace inference_backend;

// YOLOv8 constants
static constexpr int INPUT_W = 640;
static constexpr int INPUT_H = 640;
static constexpr int NUM_CLASSES = 80;
static constexpr int NUM_CANDIDATES = 8400;
static constexpr float CONF_THRESHOLD = 0.5f;
static constexpr float NMS_THRESHOLD = 0.45f;

// COCO class names
static const std::vector<std::string> COCO_NAMES = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"
};

static cv::Scalar getClassColor(int class_id) {
    float hue = fmod(class_id * 37.0f, 180.0f);
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(static_cast<int>(hue), 220, 255));
    cv::Mat bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    auto pixel = bgr.at<cv::Vec3b>(0, 0);
    return cv::Scalar(pixel[0], pixel[1], pixel[2]);
}

class YoloTRTNode {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer detect_srv_;
    image_transport::ImageTransport it_;
    image_transport::Publisher det_img_pub_;
    std::unique_ptr<TensorRTInferenceEngine> engine_;

public:
    YoloTRTNode() : it_(nh_) {

        TRTOptimizerParams params;
        params.fp16 = true;
        params.batch_size = 1;
        params.input_dims = nvinfer1::Dims4{1, 3, INPUT_H, INPUT_W};
        params.input_layer_name = "images";
        params.output_layer_names = {"output0"};

        engine_ = std::make_unique<TensorRTInferenceEngine>(params, 2);

        // Load the ONNX model
        std::string package_path = ros::package::getPath("mobile_manipulator");
        std::string model_path = package_path + "/models/yolov8m.onnx";
        if(!engine_->load_model(model_path)) {
            ROS_ERROR("Failed to load TensorRT engine!");
            ros::shutdown();
        }

        // Annotated image publisher for RViz
        det_img_pub_ = it_.advertise("/yolo/detections/image", 1);

        // Service — only runs inference when called
        detect_srv_ = nh_.advertiseService("/yolo/detect", &YoloTRTNode::detectCallback, this);

        ROS_INFO("YOLOv8 TensorRT Service Ready!");
        ROS_INFO("  Service:         /yolo/detect");
        ROS_INFO("  Annotated image: /yolo/detections/image");
    }

    bool detectCallback(mobile_manipulator::DetectObjects::Request  &req,
                        mobile_manipulator::DetectObjects::Response &res) {
        auto start_time = std::chrono::high_resolution_clock::now();

        // Convert request image to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return false;
        }
        cv::Mat img = cv_ptr->image;

        // Run inference
        ModelPredictions predictions = engine_->forward(img);
        if (predictions.empty()) {
            ROS_ERROR("TensorRT inference returned empty predictions");
            return false;
        }

        const std::vector<float>& output = predictions[0];
        if (output.size() != (4 + NUM_CLASSES) * NUM_CANDIDATES) {
            ROS_ERROR("Unexpected output tensor size: %zu (expected %d)",
                      output.size(), (4 + NUM_CLASSES) * NUM_CANDIDATES);
            return false;
        }

        const float scale_x = static_cast<float>(img.cols) / INPUT_W;
        const float scale_y = static_cast<float>(img.rows) / INPUT_H;

        // Parse candidates
        std::vector<cv::Rect> boxes;
        std::vector<float> confidences;
        std::vector<int> class_ids;

        for (int i = 0; i < NUM_CANDIDATES; ++i) {
            float cx = output[0 * NUM_CANDIDATES + i];
            float cy = output[1 * NUM_CANDIDATES + i];
            float w  = output[2 * NUM_CANDIDATES + i];
            float h  = output[3 * NUM_CANDIDATES + i];

            float max_score = 0.0f;
            int best_class = 0;
            for (int c = 0; c < NUM_CLASSES; ++c) {
                float score = output[(4 + c) * NUM_CANDIDATES + i];
                if (score > max_score) {
                    max_score = score;
                    best_class = c;
                }
            }

            if (max_score < CONF_THRESHOLD) continue;

            float x1 = (cx - w / 2.0f) * scale_x;
            float y1 = (cy - h / 2.0f) * scale_y;
            float bw = w * scale_x;
            float bh = h * scale_y;

            boxes.emplace_back(cv::Rect(
                static_cast<int>(x1), static_cast<int>(y1),
                static_cast<int>(bw), static_cast<int>(bh)));
            confidences.push_back(max_score);
            class_ids.push_back(best_class);
        }

        // NMS
        std::vector<int> nms_indices;
        cv::dnn::NMSBoxes(boxes, confidences, CONF_THRESHOLD, NMS_THRESHOLD, nms_indices);

        // Build response + annotated image
        for (int idx : nms_indices) {
            const cv::Rect& box = boxes[idx];
            float conf = confidences[idx];
            int class_id = class_ids[idx];
            cv::Scalar color = getClassColor(class_id);

            std::string class_name = (class_id < static_cast<int>(COCO_NAMES.size()))
                ? COCO_NAMES[class_id] : "id:" + std::to_string(class_id);

            int u = box.x + box.width / 2;
            int v = box.y + box.height / 2;

            // Fill Detection message
            mobile_manipulator::Detection det;
            det.class_name = class_name;
            det.class_id = class_id;
            det.confidence = conf;
            det.x = box.x;
            det.y = box.y;
            det.width = box.width;
            det.height = box.height;
            det.center_u = u;
            det.center_v = v;
            res.detections.push_back(det);

            // Draw on image
            std::string label = class_name + " " + std::to_string(static_cast<int>(conf * 100)) + "%";

            cv::rectangle(img, box, color, 2);

            int baseline = 0;
            cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.55, 1, &baseline);
            cv::Point label_tl(box.x, box.y - text_size.height - 6);
            cv::Point label_br(box.x + text_size.width + 4, box.y);
            cv::rectangle(img, label_tl, label_br, color, cv::FILLED);
            cv::putText(img, label, cv::Point(box.x + 2, box.y - 4),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

            cv::drawMarker(img, cv::Point(u, v), cv::Scalar(0, 255, 0),
                           cv::MARKER_CROSS, 12, 2, cv::LINE_AA);
        }

        // Convert annotated image to ROS message
        sensor_msgs::ImagePtr det_img = cv_bridge::CvImage(
            req.image.header, sensor_msgs::image_encodings::BGR8, img).toImageMsg();
        res.annotated_image = *det_img;

        // Also publish for RViz
        det_img_pub_.publish(det_img);

        auto end_time = std::chrono::high_resolution_clock::now();
        res.inference_time_ms = std::chrono::duration<float, std::milli>(end_time - start_time).count();

        ROS_INFO("Detected %zu objects in %.1f ms", res.detections.size(), res.inference_time_ms);
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trt_yolo_node");
    YoloTRTNode node;
    ros::spin();
    return 0;
}