// TensorRT wrapper for models/landmine.onnx -- YOLO11s-seg, one class: "landmine".
//
// Modelled on trt_yolo_node.cc (same TensorRTInferenceEngine, same service-not-stream shape),
// but this is a SEGMENTATION model, and that changes the decode entirely:
//
//   trt_yolo_node (YOLOv8m detect)      this node (YOLO11s-seg)
//   ------------------------------      ---------------------------------------------
//   1 output:  [1, 84, 8400]            2 outputs: [1, 37, 18900] + [1, 32, 240, 240]
//   84 = 4 box + 80 classes             37 = 4 box + 1 class + 32 MASK COEFFICIENTS
//   640x640                             960x960
//   returns boxes                       returns boxes AND A PER-INSTANCE PIXEL MASK
//
// The mask is the reason this node exists. The grasp needs the detonator located to a couple
// of millimetres and its yaw recovered; a bounding box gives neither. The model's single class
// is "landmine" -- the WHOLE mine, disc included -- so the mask is a region of interest, and
// the detonator is found inside it downstream (landmine_detector.py). The learned model
// answers "is there a mine, and where"; geometry answers "where exactly do the jaws go".
//
// Runs ON DEMAND (a service), not as a stream. The grasp loop wants a detection at the instant
// the arm reaches the look pose, not a 2 Hz feed of stale viewpoints.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "mobile_manipulator/Detection.h"
#include "mobile_manipulator/DetectLandmines.h"
#include "TRT_InferenceEngine/TensorRT_InferenceEngine.h"

using namespace inference_backend;

// All of these are READ OFF THE ONNX ITSELF (its metadata: imgsz [960,960], names {0:
// 'landmine'}, task segment). Do not "tidy" them to the YOLOv8 values -- they are different.
static constexpr int INPUT_W = 960;
static constexpr int INPUT_H = 960;
static constexpr int NUM_CLASSES = 1;        // "landmine"
static constexpr int NUM_CANDIDATES = 18900;
static constexpr int MASK_DIM = 32;          // mask coefficients per candidate
static constexpr int PROTO_H = 240;          // prototype masks are input/4
static constexpr int PROTO_W = 240;
static constexpr int ATTRS = 4 + NUM_CLASSES + MASK_DIM;   // 37, and output0 says so

// One class, and the scene contains at most one mine, so a false positive is cheap (the yellow
// gate downstream throws it out) while a miss costs the whole pick. Be generous.
//
// It is set where it is because of a measurement: on a Gazebo frame of our prop the network's
// best score was 0.2902. The model DOES recognise the prop -- it was trained on photographs of
// real mines and ours is a flat-shaded red disc with a yellow cube, so a middling score is
// exactly what you would expect -- but at the usual 0.35 the detection was being thrown away.
static constexpr float CONF_THRESHOLD = 0.20f;
static constexpr float NMS_THRESHOLD = 0.45f;
static constexpr float MASK_THRESHOLD = 0.5f;    // sigmoid output -> binary mask

// Grey used by Ultralytics' letterbox padding. Matching it matters: the network has only ever
// seen this colour in the padding, and filling with black instead invents an edge.
static const cv::Scalar LETTERBOX_GREY(114, 114, 114);

class LandmineTRTNode
{
public:
  LandmineTRTNode() : it_(nh_), pnh_("~")
  {
    TRTOptimizerParams params;
    params.fp16 = true;
    params.batch_size = 1;
    params.input_dims = nvinfer1::Dims4{1, 3, INPUT_H, INPUT_W};
    params.input_layer_name = "images";
    params.output_layer_names = {"output0", "output1"};

    engine_ = std::make_unique<TensorRTInferenceEngine>(params, 2);

    std::string model_path;
    pnh_.param<std::string>(
        "model_path",
        model_path,
        ros::package::getPath("mobile_manipulator") + "/models/landmine.onnx");

    // The FIRST call builds a TensorRT engine from the ONNX and caches it next to the model.
    // That takes a minute or two and looks like a hang; it is not. Later runs load the cache.
    ROS_INFO("[trt_landmine] loading %s (the first run BUILDS the engine; this takes a while)",
             model_path.c_str());
    if (!engine_->load_model(model_path))
    {
      ROS_ERROR("[trt_landmine] failed to load the TensorRT engine from %s", model_path.c_str());
      ros::shutdown();
      return;
    }

    det_img_pub_ = it_.advertise("/landmine/detections/image", 1);
    detect_srv_ = nh_.advertiseService("/landmine/detect", &LandmineTRTNode::detectCb, this);

    ROS_INFO("[trt_landmine] ready");
    ROS_INFO("  service:         /landmine/detect  (mobile_manipulator/DetectLandmines)");
    ROS_INFO("  annotated image: /landmine/detections/image");
  }

private:
  static float sigmoid(float x) { return 1.0f / (1.0f + std::exp(-x)); }

  bool detectCb(mobile_manipulator::DetectLandmines::Request &req,
                mobile_manipulator::DetectLandmines::Response &res)
  {
    const auto t0 = std::chrono::high_resolution_clock::now();

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::BGR8);
    }
    catch (const cv_bridge::Exception &e)
    {
      ROS_ERROR("[trt_landmine] cv_bridge: %s", e.what());
      return false;
    }
    cv::Mat img = cv_ptr->image;          // BGR, annotated in place at the end
    if (img.empty())
    {
      ROS_ERROR("[trt_landmine] empty image");
      return false;
    }

    // LETTERBOX FIRST. TensorRTInferenceEngine::forward() calls blobFromImage, which does a
    // PLAIN RESIZE to the network's 960x960 -- so a 640x480 frame gets stretched 33% vertically.
    // Ultralytics trains with aspect-preserving letterbox, and feeding it a distorted image
    // costs real score. We cannot change the engine's resize, but we can hand it something
    // already square: pad to a square canvas here, and the engine's uniform 640->960 rescale is
    // then distortion-free.
    const int side = std::max(img.cols, img.rows);
    const int pad_x = (side - img.cols) / 2;
    const int pad_y = (side - img.rows) / 2;
    cv::Mat canvas(side, side, CV_8UC3, LETTERBOX_GREY);
    img.copyTo(canvas(cv::Rect(pad_x, pad_y, img.cols, img.rows)));

    // FEED RGB. forward() calls blobFromImage with swapRB HARDCODED FALSE (it ignores
    // TRTOptimizerParams::swapRB), so whatever channel order we hand it is what the network
    // sees -- and Ultralytics models are trained on RGB. trt_yolo_node passes BGR straight
    // through and silently feeds its model swapped channels; do not copy that.
    cv::Mat rgb;
    cv::cvtColor(canvas, rgb, cv::COLOR_BGR2RGB);

    const ModelPredictions preds = engine_->forward(rgb);
    if (preds.size() < 2)
    {
      ROS_ERROR("[trt_landmine] expected 2 output tensors, got %zu", preds.size());
      return false;
    }

    // Identify the tensors BY SIZE, not by position: the engine returns them in binding order,
    // which is not guaranteed to match output_layer_names.
    const size_t kDetSize = static_cast<size_t>(ATTRS) * NUM_CANDIDATES;          // 699300
    const size_t kProtoSize = static_cast<size_t>(MASK_DIM) * PROTO_H * PROTO_W;  // 1843200
    const std::vector<float> *det = nullptr;
    const std::vector<float> *proto = nullptr;
    for (const auto &p : preds)
    {
      if (p.size() == kDetSize) det = &p;
      else if (p.size() == kProtoSize) proto = &p;
    }
    if (!det || !proto)
    {
      ROS_ERROR("[trt_landmine] output sizes %zu / %zu do not match the expected %zu / %zu. "
                "Is models/landmine.onnx still a 960x960 1-class YOLO11-seg?",
                preds[0].size(), preds[1].size(), kDetSize, kProtoSize);
      return false;
    }

    // output0 is ATTRIBUTE-MAJOR: value(attr, i) = det[attr * NUM_CANDIDATES + i].
    //
    // Network coords are in the 960x960 input, which is the SQUARE CANVAS rescaled -- not the
    // original frame. So map network -> canvas (one uniform scale, since the canvas is square)
    // -> original image (subtract the padding). Skipping the padding step puts every box and
    // every mask off by 80 px vertically on a 640x480 frame, which looks like a calibration bug.
    const float net_to_canvas = static_cast<float>(side) / INPUT_W;

    std::vector<cv::Rect> boxes;           // in ORIGINAL image pixels
    std::vector<float> confidences;
    std::vector<std::vector<float>> coeffs;

    // The single most useful number when this node reports nothing: the best score the network
    // produced ANYWHERE. If it is ~0, the model genuinely does not fire on what it is being
    // shown (a domain gap) and no threshold will save you. If it is high, the bug is in the
    // decode below. Without this you cannot tell those two apart, and they need opposite fixes.
    float best_seen = 0.0f;
    int best_i = 0;
    for (int i = 0; i < NUM_CANDIDATES; ++i)
    {
      const float s = (*det)[4 * NUM_CANDIDATES + i];
      if (s > best_seen) { best_seen = s; best_i = i; }
    }
    // DECODE SANITY. If the tensor layout is what we think it is, the best candidate's box is a
    // pixel box in the 960x960 network input: cx,cy in [0,960] and landing ON the object. If the
    // layout is wrong (e.g. the tensor is candidate-major, not attribute-major) these come out
    // as nonsense, and a near-zero "score" would just be some other attribute misread. Those two
    // failures look identical from outside and need opposite fixes, so print the numbers.
    ROS_INFO("[trt_landmine] best candidate #%d: score=%.4f  box=(cx %.1f, cy %.1f, w %.1f, "
             "h %.1f) in a %dx%d input", best_i, best_seen,
             (*det)[0 * NUM_CANDIDATES + best_i], (*det)[1 * NUM_CANDIDATES + best_i],
             (*det)[2 * NUM_CANDIDATES + best_i], (*det)[3 * NUM_CANDIDATES + best_i],
             INPUT_W, INPUT_H);

    for (int i = 0; i < NUM_CANDIDATES; ++i)
    {
      // One class, so its score IS the confidence -- no argmax over 80 classes needed.
      const float conf = (*det)[4 * NUM_CANDIDATES + i];
      if (conf < CONF_THRESHOLD) continue;

      const float cx = (*det)[0 * NUM_CANDIDATES + i];
      const float cy = (*det)[1 * NUM_CANDIDATES + i];
      const float w = (*det)[2 * NUM_CANDIDATES + i];
      const float h = (*det)[3 * NUM_CANDIDATES + i];

      cv::Rect box(static_cast<int>((cx - w / 2.0f) * net_to_canvas) - pad_x,
                   static_cast<int>((cy - h / 2.0f) * net_to_canvas) - pad_y,
                   static_cast<int>(w * net_to_canvas),
                   static_cast<int>(h * net_to_canvas));
      box &= cv::Rect(0, 0, img.cols, img.rows);      // clip; a box off the edge crashes the crop
      if (box.width <= 0 || box.height <= 0) continue;

      std::vector<float> c(MASK_DIM);
      for (int k = 0; k < MASK_DIM; ++k)
        c[k] = (*det)[(4 + NUM_CLASSES + k) * NUM_CANDIDATES + i];

      boxes.push_back(box);
      confidences.push_back(conf);
      coeffs.push_back(std::move(c));
    }

    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, confidences, CONF_THRESHOLD, NMS_THRESHOLD, keep);

    // Prototypes as a [32 x (240*240)] matrix, so a mask is one matrix-vector product.
    const cv::Mat protos(MASK_DIM, PROTO_H * PROTO_W, CV_32F,
                         const_cast<float *>(proto->data()));

    for (const int idx : keep)
    {
      const cv::Rect &box = boxes[idx];
      const float conf = confidences[idx];

      // mask = sigmoid(coeff . protos), reshaped to the prototype grid.
      const cv::Mat c(1, MASK_DIM, CV_32F, const_cast<float *>(coeffs[idx].data()));
      cv::Mat prod = c * protos;                          // 1 x (240*240); MatExpr -> Mat here,
      cv::Mat m = prod.reshape(1, PROTO_H);               // because MatExpr has no reshape()
      for (int r = 0; r < m.rows; ++r)
      {
        float *row = m.ptr<float>(r);
        for (int col = 0; col < m.cols; ++col) row[col] = sigmoid(row[col]);
      }

      // The prototypes describe the SQUARE CANVAS, so resize to the canvas and then cut the
      // original frame back out of it -- resizing straight to img.size() would silently squash
      // the mask by the very letterbox padding we just added.
      cv::Mat on_canvas;
      cv::resize(m, on_canvas, cv::Size(side, side), 0, 0, cv::INTER_LINEAR);
      const cv::Mat full = on_canvas(cv::Rect(pad_x, pad_y, img.cols, img.rows));

      // Threshold, and CROP TO THE BOX. The crop matters: prototype masks bleed well outside
      // the instance, and without it one mine's mask leaks over its neighbours.
      cv::Mat bin = cv::Mat::zeros(img.size(), CV_8UC1);
      cv::Mat roi_src = full(box);
      cv::Mat roi_dst = bin(box);
      roi_dst.setTo(255, roi_src > MASK_THRESHOLD);

      mobile_manipulator::Detection d;
      d.class_name = "landmine";
      d.class_id = 0;
      d.confidence = conf;
      d.x = box.x;
      d.y = box.y;
      d.width = box.width;
      d.height = box.height;
      d.center_u = box.x + box.width / 2;
      d.center_v = box.y + box.height / 2;
      res.detections.push_back(d);

      res.masks.push_back(*cv_bridge::CvImage(
          req.image.header, sensor_msgs::image_encodings::MONO8, bin).toImageMsg());

      // Annotate: box, label, and the mask tinted so a human can see what the net actually
      // segmented rather than trusting a rectangle.
      cv::Mat tint(img.size(), CV_8UC3, cv::Scalar(0, 200, 255));
      tint.copyTo(img, bin);
      cv::addWeighted(img, 0.75, cv_ptr->image, 0.25, 0.0, img);
      cv::rectangle(img, box, cv::Scalar(0, 200, 255), 2);
      const std::string label =
          "landmine " + std::to_string(static_cast<int>(conf * 100)) + "%";
      cv::putText(img, label, cv::Point(box.x + 2, std::max(box.y - 4, 12)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 200, 255), 2, cv::LINE_AA);
      cv::drawMarker(img, cv::Point(d.center_u, d.center_v), cv::Scalar(0, 255, 0),
                     cv::MARKER_CROSS, 14, 2, cv::LINE_AA);
    }

    sensor_msgs::ImagePtr out =
        cv_bridge::CvImage(req.image.header, sensor_msgs::image_encodings::BGR8, img).toImageMsg();
    res.annotated_image = *out;
    det_img_pub_.publish(out);

    const auto t1 = std::chrono::high_resolution_clock::now();
    res.inference_time_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
    if (res.detections.empty())
    {
      ROS_WARN("[trt_landmine] NO mine. Best score anywhere in the tensor: %.4f (threshold "
               "%.2f). If that number is near zero the model simply does not recognise what it "
               "is being shown; raising the threshold down will not help.",
               best_seen, CONF_THRESHOLD);
    }
    else
    {
      ROS_INFO("[trt_landmine] %zu mine(s) in %.1f ms (best score %.3f)",
               res.detections.size(), res.inference_time_ms, best_seen);
    }
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher det_img_pub_;
  ros::ServiceServer detect_srv_;
  std::unique_ptr<TensorRTInferenceEngine> engine_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trt_landmine_node");
  LandmineTRTNode node;
  ros::spin();
  return 0;
}
