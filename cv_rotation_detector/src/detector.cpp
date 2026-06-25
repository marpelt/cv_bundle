#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <cv_msg/CV_msg.h>

class RotationDetector {
public:
    RotationDetector(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
        nh_private.param<std::string>("camera_topic", input_topic_, "/screen/camera/image_raw/compressed");
        nh_private.param<std::string>("cv_msg_topic", cv_msg_topic_, "/cv_bundle");
        nh_private.param<std::string>("output_topic", output_topic_, "/motion/image/compressed");
        nh_private.param<int>("camera_id", camera_id_, 0);
        nh_private.param<int>("blur", blur_kernel_, 5);
        nh_private.param<double>("motion_threshold", motion_threshold_, 1.1); //bewegungsgeschwindigkeit als schwelle, rauschen
        nh_private.param<double>("scale_factor", scale_factor_, 0.5); //bildscale, cpulast
        nh_private.param<double>("min_area_ratio", min_area_ratio_, 0.005); // min größe kreis zum detektieren (prozent)
        nh_private.param<double>("motion_accumulator", motion_accumulator_diff, 0.98);

        image_sub_ = nh.subscribe(input_topic_, 1, &RotationDetector::imageCallback, this);
        image_pub_ = nh.advertise<sensor_msgs::CompressedImage>(output_topic_, 1);
        cv_msg_pub_ = nh.advertise<cv_msg::CV_msg>(cv_msg_topic_ + "/rotation", 1);
    }

private:
    std::string input_topic_, output_topic_, cv_msg_topic_;
    int camera_id_;
    int blur_kernel_;
    double motion_threshold_;
    double scale_factor_;
    double motion_accumulator_diff;
    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;
    ros::Publisher cv_msg_pub_;
    cv::Mat prev_gray_;
    cv::Mat motion_accumulator_;
    const int max_heatmap_value_ = 50;
    double min_area_ratio_;

    void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
        cv::Mat raw_frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (raw_frame.empty()) return;

        cv::Mat frame;
        cv::resize(raw_frame, frame, cv::Size(), scale_factor_, scale_factor_, cv::INTER_LINEAR);

        cv::Mat gray, gray_blurred;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray_blurred, cv::Size(blur_kernel_, blur_kernel_), 0);

        cv_msg::CV_msg cv_msg;
        cv_msg.header.stamp = msg->header.stamp;
        cv_msg.header.frame_id = "placeholder";
        cv_msg.camera_id = 0;

        if (prev_gray_.empty()) {
            prev_gray_ = gray_blurred.clone();
            motion_accumulator_ = cv::Mat::zeros(gray.size(), CV_32F);
            return;
        }

        cv::Mat flow;
        cv::calcOpticalFlowFarneback(prev_gray_, gray_blurred, flow, 0.5, 3, 15, 3, 5, 1.2, 0);

        cv::Mat result = frame.clone();
        float threshold_sq = motion_threshold_ * motion_threshold_;

        for (int y = 0; y < flow.rows; y++) {
            const cv::Point2f* flow_row = flow.ptr<cv::Point2f>(y);
            float* acc_row = motion_accumulator_.ptr<float>(y);
            for (int x = 0; x < flow.cols; x++) {
                const cv::Point2f& f = flow_row[x];
                if ((f.x*f.x + f.y*f.y) > threshold_sq) {
                    acc_row[x] += 3.0f;
                }
            }
        }

        motion_accumulator_ *= motion_accumulator_diff; //alte bewegungen verblassen

        cv::Mat heatmap_norm, heatmap_color, binary_mask;
        cv::threshold(motion_accumulator_, motion_accumulator_, max_heatmap_value_, max_heatmap_value_, cv::THRESH_TRUNC);
        motion_accumulator_.convertTo(heatmap_norm, CV_8U, 255.0 / max_heatmap_value_);
        cv::applyColorMap(heatmap_norm, heatmap_color, cv::COLORMAP_JET);

        cv::threshold(heatmap_norm, binary_mask, 30, 255, cv::THRESH_BINARY);
        
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(binary_mask, binary_mask, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double total_area = (double)(gray.cols * gray.rows);

        for (const auto& cnt : contours) {
            double area = cv::contourArea(cnt);
            double perimeter = cv::arcLength(cnt, true);
            if (perimeter <= 0) continue;

            double circularity = (4.0 * CV_PI * area) / (perimeter * perimeter);

            if (area > (total_area * min_area_ratio_) && circularity > 0.65) { //stellt sicher dass kreis
                
                cv::Rect roi = cv::boundingRect(cnt);
                cv::Point center(roi.x + roi.width / 2, roi.y + roi.height / 2);
                double cross_sum = 0.0;
                int count = 0;

                for (int y = roi.y; y < roi.y + roi.height; ++y) {
                    if (y < 0 || y >= flow.rows) continue;
                    const cv::Point2f* flow_ptr = flow.ptr<cv::Point2f>(y);
                    for (int x = roi.x; x < roi.x + roi.width; ++x) {
                        if (x < 0 || x >= flow.cols) continue;

                        cv::Point2f f = flow_ptr[x];
                        if ((f.x*f.x + f.y*f.y) < threshold_sq) continue;

                        cv::Point2f r = cv::Point2f(x, y) - cv::Point2f(center);
                        cross_sum += (r.x * f.y - r.y * f.x);
                        count++;
                    }
                }

                if (count > 3) {
                    cv_msg::Rotation_Detection rot_det;
                    std::string label = (cross_sum > 0) ? "CCW" : "CW";
                    cv::Scalar color = (cross_sum > 0) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
                    
                    cv::rectangle(result, roi, color, 2);
                    cv::putText(result, label, roi.tl() + cv::Point(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);

                    double img_w = static_cast<double>(frame.cols);
                    double img_h = static_cast<double>(frame.rows);

                    cv::Rect roi = cv::boundingRect(cnt);
                    rot_det.bbox.cx = (roi.x + roi.width / 2.0) / img_w;
                    rot_det.bbox.cy = (roi.y + roi.height / 2.0) / img_h;
                    rot_det.bbox.width = static_cast<double>(roi.width) / img_w;
                    rot_det.bbox.height = static_cast<double>(roi.height) / img_h;
                    rot_det.direction = label;

                    cv_msg.rotation_detections.push_back(rot_det);
                    cv_msg_pub_.publish(cv_msg);

                }
            }
        }

        cv::addWeighted(heatmap_color, 0.4, result, 0.6, 0.0, result);

        std::vector<uchar> buf;
        cv::imencode(".jpg", result, buf);
        sensor_msgs::CompressedImage out_msg;
        out_msg.header = msg->header;
        out_msg.format = "jpeg";
        out_msg.data.assign(buf.begin(), buf.end());
        image_pub_.publish(out_msg);

        prev_gray_ = gray_blurred.clone();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cv_rotation_detector");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    RotationDetector node(nh, nh_private);
    ros::spin();
    return 0;
}