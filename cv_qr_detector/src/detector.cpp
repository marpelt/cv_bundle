#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>

#include <cv_msg/CV_msg.h>

class QR {
public:
    QR() : nh_(), pnh_("~") {
        std::string sub_topic, pub_topic;
        pnh_.param<std::string>("camera_topic", sub_topic, "/screen/camera/image_raw/compressed");
        pnh_.param<std::string>("cv_msg_topic", pub_topic, "/cv_bundle");
        pnh_.param<int>("camera_id", camera_id_, 0);
        
        image_sub_ = nh_.subscribe(sub_topic, 1, &QR::compressedImageCallback, this);
        cv_pub_ = nh_.advertise<cv_msg::CV_msg>(pub_topic + "/qr", 10);
        
        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
    }

    void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
        cv::Mat raw_img;
        try {
            raw_img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Fehler Dekodieren: %s", e.what());
            return;
        }

        if (raw_img.empty()) return;

        cv::Mat gray;
        cv::cvtColor(raw_img, gray, cv::COLOR_BGR2GRAY);

        cv::Mat processed;
        cv::GaussianBlur(gray, processed, cv::Size(3, 3), 0);
        cv::adaptiveThreshold(processed, processed, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);

        zbar::Image zbar_img(processed.cols, processed.rows, "Y800", processed.data, processed.cols * processed.rows);
        
        cv_msg::CV_msg output_msg;
        output_msg.header.stamp = ros::Time::now();
        output_msg.header.frame_id = "placeholder_frame";
        output_msg.camera_id = camera_id_;

        if (scanner_.scan(zbar_img) > 0) {
            double img_w = static_cast<double>(raw_img.cols);
            double img_h = static_cast<double>(raw_img.rows);

            for (zbar::Image::SymbolIterator symbol = zbar_img.symbol_begin(); symbol != zbar_img.symbol_end(); ++symbol) {
                cv_msg::QR_Detection qr_det;
                qr_det.data = symbol->get_data();

                if (symbol->get_location_size() >= 4) {
                    int min_x = 10000, max_x = 0, min_y = 10000, max_y = 0;
                    for (int i = 0; i < symbol->get_location_size(); i++) {
                        int x = symbol->get_location_x(i);
                        int y = symbol->get_location_y(i);
                        min_x = std::min(min_x, x); max_x = std::max(max_x, x);
                        min_y = std::min(min_y, y); max_y = std::max(max_y, y);
                    }

                    double px_cx = (min_x + max_x) / 2.0;
                    double px_cy = (min_y + max_y) / 2.0;
                    double px_w  = max_x - min_x;
                    double px_h  = max_y - min_y;

                    qr_det.bbox.cx = px_cx / img_w;
                    qr_det.bbox.cy = px_cy / img_h;
                    qr_det.bbox.width = px_w / img_w;
                    qr_det.bbox.height = px_h / img_h;
                }
                output_msg.qr_detections.push_back(qr_det);
            }
            cv_pub_.publish(output_msg);
        }
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber image_sub_;
    ros::Publisher cv_pub_;
    zbar::ImageScanner scanner_;
    int camera_id_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cv_qr_detector");
    QR detector;
    ros::spin();
    return 0;
}