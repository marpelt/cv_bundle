#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>

#include <cv_msg/CV_msg.h>
#include <cv_msg/Landolt_Detection.h>

struct Gaps {
    std::vector<float> angles;
    std::vector<float> radius;
    std::vector<cv::Point2f> centers;
};

float magnitudePoint(const cv::Point2f& diff)
{
    return std::sqrt(diff.dot(diff));
}

cv::Point2f normalizePoint(const cv::Point2f& diff)
{
    return diff / magnitudePoint(diff);
}

float angleBetween(cv::Point2f origin, cv::Point2f dest)
{
    float dot = origin.x * dest.x + origin.y * dest.y;  
    float det = origin.x * dest.y - origin.y * dest.x;  
    return std::atan2(det, dot) * (float)(180.0 / M_PI) + 180.0f;  
}

class LandoltDetector
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    ros::Subscriber sub_camera_;
    ros::Publisher cv_msg_pub_;

    std::string camera_topic_;
    std::string cv_msg_topic_;
    int min_edge_;
    float min_ratio_circle_;
    int min_depth_;
    int camera_id_;

    void findLandoltGaps(const cv::Mat &imageRaw, Gaps& gaps)
    {
        cv::Mat thresholdMat;
        cv::cvtColor(imageRaw, thresholdMat, cv::COLOR_BGR2GRAY);
        cv::blur(thresholdMat, thresholdMat, cv::Size(3, 3)); 
        cv::threshold(thresholdMat, thresholdMat, 140, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours; 
        cv::findContours(thresholdMat, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        for (auto &contour : contours) {
            if (contour.size() > min_edge_)
            {
                std::vector<cv::Point> hull;
                cv::convexHull(contour, hull, true, true);
                double hullArea = cv::contourArea(hull);
                
                float contourRadius;
                cv::Point2f contourCenter;
                cv::minEnclosingCircle(contour, contourCenter, contourRadius);
                double minArea = contourRadius * contourRadius * M_PI;

                if (hullArea / minArea > min_ratio_circle_)
                {
                    std::vector<cv::Vec4i> defects;
                    std::vector<int> hullsI;
                    cv::convexHull(contour, hullsI, true, false);
                    cv::convexityDefects(contour, hullsI, defects);

                    std::vector<cv::Vec4i> deepDefects;
                    for (const auto &v : defects) {
                        float depth = (float)v[3] / 256.0f;
                        if (depth > min_depth_)
                        {
                            deepDefects.push_back(v);
                        }
                    }

                    if (deepDefects.size() == 1)
                    {
                        const cv::Vec4i& v = deepDefects[0];

                        int startidx = v[0];
                        int endidx = v[1];
                        int faridx = v[2];

                        std::vector<cv::Point> points;
                        points.emplace_back(contour[startidx]);
                        points.emplace_back(contour[endidx]);
                        
                        float defectRadius;
                        cv::Point2f defectCenter;
                        cv::minEnclosingCircle(points, defectCenter, defectRadius);

                        cv::Point2f dir = normalizePoint(cv::Point2f(contour[faridx]) - defectCenter);
                        defectCenter += dir * defectRadius;

                        float defectAngle = angleBetween(dir, cv::Point2f(1, 0));
                        
                        gaps.angles.push_back(defectAngle);
                        gaps.radius.push_back(defectRadius);
                        gaps.centers.push_back(defectCenter);
                    }
                }
            }   
        }
    }

public:
    LandoltDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh) 
        : nh_(nh), private_nh_(pnh)
    {
        private_nh_.param<std::string>("camera_topic", camera_topic_, "/screen/camera/image_raw");
        private_nh_.param<std::string>("cv_msg_topic", cv_msg_topic_, "cv_bundle");
        private_nh_.param<int>("camera_id", camera_id_, 0);

        min_edge_ = 12;
        min_ratio_circle_ = 0.8f;
        min_depth_ = 10;

        sub_camera_ = nh_.subscribe(camera_topic_, 1, &LandoltDetector::image_callback, this);
        cv_msg_pub_ = nh_.advertise<cv_msg::CV_msg>(cv_msg_topic_ + "/landolt", 1);
    }

    void image_callback(const sensor_msgs::ImageConstPtr &image_msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge Ausnahme: %s", e.what());
            return;
        }

        int img_width = cv_ptr->image.cols;
        int img_height = cv_ptr->image.rows;

        Gaps gaps;  
        findLandoltGaps(cv_ptr->image, gaps);

        cv_msg::CV_msg out_msg;
        out_msg.header.stamp = ros::Time::now();
        out_msg.header.frame_id = "placeholder_frame";
        out_msg.camera_id = camera_id_;

        for (size_t i = 0; i < gaps.angles.size(); ++i)
        {
            cv_msg::Landolt_Detection detection;
            detection.angle = gaps.angles[i];
            detection.radius = gaps.radius[i] / std::min(img_width, img_height);
            detection.center_x = gaps.centers[i].x / img_width;
            detection.center_y = gaps.centers[i].y / img_height;
            
            out_msg.landolt_detections.push_back(detection);
        }

        if (gaps.angles.size() > 0)
        {
            cv_msg_pub_.publish(out_msg);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cv_landolt_detector");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    LandoltDetector detector(nh, private_nh);

    ros::spin();
    return 0;
}