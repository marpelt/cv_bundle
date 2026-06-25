#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from cv_msg.msg import CV_msg

class Visualizer:
    def __init__(self):
        self.camera_id = rospy.get_param('~camera_id', 0)
        self.cv_base_topic = rospy.get_param('~input_cv_data', '/cv_bundle')
        
        self.bridge = CvBridge()
        
        self.fade_start = 3.0
        self.fade_duration = 2.0
        self.total_timeout = self.fade_start + self.fade_duration

        self.data_store = {
            'qr': ([], rospy.Time(0)),
            'april': ([], rospy.Time(0)),
            'hazmat': ([], rospy.Time(0)),
            'image': ([], rospy.Time(0)),
            'landolt': ([], rospy.Time(0)),
            'rotation': ([], rospy.Time(0))
        }

        self.pub = rospy.Publisher(f'{self.cv_base_topic}/viz/{self.camera_id}', Image, queue_size=10)
        self.image_sub = rospy.Subscriber("input_image", CompressedImage, self.image_callback)

        topics = ['qr', 'april', 'hazmat', 'image', 'landolt', 'rotation']
        for t in topics:
            rospy.Subscriber(f"{self.cv_base_topic}/{t}", CV_msg, self.cv_callback, t)

    def cv_callback(self, msg, detection_type):
        if msg.camera_id == self.camera_id:
            attr_name = f"{detection_type}_detections"
            if hasattr(msg, attr_name):
                self.data_store[detection_type] = (getattr(msg, attr_name), rospy.get_rostime())

    def get_alpha(self, last_time):
        age = (rospy.get_rostime() - last_time).to_sec()
        if age < self.fade_start:
            return 1.0
        elif age < self.total_timeout:
            return 1.0 - ((age - self.fade_start) / self.fade_duration)
        else:
            return 0.0

    def draw_scaled_bbox(self, img, bbox, label, color, w, h, alpha):
        if alpha <= 0: return
        
        overlay = img.copy()
        
        x1 = int((bbox.cx - bbox.width / 2) * w)
        y1 = int((bbox.cy - bbox.height / 2) * h)
        x2 = int((bbox.cx + bbox.width / 2) * w)
        y2 = int((bbox.cy + bbox.height / 2) * h)
        
        cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(overlay, (x1, y1 - th - 10), (x1 + tw, y1), color, -1)
        cv2.putText(overlay, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)

    def image_callback(self, img_msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg, "bgr8")
            h, w = cv_image.shape[:2]

            for det_type, (detections, last_time) in self.data_store.items():
                alpha = self.get_alpha(last_time)
                if alpha <= 0: continue

                for d in detections:
                    if det_type == 'landolt':
                        px, py = int(d.center_x * w), int(d.center_y * h)
                        pr = max(10, int(d.radius * w))
                        overlay = cv_image.copy()
                        cv2.circle(overlay, (px, py), pr, (0, 255, 0), 2)
                        cv2.putText(overlay, f"{int(d.angle)}deg", (px + 10, py), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        cv2.addWeighted(overlay, alpha, cv_image, 1 - alpha, 0, cv_image)
                    else:
                        label = getattr(d, 'content', getattr(d, 'data', getattr(d, 'direction', "")))
                        colors = {
                            'qr': (255, 0, 0), 'april': (0, 255, 255), 
                            'hazmat': (0, 0, 255), 'image': (255, 255, 0), 
                            'rotation': (255, 0, 255)
                        }
                        self.draw_scaled_bbox(cv_image, d.bbox, f"{label}", colors.get(det_type, (255,255,255)), w, h, alpha)

            self.pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        except Exception as e:
            rospy.logerr(f"Visualizer Error: {e}")

if __name__ == '__main__':
    rospy.init_node('cv_visualizer_node')
    Visualizer()
    rospy.spin()