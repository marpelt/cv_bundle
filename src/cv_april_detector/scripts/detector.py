#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import pupil_apriltags

from cv_bridge import CvBridge, CvBridgeError
from cv_msg.msg import CV_msg, April_Detection, BBox

import cv2

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
    
        cam_input_topic = rospy.get_param('~camera_topic', '/screen/camera/image_raw/compressed')
        tag_standard = rospy.get_param('~tag_standard', 'tagStandard41h12')
        self.camera_id = rospy.get_param('~camera_id', 0)
        self.cv_msg_topic = rospy.get_param('~cv_msg_topic', '/cv_bundle')

        self.detector = pupil_apriltags.Detector(families=tag_standard)

        self.cv_pub = rospy.Publisher(self.cv_msg_topic + "/april", CV_msg, queue_size=1)
        self.image_sub = rospy.Subscriber(cam_input_topic, CompressedImage, self.image_callback)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "mono8")
            img_h, img_w = cv_image.shape[:2]

            bundle_msg = CV_msg()
            bundle_msg.header = msg.header
            bundle_msg.camera_id = self.camera_id

            tag_detections = []

            tag_detections = self.detector.detect(cv_image)

            detections = []

            if tag_detections:
                
                for detection in tag_detections:
                    det = April_Detection()
                    det.content = str(detection.tag_id)
                    det.bbox.cx = detection.center[0] / img_w
                    det.bbox.cy = detection.center[1] / img_h
                    corners = detection.corners.astype(int)
                    min_x = int(corners[:, 0].min())
                    max_x = int(corners[:, 0].max())
                    min_y = int(corners[:, 1].min())
                    max_y = int(corners[:, 1].max())
                    
                    det.bbox.width = float(max_x - min_x) / img_w
                    det.bbox.height = float(max_y - min_y) / img_h
                    
                    detections.append(det)

            bundle_msg.april_detections = detections

            if detections:
                self.cv_pub.publish(bundle_msg)

        except CvBridgeError as e:
            rospy.logerr(f'CvBridge Error: {e}')

def main():
    rospy.init_node('cv_april_detector', anonymous=True)
    process = ImageProcessor()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down apriltag detector node.")

if __name__ == '__main__':
    main()