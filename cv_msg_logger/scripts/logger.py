#!/usr/bin/env python3
import rospy
import os
import csv
from datetime import datetime
from cv_msg.msg import CV_msg
import rospkg

class CVMsgLogger:
    def __init__(self):
        rospy.init_node('cv_msg_logger')
        
        self.cv_base_topic = rospy.get_param('~input_cv_data', '/cv_bundle')
        self.camera_id = rospy.get_param('~camera_id', 0)
        self.team_name = rospy.get_param('~team_name', "CJT-Robotics")
        self.nationality = rospy.get_param('~nationality', "Germany")
        self.mission = rospy.get_param('~mission', "Prelims")
        self.robot_name = rospy.get_param('~robot_name', "Error404")
        self.operation_mode = rospy.get_param('~operation_mode', "T")
        self.filename = rospy.get_param('~log_file', "RoboCup[year]-[teamname]-[mission]-[timestamp]-pois")

        self.already_logged = set()

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('cv_msg_logger')
        log_dir = os.path.join(pkg_path, 'logs')
        
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        now = datetime.now()
        year_str = now.strftime("%Y")
        time_str = now.strftime("%H:%M:%S")

        self.filename = self.filename.replace("[year]", year_str).replace("[teamname]", self.team_name).replace("[mission]", self.mission).replace("[timestamp]", time_str)
        self.filename = os.path.join(log_dir, self.filename + ".csv")

        self.metadata = [
            f"# pois",
            f"# 1.3",
            f"# {self.team_name}",
            f"# {self.nationality}",
            f"# {now.strftime('%Y-%m-%d')}",
            f"# {time_str}",
            f"# {self.mission}"
            " \n"
        ]

        self.header = ["detection", "time", "type", "name", "x", "y", "z", "robot", "mode"]
        
        self.init_csv()

        self.data_store = {
            'qr': ([], rospy.Time(0)),
            'april': ([], rospy.Time(0)),
            'hazmat': ([], rospy.Time(0)),
            'image': ([], rospy.Time(0))
        }

        self.id = 0;

        self.topics = ['qr', 'april', 'hazmat', 'image']
        for t in self.topics:
            rospy.Subscriber(f"{self.cv_base_topic}/{t}", CV_msg, self.cv_callback, t)
        
        # TODO: Subscriber für Kameraframes hinzufügen -> self.position_callback

    def init_csv(self):
        with open(self.filename, mode='w', newline='') as f:
            for line in self.metadata:
                f.write(f"{line}\n")
            writer = csv.writer(f)
            writer.writerow(self.header)

    def position_callback(self, msg):
        # TODO: Speichert in einer Variable die aktuellen Vekroen der Kameraframes
        return

    def cv_callback(self, msg, detection_type):

        identifier = (detection_type, msg.name if hasattr(msg, 'name') else "unknown")
        if identifier in self.already_logged:
            return
        self.already_logged.add(identifier)

        self.id = self.id + 1

        time = datetime.now().strftime("%H:%M:%S")

        if detection_type == "april": type = "ar_code"
        elif detection_type == "qr": type = "qr_code"
        elif detection_type == "hazmat": type = "hazmat_sign"
        elif detection_type == "image": type = "real_object"
        else: type = "unknown"

        name = msg.name if hasattr(msg, 'name') else "unknown"

        x = None # TODO from self.position_callback and calc method
        y = None # TODO from self.position_callback and calc method
        z = None # TODO from self.position_callback and calc method

        robot = self.robot_name
        mode = self.operation_mode
        
        with open(self.filename, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([detection_type, time, type, name, x, y, z, robot, mode])

if __name__ == '__main__':
    try:
        logger = CVMsgLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass