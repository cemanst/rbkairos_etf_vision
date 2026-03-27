#!/usr/bin/env python3
from get_base_pose import Tracker3D
from rbkairos_etf_services.srv import ActionServer, ActionServerRequest
from run_grasp_action import run_my_action
import rospy
import numpy as np

class GraspController:
    def __init__(self):
        rospy.init_node("fruit_grasp_controller")
        self.buffer = []
        self.buffer_size = 25
        self.deviation_threshold = 0.015
        self.is_processing = False
        self.tracker = Tracker3D() 
        self.det = None
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.is_processing:
                self.process_detections()
            else:
                self.det = None
                self.buffer=[]
            rate.sleep()

# running actions only once detections are stable (object is stationary) 
    def process_detections(self):
        self.det = self.tracker.get_last_detection()
        det = self.det
        if det:
            self.buffer.append([det.x, det.y, det.z])
            rospy.loginfo(f"{self.buffer}")
            if len(self.buffer) > self.buffer_size: self.buffer=self.buffer[-(self.buffer_size):]
            
            if len(self.buffer) == self.buffer_size:
                std_dev = np.std(self.buffer, axis=0)
                if all(std_dev < self.deviation_threshold):
                    self.is_processing = True
                    avg_pos = np.mean(self.buffer, axis=0)
                    processing = run_my_action(avg_pos)
                    self.is_processing = False
                    self.buffer = []
        else:
            if len(self.buffer)<0:
                self.buffer = []

if __name__ == "__main__":
    controller = GraspController()