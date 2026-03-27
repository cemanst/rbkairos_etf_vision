#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from franka_msgs.msg import FrankaState
from cv_bridge import CvBridge

class HandEyeCalibration:
    def __init__(self):
        self.bridge = CvBridge()
        self.robot_poses = []
        self.charuco_poses = [] 
        self.current_robot_pose = None
        self.K = None
        self.D = None
        
        self.pattern_size = (9, 6) 
        self.square_size = 0.025  
        
        self.objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2) * self.square_size

        rospy.Subscriber("robot/arm/franka_state_controller/franka_states", FrankaState, self.robot_cb)
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.info_cb)

    def info_cb(self, msg):
        self.K = np.array(msg.K).reshape(3, 3)
        self.D = np.array(msg.D)

    def robot_cb(self, msg):
        self.current_robot_pose = np.array(msg.O_T_EE).reshape(4, 4).T

    def image_cb(self, msg):
        if self.K is None: return
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)

        if ret:
            cv2.drawChessboardCorners(frame, self.pattern_size, corners, ret)
            cv2.putText(frame, f"Samples: {len(self.robot_poses)}. Press 's' to save", (50, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow("Hand-Eye Calibration", frame)
        key = cv2.waitKey(1)

        if key == ord('s') and ret:
            if self.current_robot_pose is not None:
                _, rvec, tvec = cv2.solvePnP(self.objp, corners, self.K, self.D)
                R, _ = cv2.Rodrigues(rvec)
                
                T_target_cam = np.eye(4)
                T_target_cam[:3, :3] = R
                T_target_cam[:3, 3] = tvec.ravel()
                
                self.robot_poses.append(self.current_robot_pose)
                self.charuco_poses.append(T_target_cam)
                print(f"Uzorak {len(self.robot_poses)} sacuvan uspešno!")
            else:
                print("⚠️ GRESKA: Nema podataka o pozi robota! Proveri Franka drajver.")

        if key == ord('q') and len(self.robot_poses) > 10:
            self.calibrate()

    def calibrate(self):
        clean_robot = [p for p in self.robot_poses if p is not None]
        clean_cam = [p for p in self.charuco_poses if p is not None]

        if len(clean_robot) < 5:
            print("Nedovoljno uzoraka za racunanje!")
            return

        try:
            R_base_ee = [np.array(p[:3, :3], dtype=np.float64) for p in clean_robot]
            t_base_ee = [np.array(p[:3, 3], dtype=np.float64).reshape(3,1) for p in clean_robot]
            
            R_target_cam = [np.array(p[:3, :3], dtype=np.float64) for p in clean_cam]
            t_target_cam = [np.array(p[:3, 3], dtype=np.float64).reshape(3,1) for p in clean_cam]

            R_cam_ee, t_cam_ee = cv2.calibrateHandEye(
                R_base_ee, t_base_ee, 
                R_target_cam, t_target_cam,
                method=cv2.CALIB_HAND_EYE_TSAI
            )

            T_cam_ee = np.eye(4)
            T_cam_ee[:3, :3] = R_cam_ee
            T_cam_ee[:3, 3] = t_cam_ee.ravel()

            print("\n" + "="*40)
            print("USPESNA KALIBRACIJA!")
            print(T_cam_ee)
            print("="*40)
            
        except Exception as e:
            print(f"Doslo je do greske u matematici: {e}")

if __name__ == '__main__':
    rospy.init_node('calibration_node')
    calib = HandEyeCalibration()
    rospy.spin()