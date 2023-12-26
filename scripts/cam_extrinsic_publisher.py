#!/usr/bin/env python3
import rospy
from unld_msgs.msg import MotionInfo
import numpy as np
import rospkg
import yaml
import tf2_ros
import geometry_msgs.msg
from tf.transformations import *

class ExtrinsicPublisher():

    def __init__(self):

        rospy.Subscriber("/unld/calibration/motion_info", MotionInfo, self.motion_info_cb)
        rospy.Subscriber("/unld/cam_tilt", MotionInfo, self.cam_tilt_cb)
        self.tilt = rospy.get_param("~tilt", default=True)
        rospack = rospkg.RosPack()
        path = rospack.get_path('unld_cam_driver')

        with open(path+"/config/calib_info_for_tf.yaml", "r") as file_handle:
            extrinsic_data = yaml.full_load(file_handle)

        self.cam0_base_T_rgb_down = extrinsic_data["cam0"]["base_T_rgb_down"] 
        self.cam0_base_T_rgb_up   = extrinsic_data["cam0"]["base_T_rgb_up"]  
        self.cam0_base_T_cam_down = extrinsic_data["cam0"]["base_T_cam_down"]
        self.cam0_base_T_cam_up   = extrinsic_data["cam0"]["base_T_cam_up"]  
        self.cam1_base_T_rgb_down = extrinsic_data["cam1"]["base_T_rgb_down"] 
        self.cam1_base_T_rgb_up   = extrinsic_data["cam1"]["base_T_rgb_up"]  
        self.cam1_base_T_cam_down = extrinsic_data["cam1"]["base_T_cam_down"]
        self.cam1_base_T_cam_up   = extrinsic_data["cam1"]["base_T_cam_up"]  


        cam0_k4abase_p_k4adepth   = extrinsic_data["cam0"]["cambase_p_Kdepth"]  
        cam0_k4abase_q_k4adepth   = extrinsic_data["cam0"]["cambase_q_Kdepth"]  
        cam0_k4abase_T_k4adepth = quaternion_matrix(cam0_k4abase_q_k4adepth)
        cam0_k4abase_T_k4adepth[:3,3] = cam0_k4abase_p_k4adepth
        cam0_k4adepth_p_k4argb   = extrinsic_data["cam0"]["Kdepth_p_rgb"]  
        cam0_k4adepth_q_k4argb   = extrinsic_data["cam0"]["Kdepth_q_rgb"] 
        cam0_k4adepth_T_k4argb = quaternion_matrix(cam0_k4adepth_q_k4argb)
        cam0_k4adepth_T_k4argb[:3,3] = cam0_k4adepth_p_k4argb

        kbTd0 = np.asarray(cam0_k4abase_T_k4adepth).reshape((4,4))
        kdTr0 = np.asarray(cam0_k4adepth_T_k4argb).reshape((4,4))
        kbTr0 = kbTd0.dot(kdTr0)
        bTrd0 = np.asarray(self.cam0_base_T_rgb_down).reshape((4,4))
        bTkbd0 = bTrd0.dot(np.linalg.inv(kbTr0))
        self.cam0_base_T_k4abase_down = bTkbd0.reshape(-1)
        bTru0 = np.asarray(self.cam0_base_T_rgb_up).reshape((4,4))
        bTkbu0 = bTru0.dot(np.linalg.inv(kbTr0))
        self.cam0_base_T_k4abase_up = bTkbu0.reshape(-1)

        cam1_k4abase_p_k4adepth   = extrinsic_data["cam1"]["cambase_p_Kdepth"]  
        cam1_k4abase_q_k4adepth   = extrinsic_data["cam1"]["cambase_q_Kdepth"]  
        cam1_k4abase_T_k4adepth = quaternion_matrix(cam1_k4abase_q_k4adepth)
        cam1_k4abase_T_k4adepth[:3,3] = cam1_k4abase_p_k4adepth
        cam1_k4adepth_p_k4argb   = extrinsic_data["cam1"]["Kdepth_p_rgb"]  
        cam1_k4adepth_q_k4argb   = extrinsic_data["cam1"]["Kdepth_q_rgb"] 
        cam1_k4adepth_T_k4argb = quaternion_matrix(cam1_k4adepth_q_k4argb)
        cam1_k4adepth_T_k4argb[:3,3] = cam1_k4adepth_p_k4argb

        kbTd1 = np.asarray(cam1_k4abase_T_k4adepth).reshape((4,4))
        kdTr1 = np.asarray(cam1_k4adepth_T_k4argb).reshape((4,4))
        kbTr1 = kbTd1.dot(kdTr1)
        bTrd1 = np.asarray(self.cam1_base_T_rgb_down).reshape((4,4))
        bTkbd1 = bTrd1.dot(np.linalg.inv(kbTr1))
        self.cam1_base_T_k4abase_down = bTkbd1.reshape(-1)
        bTru1 = np.asarray(self.cam1_base_T_rgb_up).reshape((4,4))
        bTkbu1 = bTru1.dot(np.linalg.inv(kbTr1))
        self.cam1_base_T_k4abase_up = bTkbu1.reshape(-1)


        self.motion_info_subscribed = True

        rospy.logwarn("Needs to subscribe camera tilt info...") 
        
        if self.tilt:
            self.cam0_tilt = True
            self.cam1_tilt = True
            rospy.logwarn("Default : Up position") 
        else:                
            self.cam0_tilt = False
            self.cam1_tilt = False
            rospy.logwarn("Default : Down position") 

        rospy.Timer(rospy.Duration(0.5), self.extrinsic_publisher)


    def __del__(self):
        pass

    def cam_tilt_cb(self, data):
        self.cam0_tilt = data.cam0_tilt
        self.cam1_tilt = data.cam1_tilt


    def motion_info_cb(self, data):        
        self.cam0_tilt = data.cam0_tilt
        self.cam1_tilt = data.cam1_tilt


    def send_tf(self, parent_frame, child_frame, T):
        camT = np.array(T).reshape(4,4)
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = parent_frame
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = child_frame
        t.transform.translation.x = camT[0,3]
        t.transform.translation.y = camT[1,3]
        t.transform.translation.z = camT[2,3]
        q = quaternion_from_matrix(camT)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br = tf2_ros.StaticTransformBroadcaster()
        br.sendTransform(t)


    def extrinsic_publisher(self, event):
        if self.cam0_tilt:
            rospy.set_param("/cam0/base_T_rgb", self.cam0_base_T_rgb_up)
            rospy.set_param("/cam0/base_T_cam", self.cam0_base_T_cam_up)
            self.send_tf("robot", "cam0_rgb_camera_link", self.cam0_base_T_rgb_up)
            self.send_tf("robot", "cam0_depth_camera_link", self.cam0_base_T_cam_up)
            self.send_tf("robot", "cam0_camera_base", self.cam0_base_T_k4abase_up)
        else:
            rospy.set_param("/cam0/base_T_rgb", self.cam0_base_T_rgb_down)
            rospy.set_param("/cam0/base_T_cam", self.cam0_base_T_cam_down)
            self.send_tf("robot", "cam0_rgb_camera_link", self.cam0_base_T_rgb_down)
            self.send_tf("robot", "cam0_depth_camera_link", self.cam0_base_T_cam_down)
            self.send_tf("robot", "cam0_camera_base", self.cam0_base_T_k4abase_down)
        if self.cam1_tilt:
            rospy.set_param("/cam1/base_T_rgb", self.cam1_base_T_rgb_up)
            rospy.set_param("/cam1/base_T_cam", self.cam1_base_T_cam_up)
            self.send_tf("robot", "cam1_rgb_camera_link", self.cam1_base_T_rgb_up)
            self.send_tf("robot", "cam1_depth_camera_link", self.cam1_base_T_cam_up)
            self.send_tf("robot", "cam1_camera_base", self.cam1_base_T_k4abase_up)
        else:
            rospy.set_param("/cam1/base_T_rgb", self.cam1_base_T_rgb_down)
            rospy.set_param("/cam1/base_T_cam", self.cam1_base_T_cam_down)
            self.send_tf("robot", "cam1_rgb_camera_link", self.cam1_base_T_rgb_down)
            self.send_tf("robot", "cam1_depth_camera_link", self.cam1_base_T_cam_down)
            self.send_tf("robot", "cam1_camera_base", self.cam1_base_T_k4abase_down)


if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('aruco_detector', anonymous=True)
    ep = ExtrinsicPublisher()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
