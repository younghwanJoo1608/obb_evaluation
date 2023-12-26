#!/usr/bin/python3
import rospy
from unld_msgs.msg import ObjectArray, Object
import json
from tf import transformations as t

class obb_evaluator():

    def __init__(self):
        rospy.Subscriber("/unld/detection/objects_array", ObjectArray, self.obb_cb)
        self.datanum = 0


    def obb_cb(self,msg):
        self.obb_msg = msg
        self.obb_msg_ready = True  

        anns = {}
        anns['folder'] = "pointclouds"
        anns['filename'] = "azure_stacked_boxes_2400mm.pcd"
        anns['path'] = "pointclouds/azure_stacked_boxes_2400mm.pcd"
        anns['objects'] = []

        for obj in msg.data:
            if ((obj.type == Object.TYPE_BOX) or (obj.type == Object.TYPE_ICEBOX) ):
                cent = {}
                dim = {}
                rot = {}
                print(obj)
                data = {}
                data["name"] = "box"
                cent["x"] = obj.pose.position.x 
                cent["y"] = obj.pose.position.y
                cent["z"] = obj.pose.position.z
                data["centroid"] = cent
                dim["length"] = obj.size.x
                dim["width"] = obj.size.y
                dim["height"] = obj.size.z
                data["dimensions"] = dim
                x = obj.pose.orientation.x 
                y = obj.pose.orientation.y
                z = obj.pose.orientation.z
                w = obj.pose.orientation.w
                #ex, ey, ez = t.euler_from_quaternion([x,y,z,w], axes = 'sxyz')
                rot["x"] = obj.pose.orientation.x
                rot["y"] = obj.pose.orientation.y
                rot["z"] = obj.pose.orientation.z
                rot["w"] = obj.pose.orientation.w
                data["rotations"]=rot
                anns['objects'].append(data)
        #print(anns)
        save_file = "./results/obb_json/data_" + str(self.datanum) + ".json"
        with open(save_file, "w") as json_file:
            json.dump(anns, json_file)
        json_file.close()

        print("File saved! {}".format(save_file))
        self.datanum = self.datanum + 1
        

if __name__ == '__main__':

    if not rospy.is_shutdown():
        try:
            rospy.init_node("obb_evaluator", anonymous = True)

            node = obb_evaluator()

            rospy.loginfo(' obb_evaluator Node Started! ')
            rospy.spin()
            rospy.loginfo(' obb_evaluator Node Terminated ')  

        except rospy.ROSInterruptException:
            rospy.loginfo('ROS Terminated')
            pass
