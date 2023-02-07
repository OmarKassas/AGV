#! /usr/bin/env python


# import rospy
# import collections
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Pose , PoseWithCovarianceStamped
# import time


# class SavePoses(object):
#     def __init__(self):
        
#         self._pose = PoseWithCovarianceStamped()
#         # self.poses_dict = {"pose A":self._pose, "pose B":self._pose}
#         self.poses_dict = {}
#         self._pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped , self.sub_callback)
#         self.write_to_file()

#     def sub_callback(self, msg):
        
#         self._pose = msg.pose.pose
    
#     def write_to_file(self):
        
#         # X = input("Enter the number ")
#         i = 1
#         while (1):
#             A = input("press " + str(i) + " to enter pose A or 0 to finish poses: ")
#             if A == i:
#                 # self.poses_dict[0] = "pose" + str(i)
#                 # self.poses_dict[1] = self._pose
#                 self.poses_dict.update({"Pose "+ str(i): self._pose})
#                 rospy.loginfo("Written pose " + str(i))
#                 i+=1
#             elif A == 0:
#                 break
#         self.poses_dict = collections.OrderedDict(sorted(self.poses_dict.items()))
#         print(self.poses_dict)
        

#         # A = input("press 1 to enter pose A: ")
#         # if A ==1:
#         #     self.poses_dict["pose A"] = self._pose
#         #     rospy.loginfo("Written pose A")

#         # B = input("press 2 to enter pose B: ")
#         # if B ==2:
#         #     self.poses_dict["pose B"] = self._pose
#         #     rospy.loginfo("Written pose B")
            
        
#         with open('poses.txt', 'w') as file:
            
#             for key, value in self.poses_dict.iteritems():
#                 if value:
#                     file.write(str(key) + ':\n----------\n' + str(value) + '\n===========\n')
                    
#         rospy.loginfo("Written all Poses to poses.txt file")
        


# if __name__ == "__main__":
#     rospy.init_node('getPoses', log_level=rospy.INFO) 
#     save_spots_object = SavePoses()
#     #rospy.spin() # mantain the service open.














# #!/usr/bin/env python

# import rospy
# from std_msgs.msg import Float64

# def lift():
#     pub = rospy.Publisher('/arabv/table_joint_position_controller/command', Float64, queue_size=1)
#     rospy.init_node('lift', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     for i in range(5):
#         pub.publish(0.1)
#         rate.sleep()
        

#         # stop = False
        

# if __name__ == '__main__':
#     try:
#         lift()
#     except rospy.ROSInterruptException:
#         pass