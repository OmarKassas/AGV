#! /usr/bin/env python

import rospy
import collections
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose , PoseWithCovarianceStamped
import time


class SavePoses(object):
    def __init__(self):
        
        self._pose = PoseWithCovarianceStamped()
        self.poses_dict = {}
        self._pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped , self.sub_callback)
        self.write_to_file()

    def sub_callback(self, msg):
        
        self._pose = msg.pose.pose
    
    def write_to_file(self):
        
        i = 1
        while (1):
            A = input("press " + str(i) + " to enter pose " + str(i) + " or 0 to finish poses: ")
            if A == i:
                # self.poses_dict[0] = "pose" + str(i)
                # self.poses_dict[1] = self._pose
                self.poses_dict.update({"Pose "+ str(i): self._pose})
                rospy.loginfo("Written pose " + str(i))
                i+=1
            elif A == 0:
                break
        self.poses_dict = collections.OrderedDict(sorted(self.poses_dict.items()))
        # A = input("press 1 to enter pose A: ")
        # if A ==1:
        #     self.poses_dict["pose A"] = self._pose
        #     rospy.loginfo("Written pose A")

        # B = input("press 2 to enter pose B: ")
        # if B ==2:
        #     self.poses_dict["pose B"] = self._pose
        #     rospy.loginfo("Written pose B")
            
        with open('poses.txt', 'w') as file:
            
            for key, value in self.poses_dict.iteritems():
                if value:
                    file.write(str(key) + ':\n----------\n' + str(value) + '\n===========\n')
                    
            rospy.loginfo("Written all Poses to poses.txt file")
        


if __name__ == "__main__":
    rospy.init_node('getPose', log_level=rospy.INFO) 
    save_spots_object = SavePoses()
    #rospy.spin() # mantain the service open.












# import rospy
# import sys
# import termios
# import tty
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseWithCovarianceStamped
# from select import select
# import time


# class SavePoses(object):
#     def __init__(self):
        
#         self._pose = PoseWithCovarianceStamped()
#         self.settings = self.saveTerminalSettings()
#         self.key_timeout = rospy.get_param("~key_timeout", 0.5)
#         self.poses_dict = {"pose A":self._pose, "pose B":self._pose}
#         self._pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped , self.sub_callback)
#         self.write_to_file()

#     def sub_callback(self, msg):
        
#         self._pose = msg.pose.pose
    
#     def saveTerminalSettings(self):
#         if sys.platform == 'win32':
#             return None
#         return termios.tcgetattr(sys.stdin)

#     def restoreTerminalSettings(self,old_settings):
#         if sys.platform == 'win32':
#             return
#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

#     def getKey(self,settings, timeout):
#         if sys.platform == 'win32':
#             # getwch() returns a string on Windows
#             key = msvcrt.getwch()
#         else:
#             tty.setraw(sys.stdin.fileno())
#             # sys.stdin.read() returns a string on Linux
#             rlist, _, _ = select([sys.stdin], [], [], timeout)
#             if rlist:
#                 key = sys.stdin.read(1)
#             else:
#                 key = ''
#             termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#         return key
    
#     def write_to_file(self):

#         keyy = self.getKey(self.settings, self.key_timeout)
#         while keyy != 'c':
#             if keyy == 'a':
#                 self.poses_dict["pose A"] = self._pose
#                 rospy.loginfo("Written pose A")
#             elif keyy == 'b':
#                 self.poses_dict["pose B"] = self._pose
#                 rospy.loginfo("Written pose B")
                
            
#         with open('poses.txt', 'w') as file:
                
#             for key, value in self.poses_dict.iteritems():
#                 if value:
#                     file.write(str(key) + ':\n----------\n' + str(value) + '\n===========\n')
                        
#         rospy.loginfo("Written all Poses to poses.txt file")
            
        

# if __name__ == "__main__":

#     rospy.init_node('pose_recorder', log_level=rospy.INFO) 
#     save_spots_object = SavePoses()
#     #rospy.spin() # mantain the service open.