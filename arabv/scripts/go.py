#!/usr/bin/env python
# license removed for brevity

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    with open('poses.txt', 'r') as file:
        Lines = file.readlines()
        L, X, Y, Z = [], [], [], []
        for line in Lines: # Reading file line by line
            line = line.strip()
            line = line.split(':')   # spliting the line
            L.append(line)
            # L.remove(['===========\n'])
        for i in range(0,len(L)):
            if (L[i][0]) == 'x':
                if (L[i-1][0]) == 'position':
                    cost1 = float(L[i][1])
                    X.append(cost1)
            if (L[i][0]) == 'y':
                if (L[i-2][0]) == 'position':
                    cost2 = float(L[i][1])
                    Y.append(cost2)
            if (L[i][0]) == 'z':
                if (L[i-3][0]) == 'orientation':
                    cost3 = float(L[i][1])
                    Z.append(cost3)
        print(X)
        print(Y)
        print(Z)

   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = 0.5
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")


# #!/usr/bin/env python
# # license removed for brevity
# __author__ = 'fiorellasibona'
# import rospy
# import math

# import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from actionlib_msgs.msg import GoalStatus
# from geometry_msgs.msg import Pose, Point, Quaternion
# from tf.transformations import quaternion_from_euler


# class MoveBaseSeq():

#     def __init__(self):

#         rospy.init_node('move_base_sequence')
#         with open('poses.txt', 'r') as file:
#             Lines = file.readlines()
#             L, X, Y, Z, W = [], [], [], [], []
#             for line in Lines: # Reading file line by line
#                 line = line.strip()
#                 line = line.split(':')   # spliting the line
#                 L.append(line)
#                 # L.remove(['===========\n'])
#             for i in range(0,len(L)):
#                 if (L[i][0]) == 'x':
#                     if (L[i-1][0]) == 'position':
#                         cost1 = float(L[i][1])
#                         X.append(cost1)
#                 if (L[i][0]) == 'y':
#                     if (L[i-2][0]) == 'position':
#                         cost2 = float(L[i][1])
#                         Y.append(cost2)
#                 if (L[i][0]) == 'z':
#                     if (L[i-3][0]) == 'orientation':
#                         cost3 = float(L[i][1])
#                         Z.append(cost3)
#                 if (L[i][0]) == 'w':
#                     if (L[i-4][0]) == 'orientation':
#                         cost4 = float(L[i][1])
#                         W.append(cost4)
#             # print(X)
        
#         # points_seq = [0.2,0.5,0,2,0.5,0,1.5,-0.5,0]
#         # Only yaw angle required (no ratotions around x and y axes) in deg:
#         # yaweulerangles_seq = [90,0,180]
#         #List of goal quaternions:
#         quat_seq = list()
#         #List of goal poses:
#         self.pose_seq = list()
#         self.goal_cnt = 0
#         for j in range(len(Z)):
#             #Unpacking the quaternion tuple and passing it as arguments to Quaternion message constructor
#             quat_seq.append(Quaternion(0, 0, Z[j], W[j]))
#         # print(quat_seq)
#         # Returns a list of lists [[point1], [point2],...[pointn]]
#         for k in range(len(X)):
#             points = [[X[k],Y[k],0]]
#             print(points)
#         # points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
#         rospy.loginfo(str(points))
#         n = 3
#         for point in points:
#             #Exploit n variable to cycle in quat_seq
#             self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
#             n += 1
#         #rospy.loginfo(str(self.pose_seq))
#         #Create action client
#         self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
#         rospy.loginfo("Waiting for move_base action server...")
#         #wait = self.client.wait_for_server(rospy.Duration(5.0))
#         wait = self.client.wait_for_server()
#         if not wait:
#             rospy.logerr("Action server not available!")
#             rospy.signal_shutdown("Action server not available!")
#             return
#         rospy.loginfo("Connected to move base server")
#         rospy.loginfo("Starting goals achievements ...")
#         self.movebase_client()

#     def active_cb(self):
#         rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

#     def feedback_cb(self, feedback):
#         #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
#         rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

#     def done_cb(self, status, result):
#         self.goal_cnt += 1
#     # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
#         if status == 2:
#             rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

#         if status == 3:
#             rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
#             if self.goal_cnt< len(self.pose_seq):
#                 next_goal = MoveBaseGoal()
#                 next_goal.target_pose.header.frame_id = "map"
#                 next_goal.target_pose.header.stamp = rospy.Time.now()
#                 next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
#                 rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
#                 rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
#                 self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
#             else:
#                 rospy.loginfo("Final goal pose reached!")
#                 rospy.signal_shutdown("Final goal pose reached!")
#                 return

#         if status == 4:
#             rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
#             rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
#             return

#         if status == 5:
#             rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
#             rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
#             return

#         if status == 8:
#             rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

#     def movebase_client(self):
#     #for pose in pose_seq:   
#         goal = MoveBaseGoal()
#         goal.target_pose.header.frame_id = "map"
#         goal.target_pose.header.stamp = rospy.Time.now() 
#         goal.target_pose.pose = self.pose_seq[self.goal_cnt]
#         rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
#         rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
#         self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
#         rospy.spin()

# if __name__ == '__main__':
#     try:

#         MoveBaseSeq()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Navigation finished.")