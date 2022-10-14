#!/usr/bin/env python
# license removed for brevity
__author__ = 'fiorellasibona'
import rospy
import math
from std_msgs.msg import Float64
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('move_base_sequence')
        with open('poses.txt', 'r') as file:
            Lines = file.readlines()
            L, X, Y, Z, W = [], [], [], [], []
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
                if (L[i][0]) == 'w':
                    if (L[i-4][0]) == 'orientation':
                        cost4 = float(L[i][1])
                        W.append(cost4)
            # print(X)
        
        # points_seq = [0.2,0.5,0,2,0.5,0,1.5,-0.5,0]
        # Only yaw angle required (no ratotions around x and y axes) in deg:
        # yaweulerangles_seq = [90,0,180]
        #List of goal quaternions:
        quat_seq = list()
        #List of goal poses:
        self.pose_seq = list()
        self.goal_cnt = 0
        for j in range(len(Z)):
            #Unpacking the quaternion tuple and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(0, 0, Z[j], W[j]))
        # print(quat_seq)
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = []
        for k in range(len(X)):
            # print(X[k],Y[k])
            p =[X[k],Y[k],0]
            # print('p ..... : ' , p)
            points.append(p)
        # print('points ..... : ' , points)
        # points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        rospy.loginfo(str(points))
        n = 3
        for point in points:
            # print(point)
            #Exploit n variable to cycle in quat_seq
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1
        print("poses :",self.pose_seq)
        #rospy.loginfo(str(self.pose_seq))
        #Create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        #wait = self.client.wait_for_server(rospy.Duration(5.0))
        wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        self.goal_cnt += 1
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")
            if self.goal_cnt == 1: self.lift(0.05) 
            # print("goal_cnt :",self.goal_cnt)
            # print("pose_seq :",len(self.pose_seq))
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final goal pose reached!")
                self.lift(-0.05)
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
    #for pose in pose_seq:   
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()
    def lift(self,lifting):
        pub = rospy.Publisher('/arabv/table_joint_position_controller/command', Float64, queue_size=1)
        rate = rospy.Rate(10) # 10hz
        for i in range(5):
            pub.publish(lifting)
            rate.sleep()

if __name__ == '__main__':
    try:

        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")