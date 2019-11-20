#!/usr/bin/env python
# license removed for brevity
import rospy
import random,math
import numpy as np
import copy
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped


class Reaching:
    def __init__(self):
        rospy.init_node('reaching', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.model_pose = [0.0,0.0,0.0]
        self.obstacle_pose = [0.0,0.0,0.0]
        self.set_forbidden()
        self.set_init_pose()

    def set_forbidden(self):
        #set forbidden erea
        self.planning_scene = PlanningSceneInterface("base_link")
        self.planning_scene.removeCollisionObject("my_front_ground")
        self.planning_scene.removeCollisionObject("my_back_ground")
        self.planning_scene.removeCollisionObject("my_right_ground")
        self.planning_scene.removeCollisionObject("my_left_ground")
        self.planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
        self.planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
        self.planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
        self.planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)
        self.planning_scene.removeCollisionObject("demo_cube") 
        self.planning_scene.removeCollisionObject("obstacle")
        print dir(self.planning_scene)
        import inspect
        print "addBox's variable is ",inspect.getargspec(self.planning_scene.addBox)
        print "attachBox's variable is ",inspect.getargspec(self.planning_scene.attachBox)
        print "addCube's variable is ",inspect.getargspec(self.planning_scene.addCube)
        print "setColor's variable is ",inspect.getargspec(self.planning_scene.setColor)

    def set_init_pose(sefl): 
        #set init pose
        move_group = MoveGroupInterface("manipulator","base_link")
        planning_scene = PlanningSceneInterface("base_link")
        joint_names = ["shoulder_pan_joint", "shoulder_lift_joint",
                        "elbow_joint", "wrist_1_joint",
                        "wrist_2_joint", "wrist_3_joint"]
        pose = [0.0, -1.98, 1.0, -0.64, -1.57, 0.0]      
        move_group.moveToJointPosition(joint_names, pose, wait=False)
        move_group.get_move_action().wait_for_result()
        result = move_group.get_move_action().get_result()
        move_group.get_move_action().cancel_all_goals()


    def callback(self,data):
        x = data.pose[1].position.x
        y = data.pose[1].position.y
        z = data.pose[1].position.z
        self.model_pose = [x,y,z]
        
        x = data.pose[2].position.x
        y = data.pose[2].position.y
        z = data.pose[2].position.z
        self.obstacle_pose = [x,y,z]

    def start_subscriber(self):
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)

    def target(self):
        rate = rospy.Rate(1)
        rospy.sleep(1)
        while not rospy.is_shutdown():
            self.planning_scene.removeCollisionObject("demo_cube") 
            self.planning_scene.removeCollisionObject("obstacle")
            self.planning_scene.addCube("demo_cube", 0.06,self.model_pose[0],self.model_pose[1],self.model_pose[2])
            self.planning_scene.addBox("obstacle", 0.540614, 0.083603, 0.54373, self.obstacle_pose[0],self.obstacle_pose[1],self.obstacle_pose[2])

            print self.model_pose
            group = moveit_commander.MoveGroupCommander("manipulator")
            #print group.get_current_pose().pose
            pose_target = geometry_msgs.msg.Pose()
            pose = group.get_current_pose().pose
            pose_target = geometry_msgs.msg.Pose()
            pose_target.orientation.x = pose.orientation.x
            pose_target.orientation.y = pose.orientation.y
            pose_target.orientation.z = pose.orientation.z
            pose_target.orientation.w = pose.orientation.w
            #pose_target.position.x = pose.position.x
            #pose_target.position.y = pose.position.y
            #pose_target.position.z = pose.position.z
            pose_target.position.x = self.model_pose[0]
            pose_target.position.y = self.model_pose[1]
            pose_target.position.z = self.model_pose[2]+0.05

            group.set_pose_target(pose_target)
            group.go()

            rospy.sleep(1)



if __name__ == '__main__':
    try:
        r = Reaching()
        r.start_subscriber()
        r.target()
        
    except rospy.ROSInterruptException:
        pass