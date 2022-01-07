#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import actionlib
import math
import random
from geometry_msgs.msg import Point, Pose
from gazebo_msgs.msg import ModelStates
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion

gazebo_model_states = ModelStates()

def callback(msg):
    global gazebo_model_states
    gazebo_model_states = msg


def yaw_of(object_orientation):
    #Convert quaternions to Euler angles and return yaw angles
    euler = euler_from_quaternion(
        (object_orientation.x, object_orientation.y,
        object_orientation.z, object_orientation.w))

    return euler[2]


def main():
    global gazebo_model_states

    OBJECT_NAME = "wood_cube_5cm"   # The name of the object to grab
    OBJECT_NAME_2 = "wood_cube_5cm_clone"   # The name of the object to grab
    GRIPPER_OPEN = 1.2              # Hand opening / closing angle when grasping
    GRIPPER_CLOSE = 0.42            # Hand opening / closing angle at the time of installation
    APPROACH_Z = 0.15               # Hand height when approaching
    LEAVE_Z = 0.20                  # Hand height when leaving
    PICK_Z = 0.12                   # Hand height when grabbing
    PLACE_POSITION = Point(0.45, 0.0, 0.0)  # Object place position 
    PLACE_POSITION_2 = Point(0.45, 0.0, 0.0)  # Object place position 

    sub_model_states = rospy.Subscriber("gazebo/model_states", ModelStates, callback, queue_size=1)

    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.4)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = actionlib.SimpleActionClient("crane_x7/gripper_controller/gripper_cmd", GripperCommandAction)
    gripper.wait_for_server()
    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 4.0

    rospy.sleep(0.1)
    
    #Open your hand in case you're holding something
    gripper_goal.command.position = GRIPPER_OPEN
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(0.1))

    # Make it the "home" attitude defined in #SRDF
    arm.set_named_target("home")
    arm.go()
    rospy.sleep(0.1)

    # If the object exists on gazebo, execute pick_and_place
    if OBJECT_NAME in gazebo_model_states.name:
        object_index = gazebo_model_states.name.index(OBJECT_NAME)
        #Get the posture of the object
        object_position = gazebo_model_states.pose[object_index].position
        object_orientation = gazebo_model_states.pose[object_index].orientation
        object_yaw = yaw_of(object_orientation)

        #Approach the object
        target_pose = Pose()
        target_pose.position.x = object_position.x
        target_pose.position.y = object_position.y
        target_pose.position.z = APPROACH_Z
        q = quaternion_from_euler(-math.pi, 0.0, object_yaw)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        if arm.go() is False:
            print("Failed to approach an object.")
	    return
        rospy.sleep(0.1)

        # Go grab
        target_pose.position.z = PICK_Z
        arm.set_pose_target(target_pose)
        if arm.go() is False:
            print("Failed to grip an object.")
	    return
        rospy.sleep(0.1)
        gripper_goal.command.position = GRIPPER_CLOSE
        gripper.send_goal(gripper_goal)
        gripper.wait_for_result(rospy.Duration(0.1))

        # Lift
        target_pose.position.z = LEAVE_Z
        arm.set_pose_target(target_pose)
        if arm.go() is False:
            print("Failed to pick up an object.")
	    return
        rospy.sleep(0.1)
        
        # Move to the place position
        place_position = PLACE_POSITION # Randomly select the installation position
        target_pose.position.x = place_position.x
        target_pose.position.y = place_position.y
        q = quaternion_from_euler(-math.pi, 0.0, -math.pi/2.0)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        if arm.go() is False:
            print("Failed to approach target position.")
	    return
        rospy.sleep(0.1)

        #Place
        target_pose.position.z = PICK_Z
        arm.set_pose_target(target_pose)
        if arm.go() is False:
            print("Failed to place an object.")
	    return
        rospy.sleep(0.1)
        gripper_goal.command.position = GRIPPER_OPEN
        gripper.send_goal(gripper_goal)
        gripper.wait_for_result(rospy.Duration(0.1))

        # Raise your hand
        target_pose.position.z = LEAVE_Z
        arm.set_pose_target(target_pose)
        if arm.go() is False:
            print("Failed to leave from an object.")
	    return
        rospy.sleep(0.1)

        print("Cube1 done")

    # If the object exists on gazebo, execute pick_and_place
    if OBJECT_NAME_2 in gazebo_model_states.name:
        object_index = gazebo_model_states.name.index(OBJECT_NAME_2)
        #Get the posture of the object
        object_position = gazebo_model_states.pose[object_index].position
        object_orientation = gazebo_model_states.pose[object_index].orientation
        object_yaw = yaw_of(object_orientation)

        #Approach the object
        target_pose = Pose()
        target_pose.position.x = object_position.x
        target_pose.position.y = object_position.y
        target_pose.position.z = APPROACH_Z
        q = quaternion_from_euler(-math.pi, 0.0, object_yaw)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        if arm.go() is False:
            print("Failed to approach an object.")
	    return
        rospy.sleep(0.1)

        # Go grab
        target_pose.position.z = PICK_Z
        arm.set_pose_target(target_pose)
        if arm.go() is False:
            print("Failed to grip an object.")
	    return
        rospy.sleep(0.1)
        gripper_goal.command.position = GRIPPER_CLOSE
        gripper.send_goal(gripper_goal)
        gripper.wait_for_result(rospy.Duration(0.1))

        # Lift
        target_pose.position.z = LEAVE_Z
        arm.set_pose_target(target_pose)
        if arm.go() is False:
            print("Failed to pick up an object.")
	    return
        rospy.sleep(0.1)
        
        # Move to the place position
        place_position = PLACE_POSITION_2
        target_pose.position.x = place_position.x
        target_pose.position.y = place_position.y
        q = quaternion_from_euler(-math.pi, 0.0, -math.pi/2.0)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        if arm.go() is False:
            print("Failed to approach target position.")
	    return
        rospy.sleep(0.1)

        #Place
        target_pose.position.z = PICK_Z
        arm.set_pose_target(target_pose)
        if arm.go() is False:
            print("Failed to place an object.")
	    return
        rospy.sleep(0.1)
        gripper_goal.command.position = GRIPPER_OPEN
        gripper.send_goal(gripper_goal)
        gripper.wait_for_result(rospy.Duration(0.1))

        # Raise your hand
        target_pose.position.z = LEAVE_Z
        arm.set_pose_target(target_pose)
        if arm.go() is False:
            print("Failed to leave from an object.")
	    return
        rospy.sleep(0.1)

        #Make it the "home" attitude defined in #SRDF
        arm.set_named_target("home")
        if arm.go() is False:
            print("Failed to go back to home pose.")
	    return
        rospy.sleep(0.1)

        print("Cube2 done")

   


if __name__ == '__main__':
    rospy.init_node("pick_and_place")

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass