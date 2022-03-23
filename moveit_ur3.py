#!/usr/bin/env python

from multiprocessing.connection import wait
import sys
import copy
from tkinter import Scale
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


##initializing the moveit_commander and a rospy node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)


#initializing the a Robotcommander object.
robot = moveit_commander.RobotCommander()  #this is the outer-level interface to the robot

#initializing the planning scene
scene = moveit_commander.PlanningSceneInterface()


#instantiate a MoveGroupCommander object
#in this case the robot object is panda arm and the the group is the joints in the Panda arm
group_name ='manipulator'
group = moveit_commander.MoveGroupCommander(group_name) 


#now creating Display trajectory publisher which is used to publish trajectories for RViz to visualize
publisher_name = '/move_group/display_planned_path'
display_trajectory_publisher = rospy.Publisher(publisher_name, moveit_msgs.msg.DisplayTrajectory, queue_size=20)



# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print ("============ Reference frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print ("============ End effector: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print ("============ Robot Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print ("============ Printing robot state")
print (robot.get_current_state())
print ("")


#providing initial conditions to the panda robot
joint_goal = group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 0
joint_goal[3] = 0
joint_goal[4] = 0
joint_goal[5] = 0



#use go command to set the robot to those values

group.go(joint_goal, wait=True)

#just use stop command to remove any residual movement

group.stop()



#lets plan a motion now for a endeffector

pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x =0.0
pose_goal.position.y =0.4
pose_goal.position.z =0.1
group.set_pose_target(pose_goal)


plan = group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()


# ##cartesian paths

# waypoints=[]

# scale = 2

# wpose = group.get_current_pose().pose

# print(wpose)

# wpose.position.z -= scale * 0.1  # First move up (z)
# wpose.position.y += scale * 0.2  # and sideways (y)
# waypoints.append(copy.deepcopy(wpose))

# # wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
# # waypoints.append(copy.deepcopy(wpose))

# # wpose.position.y -= scale * 0.1  # Third move sideways (y)
# # waypoints.append(copy.deepcopy(wpose))

# # We want the Cartesian path to be interpolated at a resolution of 1 cm
# # which is why we will specify 0.01 as the eef_step in Cartesian
# # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
# (plan, fraction) = group.compute_cartesian_path(
#                                    waypoints,   # waypoints to follow
#                                    0.01,        # eef_step
#                                    0.0)         # jump_threshold


# display_trajectory = moveit_msgs.msg.DisplayTrajectory()
# display_trajectory.trajectory_start = robot.get_current_state() #getting current state
# display_trajectory.trajectory.append(plan) #adding the plan 
# # Publish
# display_trajectory_publisher.publish(display_trajectory) #publishing the trajectory

# group.execute(plan, wait=True) #executing the plan

# # #adding box object for the robot
# box_pose = geometry_msgs.msg.PoseStamped()
# box_pose.header.frame_id = "ee_link"
# box_pose.pose.orientation.w = 1.0
# box_name = "box"
# scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))


