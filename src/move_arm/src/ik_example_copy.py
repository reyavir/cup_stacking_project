#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from intera_interface import gripper as robot_gripper
from planning import plan_pyramid


increment_z = 0.128
start_y = 0.285
cup_diameter = 0.378 - 0.285
cup_height = cup_diameter*1.25
quat = [0.0, 1.0, 0.0, 0.0]
neg_z = -0.099
pos_z = 0.099
quat = [0.0, 1.0, 0.0, 0.0]
num_cups = 3


def calculate_inter_trans_positions(trans):
    x,y,z = trans[0], trans[1], trans[2]
    return [x, y, z + increment_z]

def construct_request(trans, i):
    # Construct the request
    link = "right_gripper_tip"
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"
    request.ik_request.ik_link_name = link
    request.ik_request.pose_stamped.header.frame_id = "base"

    request.ik_request.pose_stamped.pose.position.x = trans[i][0]
    request.ik_request.pose_stamped.pose.position.y = trans[i][1]
    request.ik_request.pose_stamped.pose.position.z = trans[i][2]    
    request.ik_request.pose_stamped.pose.orientation.x = quat[0]
    request.ik_request.pose_stamped.pose.orientation.y = quat[1]
    request.ik_request.pose_stamped.pose.orientation.z = quat[2]
    request.ik_request.pose_stamped.pose.orientation.w = quat[3]
    return request

def move_to_position(request, position_name):
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    count = 0
    tries = 10
    while tries > count:
        response = compute_ik(request)
        print(position_name)
        inter_group = MoveGroupCommander("right_arm")
        inter_group.set_pose_target(request.ik_request.pose_stamped)
        
        # Print the response HERE
        print(response)
        end_plan = inter_group.plan()
        user_input = input("Enter 'y' if the trajectory looks safe on RVIZ: ")

        # Execute IK if safe
        if user_input == 'y':
            inter_group.execute(end_plan[1])
            break
        
        count += 1



def main():
    # get starting positiosn for the cups - hardcode for now (testing with 3 cups)
    start_trans = [[0.793, start_y, neg_z], [0.793, start_y + cup_diameter, neg_z], [0.793, start_y + cup_diameter*2, neg_z]]
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    while not rospy.is_shutdown():
        input('Press [ Enter ]: ')
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"

        start_inter_trans1 = calculate_inter_trans_positions(start_trans[0])
        start_inter_trans2 = calculate_inter_trans_positions(start_trans[1])
        start_inter_trans3 = calculate_inter_trans_positions(start_trans[2])

        start_inter_trans = [start_inter_trans1, start_inter_trans2, start_inter_trans3]

        
        end_x, end_y, end_z = 0.793, 0 - cup_diameter, neg_z
        end_trans = plan_pyramid(num_cups, end_x, end_y, end_z, cup_diameter, cup_height)

        end_inter_trans1 = calculate_inter_trans_positions(end_trans[0])
        end_inter_trans2 = calculate_inter_trans_positions(end_trans[1])
        end_inter_trans3 = calculate_inter_trans_positions(end_trans[2])

        end_inter_trans = [end_inter_trans1, end_inter_trans2, end_inter_trans3]

        # Set up the right gripper
        right_gripper = robot_gripper.Gripper('right_gripper')

        # Calibrate the gripper (other commands won't work unless you do this first)
        print('Calibrating...')
        right_gripper.calibrate()
        rospy.sleep(2.0)

        # Open the right gripper
        print('Opening...')
        right_gripper.open()
        rospy.sleep(1.0)
        print('Done!')

        for i in range(num_cups):
            # Construct the request
            start_request = construct_request(start_trans, i)
            # Construct the start inter request to move box to goal
            start_inter_request = construct_request(start_inter_trans, i)
            # Construct the end inter request to move box to goal
            end_inter_request = construct_request(end_inter_trans, i)
            # Construct the end request to move box to goal
            end_request = construct_request(end_trans, i)
            print("Trying to move cup number ", str(i + 1))

            try:
                # move to pre start
                move_to_position(start_inter_request, "pre_start")
                # move to start
                move_to_position(start_request, "start")
                # Close the right gripper
                print('Closing...')
                right_gripper.close()
                rospy.sleep(1.0)
                # move to start inter
                move_to_position(start_inter_request, "start_inter")
                # move to end inter
                move_to_position(end_inter_request, "end_inter")
                # move to end
                move_to_position(end_request, "end")
                # Open the right gripper
                print('Opening...')
                right_gripper.open()
                rospy.sleep(3.0)
                print('Done!')
                # move to end inter
                move_to_position(end_inter_request, "end_inter")

            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
