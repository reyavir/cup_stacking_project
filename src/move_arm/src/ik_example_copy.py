#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from intera_interface import gripper as robot_gripper


def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
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

        # Example with 1 cup
        # start_trans = [0.793, 0.263, neg_z]
        # start_quat = [0.050, 0.998, 0.041, -0.014]
        # inter_trans = [0.793, 0.263, pos_z]
        # inter_quat = [0.050, 0.998, 0.041, -0.014]
        # end_trans = [0.793, 0.0, neg_z]
        # end_quat = [0.050, 0.998, 0.041, -0.014]

        # Example with two cups
        cup_diameter = 0.378 - 0.285
        neg_z = -0.099
        pos_z = 0.099
        
        start_trans1 = [0.793, 0.285, neg_z]
        quat = [0.0, 1.0, 0.0, 0.0]
        # old_quat = [0.050, 0.998, 0.041, -0.014]
        start_trans2 = [0.793, 0.378, neg_z]

        start_trans = [start_trans1, start_trans2]
        start_quat = [quat, quat]

        start_inter_trans1 = [0.793, 0.285, pos_z]
        start_inter_trans2 = [0.793, 0.378, pos_z]

        start_inter_trans = [start_inter_trans1, start_inter_trans2]
        start_inter_quat = [quat, quat]

        end_inter_trans1 = [0.793, 0 - cup_diameter, pos_z]
        end_inter_trans2 = [0.793, 0, pos_z]

        end_inter_trans = [end_inter_trans1, end_inter_trans2]
        end_inter_quat = [quat, quat]

        end_trans1 = [0.793, 0 - cup_diameter, neg_z]
        end_trans2 = [0.793, 0, neg_z]

        end_trans = [end_trans1, end_trans2]
        end_quat = [quat, quat]

        

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

        og_num_try = 10

        for i in range(2):
            # Construct the request
            start_request = GetPositionIKRequest()
            start_request.ik_request.group_name = "right_arm"
            start_request.ik_request.ik_link_name = link
            start_request.ik_request.pose_stamped.header.frame_id = "base"

            start_request.ik_request.pose_stamped.pose.position.x = start_trans[i][0]
            start_request.ik_request.pose_stamped.pose.position.y = start_trans[i][1]
            start_request.ik_request.pose_stamped.pose.position.z = start_trans[i][2]    
            start_request.ik_request.pose_stamped.pose.orientation.x = start_quat[i][0]
            start_request.ik_request.pose_stamped.pose.orientation.y = start_quat[i][1]
            start_request.ik_request.pose_stamped.pose.orientation.z = start_quat[i][2]
            start_request.ik_request.pose_stamped.pose.orientation.w = start_quat[i][3]

            # Construct the start inter request to move box to goal
            start_inter_request = GetPositionIKRequest()
            start_inter_request.ik_request.group_name = "right_arm"
            start_inter_request.ik_request.ik_link_name = link
            start_inter_request.ik_request.pose_stamped.header.frame_id = "base"

            start_inter_request.ik_request.pose_stamped.pose.position.x = start_inter_trans[i][0]
            start_inter_request.ik_request.pose_stamped.pose.position.y = start_inter_trans[i][1]
            start_inter_request.ik_request.pose_stamped.pose.position.z = start_inter_trans[i][2]    
            start_inter_request.ik_request.pose_stamped.pose.orientation.x = start_inter_quat[i][0]
            start_inter_request.ik_request.pose_stamped.pose.orientation.y = start_inter_quat[i][1]
            start_inter_request.ik_request.pose_stamped.pose.orientation.z = start_inter_quat[i][2]
            start_inter_request.ik_request.pose_stamped.pose.orientation.w = start_inter_quat[i][3]

            # Construct the end inter request to move box to goal
            end_inter_request = GetPositionIKRequest()
            end_inter_request.ik_request.group_name = "right_arm"
            end_inter_request.ik_request.ik_link_name = link
            end_inter_request.ik_request.pose_stamped.header.frame_id = "base"

            end_inter_request.ik_request.pose_stamped.pose.position.x = end_inter_trans[i][0]
            end_inter_request.ik_request.pose_stamped.pose.position.y = end_inter_trans[i][1]
            end_inter_request.ik_request.pose_stamped.pose.position.z = end_inter_trans[i][2]    
            end_inter_request.ik_request.pose_stamped.pose.orientation.x = end_inter_quat[i][0]
            end_inter_request.ik_request.pose_stamped.pose.orientation.y = end_inter_quat[i][1]
            end_inter_request.ik_request.pose_stamped.pose.orientation.z = end_inter_quat[i][2]
            end_inter_request.ik_request.pose_stamped.pose.orientation.w = end_inter_quat[i][3]

            # Construct the end request to move box to goal
            end_request = GetPositionIKRequest()
            end_request.ik_request.group_name = "right_arm"
            end_request.ik_request.ik_link_name = link
            end_request.ik_request.pose_stamped.header.frame_id = "base"

            end_request.ik_request.pose_stamped.pose.position.x = end_trans[i][0]
            end_request.ik_request.pose_stamped.pose.position.y = end_trans[i][1]
            end_request.ik_request.pose_stamped.pose.position.z = end_trans[i][2]    
            end_request.ik_request.pose_stamped.pose.orientation.x = end_quat[i][0]
            end_request.ik_request.pose_stamped.pose.orientation.y = end_quat[i][1]
            end_request.ik_request.pose_stamped.pose.orientation.z = end_quat[i][2]
            end_request.ik_request.pose_stamped.pose.orientation.w = end_quat[i][3]

            print("Trying to move cup number ", str(i + 1))

            try:
                # INTER POSITION
                count = 0
                tries = og_num_try
                while tries > count:
                    response = compute_ik(start_inter_request)
                    print("Pre-Start Position")
                    inter_group = MoveGroupCommander("right_arm")
                    inter_group.set_pose_target(start_inter_request.ik_request.pose_stamped)
                    
                    # Print the response HERE
                    print(response)
                    end_plan = inter_group.plan()
                    user_input = input("Enter 'y' if the trajectory looks safe on RVIZ: ")

                    # Execute IK if safe
                    if user_input == 'y':
                        inter_group.execute(end_plan[1])
                        break
                    
                    count += 1

                # STARTING POSITION
                count = 0
                tries = og_num_try
                while tries > count:
                    response = compute_ik(start_request)
                    print("Starting Position")
                    print(response)
                    start_group = MoveGroupCommander("right_arm")
                    
                    # Setting position and orientation target
                    start_group.set_pose_target(start_request.ik_request.pose_stamped)

                    # Plan IK
                    start_plan = start_group.plan()
                    user_input = input("Enter 'y' if the trajectory looks safe on RVIZ: ")

                    # Execute IK if safe
                    if user_input == 'y':
                        start_group.execute(start_plan[1])
                        break
                    
                    count += 1

                # Close the right gripper
                print('Closing...')
                right_gripper.close()
                rospy.sleep(1.0)

                # START INTER POSITION
                count = 0
                tries = og_num_try
                while tries > count:
                    response = compute_ik(start_inter_request)
                    print("Starting Intermediate Position")
                    inter_group = MoveGroupCommander("right_arm")
                    inter_group.set_pose_target(start_inter_request.ik_request.pose_stamped)
                    
                    # Print the response HERE
                    print(response)
                    end_plan = inter_group.plan()
                    user_input = input("Enter 'y' if the trajectory looks safe on RVIZ: ")

                    # Execute IK if safe
                    if user_input == 'y':
                        inter_group.execute(end_plan[1])
                        break
                    
                    count += 1

                # END INTER POSITION
                count = 0
                tries = og_num_try
                while tries > count:
                    response = compute_ik(end_inter_request)
                    print("Ending Intermediate Position")
                    inter_group = MoveGroupCommander("right_arm")
                    inter_group.set_pose_target(end_inter_request.ik_request.pose_stamped)
                    
                    # Print the response HERE
                    print(response)
                    end_plan = inter_group.plan()
                    user_input = input("Enter 'y' if the trajectory looks safe on RVIZ: ")

                    # Execute IK if safe
                    if user_input == 'y':
                        inter_group.execute(end_plan[1])
                        break
                    
                    count += 1

                # END POSITION
                count = 0
                tries = og_num_try
                while tries > count:
                    response = compute_ik(end_request)
                    print("Ending Position")
                    end_group = MoveGroupCommander("right_arm")
                    end_group.set_pose_target(end_request.ik_request.pose_stamped)

                    
                    # Print the response HERE
                    print(response)
                    end_plan = end_group.plan()
                    user_input = input("Enter 'y' if the trajectory looks safe on RVIZ: ")

                    # Execute IK if safe
                    if user_input == 'y':
                        end_group.execute(end_plan[1])
                        break
                    
                    count += 1

                # Open the right gripper
                print('Opening...')
                right_gripper.open()
                rospy.sleep(1.0)
                print('Done!')

                # END INTER POSITION
                count = 0
                tries = og_num_try
                while tries > count:
                    response = compute_ik(end_inter_request)
                    print("Ending Intermediate Position")
                    inter_group = MoveGroupCommander("right_arm")
                    inter_group.set_pose_target(end_inter_request.ik_request.pose_stamped)
                    
                    # Print the response HERE
                    print(response)
                    end_plan = inter_group.plan()
                    user_input = input("Enter 'y' if the trajectory looks safe on RVIZ: ")

                    # Execute IK if safe
                    if user_input == 'y':
                        inter_group.execute(end_plan[1])
                        break
                    
                    count += 1

            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
