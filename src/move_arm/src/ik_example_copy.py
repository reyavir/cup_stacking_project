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
        # start_trans = [0.793, 0.263, -0.103]
        # start_quat = [0.050, 0.998, 0.041, -0.014]
        # inter_trans = [0.793, 0.263, 0.103]
        # inter_quat = [0.050, 0.998, 0.041, -0.014]
        # end_trans = [0.793, 0.0, -0.103]
        # end_quat = [0.050, 0.998, 0.041, -0.014]

        # Example with two cups
        cup_diameter = 0.378 - 0.285
        
        start_trans1 = [0.793, 0.285, -0.103]
        start_quat1 = [0.050, 0.998, 0.041, -0.014]
        start_trans2 = [0.793, 0.378, -0.103]
        start_quat2 = [0.050, 0.998, 0.041, -0.014]

        start_trans = [start_trans1, start_trans2]
        start_quat = [start_quat1, start_quat2]

        inter_trans1 = [0.793, 0.285, 0.103]
        inter_quat1 = [0.050, 0.998, 0.041, -0.014]
        inter_trans2 = [0.793, 0.378, 0.103]
        inter_quat2 = [0.050, 0.998, 0.041, -0.014]

        inter_trans = [inter_trans1, inter_trans2]
        inter_quat = [inter_quat1, inter_quat2]

        end_trans1 = [0.793, 0 + cup_diameter, -0.103]
        end_quat1 = [0.050, 0.998, 0.041, -0.014]
        end_trans2 = [0.793, 0, -0.103]
        end_quat2 = [0.050, 0.998, 0.041, -0.014]

        end_trans = [end_trans1, end_trans2]
        end_quat = [end_quat1, end_quat2]

        # Set up the right gripper
        right_gripper = robot_gripper.Gripper('right_gripper')

        # Calibrate the gripper (other commands won't work unless you do this first)
        print('Calibrating...')
        right_gripper.calibrate()
        rospy.sleep(2.0)

        for i in range(2):
            # Open the right gripper
            print('Opening...')
            right_gripper.open()
            rospy.sleep(1.0)
            print('Done!')

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

            # Construct the inter request to move box to goal
            inter_request = GetPositionIKRequest()
            inter_request.ik_request.group_name = "right_arm"
            inter_request.ik_request.ik_link_name = link
            inter_request.ik_request.pose_stamped.header.frame_id = "base"

            inter_request.ik_request.pose_stamped.pose.position.x = inter_trans[i][0]
            inter_request.ik_request.pose_stamped.pose.position.y = inter_trans[i][1]
            inter_request.ik_request.pose_stamped.pose.position.z = inter_trans[i][2]    
            inter_request.ik_request.pose_stamped.pose.orientation.x = inter_quat[i][0]
            inter_request.ik_request.pose_stamped.pose.orientation.y = inter_quat[i][1]
            inter_request.ik_request.pose_stamped.pose.orientation.z = inter_quat[i][2]
            inter_request.ik_request.pose_stamped.pose.orientation.w = inter_quat[i][3]

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
                tries = 3
                # STARTING POSITION
                # Send the request to the service
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
                elif user_input != 'stop':
                    count = 0
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

                # INTER POSITION
                # Send the request to the service
                response = compute_ik(inter_request)
                print("Intermediate Position")
                inter_group = MoveGroupCommander("right_arm")
                inter_group.set_pose_target(inter_request.ik_request.pose_stamped)
                
                # Print the response HERE
                print(response)
                end_plan = inter_group.plan()
                user_input = input("Enter 'y' if the trajectory looks safe on RVIZ: ")

                # Execute IK if safe
                if user_input == 'y':
                    inter_group.execute(end_plan[1])
                elif user_input != 'stop':
                    count = 0
                    while tries > count:
                        response = compute_ik(inter_request)
                        print("Intermediate Position")
                        inter_group = MoveGroupCommander("right_arm")
                        inter_group.set_pose_target(inter_request.ik_request.pose_stamped)
                        
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
                # Send the request to the service
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
                elif user_input != 'stop':
                    count = 0
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
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
