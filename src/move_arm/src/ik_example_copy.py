#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseArray
from logitech_cam.srv import PositionSrv
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from intera_interface import gripper as robot_gripper
from planning import plan_pyramid


increment_z = 0.150
start_y = 0.285
# Big Cups
cup_diameter = 0.378 - 0.285
cup_height = cup_diameter*1.25

# Small Cups
cup_diameter = (0.378 - 0.285) - 0.01
cup_height = cup_diameter*1.1

quat = [0.0, 1.0, 0.0, 0.0]
neg_z = -0.099
pos_z = 0.099
num_cups = 0
small_x_offset = -0.01
big_x_offset = -0.04
TR_x_offset = -0.05
big_y_offset = 0.015
small_y_offset = -0.005

sawyer_bl = [0.871, 0.046]
sawyer_tr = [0.474, 0.737]     # [0.444, 0.737] 
sawyer_tl = [0.517, 0.046]        # [0.487, 0.046]
sawyer_br = [sawyer_bl[0], sawyer_tr[1]]
sawyer_x = sawyer_bl[0] - sawyer_tl[0]
sawyer_y = sawyer_tr[1] - sawyer_bl[1]

# [min_x, max_x, min_y, max_y]
TL_grid = [sawyer_tl[0], sawyer_bl[0] - sawyer_x / 2, sawyer_tl[1], sawyer_tr[1] - sawyer_y / 2]
TR_grid = [sawyer_tl[0], sawyer_bl[0] - sawyer_x / 2, sawyer_tr[1] - sawyer_y / 2, sawyer_tr[1]]
BL_grid = [sawyer_bl[0] - sawyer_x / 2, sawyer_bl[0], sawyer_tl[1], sawyer_tr[1] - sawyer_y / 2]
BR_grid = [sawyer_bl[0] - sawyer_x / 2, sawyer_bl[0], sawyer_tr[1] - sawyer_y / 2, sawyer_tr[1]]

start_trans = []

def calculate_inter_trans_positions(trans):
    x,y,z = trans[0], trans[1], trans[2]
    return [x, y, z + increment_z]

def construct_request(trans, i, is_start_trans):
    # Construct the request
    link = "right_gripper_tip"
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"
    request.ik_request.ik_link_name = link
    request.ik_request.pose_stamped.header.frame_id = "base"

    trans_x = trans[i][0]
    trans_y = trans[i][1]

    # If cup is TL or TR
    if not is_start_trans:
        request.ik_request.pose_stamped.pose.position.x = trans_x
        request.ik_request.pose_stamped.pose.position.y = trans_y
    elif trans_x >= TL_grid[0] and trans_x <= TL_grid[1]:
        # TL grid: big x & big y
        if trans_y >= sawyer_tl[1] and trans_y <= sawyer_tr[1] - sawyer_y / 2:
            request.ik_request.pose_stamped.pose.position.x = trans_x + big_x_offset
            request.ik_request.pose_stamped.pose.position.y = trans_y + big_y_offset        # make smaller?
        # TR grid: TR_x_offset & small y
        elif trans_y >= sawyer_tr[1] - sawyer_y / 2 and trans_y <= sawyer_tr[1]:
            request.ik_request.pose_stamped.pose.position.x = trans_x + TR_x_offset
            request.ik_request.pose_stamped.pose.position.y = trans_y + small_y_offset
    elif trans_x >= BL_grid[0] and trans_x <= BL_grid[1]:
        # BL grid: small x & big y
        if trans_y >= sawyer_tl[1] and trans_y <= sawyer_tr[1] - sawyer_y / 2:
            request.ik_request.pose_stamped.pose.position.x = trans_x + small_x_offset
            request.ik_request.pose_stamped.pose.position.y = trans_y + big_y_offset
        # BR grid: big x & big y
        elif trans_y >= sawyer_tr[1] - sawyer_y / 2 and trans_y <= sawyer_tr[1]:
            request.ik_request.pose_stamped.pose.position.x = trans_x + big_x_offset
            request.ik_request.pose_stamped.pose.position.y = trans_y + big_y_offset
    

    
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

# pos_sub = rospy.Subscriber("cup_locations", PoseArray, callback)
def stacking(start_trans, num_cups):
    input('Press [ Enter ] to start stacking: ')
        
    # Construct the request
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    link = "right_gripper_tip"

    request.ik_request.ik_link_name = link
    # request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"

    start_inter_trans = []
    for i in range(num_cups):
        start_inter_trans.append(calculate_inter_trans_positions(start_trans[i]))

    # start_inter_trans = [start_inter_trans1, start_inter_trans2, start_inter_trans3]

    
    end_x, end_y, end_z = 0.793, 0 - (num_cups//2)*cup_diameter, neg_z
    end_trans = plan_pyramid(num_cups, end_x, end_y, end_z, cup_diameter, cup_height)
    num_valid_cups = len(end_trans)

    # end_inter_trans = [end_inter_trans1, end_inter_trans2, end_inter_trans3]
    end_inter_trans = []
    for i in range(num_valid_cups):
        print(num_cups)
        print(end_trans)
        end_inter_trans.append(calculate_inter_trans_positions(end_trans[i]))

    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')

    # Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(3.0)

    for i in range(num_valid_cups):
        # Construct the request
        start_request = construct_request(start_trans, i, True)
        # Construct the start inter request to move box to goal
        start_inter_request = construct_request(start_inter_trans, i, False)
        # Construct the end inter request to move box to goal
        end_inter_request = construct_request(end_inter_trans, i, False)
        # Construct the end request to move box to goal
        end_request = construct_request(end_trans, i, False)
        print("Trying to move cup number ", str(i + 1))

        # Open the right gripper
        print('Opening...')
        right_gripper.open()     
        rospy.sleep(2.0)
        print('Done!')


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
            print("Cup number ", i)
            print("End inter trans", end_inter_trans[i])
            move_to_position(end_inter_request, "end_inter")
            # move to end
            print("End trans", end_trans[i])
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

    return end_trans

def destacking(start_trans, num_cups):

    input('Press [ Enter ] for destacking: ')
        
    # Construct the request
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    link = "right_gripper_tip"

    request.ik_request.ik_link_name = link
    # request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"

    start_inter_trans = []
    for i in range(num_cups):
        start_inter_trans.append(calculate_inter_trans_positions(start_trans[i]))

    # start_inter_trans = [start_inter_trans1, start_inter_trans2, start_inter_trans3]

    
    end_x_pos = sawyer_x / 2 + sawyer_tl[0]
    end_y_pos = sawyer_y / 2 + sawyer_tl[1]
    end_z_pos = neg_z
    end_inter_trans = [[end_x_pos, end_y_pos, end_z_pos + 2*cup_height]]


    # # end_inter_trans = [end_inter_trans1, end_inter_trans2, end_inter_trans3]
    # end_inter_trans = []
    # for i in range(num_cups):
    #     print(num_cups)
    #     print(end_trans)
    #     end_inter_trans.append(calculate_inter_trans_positions(end_trans[i]))

    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')

    # # Calibrate the gripper (other commands won't work unless you do this first)
    # print('Calibrating...')
    # right_gripper.calibrate()
    # rospy.sleep(2.0)

    # Open the right gripper
    print('Opening...')
    right_gripper.open()
    rospy.sleep(1.0)
    print('Done!')

    num_valid_cups = len(start_trans)

    for i in range(num_valid_cups-1, -1, -1):
        # Construct the request
        start_request = construct_request(start_trans, i, False)
        # Construct the start inter request to move box to goal
        start_inter_request = construct_request(start_inter_trans, i, False)

        # Construct the end inter request to move box to goal
        end_inter_request = construct_request(end_inter_trans, 0, False)
        # Construct the end request to move box to goal
        end_trans = [[end_x_pos, end_y_pos, end_z_pos + (num_valid_cups-i)*0.03]]
        end_request = construct_request(end_trans, 0, False)
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
            print("Cup number ", i)
            print("End inter trans", end_inter_trans[0])
            move_to_position(end_inter_request, "end_inter")
            # move to end
            print("End trans", end_trans[0])
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

def main():
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    rospy.wait_for_service('cup_locations')
    position_service = rospy.ServiceProxy("cup_locations", PositionSrv)
    positions = position_service().positions

    print("Position in Sawyer coordinates:", positions)
    start_trans = []
    for p in positions.poses:
        p = p.position
        start_trans.append([p.x, p.y, p.z])
    start_trans.sort(key = lambda x: x[1])
    num_cups = len(positions.poses)
    print("CALLBACK:", start_trans)
    print("Num start positions: " + str(num_cups))

    while not rospy.is_shutdown():
        end_trans = stacking(start_trans=start_trans, num_cups=num_cups)

        destacking(end_trans, num_cups)

        # Terminate upon Ctrl+c
        rospy.spin()

if __name__ == '__main__':
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    
    main()
