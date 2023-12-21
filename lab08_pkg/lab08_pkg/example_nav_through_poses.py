#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from math import sin, cos , pi
from rclpy.duration import Duration

"""
Basic navigation demo to go to pose.
"""
def yaw_to_quaternion(yaw):
    return [0.0, 0.0, sin(yaw / 2.0), cos(yaw / 2.0)]

def array_to_pose_stamped(pose_array, frame_id='map'):
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame_id
    pose_stamped.header.stamp = rclpy.clock.Clock().now().to_msg()
    pose_stamped.pose.position.x = pose_array[0]
    pose_stamped.pose.position.y = pose_array[1]
    q = yaw_to_quaternion(pose_array[2])
    pose_stamped.pose.orientation.z = q[2]
    pose_stamped.pose.orientation.w = q[3]
    return pose_stamped

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    # init_pose = [-2.05, -0.50, 0.0] # x, y, yaw
    # if no initial pose is set, the robot will use the last one it received
    # navigator.setInitialPose(array_to_pose_stamped(init_pose))

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # Go to our demos first goal pose
    goal_poses = []
    
    ################################################
    #
    # Append your goal poses here!
    #
    ################################################

    # transform goal poses to pose stamped
    goal_poses = [array_to_pose_stamped(goal_pose) for goal_pose in goal_poses]

    navigator.goThroughPoses(goal_poses)

    i = 0
    while not navigator.isTaskComplete():

        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    exit(0)


if __name__ == '__main__':
    main()