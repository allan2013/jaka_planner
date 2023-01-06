#!/usr/bin/env python
# !/usr/bin/python

import time
import copy
import csv
import rospy
import moveit_commander
import actionlib
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from moveit_msgs.msg import PickupAction, PickupGoal, Grasp, \
    MoveItErrorCodes, AttachedCollisionObject, CollisionObject, \
    OrientationConstraint, RobotTrajectory
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import FollowJointTrajectoryAction
import numpy as np


class MotionPlanner:
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.moveit_group = moveit_commander.MoveGroupCommander("jaka_zu7")
        self.way_points = []
        real = False
        if real:
            fjtname = '/follow_joint_trajectory'
        else:
            fjtname = '/jaka_zu7_controller/follow_joint_trajectory'
        self.action_client = actionlib.SimpleActionClient(fjtname, FollowJointTrajectoryAction)

        rospy.loginfo('waiting for follow_joint_trajectory action.')
        self.action_client.wait_for_server()
        rospy.loginfo('ok!')

    def load_csv(self):
        with open('test_JAKA.csv') as csv_file:
            reader = csv.reader(csv_file)
            line_count = 0
            for row in reader:
                if line_count > 0:
                    point = JointTrajectoryPoint()
                    item_counter = 0
                    for a in row:
                        if item_counter == 0:
                            point.time_from_start = rospy.rostime.Duration(float(a), 0)
                            print(point.time_from_start)
                        else:
                            point.positions.append(float(a) * np.pi / 180)
                            point.velocities.append(0)
                            point.accelerations.append(0)
                        item_counter = item_counter + 1

                    point.positions = np.array(point.positions)
                    point.velocities = np.array(point.velocities)
                    point.accelerations = np.array(point.accelerations)
                    self.way_points.append(point)

                line_count = line_count + 1
        self.fill_vel_acc()
        print(self.way_points)

    def fill_vel_acc(self):
        item_counter = 0
        for i in range(len(self.way_points)):
            if i == 0:
                continue
            duration = float(
                self.way_points[i].time_from_start.to_sec() - self.way_points[i - 1].time_from_start.to_sec())
            velocities = (self.way_points[i].positions - self.way_points[i - 1].positions) / duration
            self.way_points[i].velocities = velocities

        for i in range(len(self.way_points)):
            if i == 0:
                continue
            duration = float(
                self.way_points[i].time_from_start.to_sec() - self.way_points[i - 1].time_from_start.to_sec())
            accelerations = (self.way_points[i].velocities - self.way_points[i - 1].velocities) / duration
            self.way_points[i].accelerations = accelerations

        for i in range(len(self.way_points)):
            self.way_points[i].positions = list(self.way_points[i].positions)
            self.way_points[i].velocities = list(self.way_points[i].velocities)
            self.way_points[i].accelerations = list(self.way_points[i].accelerations)


    def action_send_goal(self, trj):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trj.joint_trajectory
        self.action_client.send_goal(goal)
        rospy.loginfo("action_send_goal")

    def run(self):
        current_state = self.moveit_group.get_current_state()
        print(current_state)
        print("==================================")

        self.moveit_group.set_joint_value_target(arg1=current_state.joint_state)

        plan = self.moveit_group.plan()
        plan.joint_trajectory.points = self.way_points
        # print(plan)

        # self.moveit_group.execute(plan)
        self.action_send_goal(plan)

        pass


if __name__ == '__main__':
    rospy.init_node('asahi', anonymous=True)
    print("============ Initialize===============")

    mp = MotionPlanner()
    mp.load_csv()
    mp.run()
