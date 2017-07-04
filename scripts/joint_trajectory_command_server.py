#!/usr/bin/env python

import argparse

import rospy

from dynamic_reconfigure.server import Server

from baxter_interface.cfg import (
    PositionJointTrajectoryActionServerConfig,
    VelocityJointTrajectoryActionServerConfig,
    PositionFFJointTrajectoryActionServerConfig,
)
from baxter_interface.limb import Limb
from baxter_interface.head import Head
from joint_trajectory_action.joint_trajectory_action import (
    JointTrajectoryActionServer,
)

from baxter_core_msgs.msg import HeadPanCommand
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)


g_limb = None
g_head = None


def cb(msg, limb_name):
    for point in msg.points:
        print(limb_name, point.positions, msg.joint_names)
        positions = dict(zip(msg.joint_names, point.positions))
        g_limb[limb_name].move_to_joint_positions(positions)
        #g_limb[limb_name].set_joint_positions(positions)


def cb_head(msg):
    for point in msg.points:
        print(point.positions, msg.joint_names)
        g_head.set_pan(point.positions[0])


def start_server(limb, rate, mode, interpolation):
    print("Initializing node... ")
    rospy.init_node("rsdk_%s_joint_trajectory_command_server%s" %
                    (mode, "" if limb == 'both' else "_" + limb,))
    print("Initializing joint trajectory action server...")

    global g_limb, g_head
    g_limb = {'right': Limb('right'), 'left': Limb('left')}
    g_head = Head()

    sub_left = rospy.Subscriber('/robot/limb/left/command_joint_position', JointTrajectory, cb, callback_args='left')
    sub_right = rospy.Subscriber('/robot/limb/right/command_joint_position', JointTrajectory, cb, callback_args='right')
    sub_head = rospy.Subscriber('/robot/head/head_pan', JointTrajectory, cb_head)
    rospy.spin()


def main():
    arg_fmt = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    parser.add_argument(
        "-l", "--limb", dest="limb", default="both",
        choices=['both', 'left', 'right'],
        help="joint trajectory action server limb"
    )
    parser.add_argument(
        "-r", "--rate", dest="rate", default=100.0,
        type=float, help="trajectory control rate (Hz)"
    )
    parser.add_argument(
        "-m", "--mode", default='position_w_id',
        choices=['position_w_id', 'position', 'velocity'],
        help="control mode for trajectory execution"
    )
    parser.add_argument(
        "-i", "--interpolation", default='bezier',
        choices=['bezier', 'minjerk'],
        help="interpolation method for trajectory generation"
    )
    args = parser.parse_args(rospy.myargv()[1:])
    start_server(args.limb, args.rate, args.mode, args.interpolation)


if __name__ == "__main__":
    main()
