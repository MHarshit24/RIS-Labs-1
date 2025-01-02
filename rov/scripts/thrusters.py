#!/usr/bin/env python

from __future__ import print_function
import rospy
import sys
from geometry_msgs.msg import Wrench, Vector3, Point
from gazebo_msgs.srv import ApplyBodyWrench


class ThrustCommander(object):
    def __init__(self):
        self.fx = 0
        self.fy = 0
        self.fz = 0
        self.tx = 0
        self.ty = 0
        self.tz = 0
        self.starting_time = 1000
        self.duration = 10000

    def wrench_callback(self, msg):
        self.fx = msg.force.x
        self.fy = msg.force.y
        self.fz = msg.force.z
        self.tx = msg.torque.x
        self.ty = msg.torque.y
        self.tz = msg.torque.z

def main():
    rospy.init_node('thrusters')
    if rospy.is_shutdown():
        print('ROS master not running!')
        sys.exit(-1)
    monitor = ThrustCommander()

    if rospy.has_param('~starting_time'):
        monitor.starting_time = rospy.get_param('~starting_time')

    print('Starting time= {} s'.format(monitor.starting_time))

    if monitor.duration == 0.0:
        print('Duration not set, leaving node...')
        sys.exit(-1)

    sub = rospy.Subscriber('thruster_manager/input',
                            Wrench, monitor.wrench_callback)
    try:
        rospy.wait_for_service('/gazebo/apply_body_wrench', timeout=10)
    except rospy.ROSException:
        print('Service not available! Closing node...')
        sys.exit(-1)
 
    try:
        push_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    except rospy.ServiceException as e:
        print('Service call failed, error=', e)
        sys.exit(-1)
    ns = rospy.get_namespace().replace('/', '')
    body_name = "base_link"

    if monitor.starting_time >= 0:
        rate = rospy.Rate(100)
        while rospy.get_time() < monitor.starting_time:
            rate.sleep()

    force = [monitor.fx, monitor.fy, monitor.fz]
    torque = [monitor.tx, monitor.ty, monitor.tz]
    print(force, torque)
    wrench = Wrench()
    wrench.force = Vector3(*force)
    wrench.torque = Vector3(*torque)
    success = push_wrench(
        body_name,
        'world',
        Point(0, 0, 0),
        wrench,
        rospy.Time().now(),
        rospy.Duration(monitor.duration))
    if success:
        print('Body wrench perturbation applied!')
        print('\tFrame: ', body_name)
        print('\tDuration [s]: ', monitor.duration)
        print('\tForce [N]: ', force)
        print('\tTorque [Nm]: ', torque)
    else:
        print('Failed!')
if __name__ == '__main__':
    main()
