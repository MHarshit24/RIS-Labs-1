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
        self.starting_time = 0
        self.duration = 0


    def get_apply_body_wrench(self):
        try:
            return rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        except rospy.ServiceException as e:
            print('Service call failed, error=', e)
            sys.exit(-1)

    def wrench_callback(self, msg):
        self.fx = msg.wrench.force.x
        self.fy = msg.wrench.force.y
        self.fz = msg.wrench.force.z
        self.tx = msg.wrench.torque.x
        self.ty = msg.wrench.torque.y
        self.tz = msg.wrench.torque.z

def main():
    rospy.init_node('thrust_commander')
    if rospy.is_shutdown():
        print('ROS master not running!')
        sys.exit(-1)
    monitor = ThrustCommander()

    if rospy.has_param('~starting_time'):
        monitor.starting_time = rospy.get_param('~starting_time')

    print('Starting time= {} s'.format(monitor.starting_time))

    if rospy.has_param('~duration'):
            monitor.duration = rospy.get_param('~duration')

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

    push_wrench = monitor.get_apply_body_wrench()
    ns = rospy.get_namespace().replace('/', '')
    body_name = '%s/link1' % ns

    if monitor.starting_time >= 0:
        rate = rospy.Rate(100)
        while rospy.get_time() < monitor.starting_time:
            rate.sleep()

    force = [monitor.fx, monitor.fy, monitor.fz]
    torque = [monitor.tx, monitor.ty, monitor.tz]
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
    rospy.spin()
if __name__ == '__main__':
    main()
