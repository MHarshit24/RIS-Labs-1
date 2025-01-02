#!/usr/bin/env python

from __future__ import print_function

import threading
import rospy

from geometry_msgs.msg import Wrench

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Wrench!
---------------------------
    0               : for setting all values to zero
Force:
    i               : for force in negative x-axis  
    k               : for force in positive x-axis 
    j               : for force in negative y-axis
    l               : for force in positive y-axis 
    n               : for force in negative z-axis
    m               : for force in positive z-axis 

Torque:
    w               : for torque in negative x-axis  
    s               : for torque in positive x-axis 
    a               : for torque in negative y-axis
    d               : for torque in positive y-axis 
    z               : for torque in negative z-axis
    x               : for torque in positive z-axis 
CTRL-C to quit
"""

moveBindings = {
        'i':(-1,0,0,0,0,0),
        'k':(1,0,0,0,0,0),
        'j':(0,-1,0,0,0,0),
        'l':(0,1,0,0,0,0),
        'n':(0,0,-1,0,0,0),
        'm':(0,0,1,0,0,0),

        'w':(0,0,0,-1,0,0),
        's':(0,0,0,1,0,0),
        'a':(0,0,0,0,-1,0),
        'd':(0,0,0,0,1,0),
        'z':(0,0,0,0,0,-1),
        'x':(0,0,0,0,0,1),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/rexrov/thruster_manager/input', Wrench, queue_size = 1)
        self.fx = 0.0
        self.fy = 0.0
        self.fz = 0.0
        self.tx = 0.0
        self.ty = 0.0
        self.tz = 0.0
        self.flag = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, fx, fy, fz, tx, ty, tz, flag):
        self.condition.acquire()
        if flag == 1:
            self.fx = 0
            self.fy = 0
            self.fz = 0
            self.tx = 0
            self.ty = 0
            self.tz = 0
        else:
            self.fx += fx
            self.fy += fy
            self.fz += fz
            self.tx += tx
            self.ty += ty
            self.tz += tz
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0, 1)
        self.join()

    def run(self):
        twist = Wrench()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.force.x = self.fx
            twist.force.y = self.fy
            twist.force.z = self.fz
            twist.torque.x = self.tx
            twist.torque.y = self.ty
            twist.torque.z = self.tz

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.force.x = 0
        twist.force.y = 0
        twist.force.z = 0
        twist.torque.x = 0
        twist.torque.y = 0
        twist.torque.z = 0
        self.publisher.publish(twist)



def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_forces_torques')

    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)

    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    fx = 0
    fy = 0
    fz = 0
    tx = 0
    ty = 0
    tz = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(fx, fy, fz, tx, ty, tz, 0)

        while(1):
            flag = 0
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                fx = moveBindings[key][0]
                fy = moveBindings[key][1]
                fz = moveBindings[key][2]
                tx = moveBindings[key][3]
                ty = moveBindings[key][4]
                tz = moveBindings[key][5]
            
            elif key == '0':
                flag = 1
            else:
                # Skip updating cmd_vel if key timeout and robot already stopped.
                if key == '' and fx == 0 and fy == 0 and fz == 0 and tx == 0 and ty == 0 and tz == 0:
                    continue
                fx = 0
                fy = 0
                fz = 0
                tx = 0
                ty = 0
                tz = 0
                if (key == '\x03'):
                    break

            pub_thread.update(fx, fy, fz, tx, ty, tz, flag)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
