#!/usr/bin/env python

from __future__ import print_function

import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty
import math

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
        w
   a    s    d

space to stop!

CTRL-C to quit
"""

moveBindings = {
        'w':(1,0,0,0),
        'a':(0,0,0,1),
        'd':(0,0,0,-1),
        's':(-1,0,0,0),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard')

    maxLinearSpeed = 10
    linearSpeedIncrement = rospy.get_param("~linearSpeedIncrement", 0.2)
    maxAngularSpeed = rospy.get_param("~maxAngularSpeed", 2.0)
    angularSpeedIncrement = rospy.get_param("~angularSpeedIncrement", 0.2)
    x = 0
    th = 0
    twist = Twist()

    def timerCallback(event):
        pub.publish(twist)

    timer = rospy.Timer(rospy.Duration(0.2), timerCallback)


    try:
        print(msg)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                # TODO
                #x <-  moveBindings[key][0]
                #th <-  moveBindings[key][3]

            elif key == ' ':
                x = 0.0
                th = 0.0

            if (key == '\x03'):
                break

            twist.linear.x = x; twist.linear.y = 0.0; twist.linear.z = 0.0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

