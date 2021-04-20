#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
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

    def update(self, x, y, z, ax, ay, az, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z

        self.ax = ax
        self.ay = ay
        self.az = az

        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            # twist.angular.x = 0
            # twist.angular.y = 0
            # twist.angular.z = self.th * self.turn

            twist.angular.x = self.ax
            twist.angular.y = self.ay
            twist.angular.z = self.az

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
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


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)


class Listener():
    def __init__(self):
        self.sub = rospy.Subscriber('duck_controller', Twist, self.callback)
        self.measurements = None

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
        self.measurements = data

    def get_measurements(self):
        return self.measurements

if __name__=="__main__":
    rospy.init_node('duck_action_node', anonymous=True)
    sub_thread = Listener()

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None
    pub_thread = PublishThread(repeat)

    try:
        while not rospy.is_shutdown():
            if sub_thread.get_measurements():
                # print("Got measurements")
                vel_msg = sub_thread.get_measurements()
                x = vel_msg.linear.x
                y = vel_msg.linear.y
                z = vel_msg.linear.z
                ax = vel_msg.angular.x
                ay = vel_msg.angular.y
                az = vel_msg.angular.z
                
                th = 0
                pub_thread.update(x, y, z, ax, ay, az, th, speed, turn)


    except Exception as e:
        print(e)

    pub_thread.stop()






# settings = termios.tcgetattr(sys.stdin)

# speed = rospy.get_param("~speed", 0.5)
# turn = rospy.get_param("~turn", 1.0)
# repeat = rospy.get_param("~repeat_rate", 0.0)
# key_timeout = rospy.get_param("~key_timeout", 0.0)
# if key_timeout == 0.0:
#     key_timeout = None

# pub_thread = PublishThread(repeat)
# # sub_thread = Listener()
# rate_limiter = rospy.Rate(100)
# print("Finish init")

# rospy.Subscriber('duck_controller', String, callback)
# rospy.spin()
# pub_thread.stop()

# try:
#     pub_thread.wait_for_subscribers()
#     x = 0
#     y = 0
#     z = 0
#     th = 0
#     pub_thread.update(x, y, z, th, speed, turn)

#     while(1):
#         key = getKey(key_timeout)
#         if (key == '\x03'):
#             break    
#         # if sub_thread.measurements:
#         print(sub_thread.measurements)

# except Exception as e:
#     print(e)

# try:
#     # while not rospy.is_shutdown():
#         # key = getKey(key_timeout)
#         # print("key", key)
#         # if (key == '\x03'):
#         #     break    
#         # if sub_thread.measurements:
#     print(sub_thread.measurements)
#     # rate_limiter.sleep()
#     rospy.spin()

# except Exception as e:
#     print(e)
# termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


# settings = termios.tcgetattr(sys.stdin)
# rospy.init_node('duck_action_node')

# speed = rospy.get_param("~speed", 0.5)
# turn = rospy.get_param("~turn", 1.0)
# repeat = rospy.get_param("~repeat_rate", 0.0)
# key_timeout = rospy.get_param("~key_timeout", 0.0)
# if key_timeout == 0.0:
#     key_timeout = None

# pub_thread = PublishThread(repeat)

# x = 0
# y = 0
# z = 0
# th = 0
# status = 0

# try:
#     pub_thread.wait_for_subscribers()
#     pub_thread.update(x, y, z, th, speed, turn)

#     print(msg)
#     print(vels(speed,turn))
#     i = 0
#     toggle = 2 #TODO motion update rate
#     while(1):
#         key = getKey(key_timeout)
#         print("key", key)
#         if i == 0:
#             # TODO subscribe to controller
#             x = 0
#             y = 0
#             z = 1
#             th = 0
#             speed = 0.5
#             turn = 1
#             i += 1
#         elif (key == '\x03'):
#             break
#         else:
#             i += 1
#             x = 0
#             y = 0
#             z = 0
#             th = 0
#             if i == toggle:
#                 i = 0
#             continue

#         pub_thread.update(x, y, z, th, speed, turn)

# except Exception as e:
#     print(e)

# finally:
#     pub_thread.stop()

#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
