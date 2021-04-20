#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import threading
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
# from TFMessage.msg import tf2_msgs
from sensor_msgs.msg import PointCloud2 as pc2
from laser_geometry import LaserProjection
from geometry_msgs.msg import Twist
import tf


class PublishThread(threading.Thread):
  def __init__(self, rate):
    super(PublishThread, self).__init__()
    self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    self.x = 0.0
    self.y = 0.0
    self.z = 0.0
    self.ax = 0.0
    self.ay = 0.0
    self.az = 0.0
    self.condition = threading.Condition()
    self.done = False
    if rate != 0.0:
        self.timeout = 1.0 / rate
    else:
        self.timeout = None
    self.start()

  def update(self, x, y, z, ax, ay, az):
    self.condition.acquire()
    self.x = x
    self.y = y
    self.z = z
    self.ax = ax
    self.ay = ay
    self.az = az
    # Notify publish thread that we have a new message.
    self.condition.notify()
    self.condition.release()

  def stop(self):
    self.done = True
    self.update(0, 0, 0, 0, 0, 0)
    self.join()

  def run(self):
    twist = Twist()
    while not self.done:
      self.condition.acquire()
      # Wait for a new message or timeout.
      self.condition.wait(self.timeout)

      # Copy state into twist message.
      twist.linear.x = self.x
      twist.linear.y = self.y
      twist.linear.z = self.z
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

class Laser2PC():
  def __init__(self):
    self._measurements = [float('inf')]
    self.laserProj = LaserProjection()
    self.pcPub = rospy.Publisher('/laserPointCloud', pc2, queue_size=1)
    self.laserSub = rospy.Subscriber('/scan', LaserScan, self.laserCallBack)

  def laserCallBack(self, data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
    cloud_out = self.laserProj.projectLaser(data)
    self._measurements = cloud_out
    self.pcPub.publish(cloud_out)

  @property
  def ready(self):
    return not np.isnan(self._measurements)

  @property
  def measurements(self):
    return self._measurements

if __name__ == '__main__':
  rospy.init_node('duck_controller_node') #Name of publisher

  pub_thread = PublishThread(0.0)
  pub_thread.update(0, 0, 1, 0, 0, 0)
  rate_limiter = rospy.Rate(100)

  laser = Laser2PC()

  try:
    while not rospy.is_shutdown():
      pub_thread.update(0, 0, 0.5, 1, 1, 0)
      rate_limiter.sleep()
  except Exception as e:
    print(e)

  pub_thread.stop()








#---------------------------------------------------------------------------------------


# import rospy
# from geometry_msgs.msg import Twist #ros msg that deals with moving the robot
# from sensor_msgs.msg import LaserScan #ros msg that gets the laser scans
# import time

# # obstacle threshhold, objects a this distance or below it
# #are considered obstacles
# OBSTACLE_DIST = 0.5
# #the angle in which each region extends
# REGIONAL_ANGLE = 30
# PI = 3.141592653

# #when there's no obstacles, the robot will move with this linear velocity
# NORMAL_LIN_VEL = 0.50 #meters/second
# #after detecting an obstacle, the robot shall back up a bit (negative) while
# # rotating to help in case it can't perform a stationary rotation
# TRANS_LIN_VEL = -0.05
# #the robot always rotates with the same value of angular velocity
# TRANS_ANG_VEL = 1.75

# #this list keeps track of the order in which the regions' readings are obtained
# REGIONS = [
#              "front_C", "front_L", "left_R",
#              "left_C", "left_L", "back_R",
#              "back_C", "back_L", "right_R",
#              "right_C", "right_L", "front_R",
#           ]
# #this is a global variable that keeps handles the orders for the robot to follow
# #if there's a detected object, "act" is turned to True
# #and the angular_vel and sleep values are calculated appropriately
# Urgency_Report = {
#                     "act": False, "angular_vel": 0.0, "sleep": 0
#                  }
# #this dict keeps track of the distance measures for each region
# Regions_Report = {
#                      "front_C":[], "front_L":[], "left_R":[],
#                      "left_C":[], "left_L":[], "back_R":[],
#                      "back_C":[], "back_L":[], "right_R":[],
#                      "right_C":[], "right_L":[], "front_R":[],
#                  }
# #These are the costs to deviate from each region to the goal region (front_C)
# Regions_Distances = {
#                      "front_C": 0, "front_L": 1, "left_R": 2,
#                      "left_C": 3, "left_L": 4, "back_R": 5,
#                      "back_C": 6, "back_L": -5, "right_R": -4,
#                      "right_C": -3, "right_L": -2, "front_R": -1,
#                  }

# #in this function the clearest paths are calculated and the appropriate
# #values for the angular_vel and the execution times are assigned
# def ClearanceTest():
#     global Urgency_Report

#     goal = "front_C"
#     closest = 10e6
#     regional_dist = 0
#     maxima = {"destination": "back_C", "distance": 10e-6}
#     for region in Regions_Report.items():
#         regional_dist = abs(Regions_Distances[region[0]]-Regions_Distances[goal])
#         #if there're no obstacles in that region
#         if not len(region[1]):
#             #check if it's the cheapest option
#             if (regional_dist < closest):
#                 closest = regional_dist
#                 maxima["distance"] = OBSTACLE_DIST
#                 maxima["destination"] = region[0]
#         #check if it's the clearest option
#         elif(max(region[1]) > maxima["distance"]):
#             maxima["distance"] = max(region[1])
#             maxima["destination"] = region[0]

#     #calculate the cost to the chosen orientation
#     regional_dist = Regions_Distances[maxima["destination"]]-Regions_Distances[goal]

#     #we act whenever the clearest path is not the front_C (front center)
#     Urgency_Report["act"] = (closest != 0)
#     Urgency_Report["angular_vel"] = ((regional_dist/max(1, abs(regional_dist)))
#                                     *TRANS_ANG_VEL)
#     Urgency_Report["sleep"] = ((abs(regional_dist)*REGIONAL_ANGLE*PI)
#                               /(180*TRANS_ANG_VEL))

# def IdentifyRegions(scan):
#     global Regions_Report
#     for i, region in enumerate(REGIONS):
#         Regions_Report[region] = [
#                 x for x in scan.ranges[REGIONAL_ANGLE*i : REGIONAL_ANGLE*(i+1)]
#                         if x <= OBSTACLE_DIST and x != 'inf']

# def Steer(velocity):
#     global Urgency_Report

#     #since we're moving only on the plane, all we need is move in the x axis,
#     #and rotate in the z (zeta) axis.
#     velocity.linear.x = TRANS_LIN_VEL
#     velocity.linear.y = 0
#     velocity.linear.z = 0.5
#     velocity.angular.x = 0
#     velocity.angular.y = 0
#     velocity.angular.z = Urgency_Report["angular_vel"]

#     return velocity

# def main():
#     #Initialize our node
#     rospy.init_node("duck_controller_node")
#     #Subscribe to the "/scan" topic in order to read laser scans data from it
#     rospy.Subscriber("/scan", LaserScan, IdentifyRegions)
#     #create our publisher that'll publish to the "/cmd_vel" topic
#     pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
#     vel = Twist()
#     #ros will try to run this code 10 times/second
#     rate = rospy.Rate(10) #10Hz
#     #keep running while the ros-master isn't isn't shutdown
#     while not rospy.is_shutdown():
#         ClearanceTest()
#         if(Urgency_Report["act"]):
#             vel = Steer(vel)
#         else:
#             vel.linear.x = NORMAL_LIN_VEL
#             vel.linear.y = 0
#             vel.linear.z = 0.5
#             vel.angular.x = 0
#             vel.angular.y = 0
#             vel.angular.z = 0
#         pub.publish(vel)
#         #after publishing our action, we give it some time to execute the
#         #needed actions before reading the data again.
#         time.sleep(Urgency_Report["sleep"])
#         rate.sleep()

# if __name__ == "__main__":
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass


















#---------------------------------------------------------------------------------------











# def run():
#   rospy.init_node('duck_controller_node') #Name of publisher

#   # Update control every 100 ms.
#   rate_limiter = rospy.Rate(100)
#   publisher = rospy.Publisher('/duck_controller', Twist, queue_size=10) #Topic being published to
#   vel_msg = Twist()
#   # laser = Laser2PC()

#   while not rospy.is_shutdown():
#     # if not laser.ready:
#     #   rate_limiter.sleep()
#     #   continue
    
#     # u, w = move(*laser.measurements)
#     # u = 1 #TODO change to func
#     # w = 1 #TODO change to func
#     vel_msg.linear.x = 0
#     vel_msg.linear.y = 0
#     vel_msg.linear.z = 0.5
#     vel_msg.angular.x = 0
#     vel_msg.angular.y = 0
#     vel_msg.angular.z = 0 
#     rospy.loginfo(vel_msg)
#     publisher.publish(vel_msg)

#     rate_limiter.sleep()









# def move(front, front_left, front_right, left, right):
#   def fin(a):
#     # Sigmoid func
#     return (1/(1 + np.exp(-(a-0.5))))

#   au = [0.2,0.1,0.1,0,0]
#   aw = [0,1.1,-1.1,0.9,-0.9,0.5]
#   front, front_left, front_right, left, right = fin(front), fin(front_left), fin(front_right), fin(left), fin(right)
#   u = au[0]*front + au[1]*front_left + au[2]*front_right + au[3]*left + au[4]*right
#   w = aw[0]*front + aw[1]*front_left + aw[2]*front_right + aw[3]*left + aw[4]*right + aw[5]*front*left
#   return 10,w

# class Laser2PC(object):
#   def __init__(self):
#     rospy.Subscriber('/scan', LaserScan, self.callback)
#     self._angles = [0., np.pi / 4., -np.pi / 4., np.pi / 2., -np.pi / 2.]
#     self._width = np.pi / 180. * 10.  # 10 degrees cone of view.
#     self._measurements = [float('inf')] * len(self._angles)
#     self._indices = None

#     self.pcPub = rospy.Publisher('/laserPointCloud', pc2, queue_size=1)
#     self.laserProj = LaserProjection()

#   def callback(self, msg):
#     # print("Using CALLBACK")
#     # Helper for angles.
#     def _within(x, a, b):
#       pi2 = np.pi * 2.
#       x %= pi2
#       a %= pi2
#       b %= pi2
#       if a < b:
#         return a <= x and x <= b
#       return a <= x or x <= b;

#     # Compute indices the first time.
#     if self._indices is None:
#       self._indices = [[] for _ in range(len(self._angles))]
#       for i, d in enumerate(msg.ranges):
#         angle = msg.angle_min + i * msg.angle_increment
#         for j, center_angle in enumerate(self._angles):
#           if _within(angle, center_angle - self._width / 2., center_angle + self._width / 2.):
#             self._indices[j].append(i)

#     ranges = np.array(msg.ranges)
#     for i, idx in enumerate(self._indices):
#       # We do not take the minimum range of the cone but the 10-th percentile for robustness.
#       self._measurements[i] = np.percentile(ranges[idx], 10)

#     cloud_out = self.laserProj.projectLaser(msg)
#     # print("Cloud out")
#     # print(cloud_out)
#     self.pcPub.publish(cloud_out)

#   @property
#   def ready(self):
#     return not np.isnan(self._measurements[0])

#   @property
#   def measurements(self):
#     return self._measurements




  # rospy.init_node('laser2PointCloud')
  # l2pc = Laser2PC()
  # rospy.spin()

# import argparse
# import numpy as np
# import rospy

# # Robot motion commands:
# # http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
# from geometry_msgs.msg import Twist
# # Laser scan message:
# # http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
# from sensor_msgs.msg import LaserScan
# # For groundtruth information.
# from gazebo_msgs.msg import ModelStates
# from tf.transformations import euler_from_quaternion

# def fin(a):
#   # Sigmoid func
#   return (1/(1 + np.exp(-(a-0.5))))

# def braitenberg(front, front_left, front_right, left, right):
#   # MISSING: Implement a braitenberg controller that takes the range
#   # measurements given in argument to steer the robot.
#   # sensors: no obstacle high value, have obstacle low value
#   au = [0.2,0.1,0.1,0,0]
#   aw = [0,1.1,-1.1,0.9,-0.9,0.5]

#   front, front_left, front_right, left, right = fin(front), fin(front_left), fin(front_right), fin(left), fin(right)
  
#   u = au[0]*front + au[1]*front_left + au[2]*front_right + au[3]*left + au[4]*right
#   w = aw[0]*front + aw[1]*front_left + aw[2]*front_right + aw[3]*left + aw[4]*right + aw[5]*front*left

#   return u,w


# def rule_based(front, front_left, front_right, left, right):
#   # MISSING: Implement a rule-based controller that avoids obstacles.
#   u = 0.25
#   w = 0

#   front, front_left, front_right, left, right = fin(front), fin(front_left), fin(front_right), fin(left), fin(right)

#   c_left_front_obs = (front_right-front_left) > 0.1 and (right-left) > 0.1
#   c_right_front_obs = (front_left-front_right) > 0.1 and (left-right) > 0.1
#   c_head_on = (front < 0.5 and abs(front_right-front_left) < 0.1 and abs(front-front_right) < 0.3)
#   sth_wrong = min(front, front_left, front_right, left, right)<0.2
  
#   if (c_head_on or sth_wrong):
#     #u-turn
#     print("U-turn")
#     u = 0.05
#     w = 1/(front - min(left,right))
#   elif (c_left_front_obs):
#     print("Left Obstacle. Turn Right.")
#     u = 0.1
#     w = (front_right-front_left) + 0.2*np.exp(front_right-front_left)
#   elif (c_right_front_obs):
#     print("Right Obstacle. Turn Left.")
#     u = 0.1
#     w = (front_left-front_right) + 0.2*np.exp(front_left-front_right)
#   else:
#     u = 0.5
#     w = 0

#   return u, w


# class SimpleLaser(object):
#   def __init__(self):
#     rospy.Subscriber('/scan', LaserScan, self.callback)
#     self._angles = [0., np.pi / 4., -np.pi / 4., np.pi / 2., -np.pi / 2.]
#     self._width = np.pi / 180. * 10.  # 10 degrees cone of view.
#     self._measurements = [float('inf')] * len(self._angles)
#     self._indices = None

#   def callback(self, msg):
#     # Helper for angles.
#     def _within(x, a, b):
#       pi2 = np.pi * 2.
#       x %= pi2
#       a %= pi2
#       b %= pi2
#       if a < b:
#         return a <= x and x <= b
#       return a <= x or x <= b;

#     # Compute indices the first time.
#     if self._indices is None:
#       self._indices = [[] for _ in range(len(self._angles))]
#       for i, d in enumerate(msg.ranges):
#         angle = msg.angle_min + i * msg.angle_increment
#         for j, center_angle in enumerate(self._angles):
#           if _within(angle, center_angle - self._width / 2., center_angle + self._width / 2.):
#             self._indices[j].append(i)

#     ranges = np.array(msg.ranges)
#     for i, idx in enumerate(self._indices):
#       # We do not take the minimum range of the cone but the 10-th percentile for robustness.
#       self._measurements[i] = np.percentile(ranges[idx], 10)

#   @property
#   def ready(self):
#     return not np.isnan(self._measurements[0])

#   @property
#   def measurements(self):
#     return self._measurements


# class GroundtruthPose(object):
#   def __init__(self):
#     rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
#     self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
#     self._name = 'quadrotor'

#   def callback(self, msg):
#     idx = [i for i, n in enumerate(msg.name) if n == self._name]
#     if not idx:
#       raise ValueError('Specified name "{}" does not exist.'.format(self._name))
#     idx = idx[0]
#     self._pose[0] = msg.pose[idx].position.x
#     self._pose[1] = msg.pose[idx].position.y
#     _, _, yaw = euler_from_quaternion([
#         msg.pose[idx].orientation.x,
#         msg.pose[idx].orientation.y,
#         msg.pose[idx].orientation.z,
#         msg.pose[idx].orientation.w])
#     self._pose[2] = yaw

#   @property
#   def ready(self):
#     return not np.isnan(self._pose[0])

#   @property
#   def pose(self):
#     return self._pose
  

# def run(args):
#   rospy.init_node('obstacle_avoidance')
#   avoidance_method = globals()[args.mode]

#   # Update control every 100 ms.
#   rate_limiter = rospy.Rate(100)
#   publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
#   laser = SimpleLaser()
#   # Keep track of groundtruth position for plotting purposes.
#   groundtruth = GroundtruthPose()
#   pose_history = []
#   # with open('/tmp/gazebo_exercise.txt', 'w'):
#   #   pass

#   while not rospy.is_shutdown():
#     # Make sure all measurements are ready.
#     if not laser.ready or not groundtruth.ready:
#       rate_limiter.sleep()
#       continue
    
#     print("Laser measurements:", laser.measurements)
#     u, w = avoidance_method(*laser.measurements)
#     vel_msg = Twist()
#     vel_msg.linear.x = u
#     vel_msg.angular.z = w
#     publisher.publish(vel_msg)

#     # Log groundtruth positions in /tmp/gazebo_exercise.txt
#     pose_history.append(groundtruth.pose)
#     if len(pose_history) % 10:
#       # with open('/tmp/gazebo_exercise.txt', 'a') as fp:
#       #   fp.write('\n'.join(','.join(str(v) for v in p) for p in pose_history) + '\n')
#       pose_history = []
#     rate_limiter.sleep()


# if __name__ == '__main__':
#   parser = argparse.ArgumentParser(description='Runs obstacle avoidance')
#   parser.add_argument('--mode', action='store', default='braitenberg', help='Method.', choices=['braitenberg', 'rule_based'])
#   args, unknown = parser.parse_known_args()
#   try:
#     run(args)
#   except rospy.ROSInterruptException:
#     pass
