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
from geometry_msgs.msg import PoseStamped
import tf
from nav_msgs.msg import Path
import math
from hector_nav_msgs.srv import GetRobotTrajectory

# hector_nav_msgs::GetRobotTrajectory::Response trajectory_

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

class TrajectoryListener():
  def __init__(self):
    self._path = [float('inf')]
    self.sub = rospy.Subscriber('/exploration_path', Path, self.CallBack)
    self.used_path = False

  def CallBack(self, data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
    self._path = data.poses
    self.used_path = False
  
  def get_measurements(self):
    self.used_path = True
    return self._path
  
  def ready(self):
    return self._path != [float('inf')]

# https://gist.github.com/gajena/ed61f7eea7c6a967cc9a4171a07fc13f
def quaternion_to_euler_angle(w, x, y, z):
	ysqr = y * y

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.degrees(math.atan2(t0, t1))

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.degrees(math.atan2(t3, t4))

	return X, Y, Z

def getTwist(path):
  counter = 0
  dt = 1./50.
  x = 0
  y = 0
  twist_path = []

  for idx, elem in enumerate(trajectorySub._path):
    (v_roll,v_pitch,v_yaw) = quaternion_to_euler_angle(elem.pose.orientation.w, elem.pose.orientation.x , elem.pose.orientation.y, elem.pose.orientation.z)
    v_psi = float((v_yaw))
    yaw = math.radians(v_psi)

    x = elem.pose.position.x
    y = elem.pose.position.y
    z = elem.pose.position.z

    if idx == 0:
      x_prev = x
      y_prev = y
      z_prev = z
      counter += 1
      continue

    vel_x_world = (x - x_prev) / dt
    vel_y_world = (y - y_prev) / dt
    x_prev = x
    y_prev = y
    twist_x = math.cos(yaw) * vel_x_world + math.sin(yaw) * vel_y_world
    twist_y = math.cos(yaw) * vel_y_world - math.sin(yaw) * vel_x_world

    tx = twist_x
    ty = twist_y
    tz = (z - z_prev) / dt
    z_prev = z

    twist_path.append((tx,ty,tz))

  return twist_path

if __name__ == '__main__':
  rospy.init_node('duck_action_node') #Name of publisher

  pub_thread = PublishThread(0.0)
  pub_thread.update(0, 0, 1, 0, 0, 0)
  rate_limiter = rospy.Rate(100)
  previous_time = rospy.Time.now().to_sec()
  frame_id = 0
  path_publisher = rospy.Publisher('/path', Path, queue_size=1)

  trajectorySub = TrajectoryListener()
  get_exploration_path = rospy.ServiceProxy('get_exploration_path', GetRobotTrajectory)
  rospy.wait_for_service('get_exploration_path')

  try:
    get_exploration_path()
    while not rospy.is_shutdown():



      if not trajectorySub.ready():
        rate_limiter.sleep()
        continue
      
      # Update plan every 9s.
      current_time = rospy.Time.now().to_sec()
      time_since = current_time - previous_time
      if time_since < 10.:
        trajectory = [float('inf')]
      else:
        get_exploration_path()
      previous_time = current_time

      if not trajectorySub.used_path:
        print("Moving along new path")
        trajectory = trajectorySub.get_measurements()
        twist_path = getTwist(trajectory)

        # Publish path to RViz.
        path_msg = Path()
        path_msg.header.seq = frame_id
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'map'

        for tx,ty,tz in twist_path:
          
          pose_msg = PoseStamped()
          pose_msg.header.seq = frame_id
          pose_msg.header.stamp = path_msg.header.stamp
          pose_msg.header.frame_id = 'map'
          pose_msg.pose.position.x = tx
          pose_msg.pose.position.y = ty
          pose_msg.pose.position.z = tz
          path_msg.poses.append(pose_msg)
          pub_thread.update(tx, ty, tz, 0, 0, 0)
          rate_limiter.sleep()

        path_publisher.publish(path_msg)

      rate_limiter.sleep()
      frame_id += 1

  except Exception as e:
    print(e)

  pub_thread.stop()


# if __name__ == '__main__':
#   rospy.init_node('duck_action_node') #Name of publisher

#   pub_thread = PublishThread(0.0)
#   pub_thread.update(0, 0, 1, 0, 0, 0)
#   rate_limiter = rospy.Rate(100)

#   trajectorySub = TrajectoryListener()
#   counter = 0
#   dt = 1./50.
#   x = 0
#   y = 0

#   try:
#     while not rospy.is_shutdown():
#       if trajectorySub._path != [float('inf')]:

#         for idx, elem in enumerate(trajectorySub._path):
#           (v_roll,v_pitch,v_yaw) = quaternion_to_euler_angle(elem.pose.orientation.w, elem.pose.orientation.x , elem.pose.orientation.y, elem.pose.orientation.z)
#           v_psi = float((v_yaw))
#           yaw = math.radians(v_psi)

#           x = elem.pose.position.x
#           y = elem.pose.position.y
#           z = elem.pose.position.z

#           if idx == 0:
#             x_prev = x
#             y_prev = y
#             z_prev = z
#             counter += 1
#             continue

#           vel_x_world = (x - x_prev) / dt
#           vel_y_world = (y - y_prev) / dt
#           x_prev = x
#           y_prev = y
#           twist_x = math.cos(yaw) * vel_x_world + math.sin(yaw) * vel_y_world
#           twist_y = math.cos(yaw) * vel_y_world - math.sin(yaw) * vel_x_world

#           tx = twist_x
#           ty = twist_y
#           tz = (z - z_prev) / dt
#           z_prev = z

#           pub_thread.update(tx, ty, tz, 0, 0, 0)

#       rate_limiter.sleep()
#   except Exception as e:
#     print(e)

#   pub_thread.stop()