#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import sys
import numpy as np
import os
import re
import scipy.signal
import yaml
import random

import threading
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
# from TFMessage.msg import tf2_msgs
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import PointCloud as pc
from laser_geometry import LaserProjection
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid as nav_msgs_OccupancyGrid
import math
from hector_uav_msgs.msg import PoseAction
from hector_uav_msgs.msg import PoseGoal
import actionlib

from octomap_msgs.msg import Octomap as om

from pyoctree import pyoctree as ot
# from rrt_exploration.octomap_rrt import RRT3D
import copy


# Import the potential_field.py code rather than copy-pasting.
directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), './rrt.py')
sys.path.insert(0, directory)
try:
  import rrt
except ImportError:
  raise ImportError('Unable to import simple_algo.py. Make sure this file is in "{}"'.format(directory))


SPEED = .2
EPSILON = .1

# Constants used for indexing.
X = 0
Y = 1
Z = 2
ROLL = 3
PITCH = 4
YAW = 5

# Constants for occupancy grid.
FREE = 0
UNKNOWN = 1
OCCUPIED = 2
# ROBOT_SIZE = [1,1,0.3]
ROBOT_RADIUS = 1 / 2.

#Defines an occupancy grid.
# class OccupancyGrid(object):
#   def __init__(self, sz, origin, resolution):
#     # a 3D occupancy grid with more focus on 2D planes
#     full_sz = (int(sz[0]/resolution), int(sz[0]/resolution), int(sz[0]/resolution))
#     self._original_values = np.ones((full_sz))
#     # self._values = self.convolution(self._original_values, resolution)
#     self._values = self._original_values.copy()

#     # Inflate obstacles (using a convolution in 2d only).
#     # inflated_grid = np.zeros_like(values)
#     # inflated_grid[values == OCCUPIED] = 1.
#     # w = 2 * int(ROBOT_RADIUS / resolution) + 1
#     # inflated_grid = scipy.signal.convolve2d(inflated_grid, np.ones((w, w)), mode='same')
#     # self._values[inflated_grid > 0.] = OCCUPIED
#     self._origin = np.array(origin[:3], dtype=np.float32)
#     self._origin -= resolution / 2.
#     self._resolution = resolution

#   def get_index(self, position):
#     # position is (x,y,z)
#     idx = ((position - self._origin) / self._resolution).astype(np.int32)
#     if len(idx.shape) == 3:
#       idx[:, 0] = np.clip(idx[:, 0], 0, self._values.shape[0] - 1)
#       idx[:, 1] = np.clip(idx[:, 1], 0, self._values.shape[1] - 1)
#       idx[:, 2] = np.clip(idx[:, 2], 0, self._values.shape[2] - 1)
#       return (idx[:, 0], idx[:, 1], idx[:, 2])
#     idx[0] = np.clip(idx[0], 0, self._values.shape[0] - 1)
#     idx[1] = np.clip(idx[1], 0, self._values.shape[1] - 1)
#     idx[2] = np.clip(idx[2], 0, self._values.shape[2] - 1)
#     return tuple(idx)

#   def is_free(self, position):
#     # dependency
#     return self._values[self.get_index(position)] == FREE

#   def convolution(self, grid, resolution):
#     # Inflate obstacles (using a convolution in 2d only).
#     grid3d = copy.deepcopy(grid)
#     for h in range(grid.shape[-1]):
#       values = grid3d[:,:,h]
#       inflated_grid = np.zeros_like(values)
#       inflated_grid[values == OCCUPIED] = 1.
#       w = 2 * int(ROBOT_RADIUS / resolution) + 1
#       inflated_grid = scipy.signal.convolve2d(inflated_grid, np.ones((w, w)), mode='same')
#       values[inflated_grid > 0.] = OCCUPIED
#       grid3d[:,:,h] = values
#     return grid3d

#   def update(self, positions):
#     # occupied_cell = [[x,y,z], [x,y,z] ...]
#     for p in positions:
#       self._values[self.get_index(p[:3])] == OCCUPIED
#     rospy.loginfo(rospy.get_caller_id() + ' Updated OG')
#     return True


class pcScan_to_pcWorld():
  def __init__(self):
    self.pcSub = rospy.Subscriber('/slam_cloud', pc, self.callback)
    self._tf = TransformListener()
    self.laser_pc = None
    self.mat_laser2world = None
    self.world_pc = None
  
  def callback(self,data):
    self.laser_pc = data
    target_frame = "world"
    src_frame = "laser0_frame"
    if self._tf.frameExists(target_frame) and self._tf.frameExists(src_frame):
      try:
        self.mat_laser2world = self._tf.asMatrix(target_frame, data.header)
      except Exception as e:
        print(e)
    else:
      print('Unable to find:', self._tf.frameExists(target_frame), self._tf.frameExists(src_frame))

  def update(self):
    if not isinstance(self.laser_pc,pc) or not isinstance(self.mat_laser2world, np.ndarray) :
      return
    try:
      lpc = copy.deepcopy(self.laser_pc)
      mat = copy.deepcopy(self.mat_laser2world)
      wpc = copy.deepcopy(self.laser_pc)
      for idx,p in enumerate(np.array(lpc.points)):
        col = np.array([[float(p.x), float(p.y), float(p.z), 1.0]]).T
        tran_p = np.matmul(mat, col).reshape((4))
        wpc.points[idx].x = tran_p[X]
        wpc.points[idx].y = tran_p[Y]
        wpc.points[idx].z = tran_p[Z]
      self.world_pc = wpc
    except Exception as e:
      print(e)

  def get_Cell(self):
    pt = copy.deepcopy(self.world_pc.points)
    val = copy.deepcopy(self.world_pc.channels[0].values) #'intensities'
    pt = list(pt)
    val = np.array(val)
    thres = 0.0 #threshold to be considered as occupied

    trans_pt = [[p.x, p.y, p.z] for p in pt]
    trans_pt = np.array(trans_pt)
    occupied = trans_pt[val<thres]
    free = trans_pt[val>thres]
    return occupied,free

  def ready(self):
    return isinstance(self.world_pc,pc)

class Localisation():
  def __init__(self):
    self.sub = rospy.Subscriber('ground_truth_to_tf/pose', PoseStamped, self.callback)
    self.pose = None

  def callback(self,data):
    self.pose = data

  def ready(self):
    return self.pose != None

  def get_pose(self):
    return (self.pose.pose.position.x,self.pose.pose.position.y,self.pose.pose.position.z)

def get_velocity(position, path_points):
  MAX_SPEED = 5
  v = np.zeros_like(position)
  if len(path_points) == 0:
    return v
  # Stop moving if the goal is reached.
  if np.linalg.norm(position - path_points[-1]) < .2:
    return v

  # MISSING: Return the velocity needed to follow the
  # path defined by path_points. Assume holonomicity of the
  # point located at position.

  distance = []
  for points in path_points:
    distance.append(np.linalg.norm(position - points))

  distance = np.array(distance)
  min_index = np.argmin(distance.flatten())
  error = distance[min_index]
  print("Min index", min_index)
  print("Error", error)

  scale = (2/(1+np.exp((1/0.3)*(error-0.6))))

  v = (path_points[min_index + 1] - path_points[min_index]) * scale
  v.clip(MAX_SPEED, MAX_SPEED)
  target_position = position + v
  return target_position

def get_path(final_node):
  # Construct path from RRT solution.
  if final_node is None:
    return []
  path_reversed = []
  path_reversed.append(final_node)
  while path_reversed[-1].parent is not None:
    path_reversed.append(path_reversed[-1].parent)
  path = list(reversed(path_reversed))
  # Put a point every 5 cm.
  # distance = 0.05
  # offset = 0.
  # points_x = []
  # points_y = []
  # points_z = []
  # for u, v in zip(path, path[1:]):
  #   du = u.position - center
  #   dv = v.position - center

  #   # Generate a point every 5cm apart.
  #   da = distance / radius
  #   offset_a = offset / radius

  #   angles = np.arange(theta1 + offset_a, theta2, da)
  #   offset = distance - (theta2 - angles[-1]) * radius
  #   points_x.extend(center[X] + np.cos(angles) * radius)
  #   points_y.extend(center[Y] + np.sin(angles) * radius)
  # return zip(points_x, points_y, points_z)
  return path

def getRandomPose():
  while(True):
    num = np.random.uniform(1,9,1)
    if num < 4 or num > 6:
      break
  x = num
  while(True):
    num = np.random.uniform(1,9,1)
    if num < 4 or num > 6:
      break
  y = num

  z = np.random.uniform(1,5,1)
  return [x,y,z]

def run():
  rospy.init_node('duck_action_node') #Name of publisher
  point_cloud = pcScan_to_pcWorld()
  localisation = Localisation()
  rate_limiter = rospy.Rate(100)
  occupancy_grid = rrt.OccupancyGrid((10,10,3),(0,0,0),0.5)
  path_publisher = rospy.Publisher('/twist_path', Path, queue_size=1)


  rospy.loginfo("Creating Action Client.")
  client = actionlib.SimpleActionClient('/action/pose', PoseAction)
  client.wait_for_server()
  rospy.loginfo("Client created.")

  current_path = []
  previous_time = rospy.Time.now().to_sec()
  frame_id = 0

  while not rospy.is_shutdown():
    print("here")
    point_cloud.update()
    current_time = rospy.Time.now().to_sec()
    if not point_cloud.ready():
      rospy.loginfo("Not Ready")
      rate_limiter.sleep()
      continue

    # Follow path
    position = localisation.get_pose()
    if current_path:
      moveit = get_velocity(position, current_path)
      # Send it to action library
      g = PoseGoal()
      g.target_pose.header.seq = frame_id
      g.target_pose.header.frame_id = 'world'
      g.target_pose.pose.position.x = moveit[X]
      g.target_pose.pose.position.y = moveit[Y]
      g.target_pose.pose.position.z = moveit[Z]
      rospy.loginfo("Sending goal")
      print("cur path", current_path)
      print("Moveit",position)
      client.send_goal(g)
      client.wait_for_result()

      # Increment Frame
      frame_id += 1

    print("debug")

    # # Update plan every 1s.
    # time_since = current_time - previous_time
    # if current_path and time_since < 2.:
    #   # trajectory = [float('inf')]
    #   rate_limiter.sleep()
    #   continue
    # previous_time = current_time

    # Get Occuiped Cell (in world coordinate)
    occupied_cells, free_cells = point_cloud.get_Cell()
    occupancy_grid.update(occupied_cells, OCCUPIED)
    occupancy_grid.update(free_cells, FREE)

    # Run Exploration Planner
    start = np.array(position, dtype=np.float32)
    rand_pose = getRandomPose()
    end = np.array([1.5,1.5,3], dtype=np.float32)
    start_node, final_node = rrt.rrt(start, end, occupancy_grid)
    print("New Path Avalibale")
    # print("start", start_node)
    # current_path = get_path(final_node)

    # # Publish path to RViz.
    # path_msg = Path()
    # path_msg.header.seq = frame_id
    # path_msg.header.stamp = rospy.Time.now()
    # path_msg.header.frame_id = 'map'
    # for u in current_path:
    #   pose_msg = PoseStamped()
    #   pose_msg.header.seq = frame_id
    #   pose_msg.header.stamp = path_msg.header.stamp
    #   pose_msg.header.frame_id = 'map'
    #   pose_msg.pose.position.x = u[X]
    #   pose_msg.pose.position.y = u[Y]
    #   pose_msg.pose.position.z = u[Z]
    #   path_msg.poses.append(pose_msg)
    # path_publisher.publish(path_msg)

    rate_limiter.sleep()

if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException:
    pass


#Defines an occupancy grid.
# class OccupancyGrid(object):
#   def __init__(self, world_size, robot_size):
#     self.origin = (0,0,0)
#     grid_size = np.divide(world_size, robot_size).astype(int)
#     self.values = np.ones(grid_size) # all cell are unknown to start with
  
#   def get_index(self, position):
#     # position is (x,y,z)
#     idx = ((position - self._origin) / self._resolution).astype(np.int32)
#     if len(idx.shape) == 3:
#       idx[:, 0] = np.clip(idx[:, 0], 0, self._values.shape[0] - 1)
#       idx[:, 1] = np.clip(idx[:, 1], 0, self._values.shape[1] - 1)
#       idx[:, 2] = np.clip(idx[:, 2], 0, self._values.shape[2] - 1)
#       return (idx[:, 0], idx[:, 1], idx[:, 2])
#     idx[0] = np.clip(idx[0], 0, self._values.shape[0] - 1)
#     idx[1] = np.clip(idx[1], 0, self._values.shape[1] - 1)
#     idx[2] = np.clip(idx[2], 0, self._values.shape[2] - 1)
#     return tuple(idx)

#   def is_occupied(self, position):
#     # dependency
#     return self._values[self.get_index(position)] == OCCUPIED

#   def is_free(self, position):
#     # dependency
#     return self._values[self.get_index(position)] == FREE

#   def convolution(self, grid, resolution):
#     # Inflate obstacles (using a convolution in 2d only).
#     grid3d = copy.deepcopy(grid)
#     for h in range(grid.shape[-1]):
#       values = grid3d[:,:,h]
#       inflated_grid = np.zeros_like(values)
#       inflated_grid[values == OCCUPIED] = 1.
#       w = 2 * int(ROBOT_RADIUS / resolution) + 1
#       inflated_grid = scipy.signal.convolve2d(inflated_grid, np.ones((w, w)), mode='same')
#       values[inflated_grid > 0.] = OCCUPIED
#       grid3d[:,:,h] = values
#     return grid3d

#   def update(self, positions):
#     # occupied_cell = [[x,y,z], [x,y,z] ...]
#     for p in positions:
#       self._values[self.get_index(p[:3])] == OCCUPIED
#     return True

#   def update(self, cell_info): #location of occupied cell
#     # cell_info = (x,y,z,val)
#     for loc in cell_info:
#       self.values[loc[:3]] = loc[:-1]



# class Laser2PC():
#   #translate laser to pc and publish point cloud to /cloud_in
#   def __init__(self):
#     self.laserSub = rospy.Subscriber('/scan', LaserScan, self.laserCallBack)
#     self.poseSub = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, self.poseCallBack)
#     self._tf = TransformListener()
#     # self.grid = OccupancyGrid(np.ones((10,10,3)), [0,0,0], 0.1)
#     self.laser = LaserScan()
#     self.continuous_pose_map= PoseStamped()
#     self.pose_map = PoseStamped()
#     self.continuous_pose_laser = PoseStamped()
#     self.pose_laser = PoseStamped()
#     self.grid = []
#     self.transformation_laser_to_map = None

#   def laserCallBack(self,data):
#     self._tf.transformPointCloud("world", world_pc)


#   def laserCallBack(self, data):
#     self.laser = data
#     self.pose_map = self.continuous_pose_map #update the pose according to the stored laser
#     self.pose_laser = self.continuous_pose_map

#   def poseCallBack(self, data):
#     self.continuous_pose_map = data
#     # Get pose in world w.r.t. base_laser.
#     a = 'laser0_frame'
#     b = 'world'
#     if self._tf.frameExists(a) and self._tf.frameExists(b):
#       try:
#         t = rospy.Time(0)
#         self.continuous_pose_laser = self._tf.lookupTransform('/' + a, '/' + b, t)
#         rospy.loginfo(rospy.get_caller_id() + 'Trans %s', self.transformation_laser_to_map)
#       except Exception as e:
#         print(e)
#     else:
#       print('Unable to find:', self._tf.frameExists(a), self._tf.frameExists(b))


  # def getCartesianGrid(self):
  #   angle_min = self.laser.angle_min.copy
  #   angle_max = self.laser.angle_max
  #   angle_increment = self.laser.angle_increment
  #   ranges = self.laser.ranges
  #   intensity = self.laser.intensity

  #   l = copy.deepcopy(self.laser)
  #   laser_pose = copy.deepcopy(self.pose_laser)
  #   map_pose = copy.deepcopy(self.pose_map)

  #   for idx, elem in enumerate(l.ranges):
  #     phi = l.angle_increment * idx + l.angle_min
  #     phi = phi*180/np.pi()

  #     temp = laser_pose.copy()
  #     laser_pose.position.x = laser_pose.position.x + np.cos(phi)
  #     laser_pose.position.y = laser_pose.position.y + np.sin(phi)

  #     a = 'world'
  #     b = 'laser0_frame'
  #     if self._tf.frameExists(a) and self._tf.frameExists(b):
  #       try:
  #         self._tf.transformPose('/' + a, '/' + b, t)
  #         t = rospy.Time(0)
  #         self.continuous_pose_laser = self._tf.lookupTransform('/' + a, '/' + b, t)
  #         rospy.loginfo(rospy.get_caller_id() + 'Trans %s', self.transformation_laser_to_map)
  #       except Exception as e:
  #         print(e)
  #     else:
  #     print('Unable to find:', self._tf.frameExists(a), self._tf.frameExists(b))



      

      #coord according to laser sensor center
    
    #transfer to world coord
    


# class PublishThread(threading.Thread):
#   def __init__(self, rate):
#     super(PublishThread, self).__init__()
#     self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
#     self.x = 0.0
#     self.y = 0.0
#     self.z = 0.0
#     self.ax = 0.0
#     self.ay = 0.0
#     self.az = 0.0
#     self.condition = threading.Condition()
#     self.done = False
#     if rate != 0.0:
#         self.timeout = 1.0 / rate
#     else:
#         self.timeout = None
#     self.start()

#   def update(self, x, y, z, ax, ay, az):
#     self.condition.acquire()
#     self.x = x
#     self.y = y
#     self.z = z
#     self.ax = ax
#     self.ay = ay
#     self.az = az
#     # Notify publish thread that we have a new message.
#     self.condition.notify()
#     self.condition.release()

#   def stop(self):
#     self.done = True
#     self.update(0, 0, 0, 0, 0, 0)
#     self.join()

#   def run(self):
#     twist = Twist()
#     while not self.done:
#       self.condition.acquire()
#       # Wait for a new message or timeout.
#       self.condition.wait(self.timeout)

#       # Copy state into twist message.
#       twist.linear.x = self.x
#       twist.linear.y = self.y
#       twist.linear.z = self.z
#       twist.angular.x = self.ax
#       twist.angular.y = self.ay
#       twist.angular.z = self.az
#       self.condition.release()
#       # Publish.
#       self.publisher.publish(twist)
#     # Publish stop message when thread exits.
#     twist.linear.x = 0
#     twist.linear.y = 0
#     twist.linear.z = 0
#     twist.angular.x = 0
#     twist.angular.y = 0
#     twist.angular.z = 0
#     self.publisher.publish(twist)

# class TrajectoryListener():
#   def __init__(self):
#     self._path = [float('inf')]
#     self._tf = TransformListener()
#     self.sub = rospy.Subscriber('/rrt_path', Path, self.CallBack)
#     self.used_path = False

#   def CallBack(self, data):
#     # rospy.loginfo(rospy.get_caller_id() + 'Path %s', data)
#     print("New Path Avaliable")
#     path = []

#     # Get pose in path w.r.t. map.
#     a = 'map'
#     b = 'base_link'
#     if self._tf.frameExists(a) and self._tf.frameExists(b):
#       try:
#         for u in data.poses:
#           t = rospy.Time(0)
#           position, orientation = self._tf.lookupTransform('/' + a, '/' + b, t)
#           x = position[X]
#           y = position[Y]
#           z = position[Z]
#           roll, pitch, yaw = euler_from_quaternion(orientation)
#           path.append((x,y,z,roll,pitch,yaw))

#       except Exception as e:
#         print(e)
#     else:
#       print('Unable to find:', self._tf.frameExists(a), self._tf.frameExists(b))

#     self._path = path
#     self.used_path = False

  
#   def get_measurements(self):
#     self.used_path = True
#     return self._path
  
#   def ready(self):
#     return self._path != [float('inf')]

# # Defines an occupancy grid.
# class OccupancyGrid(object):
#   def __init__(self, values, origin, resolution):
#     self._original_values = values.copy()
#     self._values = values.copy()
#     # Inflate obstacles (using a convolution).
#     inflated_grid = np.zeros_like(values)
#     inflated_grid[values == OCCUPIED] = 1.
#     w = 2 * int(ROBOT_RADIUS / resolution) + 1
#     inflated_grid = scipy.signal.convolve2d(inflated_grid, np.ones((w, w)), mode='same')
#     self._values[inflated_grid > 0.] = OCCUPIED
#     self._origin = np.array(origin[:2], dtype=np.float32)
#     self._origin -= resolution / 2.
#     assert origin[YAW] == 0.
#     self._resolution = resolution

#   @property
#   def values(self):
#     return self._values

#   @property
#   def resolution(self):
#     return self._resolution

#   @property
#   def origin(self):
#     return self._origin

#   def draw(self):
#     plt.imshow(self._original_values.T, interpolation='none', origin='lower',
#                extent=[self._origin[X],
#                        self._origin[X] + self._values.shape[0] * self._resolution,
#                        self._origin[Y],
#                        self._origin[Y] + self._values.shape[1] * self._resolution])
#     plt.set_cmap('gray_r')

#   def get_index(self, position):
#     idx = ((position - self._origin) / self._resolution).astype(np.int32)
#     if len(idx.shape) == 2:
#       idx[:, 0] = np.clip(idx[:, 0], 0, self._values.shape[0] - 1)
#       idx[:, 1] = np.clip(idx[:, 1], 0, self._values.shape[1] - 1)
#       return (idx[:, 0], idx[:, 1])
#     idx[0] = np.clip(idx[0], 0, self._values.shape[0] - 1)
#     idx[1] = np.clip(idx[1], 0, self._values.shape[1] - 1)
#     return tuple(idx)

#   def get_position(self, i, j):
#     return np.array([i, j], dtype=np.float32) * self._resolution + self._origin

#   def is_occupied(self, position):
#     return self._values[self.get_index(position)] == OCCUPIED

#   def is_free(self, position):
#     return self._values[self.get_index(position)] == FREE

# class SLAM(object):
#   def __init__(self):
#     rospy.Subscriber('/map', nav_msgs_OccupancyGrid, self.callback)
#     self._tf = TransformListener()
#     self._occupancy_grid = None
#     self._pose = np.array([np.nan, np.nan, np.nan, np.nan, np.nan, np.nan], dtype=np.float32)
    
#   def callback(self, msg):
#     values = np.array(msg.data, dtype=np.int8).reshape((msg.info.width, msg.info.height))
#     processed = np.empty_like(values)
#     processed[:] = FREE
#     processed[values < 0] = UNKNOWN
#     processed[values > 50] = OCCUPIED
#     processed = processed.T
#     origin = [msg.info.origin.position.x, msg.info.origin.position.y, msg.info.origin.position.z, 0, 0, 0]
#     resolution = msg.info.resolution
#     self._occupancy_grid = OccupancyGrid(processed, origin, resolution)

#   def update(self):
#     # Get pose w.r.t. map.
#     a = 'world'
#     b = 'base_link'
#     if self._tf.frameExists(a) and self._tf.frameExists(b):
#       try:
#         t = rospy.Time(0)
#         position, orientation = self._tf.lookupTransform('/' + a, '/' + b, t)
#         self._pose[X] = position[X]
#         self._pose[Y] = position[Y]
#         self._pose[Z] = position[Z]
#         self._pose[ROLL], self._pose[PITCH], self._pose[YAW] = euler_from_quaternion(orientation)
#       except Exception as e:
#         print(e)
#     else:
#       print('Unable to find:', self._tf.frameExists(a), self._tf.frameExists(b))
#     pass

#   @property
#   def ready(self):
#     return self._occupancy_grid is not None and not np.isnan(self._pose[0])

#   @property
#   def pose(self):
#     return self._pose

#   @property
#   def occupancy_grid(self):
#     return self._occupancy_grid

# class OctomapListener():
#   def __init__(self):
#     self.sub = rospy.Subscriber('octomap_binary', om, self.callback)
#     rospy.loginfo(rospy.get_caller_id() + 'Initialised OctomapListener')
#     self.map = None

#   def callback(self,data):
#     print("Saved Octomap Binary")
#     self.map = data.data

# def velocity_control(pose):
#   scaled_pose = []
#   for elem in pose:
#     scaled = (0.5/(1+np.exp((1/0.3)*(-elem+0.6))))
#     scaled_pose.append(scaled)
#   return np.array(scaled_pose)

# def get_path(final_node):
#   # Construct path from RRT solution.
#   if final_node is None:
#     return []
#   path_reversed = []
#   path_reversed.append(final_node)
#   while path_reversed[-1].parent is not None:
#     path_reversed.append(path_reversed[-1].parent)
#   path = list(reversed(path_reversed))
#   # Put a point every 5 cm.
#   distance = 0.05
#   offset = 0.
#   points_x = []
#   points_y = []
#   for u, v in zip(path, path[1:]):
#     center, radius = rrt.find_circle(u, v)
#     du = u.position - center
#     theta1 = np.arctan2(du[1], du[0])
#     dv = v.position - center
#     theta2 = np.arctan2(dv[1], dv[0])
#     # Check if the arc goes clockwise.
#     clockwise = np.cross(u.direction, du).item() > 0.
#     # Generate a point every 5cm apart.
#     da = distance / radius
#     offset_a = offset / radius
#     if clockwise:
#       da = -da
#       offset_a = -offset_a
#       if theta2 > theta1:
#         theta2 -= 2. * np.pi
#     else:
#       if theta2 < theta1:
#         theta2 += 2. * np.pi
#     angles = np.arange(theta1 + offset_a, theta2, da)
#     offset = distance - (theta2 - angles[-1]) * radius
#     points_x.extend(center[X] + np.cos(angles) * radius)
#     points_y.extend(center[Y] + np.sin(angles) * radius)
#   return zip(points_x, points_y)

# # /occupied_cells_v_array
# # /octomap_binary
# def run():
#   rospy.init_node('duck_action_node') #Name of publisher

#   # Publish PointCloud
#   pc_publisher = Laser2PC()

#   # Update control every 100 ms.
#   # pub_thread = PublishThread(0.0)
#   rate_limiter = rospy.Rate(100)
#   publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
#   path_publisher = rospy.Publisher('/twist_path', Path, queue_size=1)
#   slam = SLAM()
#   # goal = GoalPose()
#   frame_id = 0
#   current_path = []
#   previous_time = rospy.Time.now().to_sec()
#   previous_time_g = rospy.Time.now().to_sec()

#   print("debug00")
#   oc = OctomapListener()

#   print("debug0")
#   trajectorySub = TrajectoryListener()
#   get_rrt_path = rospy.ServiceProxy('get_exploration_path', GetRobotTrajectory)
#   pose_msg = PoseStamped()
#   pose_msg.header.seq = frame_id
#   pose_msg.header.stamp = rospy.Time.now()
#   pose_msg.header.frame_id = 'map'
#   pose_msg.pose.position.x = 0
#   pose_msg.pose.position.y = 0
#   pose_msg.pose.position.z = 0
#   # get_rrt_path(PoseStamped())
#   rospy.wait_for_service('get_rrt_path')
#   print("debug1")

#   # Stop moving message.
#   stop_msg = Twist()
#   stop_msg.linear.x = 0.
#   stop_msg.angular.z = 0.

#   # Make sure the robot is stopped.
#   i = 0
#   while i < 10 and not rospy.is_shutdown():
#     publisher.publish(stop_msg)
#     rate_limiter.sleep()
#     i += 1

#   # Run the service once to initialise
#   # get_exploration_path()
#   while not rospy.is_shutdown():
#     if not trajectorySub.ready():
#       rate_limiter.sleep()
#     else:
#       current_path = trajectorySub.get_measurements()
#       break

#   while not rospy.is_shutdown():
#     slam.update()
#     current_time = rospy.Time.now().to_sec()

#     if not trajectorySub.ready() or not slam.ready:
#       rate_limiter.sleep()
#       continue

#     # Follow path
#     position = np.array([
#         slam.pose[X] + EPSILON * np.cos(slam.pose[YAW]),
#         slam.pose[Y] + EPSILON * np.sin(slam.pose[YAW]),
#         slam.pose[Z]], dtype=np.float32)


#     vel_msg = Twist()
#     scaled_pose = velocity_control(slam.pose)
#     vel_msg.linear.x = scaled_pose[X]
#     vel_msg.linear.y = scaled_pose[Y]
#     vel_msg.linear.z = scaled_pose[Z]
#     vel_msg.angular.x = scaled_pose[ROLL]
#     vel_msg.angular.y = scaled_pose[PITCH]
#     vel_msg.angular.z = scaled_pose[YAW]
#     publisher.publish(vel_msg)


#     # Update plan every 1s.
#     time_since = current_time - previous_time
#     if current_path and time_since < 2.:
#       rate_limiter.sleep()
#       continue
#     previous_time = current_time

#     # Update goal every 10s.
#     time_since_g = current_time - previous_time_g
#     if time_since > 9:
#       goal_position = rrt.sample_random_position(slam.occupancy_grid)
#       previous_time_g = current_time

#     # Run Exploration Planner
#     # rospy.wait_for_service('get_exploration_path')
#     # get_exploration_path()
#     # current_path = trajectorySub.get_measurements()

#     # Run RRT.
#     # start_node, final_node = rrt.rrt(slam.pose, goal_position, slam.occupancy_grid)
#     # current_path = get_path(final_node)
#     # if not current_path:
#     #   print('Unable to reach goal position:', goal_position)

#     # Run 3D RRT
#     pose_msg = PoseStamped()
#     pose_msg.header.seq = frame_id
#     pose_msg.header.stamp = rospy.Time.now()
#     pose_msg.header.frame_id = 'map'
#     pose_msg.pose.position.x = position[X]
#     pose_msg.pose.position.y = position[Y]
#     pose_msg.pose.position.z = position[Z]
#     rospy.wait_for_service('get_rrt_path')
#     get_rrt_path(pose_msg)
#     current_path = trajectorySub.get_measurements()

#     # if not current_path:
#     #   print('Unable to reach goal position:', goal_position)

#     # Publish path to RViz.
#     path_msg = Path()
#     path_msg.header.seq = frame_id
#     path_msg.header.stamp = rospy.Time.now()
#     path_msg.header.frame_id = 'map'
#     for u in current_path:
#       pose_msg = PoseStamped()
#       pose_msg.header.seq = frame_id
#       pose_msg.header.stamp = path_msg.header.stamp
#       pose_msg.header.frame_id = 'map'
#       pose_msg.pose.position.x = u[X]
#       pose_msg.pose.position.y = u[Y]
#       pose_msg.pose.position.z = u[Z]
#       path_msg.poses.append(pose_msg)
#     path_publisher.publish(path_msg)

#     rate_limiter.sleep()
#     frame_id += 1


# if __name__ == '__main__':
#   try:
#     run()
#   except rospy.ROSInterruptException:
#     pass



# def feedback_linearized(pose, velocity, epsilon):
#   u = 0.  # [m/s]
#   w = 0.  # [rad/s] going counter-clockwise.

#   # MISSING: Implement feedback-linearization to follow the velocity
#   # vector given as argument. Epsilon corresponds to the distance of
#   # linearized point in front of the robot.

#   theta = pose[2]
#   dx = velocity[0]
#   dy = velocity[1]

#   u = dx*np.cos(theta) + dy*np.sin(theta)
#   w = (1/epsilon)*(-dx*np.sin(theta)+dy*np.cos(theta))

#   return u, w


# def get_velocity(full_position, full_path_points):
#   MAX_SPEED = 1
#   # print("full_position", full_position.shape)
#   # print("full_path_position", full_path_points.shape)
#   position = full_position[:3]
#   path_points = full_path_points[:,:3]

#   v = np.zeros_like(position)
#   if len(path_points) == 0:
#     return v
#   # Stop moving if the goal is reached.
#   if np.linalg.norm(position - path_points[-1]) < .2:
#     return v

#   # MISSING: Return the velocity needed to follow the
#   # path defined by path_points. Assume holonomicity of the
#   # point located at position.

#   distance = []
#   for points in path_points:
#     distance.append(np.linalg.norm(position - points))

#   distance = np.array(distance)
#   min_index = np.argmin(distance.flatten())
#   error = distance[min_index]
#   print("Min index", min_index)
#   print("Error", error)

#   scale = (2/(1+np.exp((1/0.3)*(error-0.6))))

#   v = (path_points[min_index + 1] - path_points[min_index]) * scale
#   v.clip(MAX_SPEED, MAX_SPEED)

#   return v


# # https://gist.github.com/gajena/ed61f7eea7c6a967cc9a4171a07fc13f
# def quaternion_to_euler_angle(w, x, y, z):
# 	ysqr = y * y

# 	t0 = +2.0 * (w * x + y * z)
# 	t1 = +1.0 - 2.0 * (x * x + ysqr)
# 	X = math.degrees(math.atan2(t0, t1))

# 	t2 = +2.0 * (w * y - z * x)
# 	t2 = +1.0 if t2 > +1.0 else t2
# 	t2 = -1.0 if t2 < -1.0 else t2
# 	Y = math.degrees(math.asin(t2))

# 	t3 = +2.0 * (w * z + x * y)
# 	t4 = +1.0 - 2.0 * (ysqr + z * z)
# 	Z = math.degrees(math.atan2(t3, t4))

# 	return X, Y, Z

# def getTwist(path):
#   counter = 0
#   dt = 1./50.
#   x = 0
#   y = 0
#   twist_path = []

#   for idx, elem in enumerate(path):
#     (v_roll,v_pitch,v_yaw) = quaternion_to_euler_angle(elem.pose.orientation.w, elem.pose.orientation.x , elem.pose.orientation.y, elem.pose.orientation.z)
#     v_psi = float((v_yaw))
#     yaw = math.radians(v_psi)

#     x = elem.pose.position.x
#     y = elem.pose.position.y
#     z = elem.pose.position.z

#     if idx == 0:
#       x_prev = x
#       y_prev = y
#       z_prev = z
#       counter += 1
#       continue

#     vel_x_world = (x - x_prev) / dt
#     vel_y_world = (y - y_prev) / dt
#     x_prev = x
#     y_prev = y
#     twist_x = math.cos(yaw) * vel_x_world + math.sin(yaw) * vel_y_world
#     twist_y = math.cos(yaw) * vel_y_world - math.sin(yaw) * vel_x_world

#     tx = twist_x
#     ty = twist_y
#     tz = (z - z_prev) / dt
#     z_prev = z

#     twist_path.append((tx,ty,tz,v_roll,v_pitch,v_yaw))

#   return twist_path
