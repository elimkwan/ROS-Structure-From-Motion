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
class OccupancyGrid(object):
  def __init__(self, sz, origin, resolution):
    # a 3D occupancy grid with more focus on 2D planes
    full_sz = (int(sz[0]/resolution), int(sz[0]/resolution), int(sz[0]/resolution))
    self._original_values = np.ones((full_sz))
    # self._values = self.convolution(self._original_values, resolution)
    self._values = self._original_values.copy()

    # Inflate obstacles (using a convolution in 2d only).
    # inflated_grid = np.zeros_like(values)
    # inflated_grid[values == OCCUPIED] = 1.
    # w = 2 * int(ROBOT_RADIUS / resolution) + 1
    # inflated_grid = scipy.signal.convolve2d(inflated_grid, np.ones((w, w)), mode='same')
    # self._values[inflated_grid > 0.] = OCCUPIED
    self._origin = np.array(origin[:3], dtype=np.float32)
    self._origin -= resolution / 2.
    self._resolution = resolution

  def get_index(self, position):
    # position is (x,y,z)
    idx = ((position - self._origin) / self._resolution).astype(np.int32)
    if len(idx.shape) == 3:
      idx[:, 0] = np.clip(idx[:, 0], 0, self._values.shape[0] - 1)
      idx[:, 1] = np.clip(idx[:, 1], 0, self._values.shape[1] - 1)
      idx[:, 2] = np.clip(idx[:, 2], 0, self._values.shape[2] - 1)
      return (idx[:, 0], idx[:, 1], idx[:, 2])
    idx[0] = np.clip(idx[0], 0, self._values.shape[0] - 1)
    idx[1] = np.clip(idx[1], 0, self._values.shape[1] - 1)
    idx[2] = np.clip(idx[2], 0, self._values.shape[2] - 1)
    return tuple(idx)

  def is_occupied(self, position):
    # dependency
    return self._values[self.get_index(position)] == OCCUPIED

  def is_free(self, position):
    # dependency
    if (position[X] > 4 or position[X] < 6) and (position[Y] > 4 or position[Y] < 6)
      return False
    if position[Z] < 1 or position[Z] > 4
      return False

    return self._values[self.get_index(position)] == FREE

  def convolution(self, grid, resolution):
    # Inflate obstacles (using a convolution in 2d only).
    grid3d = copy.deepcopy(grid)
    for h in range(grid.shape[-1]):
      values = grid3d[:,:,h]
      inflated_grid = np.zeros_like(values)
      inflated_grid[values == OCCUPIED] = 1.
      w = 2 * int(ROBOT_RADIUS / resolution) + 1
      inflated_grid = scipy.signal.convolve2d(inflated_grid, np.ones((w, w)), mode='same')
      values[inflated_grid > 0.] = OCCUPIED
      grid3d[:,:,h] = values
    return grid3d

  def update(self, positions):
    # occupied_cell = [[x,y,z], [x,y,z] ...]
    for p in positions:
      self._values[self.get_index(p[:3])] == OCCUPIED
    # rospy.loginfo(rospy.get_caller_id() + ' Updated OG')
    return True


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
      assert len(lpc.points) == len(wpc.points)
      for idx,p in enumerate(np.array(lpc.points)):
        col = np.array([[float(p.x), float(p.y), float(p.z), 1.0]]).T
        tran_p = np.matmul(mat, col).reshape((4))
        wpc.points[idx].x = tran_p[X]
        wpc.points[idx].y = tran_p[Y]
        wpc.points[idx].z = tran_p[Z]
      self.world_pc = wpc
    except Exception as e:
      print(e)

  def get_Occuiped_Cell(self):
    pt = copy.deepcopy(self.world_pc.points)
    val = copy.deepcopy(self.world_pc.channels[0].values) #'intensities'
    pt = list(pt)
    val = np.array(val)
    thres = 0.0 #threshold to be considered as occupied

    trans_pt = [[p.x, p.y, p.z] for p in pt]
    trans_pt = np.array(trans_pt)
    final = trans_pt[val>thres]
    return final

  def ready(self):
    return isinstance(self.world_pc,pc)

def getRandomPose():
  #TODO add speed control
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
  rate_limiter = rospy.Rate(500)
  occupancy_grid = OccupancyGrid((10,10,3),(0,0,0),0.5)


  rospy.loginfo("Creating Action Client.")
  client = actionlib.SimpleActionClient('/action/pose', PoseAction)
  client.wait_for_server()
  rospy.loginfo("Client created.")

  while not rospy.is_shutdown():
    point_cloud.update()
    if not point_cloud.ready():
      rate_limiter.sleep()
      continue
    
    # Get Occuiped Cell (in world coordinate)
    # cells = point_cloud.get_Occuiped_Cell()
    # occupancy_grid.update(cells)

    # Create a random goal
    g = PoseGoal()
    g.target_pose.header.frame_id = 'world'
    pp = getRandomPose()
    g.target_pose.pose.position.x = pp[X]
    g.target_pose.pose.position.y = pp[Y]
    g.target_pose.pose.position.z = pp[Z]

    rospy.loginfo("Sending goal")
    client.send_goal(g)
    client.wait_for_result()
    rate_limiter.sleep()

  rospy.spin()

if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException:
    pass
  
  landclient = actionlib.SimpleActionClient('/action/landing', PoseAction)
  landclient.wait_for_server()
  landclient.send_goal()
  landclient.wait_for_result()


