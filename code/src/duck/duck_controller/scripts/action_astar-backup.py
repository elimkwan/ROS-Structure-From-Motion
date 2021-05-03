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
from hector_uav_msgs.msg import TakeoffAction
from hector_uav_msgs.msg import PoseGoal
import actionlib
from hector_nav_msgs.srv import GetRobotTrajectory

from octomap_msgs.msg import Octomap as om

from pyoctree import pyoctree as ot

import copy


directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), './astar.py')
sys.path.insert(0, directory)
import astar
# try:
#   import astar
# except ImportError:
#   raise ImportError('Unable to import astar.py. Make sure this file is in "{}"'.format(directory))


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
ROBOT_SIZE = [0.5,0.5,0.3]

class Laser2PC():
  #translate laser to pc and publish point cloud to /cloud_in
  def __init__(self):
    self._measurements = [float('inf')]
    self.laserProj = LaserProjection()
    self.pcPub = rospy.Publisher('/point_cloud2', pc2, queue_size=5)
    self.laserSub = rospy.Subscriber('/scan', LaserScan, self.laserCallBack)

  def laserCallBack(self, data):
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
    cloud_out = self.laserProj.projectLaser(data)
    cloud_out.header.seq = data.header.frame_id
    cloud_out.header.stamp = data.header.stamp
    cloud_out.header.frame_id = 'laser0_frame'
    self._measurements = cloud_out
    self.pcPub.publish(cloud_out)

  @property
  def ready(self):
    return not np.isnan(self._measurements)

  @property
  def measurements(self):
    return self._measurements

class pcScan_to_pcWorld():
  def __init__(self):
    self.pcSub = rospy.Subscriber('/slam_cloud', pc, self.callback)
    self._tf = TransformListener()
    self.laser_pc = None
    self.mat_laser2world = None
    self.world_pc = None
  
  def callback(self,data):
    self.laser_pc = data

  def update(self):
    target_frame = "world"
    src_frame = "laser0_frame"
    if self._tf.frameExists(target_frame) and self._tf.frameExists(src_frame):
      try:
        self.mat_laser2world = self._tf.asMatrix(target_frame, self.laser_pc.header)
      except Exception as e:
        print(e)
    else:
      print('Unable to find:', self._tf.frameExists(target_frame), self._tf.frameExists(src_frame))

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

class Cube:
  def __init__(self, center):
      self.key = np.array(center)
      self.center = np.array(center) + np.array(ROBOT_SIZE)/2
      self.free = 0 # number of free cells in cube
      self.occuipied = 0 
      self.unknown = 0 
      self.type = UNKNOWN

class OccupancyGrid():
  def __init__(self, world_sz):
    self.grid_sz = np.divide(world_sz, np.array(ROBOT_SIZE))
    grid = np.ones((self.grid_sz))
    grid = np.nonzero(grid)
    coord = zip(grid[0],grid[1], grid[2])
    self.world_sz = world_sz
    self.values = {key: Cube(key) for key in coord}
    self.free_space = []

    self.resolution = [0.05,0.05,0.05]
    self.high_res_grid_sz = np.divide(world_sz, self.resolution)
    self.high_resolution_grid = np.ones((self.high_res_grid_sz))

  def is_valid(self, position):
    valid1 = np.all(position < self.world_sz)
    valid2 = np.all(position >= [0,0,0])
    return valid1 and valid2

  def is_free(self, position):
    return self.values[self.get_index(position[:3])] == FREE

  def get_random_goal(self, cur_postion):
    # idx = np.random.randint(len(self.free_space))
    if len(self.free_space) < 1:
      rospy.loginfo("No Free Space Detected. Move to neighbour")
      # table = self.next(self.values[self.get_index(cur_postion)], 0)
      # return table[0][0]
      target = np.array(cur_postion) + [-1,-1,0]
      return self.values[self.get_index(target)]

    max_distance = 2
    for n in self.free_space:
      if np.linalg.norm(np.array(cur_postion)-np.array(n)) < max_distance:
        return self.values[n]

    rospy.loginfo("Using Random Random Goal Location")
    idx = np.random.randint(len(self.free_space)-1)
    return self.values[self.free_space[idx]]

  def random_walk(self, cur_postion):
    cur = self.values[self.get_index(cur_postion)]
    neighbour = next(self, cur, 0)
    if len(neighbour) == 0:
      target = np.array(cur_postion) + [-1,-1,0]
      target_p = self.values[self.get_index(target)]
      return [cur,target_p]

    idx = np.random.randint(len(neighbour)-1)
    target = neighbour[idx][0]
    path = [cur,target]
    return path

  def get_index_high_res(self, position):
    idx = ((np.array(position) - np.array([0,0,0])) / np.array(self.resolution)).astype(np.int32)
    idx[0] = np.clip(idx[0], 0, self.high_res_grid_sz[0] - 1)
    idx[1] = np.clip(idx[1], 0, self.high_res_grid_sz[1] - 1)
    idx[2] = np.clip(idx[2], 0, self.high_res_grid_sz[2] - 1)
    return tuple(idx)

  def get_index(self, position):
    idx = ((np.array(position) - np.array([0,0,0])) / np.array(ROBOT_SIZE)).astype(np.int32)
    idx[0] = np.clip(idx[0], 0, self.grid_sz[0] - 1)
    idx[1] = np.clip(idx[1], 0, self.grid_sz[1] - 1)
    idx[2] = np.clip(idx[2], 0, self.grid_sz[2] - 1)
    return tuple(idx)

  def occupied_cell_in_region(self, position):
    region = np.array(position)
    l = (region*np.array(ROBOT_SIZE)/np.array(self.resolution)).astype(np.int32) #convert region index to high resolution index
    h = ((region+1)*np.array(ROBOT_SIZE)/np.array(self.resolution)).astype(np.int32)
    print("l",l)
    print("h",h)
    total = abs(l[0]-h[0]) * abs(l[1]-h[1]) * abs(l[2]-h[2])
    grid = self.high_resolution_grid[l[0]:h[0], l[1]:h[1], l[2]:h[2]]
    print("grid shape", grid)
    o = np.count_nonzero(grid == OCCUPIED)
    f = np.count_nonzero(grid == FREE)
    per1 = o/total
    per2 = f/total
    print("o", o, "f", f, "total", total)
    return per1, per2

  def update(self, cells, cells_type):
    occ_dict = {}
    if cells_type == OCCUPIED:
      for p in cells:
        # self.values[self.get_index(p[:3])].occuipied += 1
        self.high_resolution_grid[self.get_index_high_res(p[:3])] = OCCUPIED
    elif cells_type == FREE:
      for p in cells:
        self.high_resolution_grid[self.get_index_high_res(p[:3])] = FREE

    for key in self.values.keys():
      occupied, free = self.occupied_cell_in_region(key)
      if free > 0.1:
        self.values[key].type = FREE
      if occupied > 0.1:
        self.values[key].type = OCCUPIED

  def next(self, node, t):
    cur_key = node.key
    n = []
    transforms = np.array([
      [ 1 , 1 , 1 ],
      [ 1 , 1 , 0 ],
      [ 1 , 1 , -1 ],
      [ 1 , 0 , 1 ],
      [ 1 , 0 , 0 ],
      [ 1 , 0 , -1 ],
      [ 1 , -1 , 1 ],
      [ 1 , -1 , 0 ],
      [ 1 , -1 , -1 ],
      [ 0 , 1 , 1 ],
      [ 0 , 1 , 0 ],
      [ 0 , 1 , -1 ],
      [ 0 , 0 , 1 ],
      [ 0 , 0 , -1 ],
      [ 0 , -1 , 1 ],
      [ 0 , -1 , 0 ],
      [ 0 , -1 , -1 ],
      [ -1 , 1 , 1 ],
      [ -1 , 1 , 0 ],
      [ -1 , 1 , -1 ],
      [ -1 , 0 , 1 ],
      [ -1 , 0 , 0 ],
      [ -1 , 0 , -1 ],
      [ -1 , -1 , 1 ],
      [ -1 , -1 , 0 ],
      [ -1 , -1 , -1 ]])
    # k = [1, 0,-1]
    # for a in k:
    #   for b in k:
    #     for c in k:
    #       print("[",a,",",b,",",c,"],")
    for trans in transforms:
      new_key = cur_key + trans
      if self.is_valid(new_key) and self.is_free(new_key):
        n.append((self.values[tuple(new_key)],t))
    return n

  def estimate(self, node, goal, t):
    return np.linalg.norm(node.center - goal.center)

def get_velocity(position, path_points, goal = None):
  # PID Controller for velocity
  def altitude_control(p):
    p[X] = np.clip(p[X], 1.5, 8.5)
    p[Y] = np.clip(p[X], 1.5, 8.5)
    p[Z] = np.clip(p[Z], 0.5, 5)
    return p

  if len(path_points) == 1 and isinstance(goal, np.ndarray):
    return altitude_control(goal)
  if len(path_points) < 2:
    return altitude_control(position)

  MAX_SPEED = 1
  v = np.zeros_like(position)
  if len(path_points) == 0:
    return v
  # # Stop moving if the goal is reached.
  # if np.linalg.norm(position - path_points[-1]) < .1:
  #   return v

  distance = []
  for points in path_points:
    distance.append(np.linalg.norm(position - points))

  distance = np.array(distance)
  min_index = np.argmin(distance.flatten())

  if min_index+1 >= len(path_points):
    return altitude_control(position)
    
  error = distance[min_index]
  rospy.loginfo("Min index: %d", min_index)
  # print("Error", error)

  scale = (MAX_SPEED/(1+np.exp((1/0.3)*(error-0.6))))

  # print("Scaling velocity", scale)
  v = (path_points[min_index + 1] - path_points[min_index]) * scale
  target_position = path_points[min_index] + v
  # target_position = path_points[min_index + 1]
  return altitude_control(target_position)

def get_path(cube_path, suggested_goal):
  path = []
  distance = 0.1

  # Random Walk Mode
  if len(cube_path) == 1:
    cube_path.append(suggested_goal)
    rospy.loginfo("Random Walk Big Steps")
  else:
    rospy.loginfo("ASar")
  
  # AStar Mode
  # print("Init Path Length", len(cube_path))
  for idx in range(1, len(cube_path)):
    past = cube_path[idx-1].center
    cur = cube_path[idx].center

    a = abs(past[X]-cur[X])/distance
    b = abs(past[Y]-cur[Y])/distance
    c = abs(past[Z]-cur[Z])/distance
    num_of_sample = max(a,b,c)
    # print("Num of sample", num_of_sample)

    x = np.linspace(past[X], cur[X], num_of_sample)
    y = np.linspace(past[Y], cur[Y], num_of_sample)
    z = np.linspace(past[Z], cur[Z], num_of_sample)

    coords = zip(x,y,z)
    for pt in coords:
      path.append(np.array(pt))
  # print("New Path Length", len(path))
  return path

def run():
  rospy.init_node('duck_action_node') #Name of publisher
  # Publish PointCloud

  #for the octomap
  pointcloud2_publisher = Laser2PC()

  point_cloud = pcScan_to_pcWorld()
  localisation = Localisation()
  rate_limiter = rospy.Rate(100)
  occupancy_grid = OccupancyGrid((10,10,5))
  path_publisher = rospy.Publisher('/twist_path', Path, queue_size=1)

  # For Unit Testing
  # a = occupancy_grid.next(Cube((2,2,2)),5)
  # a = occupancy_grid.estimate(Cube((5,5,0)),Cube((0,0,0)),5)
  # path = astar.astar(occupancy_grid,Cube((1,1,1)),Cube((4,4,4)))

  frame_id = 0
  rospy.loginfo("Creating Action Client TakeOff")
  takeoff_client = actionlib.SimpleActionClient('/action/takeoff', TakeoffAction)
  takeoff_client.wait_for_server()
  rospy.loginfo("Takeoff Client created.")
  g = PoseGoal()
  g.target_pose.header.seq = frame_id
  g.target_pose.header.frame_id = 'world'
  g.target_pose.pose.position.x = 2
  g.target_pose.pose.position.y = 2
  g.target_pose.pose.position.z = 2

  rospy.loginfo("Creating Action Client.")
  client = actionlib.SimpleActionClient('/action/pose', PoseAction)
  client.wait_for_server()
  rospy.loginfo("Client created.")

  takeoff_client.send_goal(g)
  takeoff_client.wait_for_result()
  client.send_goal(g)
  client.wait_for_result()

  current_path = []
  previous_time = rospy.Time.now().to_sec()
  suggested_goal = None

  while not rospy.is_shutdown():
    point_cloud.update()
    current_time = rospy.Time.now().to_sec()

    if not point_cloud.ready():
      rospy.loginfo("Not Ready")
      rate_limiter.sleep()
      continue

    # Follow path
    position = localisation.get_pose()
    if current_path:
      moveit = get_velocity(position, current_path, goal=suggested_goal.center)
      # Send it to action library
      g = PoseGoal()
      g.target_pose.header.seq = frame_id
      g.target_pose.header.frame_id = 'world'
      g.target_pose.pose.position.x = moveit[X]
      g.target_pose.pose.position.y = moveit[Y]
      g.target_pose.pose.position.z = moveit[Z]
      rospy.loginfo("Sending goal")
      client.send_goal(g)
      client.wait_for_result()
      # Increment Frame
      frame_id += 1

    # Update plan every 3s.
    time_since = current_time - previous_time
    if current_path and time_since < 4.:
      # trajectory = [float('inf')]
      rate_limiter.sleep()
      continue
    previous_time = current_time

    # Get Occuiped Cell (in world coordinate)
    occupied_cells, free_cells = point_cloud.get_Cell()
    occupancy_grid.update(occupied_cells, OCCUPIED)
    occupancy_grid.update(free_cells, FREE)

    # Run Exploration Planner
    center = occupancy_grid.get_index(position)
    # start = Cube(center)
    suggested_goal = occupancy_grid.get_random_goal(position) #random walk get avaliable neighbour
    cube_path = occupancy_grid.random_walk(position)
    print("Set Goal Location:", path[-1].key)
    # cube_path = astar.astar(occupancy_grid, start, suggested_goal)
    # print("New Path Avalibale")
    current_path = get_path(cube_path, suggested_goal)

    # Publish path to RViz.
    path_msg = Path()
    path_msg.header.seq = frame_id
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = 'world'
    for u in current_path:
      pose_msg = PoseStamped()
      pose_msg.header.seq = frame_id
      pose_msg.header.stamp = path_msg.header.stamp
      pose_msg.header.frame_id = 'world'
      pose_msg.pose.position.x = u[X]
      pose_msg.pose.position.y = u[Y]
      pose_msg.pose.position.z = u[Z]
      path_msg.poses.append(pose_msg)
    path_publisher.publish(path_msg)

    rate_limiter.sleep()

if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException:
    pass