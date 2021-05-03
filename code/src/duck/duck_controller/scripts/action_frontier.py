#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import sys
import numpy as np
import os
import re
import scipy.signal
from scipy import ndimage, misc
import skimage.filter
from skimage.util import img_as_float
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
ROBOT_SIZE = [0.5,0.5,0.5]
np.random.seed(42)


class Laser2PCWorld():
  #translate laser to pc and publish point cloud to /cloud_in
  def __init__(self):
    self.laserSub = rospy.Subscriber('/scan', LaserScan, self.laserCallBack)
    self.laser = [float('inf')]
    self.pc_laser = None
    self.pc_world = None
    self._tf = TransformListener()
    self.occupied_cells = []
    self.free_cells = []
    self.mat_laser2world = None

  def laserCallBack(self, data):
    self.laser = data

  def update(self):
    # Get the transformation matrix for laser to world
    target_frame = "world"
    src_frame = "laser0_frame"
    if self._tf.frameExists(target_frame) and self._tf.frameExists(src_frame):
      try:
        self.mat_laser2world = self._tf.asMatrix(target_frame, self.laser.header)
      except Exception as e:
        print(e)
    else:
      print('Unable to find:', self._tf.frameExists(target_frame), self._tf.frameExists(src_frame))

    # Transform laser data to world coordinate
    if not isinstance(self.laser, LaserScan) or not isinstance(self.mat_laser2world, np.ndarray) :
      return
    try:
      l = copy.deepcopy(self.laser)
      mat = copy.deepcopy(self.mat_laser2world)

      free = [] #in world coord
      occuiped = [] #in world coord
      pc_laser = []
      pc_world = []
      for r in range(len(l.ranges)):
        radius = 0
        encountered_obs = True
        if l.ranges[r] == float('inf'):
          #append free cells till max range
          radius = l.range_max
          encountered_obs = False

        theta = 180 - (l.angle_min + r * l.angle_increment) #due to differences in ros corrdinate frame
        x = l.ranges[radius] * np.cos(theta)
        y = l.ranges[radius] * np.sin(theta)
        z = 0

        pc_laser.append([x,y,z])
        col = np.array([[x,y,z,1]]).T
        tran = np.matmul(mat, col).reshape((4))
        pc_world.append([tran[X],tran[Y],tran[X]])

        discretized_pt = self.append_free_space([tran[X],tran[Y],tran[X]])
        # print("discretized_pt", len(discretized_pt))
        if encountered_obs:
          # print("before", len(occuiped))
          occuiped.extend(discretized_pt[-10:])
          # print("pt", discretized_pt[-10:])
          free.extend(discretized_pt[:-10])
          # print("after", len(occuiped))
        else:
          free.extend(discretized_pt)
        # print("encoutered obs", encountered_obs)
        # print("occuipied", len(occuiped))
        # print("free", len(free))

      self.pc_laser = pc_laser
      self.pc_world = pc_world
      self.occupied_cells = occuiped
      self.free_cells = free
    except Exception as e:
      print(e)

  def append_free_space(self, pt):
    distance = 0.05
    a = abs(pt[X])/distance
    b = abs(pt[Y])/distance
    c = abs(pt[Z])/distance
    num_of_sample = max(a,b,c)

    x = np.linspace(0, pt[X], num_of_sample)
    y = np.linspace(0, pt[Y], num_of_sample)
    z = np.linspace(0, pt[Z], num_of_sample)
    coords = zip(x,y,z)
    return coords

    # arr = [[c[X],c[Y],c[Z]] for c in coords]
    # return np.array(arr)

  @property
  def ready(self):
    laser_ready = isinstance(self.laser, LaserScan)
    mat_ready = isinstance(self.mat_laser2world, np.ndarray)
    return laser_ready and mat_ready

  @property
  def measurements(self):
    return self._measurements


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
  def __init__(self, key, center):
      self.key = np.array(key)
      self.center = np.array(center)
      self.free = 0 # number of free cells in cube
      self.occuipied = 0 
      self.unknown = 0 
      self.frontier = True
      self.type = UNKNOWN

class OccupancyGrid():
  def __init__(self, world_sz):
    self.grid_sz = np.divide(world_sz, np.array(ROBOT_SIZE)).astype(np.int32)
    self.grid = np.zeros((self.grid_sz)) #all unknown
    grid = np.ones((self.grid_sz)) 
    grid_c = np.nonzero(grid)
    coord = zip(grid_c[0],grid_c[1], grid_c[2])
    self.world_sz = world_sz
    self.values = {key: Cube(key, self.key_to_center_pos(key)) for key in coord}
    self.free_space = []
    self.map_percentage = 0

    # self.resolution = [0.05,0.05,0.05]
    # self.high_res_grid_sz = np.divide(world_sz, self.resolution)
    # self.high_resolution_grid = np.ones((self.high_res_grid_sz))
  
  def get_map_percentage(self):
    total = len(self.values.keys())
    unknown = 0
    for key in self.values.keys():
      if self.values[key].type == UNKNOWN:
        unknown += 1
    self.map_percentage = (total-unknown)/total

  def key_to_center_pos(self, x):
    y = (np.array(x)*np.array(ROBOT_SIZE)).astype(np.int32)
    return y

  def is_valid(self, position):
    valid1 = np.all(position < self.grid_sz)
    valid2 = np.all(position >= [0,0,0])
    return valid1 and valid2

  def is_free(self, position):
    # print("POsition", position, "Type", self.values[self.get_index(position[:3])].type)
    return self.values[self.get_index(position[:3])].type == FREE

  # Frontier with random noise
  def get_random_goal(self, cur_postion):
    # idx = np.random.randint(len(self.free_space))
    #With random parbability to use random walk
    probability = np.random.uniform()
    if len(self.free_space) < 1 or probability < 0.3:
      rospy.loginfo("No Free Space Detected. Move to neighbour")
      path = self.random_walk(cur_postion)
      return path[-1]

    expl_probability = np.random.uniform()
    max_distance = 6 if expl_probability > 0.5 else 4
    min_distance = 3 if expl_probability > 0.5 else 2
    #Proritize Frontier Points
    for n in self.free_space:
      frontier = self.values[tuple(n)].frontier 
      cur_dist = np.linalg.norm(np.array(cur_postion)-np.array(n))
      if cur_dist < max_distance and cur_dist > min_distance and frontier:
        rospy.loginfo("Frontier Guided")
        return self.values[n]

    for n in self.free_space:
      if cur_dist < max_distance:
        return self.values[n]

    rospy.loginfo("Using Random Random Goal Location")
    idx = np.random.randint(len(self.free_space)-1)
    return self.values[self.free_space[idx]]

  #Frontier without random noise
  # def get_random_goal(self, cur_postion):
  #   # idx = np.random.randint(len(self.free_space))
  #   #With random parbability to use random walk
  #   probability = np.random.uniform()
  #   if len(self.free_space) < 1:
  #     rospy.loginfo("No Free Space Detected. Move to neighbour")
  #     path = self.random_walk(cur_postion)
  #     # target = np.array(cur_postion) + [-1,-1,0]
  #     return path[-1]

  #   expl_probability = np.random.uniform()
  #   max_distance = 4 if expl_probability > 0.5 else 4
  #   min_distance = 2 if expl_probability > 0.5 else 2
  #   #Proritize Frontier Points
  #   for n in self.free_space:
  #     frontier = self.values[tuple(n)].frontier 
  #     cur_dist = np.linalg.norm(np.array(cur_postion)-np.array(n))
  #     if cur_dist < max_distance and cur_dist > min_distance and frontier:
  #       rospy.loginfo("Frontier Guided")
  #       return self.values[n]

  #   for n in self.free_space:
  #     if cur_dist < max_distance:
  #       return self.values[n]

  #   rospy.loginfo("Using Random Random Goal Location")
  #   idx = np.random.randint(len(self.free_space)-1)
  #   return self.values[self.free_space[idx]]

  def random_walk(self, cur_postion):
    def altitude_control(pos):
      p = np.array(pos)
      # p[X] = np.clip(p[X], 0.3, 8.5)
      # p[Y] = np.clip(p[X], 0.3, 8.5)
      # p[Z] = np.clip(p[Z], 0.1, 2.3)
      p[X] = np.random.randint(3,8) if (p[X] < 2 or p[X] > 8.5) else p[X]
      p[Y] = np.random.randint(3,8) if (p[Y] < 2 or p[Y] > 8.5) else p[Y]
      p[Z] = 2 if (p[Z] < 0.5 or p[Z] > 3) else p[Z]
      return tuple(p)
    transforms = np.array([
      [ 1 , 1 , 0 ],
      [ 1 , 1 , -1 ],
      [ 1 , 0 , 0 ],
      [ 1 , 0 , -1 ],
      [ 1 , -1 , 0 ],
      [ 1 , -1 , -1 ],
      [ 0 , 1 , 0 ],
      [ 0 , 1 , -1 ],
      [ 0 , 0 , -1 ],
      [ 0 , -1 , 1 ],
      [ 0 , -1 , 0 ],
      [ 0 , -1 , -1 ],
      [ -1 , 1 , 0 ],
      [ -1 , 1 , -1 ],
      [ -1 , 0 , 0 ],
      [ -1 , 0 , -1 ],
      [ -1 , -1 , 0 ],
      [ -1 , -1 , -1 ]])

    # cur = self.values[self.get_index(cur_postion)]
    # idx = np.random.randint(len(transforms))
    # target_key = self.get_index(np.array(cur.key + transforms[idx]))
    # print("cur_postion", cur_postion)
    # print("cur_key", cur)
    # print("target key", target_key)
    # print("transform", transforms[idx])
    # target_pos = self.key_to_center_pos(target_key)

    # if np.all(target_pos == np.array([9,9,4])):
    #   print("Change position")
    #   target_pos = [7, 9, 2]
    #   target_key = self.get_index(target_pos)

    # print("target pos", target_pos)
    # path = [Cube(self.get_index(cur_postion),cur_postion),Cube(target_key,target_pos)]
    # print("path", path)
    # return path

    cur = self.values[self.get_index(cur_postion)]
    neighbour = self.next(cur, 0)
    if len(neighbour) == 0:
      rospy.loginfo("No Neighbour")
      idx = np.random.randint(len(transforms))
      print("random idx", idx)
      target_key = self.get_index(np.array(cur.key + transforms[idx]))
      target_pos = altitude_control(self.key_to_center_pos(target_key))
      return [Cube(self.get_index(cur_postion),cur_postion),Cube(target_key,target_pos)]

    idx = np.random.randint(len(neighbour))
    target = neighbour[idx][0]

    if np.all(self.get_index(target.key) == self.get_index(cur.key)):
      rospy.loginfo("No Neighbour")
      idx = np.random.randint(len(transforms))
      target_key = self.get_index(np.array(cur.key + transforms[idx]))
      target_pos = altitude_control(self.key_to_center_pos(target_key))
      return [Cube(self.get_index(cur_postion),cur_postion),Cube(target_key,target_pos)]

    path = [cur,target]
    return path

  def get_index(self, position):
    idx = ((np.array(position) - np.array([0,0,0])) / np.array(ROBOT_SIZE)).astype(np.int32)
    idx[0] = np.clip(idx[0], 0, self.grid_sz[0] - 1)
    idx[1] = np.clip(idx[1], 0, self.grid_sz[1] - 1)
    idx[2] = np.clip(idx[2], 0, self.grid_sz[2] - 1)
    return tuple(idx)

  def update(self, occupied_cells, free_cells):
    occuiped_space = np.zeros((self.grid_sz))
    free_space = np.zeros((self.grid_sz))

    for p in free_cells:
      free_space[self.get_index(p[:3])] += 1

    for p in occupied_cells:
      occuiped_space[self.get_index(p[:3])] += 1

    print("Occuiped", np.count_nonzero(occuiped_space))
    print("Free", np.count_nonzero(free_space))
    # print(occuiped_space[:10,:10,:10])


    for z in range(self.grid_sz[2]):
      for y in range(self.grid_sz[1]):
        for x in range(self.grid_sz[0]):
          updatable = (self.values[(x,y,z)].type == UNKNOWN) and (occuiped_space[x,y,z] + free_space[x,y,z]) > 5
          if free_space[x,y,z] >= 5 and updatable:
            self.values[(x,y,z)].type = FREE
            self.free_space.append((x,y,z))
            self.grid[x,y,z] = 255 #Set to known
          if occuiped_space[x,y,z] >= 5 and updatable:
            self.values[(x,y,z)].type = OCCUPIED
            self.grid[x,y,z] = 255 #Set to known

    self.is_frontier(self.grid) #To set edges as frontier

  def is_frontier(self, value):
    # set frontier points
    height = value.shape[-1]
    for h in range(1):
      im = np.array(value[:,:,h]).astype(np.uint8)
      canny = skimage.filter.canny(im, sigma=1)
      canny = np.nonzero(canny.reshape(value.shape[:2]))
      idx = zip(*canny)
      # print("canny", canny)
      # print("idx",idx)
      for elem in idx:
        key = (elem[0], elem[1], h)
        self.values[key].is_frontier = True

    # Working example of skimage canny 
    # im = [
    #   [0,0,0,0,0,255,255,255,255,255,255,255],
    #   [0,0,0,0,0,255,255,255,255,255,255,255],
    #   [0,0,0,0,0,255,255,255,255,255,255,255],
    # ]
    # im = np.array(im).astype(np.uint8)
    # # sx = ndimage.sobel(im, axis=0, mode='constant')
    # # sy = ndimage.sobel(im, axis=1, mode='constant')
    # # sob = np.hypot(sx, sy)
    # sob = skimage.filter.canny(im, sigma=1)
    # print(sob)

  # def is_frontier(self, position):
  #   unknown_count = 0
  #   cur = self.values[self.get_index(position)]
  #   arr = self.next(cur, 0, no_collision_check=True)
  #   for elem in arr:
  #     if elem[0].type == UNKNOWN:
  #       unknown_count += 1
  #     if unknown_count > 9:
  #       return True
  #   return False


  def next(self, node, t, no_collision_check=False):
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

    if no_collision_check:
      for trans in transforms:
        new_key = cur_key + trans
        if self.is_valid(new_key):
          n.append((self.values[tuple(new_key)],t))
      return n

    for trans in transforms:
      new_key = cur_key + trans
      if self.is_valid(new_key) and self.is_free(self.key_to_center_pos(new_key)):
        # print("Found free space")
        n.append((self.values[tuple(new_key)],t))
    return n

  def estimate(self, node, goal, t):
    return np.linalg.norm(node.center - goal.center)

def get_velocity(position, path_points, goal = None):
  # PID Controller for velocity
  def altitude_control(pos):
      p = np.array(pos)
      # p[X] = np.clip(p[X], 0.3, 8.5)
      # p[Y] = np.clip(p[X], 0.3, 8.5)
      # p[Z] = np.clip(p[Z], 0.1, 2.3)
      p[X] = np.random.randint(3,8) if (p[X] < 2 or p[X] > 8.5) else p[X]
      p[Y] = np.random.randint(3,8) if (p[Y] < 2 or p[Y] > 8.5) else p[Y]
      p[Z] = 2 if (p[Z] < 0.5 or p[Z] > 3) else p[Z]
      return tuple(p)

  if len(path_points) == 1 and isinstance(goal, np.ndarray):
    print("Path Point Size = 1 and suggested goal is avaliable")
    return altitude_control(goal)
  if len(path_points) < 2:
    print("Path Point Size = 1")
    return altitude_control(position)

  MAX_SPEED = 2
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
  # rospy.loginfo("Min index: %d", min_index)
  # print("Error", error)

  # scale = (MAX_SPEED/(1+np.exp((1/0.3)*(error-0.6))))
  scale = (MAX_SPEED/(1+np.exp((1/0.8)*(error-0.9))))

  # print("Scaling velocity", scale)
  # v = (path_points[min_index + 1] - path_points[min_index]) * scale
  # target_position = path_points[min_index] + v
  v = (path_points[min_index + 1] - position) * scale
  target_position = position + v
  return altitude_control(target_position)

def get_path(cube_path, suggested_goal):
  path = []
  distance = 0.5

  # Random Walk Mode
  if len(cube_path) == 1:
    cube_path.append(suggested_goal)
    rospy.loginfo("Random Walk Big Steps")
  
  # AStar Mode
  # print("Init Path Length", len(cube_path))
  for idx in range(1, len(cube_path)):
    past = cube_path[idx-1].center
    cur = cube_path[idx].center

    a = abs(past[X]-cur[X])/distance
    b = abs(past[Y]-cur[Y])/distance
    c = abs(past[Z]-cur[Z])/distance
    num_of_sample = round(max(a,b,c))
    num_of_sample = np.clip(num_of_sample, 2, None)
    print("Num of sample", num_of_sample)

    x = np.linspace(past[X], cur[X], num_of_sample)
    y = np.linspace(past[Y], cur[Y], num_of_sample)
    z = np.linspace(past[Z], cur[Z], num_of_sample)

    coords = zip(x,y,z)
    for pt in coords:
      path.append(np.array(pt))
  # print("New Path Length", len(path))
  return path

def run():
  f = open("/root/hector_quadrotor_tutorial/front_results.csv", "a")
  rospy.init_node('duck_action_node') #Name of publisher
  
  #for the octomap
  pointcloud2_publisher = Laser2PC()

  lasersub = Laser2PCWorld()
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
  start_time = rospy.Time.now().to_sec()
  suggested_goal = None

  while not rospy.is_shutdown():
    if len(current_path) == 0:
      lasersub.update()

    current_time = rospy.Time.now().to_sec()

    if not lasersub.ready:
      rospy.loginfo("Not Ready")
      rate_limiter.sleep()
      continue

    # Follow path
    position = localisation.get_pose()
    if len(current_path) > 0:
      moveit = get_velocity(position, current_path, goal=suggested_goal.center)
      print("Move it to", moveit)
      # moveit = current_path[-1]
      # Send it to action library
      g = PoseGoal()
      g.target_pose.header.seq = frame_id
      g.target_pose.header.frame_id = 'world'
      g.target_pose.pose.position.x = moveit[X]
      g.target_pose.pose.position.y = moveit[Y]
      g.target_pose.pose.position.z = moveit[Z]
      # print("move it", moveit[X], moveit[Y], moveit[Z])
      # rospy.loginfo("Sending goal")
      client.send_goal(g)
      client.wait_for_result()
      # Increment Frame
      frame_id += 1

    # Update plan every 10s.
    time_since = current_time - previous_time
    if time_since >= 10.:
      print("Update Occupancy")
      lasersub.update()
    if current_path and time_since < 10.:
      rate_limiter.sleep()
      continue
    previous_time = current_time

    # Print map exploration progress every 10 sec
    time_since = current_time - start_time
    if time_since > 5:
      occupancy_grid.get_map_percentage()
      rospy.loginfo("-------------------------")
      rospy.loginfo("Mapping percentage %s", occupancy_grid.map_percentage)
      rospy.loginfo("Mapping percentage %s", occupancy_grid.map_percentage)
      start_time = current_time
      f.write(str(occupancy_grid.map_percentage) + "\n")

    # Get Occuiped Cell (in world coordinate)
    occupied_cells, free_cells = lasersub.occupied_cells,lasersub.free_cells 
    occupancy_grid.update(occupied_cells, free_cells)

    # Run Exploration Planner
    center = occupancy_grid.get_index(position)
    start = Cube(center, position)
    suggested_goal = occupancy_grid.get_random_goal(position) #random walk get avaliable neighbour
    print("Set Goal Location:", suggested_goal.center)
    cube_path = astar.astar(occupancy_grid, start, suggested_goal)
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
  f.close()

if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException:
    pass