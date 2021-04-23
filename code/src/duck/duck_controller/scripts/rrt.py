from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import matplotlib.pylab as plt
import matplotlib.patches as patches
import numpy as np
import os
import re
import scipy.signal
import yaml
import math
import random
from mpl_toolkits import mplot3d


# https://github.com/ycaibb/octomap_rrt

# Constants used for indexing.
X = 0
Y = 1
YAW = 2

# Constants for occupancy grid.
FREE = 0
UNKNOWN = 1
OCCUPIED = 2

ROBOT_RADIUS = 0.3 / 2.
ROBOT_RADIUS = 1 / 2.
# GOAL_POSITION = np.array([1.5, 1.5], dtype=np.float32)  # Any orientation is good.
START_POSE = np.array([-1.5, -1.5, 0.], dtype=np.float32)
MAX_ITERATIONS = 500
# np.random.seed(42)


def rrt(start_pose, goal_position, occupancy_grid):
  # RRT builds a graph one node at a time.
  graph = []
  start_node = Node(start_pose)
  final_node = None
  if not occupancy_grid.is_free(goal_position):
    print('Goal position is not in the free space. Resample Goal')
    occupancy_grid.draw()
    fp = sample_random_position(occupancy_grid)
    final_node = Node(fp)
    return start_node, final_node
  graph.append(start_node)
  print("Start RRT")
  for _ in range(MAX_ITERATIONS): 
    position = sample_random_position(occupancy_grid)
    # With a random chance, draw the goal position.
    if np.random.rand() < .05:
      position = goal_position
    # Find closest node in graph.
    # In practice, one uses an efficient spatial structure (e.g., quadtree).
    potential_parent = sorted(((n, np.linalg.norm(position - n.position)) for n in graph), key=lambda x: x[1])
    # Pick a node at least some distance away but not too far.
    # We also verify that the angles are aligned (within pi / 4).
    u = None
    for n, d in potential_parent:
      if d > .2 and d < 3:
        u = n
        break
    else:
      continue
    v = adjust_pose(u, position, occupancy_grid)
    if v is None:
      continue
    u.add_neighbor(v)
    v.parent = u
    graph.append(v)
    if np.linalg.norm(v.position - goal_position) < .2:
      final_node = v
      break
  return start_node, final_node


def sample_random_position(occupancy_grid):
  position = np.zeros(3, dtype=np.float32)

  width = 2
  sign = np.ones(3)
  sign[0] = -1 if np.random.sample(1) < 0.5 else 1
  sign[1] = -1 if np.random.sample(1) < 0.5 else 1
  sign[2] = -1 if np.random.sample(1) < 0.5 else 1
  coord = np.round(np.random.sample(3)*width,2)
  # coord = np.multiply(coord , sign)
  
  while not occupancy_grid.is_free(coord):
    sign[0] = -1 if np.random.sample(1) < 0.5 else 1
    sign[1] = -1 if np.random.sample(1) < 0.5 else 1
    sign[2] = -1 if np.random.sample(1) < 0.5 else 1
    coord = np.round(np.random.sample(3)*width, 2)
    # coord = np.multiply(coord , sign)
  position = coord
  return position


def adjust_pose(node, final_position, occupancy_grid):
  final_pose = node.pose.copy()
  final_pose[:3] = final_position
  final_node = Node(final_pose)

  if occupancy_grid.is_free(final_node.position):
    return final_node
  else:
    return None

# Defines a node of the graph.
class Node(object):
  def __init__(self, pose):
    self._pose = pose.copy()
    self._neighbors = []
    self._parent = None
    self._cost = 0.

  @property
  def pose(self):
    return self._pose

  def add_neighbor(self, node):
    self._neighbors.append(node)

  @property
  def parent(self):
    return self._parent

  @parent.setter
  def parent(self, node):
    self._parent = node

  @property
  def neighbors(self):
    return self._neighbors

  @property
  def position(self):
    return self._pose[:3]

  @property
  def cost(self):
      return self._cost

  @cost.setter
  def cost(self, c):
    self._cost = c

#Defines an occupancy grid.
class OccupancyGrid(object):
  def __init__(self, sz, origin, resolution):
    # a 3D occupancy grid with more focus on 2D planes
    full_sz = (int(sz[0]/resolution), int(sz[0]/resolution), int(sz[0]/resolution))
    # self._original_values = np.ones((full_sz))
    self._original_values = self.init_obstacles(np.ones((full_sz)))
    # self._values = self.convolution(self._original_values, resolution)
    self._values = self._original_values.copy()

    self._origin = np.array(origin[:3], dtype=np.float32)
    self._origin -= resolution / 2.
    self._resolution = resolution

  def draw(self):
    # # combine the objects into a single boolean array
    # voxels = self._original_values
    # # and plot everything
    # ax = plt.figure().add_subplot(projection='3d')
    # ax.voxels(voxels, edgecolor='k')
    # plt.imshow()


    fig = plt.figure()
    ax = plt.axes(projection='3d')
    mask = np.array([self._original_values > 1]).reshape((20,20,20))
    k = np.nonzero(mask)
    coord = zip(k[0], k[1], k[2])
    X = []
    Y = []
    Z = []
    for e in coord:
      X.append(e[0])
      Y.append(e[1])
      Z.append(e[2])
      
    # X = np.arange(10)
    # Y = np.arange(10)
    # Z = np.array((10,10))
    # for x in X:
    #   for y in Y:
    #     Z[x,y] = self.is_free(())

    # Z = np.array(Z)
    # print(X.shape)

    ax.voxels(X, Y, Z)
    plt.show()


    # plt.imshow(self._original_values[:,:,1], interpolation='none', origin='lower')
    #           #  extent=[self._origin[X],
    #           #          self._origin[X] + self._values.shape[0] * self._resolution,
    #           #          self._origin[Y],
    #           #          self._origin[Y] + self._values.shape[1] * self._resolution])
    # plt.set_cmap('gray_r')

  def init_obstacles(self, grid):
    grid[:,:,0] = OCCUPIED
    grid[8:12,8:12,0:2] = OCCUPIED
    return grid

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

  def is_free(self, position):
    # dependency on rrt loop
    if (position[X] > 4 or position[X] < 6) and (position[Y] > 4 or position[Y] < 6):
      return False
    if position[Z] < 1 or position[Z] > 5:
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

  def update(self, positions, location_property):
    # dependency on controller update
    # occupied_cell = [[x,y,z], [x,y,z] ...]
    for p in positions:
      self._values[self.get_index(p[:3])] == location_property
    # rospy.loginfo(rospy.get_caller_id() + ' Updated OG')
    return True


if __name__ == '__main__':
  # Plot environment.
  occupancy_grid = OccupancyGrid((10,10,3),(0,0,0),0.5)
  occupancy_grid.draw()
  # Run RRT.
  # GOAL_POSITION = sample_random_position(occupancy_grid)
  # print("Goal is at", GOAL_POSITION)
  # start_node, final_node = rrt(START_POSE, GOAL_POSITION, occupancy_grid)
  # print("Finish RRT")



# if __name__ == '__main__':
#   parser = argparse.ArgumentParser(description='Uses RRT to reach the goal.')
#   parser.add_argument('--map', action='store', default='map', help='Which map to use.')
#   args, unknown = parser.parse_known_args()

#   # Load map.
#   with open(args.map + '.yaml') as fp:
#     data = yaml.load(fp)
#   img = read_pgm(os.path.join(os.path.dirname(args.map), data['image']))
#   occupancy_grid = np.empty_like(img, dtype=np.int8)
#   occupancy_grid[:] = UNKNOWN
#   occupancy_grid[img < .1] = OCCUPIED
#   occupancy_grid[img > .9] = FREE
#   # Transpose (undo ROS processing).
#   occupancy_grid = occupancy_grid.T
#   # Invert Y-axis.
#   occupancy_grid = occupancy_grid[:, ::-1]
#   occupancy_grid = OccupancyGrid(occupancy_grid, data['origin'], data['resolution'])

#   # Run RRT.
#   GOAL_POSITION = sample_random_position(occupancy_grid)
#   print("Goal is at", GOAL_POSITION)
#   start_node, final_node = rrt(START_POSE, GOAL_POSITION, occupancy_grid)

#   # Plot environment.
#   fig, ax = plt.subplots()
#   occupancy_grid.draw()
#   plt.scatter(.3, .2, s=10, marker='o', color='green', zorder=1000)
#   draw_solution(start_node, final_node)
#   plt.scatter(START_POSE[0], START_POSE[1], s=10, marker='o', color='green', zorder=1000)
#   plt.scatter(GOAL_POSITION[0], GOAL_POSITION[1], s=10, marker='o', color='red', zorder=1000)
  
#   plt.axis('equal')
#   plt.xlabel('x')
#   plt.ylabel('y')
#   plt.xlim([-.5 - 2., 2. + .5])
#   plt.ylim([-.5 - 2., 2. + .5])
#   plt.show()
  