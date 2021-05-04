Structure reconstruction using drone.

### Demo
Demo video available - frontier_exploration_demo.mp4

### Run Scheme - Predefined Path
Run the following commands in separate terminal windows. 
```
roslaunch duck_demo duck_flight_gazebo.launch
rosservice call enable_motors true
roslaunch hector_exploration_node exploration_planner.launch 
roslaunch octomap_server octomap_mapping.launch
roslaunch duck_controller action_openloop.launch
```

### Run Scheme - Random Walk
Run the following commands in separate terminal windows. 
```
roslaunch duck_demo duck_flight_gazebo.launch
rosservice call enable_motors true
roslaunch hector_exploration_node exploration_planner.launch 
roslaunch octomap_server octomap_mapping.launch
roslaunch duck_controller action_astar.launch
```

### Run Scheme - Frontier Exploration
Run the following commands in separate terminal windows. 
```
roslaunch duck_demo duck_flight_gazebo.launch
rosservice call enable_motors true
roslaunch hector_exploration_node exploration_planner.launch 
roslaunch octomap_server octomap_mapping.launch
roslaunch duck_controller action_frontier.launch
```

### Code Files
Code files are located at `code/src/duck/`


### Other Remarks
https://gist.github.com/elimkwan/f5e3c09187ee5dbe48e5740af9064707
