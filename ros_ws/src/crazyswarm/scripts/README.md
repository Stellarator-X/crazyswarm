# Usage

## Setting Up

1. Add all initial positions of the crazyflies to [allCrazyflies.yaml](https://github.com/Stellarator-X/crazyswarm/blob/dev_utt/ros_ws/src/crazyswarm/launch/allCrazyflies.yaml)
2. Run `$ python3 chooser.py` and select the crazyflies.

## Runing a Script

### Simulation

1. `$ python3 <script-name>.py --sim`

### Real World

In separate terminals, run the following - 
1. `$ roscore`
2. `$ roslaunch crazyswarm hover_swarm.launch`
3. `$ python3 script.py`


## Human Detection and Following

1. Run `$ python3 calibrate_cam.py` to calibrate the system by selecting the boundary nodes of the grid (double-click).
2. Node for Human Position Detection - yolo_publisher.py
3. Follower Node - yolo_follower.py
