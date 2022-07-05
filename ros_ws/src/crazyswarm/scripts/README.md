# Usage

## Setting the Crazyflies

1. Add all initial positions of the crazyflies to (https://github.com/Stellarator-X/crazyswarm/blob/dev_utt/ros_ws/src/crazyswarm/launch/allCrazyflies.yaml)[allCrazyflies.yaml]
2. Run `$ python3 chooser.py` and select the crazyflies.

## Runing the Scripts

### Simulation

1. `$ python3 <script-name>.py --sim`

### Real World

In separate terminals, run the following - 
1. `$ roscore`
2. `$ roslaunch crazyswarm hover_swarm.launch`
3. `$ python3 script.py`

