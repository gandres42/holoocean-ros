A Holoocean ROS2 wrapper built using the python API with multiagent support

## Setup
1. Install holoocean python client using directions from the main repo

2. Install requirements
```
pip install -r r requirements.txt
```
Note that if using Holoocean 1.0.0 or earlier numpy must be version 1.26.4 or lower.

3. Create a ros workspace
```
mkdir holoocean_ws
cd holoocean_ws
```

4. Clone holoocean-ros
```
mkdir src
cd src
git clone https://github.com/gandres42/holoocean-ros.git
cd ..
```

5. Build and source the project
```
colcon build
source install/setup.bash
```

## Use
Start the simulator by running
```
ros2 run holoocean_ros sim --ros-args -p config_path:={config file path}
```
where `config_path` is the filepath of a valid configuration json file.  Configuration is the same as the main simulator, just provide the path to your scenario config to get started.

To run a single agent in the SimpleUnderwater environment, run
```
ros2 run holoocean_ros sim --ros-args -p config_path:=./src/holoocean-ros/holoocean_ros/configs/config_demo.json
```

## Manually Controlling
To manually control an agent, run the manual control node:
```
ros2 run holoocean_ros manual --ros-args -p agent_name:={agent_name}
```
where `agent_name` matches an agent name provided in the config file

## Migeran Compatiblity
To maintain compatilbity with the ROS2 wrapper created by Migeran, we include a point cloud generation node that converts sonar data into a PointCloud2 topic:
```
ros2 run holoocean_ros migeran --ros-args -p config_path:={config path} agent_name:={agent name} sensor_name:={sensor name}
```
Parameters are as follows:

`config_path`: path to configuration file used for current simulation

`sensor_name`: name of sonar sensor to map as pointcloud

`agent_name`: name of agent that has said sensor

`frame_id` (optional): name of frame pointcloud should be published to, defaults to 'map'

`intensity_threshold` (optional): pixel value at which to add a point, ranges from 0-255, defaults to 85