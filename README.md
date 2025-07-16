## Installation
Set up the project:
```bash
cd
mkdir -p multi_nav/src
cd multi_nav
git clone https://github.com/Singh-0016/Multi-Map-Navigation.git
mv Multi-Map-Navigation/* Multi-Map-Navigation/.[!.]* src/
rm -rf Multi-Map-Navigation
colcon build
```
(ignore any warnings)
## Launch Sequence
Make a new file, mycommands/sh, in the multi_nav directory. 
```bash
touch mycommands.sh
nano mycommands
```
Copy and paste these commands. 
```bash
trap "pkill -f ros2; pkill -f gazebo; exit" SIGINT SIGTERM
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch gazebo_ros gazebo.launch.py world:=$(pwd)/src/experiment_rooms/worlds/room2/world.model &

sleep 5

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py x_pose:=-6.0 y_pose:=0.0 z_pose:=0.0 &

sleep 5

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$(pwd)/src/maps/room2_map.yaml &

sleep 8

source install/setup.bash
ros2 run nav_server nav_server_node
```
Write, save, and exit. Make the file executable:
```bash
chmod +x mycommands
```
Now run the commands from 1 to 4 in a single terminal using ./mycommands  
Run 5 and 6 each in a new terminal.

1. Launch facility with three rooms in the Gazebo
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch gazebo_ros gazebo.launch.py world:=$(pwd)/src/experiment_rooms/worlds/room2/world.model
```
2. Launch Turtlebot
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py x_pose:=-6.0 y_pose:=0.0 z_pose:=0.0
```
3. Launch Navigation and Map Server(Nav2)
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$(pwd)/src/maps/room1_map.yaml
```
4. Run custom nav_server
```bash
source install/setup.bash
ros2 run nav_server nav_server_node
```
5. Run Wormhole Service
```bash
source install/setup.bash
ros2 run wormhole_ser wormhole_service
```
6. Run Multi-Map Manager Node
```bash
source install/setup.bash
ros2 run multi_map_manager multi_map_manager_node
```
## Example 
```bash
ros2 action send_goal /multi_map_navigate multi_map_manager/action/MultiMapNavigate "{target_map: 'room2', target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
