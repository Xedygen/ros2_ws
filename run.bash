#python3 maze_generator.py --cells-x 8
colcon build
source install/setup.bash
ros2 launch jackal_wall_follower start.launch.py #rviz:=true
#python3 visualize.py