source install/setup.bash
ros2 launch jackal_wall_follower restart_controllers.launch.py
sleep 1
ros2 launch jackal_wall_follower start_robot.launch.py
python3 visualize.py