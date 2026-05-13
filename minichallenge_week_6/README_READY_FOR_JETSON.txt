MiniChallenge Week 6 corrected for Puzzlebot Jetson

Changes:
- Fixed OpenCV font constant in color_detector_node.py.
- Route changed to two waypoints: (0.0, 1.2) then (1.2, 1.2).
- challenge.launch.py can launch the Jetson camera using ros_deep_learning video_source.ros2.launch.

Run on Jetson:
cd ~/ros2_ws
colcon build --packages-select minichallenge_week_6
source install/setup.bash
ros2 launch minichallenge_week_6 challenge.launch.py

If the camera is already running separately:
ros2 launch minichallenge_week_6 challenge.launch.py launch_camera:=false
