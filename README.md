# ros-cloud-transfer

1. Launch User Interface

```bash
cd 5G_ws/src/ros-cloud-transfer
python cloud.py
```

2. Run Path Planning

```bash
cd 5G_ws/src/global_path_planning
catkin_make
source devel/setup.bash
roslaunch global_path_planning global_path_planning.launch
rosrun global_path_planning receiver.py
```

3. Run Gesture Recognition

```bash
cd 5G_ws/src/Real-time-GesRec
./run_online_video_egogesture_depth.sh
```

