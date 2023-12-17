# gp4-lego-assembly

## ROS Topic Message for Integration
* `/namespace/execution_status` (Pub): Int64 (1: robot executing a task, 0: robot is idle).
* `/namespace/task_type` (Sub): Int64 (0: Disassemble, 1: Assemble).
* `/namepsace/assembly_task` (Sub): Int64 (0: Human, 1: Heart, 2: Stairs).
* `/namespace/start_task` (Sub): Int64 (0: No new task, 1: New task coming).
* `/namespace/assembly_plate_state` (Sub): Float32MultiArray ([x, y, z, roll, pitch, yaw]).
* `/namespace/kit_plate_state` (Sub): Float32MultiArray ([x, y, z, roll, pitch, yaw]).
* `/fts` (Sub): WrenchStamped ([force.x, force.y, force.z, torque.x, torque.y, torque.z]). 



## Integration Execution Instructions
On mfi-twin to launch yk_builder robot
```
roslaunch testbed_utils test_integration.launch
```

On yk_god to launch azure kinect cameras
```
cd ~/
roslaunch yk_god.launch
```

On yk_god to launch amr detection pipeline
```
roslaunch testbed_camera_utils detect_amr.launch namespace:=yk_builder
```

On mfi-twin to run pallet pick and place server
```
roslaunch pallet_pick_and_place pallet_pick_and_place_server.launch namespace:=yk_builder
```

On mfi-twin to run flask server
```
cd ~/Downloads
python resource-server.py
```