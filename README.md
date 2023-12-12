# gp4-lego-assembly

## ROS Topic Message for Integration
* `/namespace/execution_status` (Pub): Int64 (0: robot executing a task, 1: robot is idle).
* `/namespace/task_type` (Sub): Int64 (0: Disassemble, 1: Assemble).
* `/namepsace/assembly_task` (Sub): Int64 (0: Human, 1: Heart, 2: Stairs).
* `/namespace/start_task` (Sub): Int64 (0: No new task, 1: New task coming).
* `/namespace/assembly_plate_state` (Sub): Float32MultiArray ([x, y, z, roll, pitch, yaw]).
* `/namespace/kit_plate_state` (Sub): Float32MultiArray ([x, y, z, roll, pitch, yaw]).
* `/fts` (Sub): WrenchStamped ([force.x, force.y, force.z, torque.x, torque.y, torque.z]). 
