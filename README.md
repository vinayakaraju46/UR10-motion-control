# ROS Trajectory Publisher for Robot Manipulators

This repository contains a ROS (Robot Operating System) node for controlling a robot manipulator by sending joint trajectory commands. The code provides two modes of operation for controlling the robot using either ROS publishers/subscribers or ActionLib clients. It supports trajectory planning to move the robot through a series of waypoints.


https://github.com/user-attachments/assets/4c603a20-0878-4303-af51-b1fb0d76d9ea




## Features
- **Trajectory Planning**: The robot moves through predefined waypoints, defined as joint positions for the robot manipulator.
- **Two Modes of Operation**:
  - **Publisher/Subscriber Mode**: Sends joint trajectory commands through ROS topics using publishers/subscribers.
  - **ActionLib Mode**: Uses ROS ActionLib to send joint trajectory goals and wait for results.
- **Real-Time Feedback**: The robot’s joint states are continuously monitored, and feedback is provided in the form of joint positions and velocities.

## Dependencies
- ROS Noetic or compatible version
- `actionlib` (actionlib, control_msgs, trajectory_msgs)
- ROS `std_msgs`, `sensor_msgs`

## Installation

Clone this repository into your ROS workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/your-username/trajectory_publisher.git
cd ..
catkin_make
source devel/setup.bash
```

# Trajectory Publisher Node

This ROS node allows you to control the movement of a robot by sending joint trajectory commands. It supports two modes of operation: **Publisher/Subscriber mode** and **ActionLib mode**. The mode is selected by passing a command-line argument.

### Run the Node
This code supports two modes of operation, selected by passing a command-line argument (1 for Publisher/Subscriber mode and 2 for ActionLib mode).

#### Publisher/Subscriber Mode (Mode 1)

This mode publishes joint trajectory messages directly to the robot’s trajectory controller via a ROS topic.

```bash
rosrun trajectory_publisher trajectory_publisher_1 1
```

#### ActionLib Mode (Mode 2)

This mode sends trajectory goals using ActionLib, waiting for the action server to complete the motion before proceeding to the next goal.

```bash
rosrun trajectory_publisher trajectory_publisher_1 2
```
## Configuration
- The waypoints for the robot's trajectory are predefined in the code.

- The robot’s joint positions are updated in real-time, and the program checks whether the robot has reached the target positions based on a defined tolerance.


## ROS Topics and Action:

- The publisher uses /ur10/vel_based_pos_traj_controller/command to send trajectory commands.

 - The action client communicates with /ur10/vel_based_pos_traj_controller/follow_joint_trajectory to send goals and wait for results.

## Waypoints:

The robot moves through the following waypoints:

- home_position: The robot’s initial or home position.

- target_position: The desired position for the robot’s joints.

- pose_1: Another predefined position.

- final_position: The final position to reach before completing the motion.

## Error Checking:

The node continuously monitors the robot’s joint states and compares them with the desired goal. If the robot reaches the goal within a predefined tolerance (tolerance = 0.01 radians), the next waypoint is queued.

## Code Structure

### Main Node:
- The main logic of the trajectory publisher is in the `main` function.
- The waypoints are stored in a queue and processed sequentially.

### Trajectory Message Preparation:
- Functions like `prepareJointTrajectoryMessage` and `prepareJointTrajectoryGoalMessage` create trajectory messages for either publisher or action client modes.

### Pose Error Calculation:
- The robot’s current pose is compared to the desired pose, and errors are calculated for each joint.

### ActionLib Client:
- In ActionLib mode, the `SimpleActionClient` sends the trajectory goals to the robot and waits for the completion of each action.

## Functions

### printVector:
- Prints the joint positions in the current waypoint.

### poseError:
- Calculates the error between the robot’s current position and the target position.

### calculateAvg:
- Calculates the average error across all joints.

### joint_states_callback:
- Callback function that updates the current joint positions and checks if the goal has been reached.

### posePublisher:
- Creates ROS publishers for joint positions and velocities.

### addWayPointsToQueue:
- Adds the predefined waypoints to a queue for sequential processing.

### performTaskFromActionLibClient:
- Performs the task using ActionLib to send and wait for joint trajectory goals.

## Notes
- The robot controller should be running and subscribed to the correct trajectory topics for the robot to move.
- Ensure that the robot has a valid URDF description loaded in the ROS parameter server (`robot_description`).
- The tolerance for goal completion is set to 0.01 radians, which can be adjusted based on the required precision.

## License
This project is licensed under the MIT License - see the [LICENSE](./LICENSE) file for details.
