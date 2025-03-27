#include "ros/ros.h"
#include <iostream>
#include <queue>
#include <vector>
#include <sstream>
#include <typeinfo>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

using namespace std;

vector<double> home_position = {0, 0, 0, 0, 0, 0};
vector<double> target_position = {-0.38244612, 0.98038946, -2.00329962, 2.21999805, 0.86258962, 1.32790853};
vector<double> pose_1 = {0, 0, 0, -1, -1, -1};
vector<double> final_position = {-1, 0, -1, 1, 0, 2};
vector<double> current_waypoint(6, 1000);
vector<double> current_position(6, 0);  // Store current joint positions
vector<double> errors(6, 0);
queue<vector<double>> q_targets;

const double tolerance = 0.01; // Allowable error in radians
bool goal_reached = false;
bool is_final_waypoint = false;

ros::Publisher joint_position_pub[6];  // Array of publishers for positions
ros::Publisher joint_velocity_pub[6];  // Array of publishers for velocities

ros::Publisher joint_states_pub;
trajectory_msgs::JointTrajectory joint_trajectory;

// <------------------------- Function Declarations ------------------------->
void printVector(vector<double>& matrix);

void poseError();

double calculateAvg(vector<double>& inputVector);

void joint_states_callback(const sensor_msgs::JointState::ConstPtr& joint_states_msg);

void posePublisher(ros::NodeHandle& n);

void addWayPointsToQueue(vector<vector<double>>& waypoints);

void performTastFromActionLibClient();

trajectory_msgs::JointTrajectory prepareJointTrajectoryMessage(vector<double>& current_waypoint);
control_msgs::FollowJointTrajectoryGoal prepareJointTrajectoryGoalMessage(vector<double>& current_waypoint);

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

// <------------------------ End of Function definitions ----------------------->

// Main function --> Starting point
int main(int argc, char **argv) {
    ros::init(argc, argv,"trajectory_publisher_1");
    ros::NodeHandle n;
    vector<vector<double>> waypoints_list = {final_position, target_position, pose_1, home_position};

    // ros::Publisher joint_states_pub = n.advertise<std_msgs::String>("trajectories_1", 1000);
    ros::Publisher joint_states_pub = n.advertise<trajectory_msgs::JointTrajectory>("/ur10/vel_based_pos_traj_controller/command", 1000);
    ros::Subscriber sub = n.subscribe("/ur10/joint_states", 1000, joint_states_callback);

    posePublisher(n); // Generate publishers

    string mode = argv[1];
    // Switching between modes in order to decide the trajectory being sent to robot via 1. ROS pub/sub 2. ROS actionlib
    if(mode == "1") {
        addWayPointsToQueue(waypoints_list);
        current_waypoint = q_targets.front();
        q_targets.pop();

        ros::Rate loop_rate(1);

        while(ros::ok()) {
            std_msgs::String msg;
            // std::cout << "Current waypoint :: " << endl;
            printVector(current_waypoint);
            // // Preparing a message that need to be published
            joint_trajectory = prepareJointTrajectoryMessage(current_waypoint);

            cout << "Joint Trajectory ::"<< joint_trajectory << endl;

            joint_states_pub.publish(joint_trajectory);
            
            ros::spinOnce();

            ros::Duration(2.5).sleep();

            loop_rate.sleep();
        }
    } else {
        performTastFromActionLibClient();   
    }
    return 0;
}

// <------------------------- Function definitions ----------------------------->

// Method to prepare a goal message
trajectory_msgs::JointTrajectory prepareJointTrajectoryMessage(vector<double>& current_waypoint) {
    trajectory_msgs::JointTrajectory joint_trajectory;
    trajectory_msgs::JointTrajectoryPoint point;

    point.positions = current_waypoint;
    point.velocities = {0, 0, 0, 0, 0, 0};
    point.accelerations = {0, 0, 0, 0, 0, 0};
    point.time_from_start = ros::Duration(2.0);

    joint_trajectory.points.push_back(point);
    joint_trajectory.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    joint_trajectory.header.stamp = ros::Time::now();
    return joint_trajectory;
}

// Method to prepare a goal message with joint tolerences
control_msgs::FollowJointTrajectoryGoal prepareJointTrajectoryGoalMessage(vector<double>& current_waypoint) {
    control_msgs::FollowJointTrajectoryGoal joint_trajectory;
    trajectory_msgs::JointTrajectoryPoint point;

    trajectory_msgs::JointTrajectory& traj = joint_trajectory.trajectory;

    // Set the joint positions (current waypoint)
    point.positions = current_waypoint;
    
    // Set the velocities and accelerations to zeros
    point.velocities = {0, 0, 0, 0, 0, 0}; // Assuming the robot has 6 joints
    point.accelerations = {0, 0, 0, 0, 0, 0}; // Same assumption as above
    
    // Set the time from start (10 seconds in your Python code)
    point.time_from_start = ros::Duration(5.0);

    // Add the point to the trajectory
    traj.points.push_back(point);

    // Set the joint names (same as your Python code)
    traj.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    // Set the current timestamp for the header
    traj.header.stamp = ros::Time::now();

    // Initialize tolerances
    double qtol = -1;        // Position tolerance
    double qveltol = -1;     // Velocity tolerance
    double qacctol = -1;     // Acceleration tolerance

    // Create lists to hold tolerances
    std::vector<control_msgs::JointTolerance> goal_tolerances;
    std::vector<control_msgs::JointTolerance> path_tolerances;

    // Populate tolerances for each joint
    for (size_t i = 0; i < current_waypoint.size(); ++i) {
        control_msgs::JointTolerance goal_tolerance;
        control_msgs::JointTolerance path_tolerance;

        // Set joint name from name_list
        goal_tolerance.name = traj.joint_names[i];
        path_tolerance.name = traj.joint_names[i];

        // Set the position, velocity, and acceleration tolerances
        goal_tolerance.position = qtol;
        goal_tolerance.velocity = qveltol;
        goal_tolerance.acceleration = qacctol;
        
        path_tolerance.position = qtol;
        path_tolerance.velocity = qveltol;
        path_tolerance.acceleration = qacctol;

        // Add tolerances to the lists
        goal_tolerances.push_back(goal_tolerance);
        path_tolerances.push_back(path_tolerance);
    }

    // Set the tolerances in the goal message
    joint_trajectory.goal_tolerance = goal_tolerances;
    joint_trajectory.path_tolerance = path_tolerances;

    // Set the goal time tolerance (equivalent to rospy.Duration(0) in Python)
    joint_trajectory.goal_time_tolerance = ros::Duration(0);

    return joint_trajectory;
}

// To print the way point (Type vector)
void printVector(vector<double>& matrix) {
    for(int i=0; i<matrix.size(); i++) {
        cout << matrix[i] << " ";
    }
    cout << endl;
}

// PoseError (Current pose - actual pose)
void poseError() {
    for(int i=0; i<current_waypoint.size(); i++) {
        // In current position, joint names are swapped (1st and 3rd postions are swapped)
        if(i == 0) {
            errors[i] = abs(current_waypoint[0] - current_position[2]);
        } else if(i == 2) {
            errors[i] = abs(current_waypoint[2] - current_position[0]);
        } else {
            errors[i] = abs(current_waypoint[i] - current_position[i]);
        }
    }
}

// To calculate Average pose error
double calculateAvg(vector<double>& inputVector) {
    int n = inputVector.size();
    double avg = 0;
    double sum = 0;
    for(int i=0; i<n; i++) {
        sum += inputVector[i];
    }
    avg = sum/n;
    return avg;
}

// Method to check if the manipulator has reached the given point and also publishes the joint states
void joint_states_callback(const sensor_msgs::JointState::ConstPtr& joint_states_msg) {
    current_position = joint_states_msg->position; // Update current position

    poseError();
    printVector(errors);
    
    if (calculateAvg(errors) < tolerance) {
        cout <<  "Goal Reached !" << endl;
        if(!q_targets.size()) {
            ros::shutdown();
            return;
        }
        current_waypoint = q_targets.front();
        q_targets.pop();
    }

    for (size_t i = 0; i < current_position.size(); i++) {
        std_msgs::Float64 pos_msg, vel_msg;
        pos_msg.data = joint_states_msg->position[i];
        vel_msg.data = joint_states_msg->velocity[i];

        joint_position_pub[i].publish(pos_msg);
        joint_velocity_pub[i].publish(vel_msg);
    }
}

void posePublisher(ros::NodeHandle& n) {
    for (int i = 0; i < 6; ++i) {
        string pos_topic = "/joint_" + to_string(i+1) + "_position";
        string vel_topic = "/joint_" + to_string(i+1) + "_velocity";
        
        joint_position_pub[i] = n.advertise<std_msgs::Float64>(pos_topic, 1000);
        joint_velocity_pub[i] = n.advertise<std_msgs::Float64>(vel_topic, 1000);
    }
}

void addWayPointsToQueue(vector<vector<double>>& waypoints) {
    for(int i=0; i<waypoints.size(); i++) {
        q_targets.push(waypoints[i]);
    }
}

void performTastFromActionLibClient() {
      // Define the action client (true -> spin a separate thread)
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> 
        followJointTrajectoryActClient("/ur10/vel_based_pos_traj_controller/follow_joint_trajectory", true);

    ROS_INFO("Waiting for the action server to start...");
    followJointTrajectoryActClient.waitForServer();  // Wait for the action server to start

    ROS_INFO("Action server started, sending goal.");

    vector<vector<double>> waypoints_list = {final_position, target_position, home_position};

    for(int i=0; i<waypoints_list.size(); i++) {
        control_msgs::FollowJointTrajectoryGoal goal_pose_msg = prepareJointTrajectoryGoalMessage(waypoints_list[i]);

        followJointTrajectoryActClient.sendGoalAndWait(goal_pose_msg);

        cout << "GOAL in Progress" << endl;

        // Wait for the result
        bool finishedBeforeTimeout = followJointTrajectoryActClient.waitForResult(ros::Duration(30.0));

        // Process the result
        if (finishedBeforeTimeout) {
            actionlib::SimpleClientGoalState state = followJointTrajectoryActClient.getState();
            // ROS_INFO("UR arm motion completed with state: %s", state.toString().c_str());
            cout << "Arm motion completed with goal :: " << state.toString().c_str() <<endl;
        } else {
            ROS_WARN("Action did not finish before the timeout.");
        }
    }

}

// <----------------------------------- END ------------------------------------>