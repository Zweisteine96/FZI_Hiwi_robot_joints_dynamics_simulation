# Overview of the implementation and modifications

## PACKAGE ar_conveyor_launch

### target_publisher_node
In the file target_publisher_node.cpp, a function listener_.waitForTransform is added. The file can be checked in branch "test".

```
tf::StampedTransform transform;
    try
    {
      //Here adding the function listener_.waitForTransform() by Cheng
      listener_.waitForTransform(base_frame_, end_effector_frame_, ros::Time(0), ros::Duration(10.0) );
      listener_.lookupTransform(base_frame_, end_effector_frame_, ros::Time(0), transform);
    }
    
```
### controller_ur_modern
In the file controller_ur_modern.yaml, the joint_trajectory_controller is modified such that the effort controller can be applied. The PID parameters need to be further tuned for real tests.

```
joint_trajectory_controller:
    #type: "position_controllers/JointTrajectoryController"
    #type: "velocity_controllers/JointTrajectoryController"
    type: "effort_controllers/JointTrajectoryController"
    joints:
    - ur10_shoulder_pan_joint
    - ur10_shoulder_lift_joint
    - ur10_elbow_joint
    - ur10_wrist_1_joint
    - ur10_wrist_2_joint
    - ur10_wrist_3_joint
    constraints:
      goal_time: 0.6
      stopped_velocity_tolerance: 0.05
      ur10_shoulder_pan_joint: {trajectory: 0.1, goal: 0.1}
      ur10_shoulder_lift_joint: {trajectory: 0.1, goal: 0.1}
      ur10_elbow_joint: {trajectory: 0.1, goal: 0.1}
      ur10_wrist_1_joint: {trajectory: 0.1, goal: 0.1}
      ur10_wrist_2_joint: {trajectory: 0.1, goal: 0.1}
      ur10_wrist_3_joint: {trajectory: 0.1, goal: 0.1}
    gains:
      ur10_shoulder_pan_joint: {p: 100, d: 10, i: 1, i_clamp: 1}
      ur10_shoulder_lift_joint: {p: 100, d: 10, i: 1, i_clamp: 1}
      ur10_elbow_joint: {p: 100, d: 10, i: 1, i_clamp: 1}
      ur10_wrist_1_joint: {p: 100, d: 10, i: 1, i_clamp: 1}
      ur10_wrist_2_joint: {p: 100, d: 10, i: 1, i_clamp: 1}
      ur10_wrist_3_joint: {p: 100, d: 10, i: 1, i_clamp: 1}
    state_publish_rate: 50
    action_monitor_rate: 50
    stop_trajectory_duration: 0.6

```

## PACKAGE ros_controller_boilerplate

In the file sim_hw_interface.cpp, the joint dynamicla model is implemented under effort command mode.
```
void SimHWInterface::effortControlSimulation(ros::Duration &elapsed_time, const std::size_t joint_id)
{
  double iA = 10.0; // joint inertia at arm side
  double iM = 10.0; // joint inertia at motor side
  double dA = 2.0; // damping coefficient at arm side
  double dM = 2.0; // damping coefficient at motor side
  double sA = 2.0; // spring coefficient at arm side
  double sM = 2.0; // spring coefficient at motor side
  double fT = 5.0; // friction torque
  int gR = 20.0; //gear reduction ratio

  //calculate joint angle acceleration
  joint_acc_[joint_id] = (gR*joint_effort_command_[joint_id]-(gR+1)*fT)/((iA+iM)+(dA+dM)*elapsed_time.toSec()+0.5*(sA+sM)*pow(elapsed_time.toSec(), 2));

  //calculate joint angle position
  joint_position_[joint_id] += joint_velocity_[joint_id]*elapsed_time.toSec()+0.5*joint_acc_[joint_id]*pow(elapsed_time.toSec(), 2);

  //calculate joint angle velocity
  joint_velocity_[joint_id] += joint_acc_[joint_id]*elapsed_time.toSec();
}

```

