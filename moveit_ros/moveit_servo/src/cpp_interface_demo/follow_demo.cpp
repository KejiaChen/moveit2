/*******************************************************************************
 *      Title     : pose_tracking_example.cpp
 *      Project   : moveit_servo
 *      Created   : 09/04/2020
 *      Author    : Adam Pettinger
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_ros/buffer.h>

#include <moveit_servo/servo.h>
#include <moveit_servo/follow.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>
#include <queue>
#include <std_msgs/msg/bool.hpp>
#include <atomic>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.follow_demo");
std::atomic<bool> start_tracking(false);  // Shared variable to indicate tracking state
constexpr size_t MAX_QUEUE_SIZE = 10; 
// Shared flag to indicate waypoint completion
std::atomic<bool> trajectory_done(false);
std::condition_variable trajectory_done_cv;  // To wait for trajectory execution
std::mutex trajectory_done_mutex;   

// Callback for start tracking signal
void startSignalCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  // start_tracking.store(msg->data);  // Update tracking state
  if (!start_tracking.load()){
    if (msg->data){
      start_tracking.store(true);
      RCLCPP_INFO(LOGGER, "Start signal received. Tracking started.");
    }
  }
}

// Class for monitoring status of moveit_servo
class StatusMonitor
{
public:
  StatusMonitor(const rclcpp::Node::SharedPtr& node, const std::string& topic)
  {
    sub_ = node->create_subscription<std_msgs::msg::Int8>(topic, rclcpp::SystemDefaultsQoS(),
                                                          [this](const std_msgs::msg::Int8::ConstSharedPtr& msg) {
                                                            return statusCB(msg);
                                                          });
  }

private:
  void statusCB(const std_msgs::msg::Int8::ConstSharedPtr& msg)
  {
    moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      RCLCPP_INFO_STREAM(LOGGER, "Servo status: " << status_str);
    }
  }

  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_;
};

void targetJointTrajectoryRepublishCallback(const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr& msg,
                                            rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr& pub,
                                            double factor)
{   
    RCLCPP_INFO(LOGGER, "Received new trajectory with %zu waypoints.", msg->points.size());

    /* Split only follow arm joints from the trajectory msg*/
    // get desired joint names
    std::vector<std::string> follow_joint_names;
    std::vector<size_t> follow_joint_indices; // To map filtered joints back to their original indices

    for (size_t i = 0; i < msg->joint_names.size(); ++i) {
        const auto& joint_name = msg->joint_names[i];
        if (joint_name.find("left_") != std::string::npos) { // Filter left-side joints
            // Filter out finger joints
            if (joint_name.find("finger") != std::string::npos) {
                RCLCPP_INFO(LOGGER, "Skipping finger trajectory");
                return;
            }
            follow_joint_names.push_back(joint_name);
            follow_joint_indices.push_back(i);
        }
    }

    if (follow_joint_names.empty()) {
        RCLCPP_WARN(LOGGER, "No follow arm joints found in trajectory. Skipping...");
        return;
    }else{
        RCLCPP_INFO(LOGGER, "Joint names: ");
        for (const auto& name : follow_joint_names)
        {
            RCLCPP_INFO_STREAM(LOGGER, name << " ");
        }
        if (!start_tracking.load()){
          start_tracking.store(true);
          RCLCPP_INFO(LOGGER, "Received valid trajectory. Start tracking.");
        }
    }

    // Construct msg with only follow arm joints
    trajectory_msgs::msg::JointTrajectory new_msg;
    new_msg.header = msg->header;
    new_msg.joint_names = follow_joint_names;

    for (const auto& point : msg->points) {
        trajectory_msgs::msg::JointTrajectoryPoint follow_point;

        // Filter positions
        for (const auto& index : follow_joint_indices) {
            follow_point.positions.push_back(point.positions[index]);
            if (!point.velocities.empty()) {
                follow_point.velocities.push_back(point.velocities[index] * factor);
            }
            if (!point.accelerations.empty()) {
                follow_point.accelerations.push_back(point.accelerations[index]);
            }
            if (!point.effort.empty()) {
                follow_point.effort.push_back(point.effort[index]);
            }
        }

        follow_point.time_from_start = point.time_from_start;
        new_msg.points.push_back(follow_point);
    }

    pub->publish(new_msg);
    RCLCPP_INFO(LOGGER, "Republished new trajectory with %zu waypoints.", new_msg.points.size());
}

void targetJointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr& msg, 
                                  std::mutex& queue_mutex,
                                  std::queue<trajectory_msgs::msg::JointTrajectory>& trajectory_queue
                                  )
{
    std::lock_guard<std::mutex> lock(queue_mutex);
    if (trajectory_queue.size() >= MAX_QUEUE_SIZE)
    {
        RCLCPP_WARN(LOGGER, "Trajectory queue is full. Dropping oldest trajectory.");
        trajectory_queue.pop();  // Remove oldest trajectory
    }
    trajectory_queue.push(*msg);
    RCLCPP_INFO(LOGGER, "Received new trajectory with %zu waypoints.", msg->points.size());
}

void sendTrajectoryToController(const trajectory_msgs::msg::JointTrajectory& trajectory, 
                                rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr& trajectory_client) {
    auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
    goal_msg.trajectory = trajectory;

    if (!trajectory_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(LOGGER, "Action server not available");
        {
            std::lock_guard<std::mutex> lock(trajectory_done_mutex);
            trajectory_done = false;  // Reset processing state
        }
        trajectory_done_cv.notify_one();  // Notify waiting thread
        return;
    }

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.result_callback = [](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(LOGGER, "Trajectory execution finished successfully");
        } else {
            RCLCPP_ERROR(LOGGER, "Trajectory execution failed or was canceled");
        }

        // Signal the condition_variable to unblock the main loop
        {
            std::lock_guard<std::mutex> lock(trajectory_done_mutex);
            trajectory_done = true;
        }
        trajectory_done_cv.notify_one();
    };

    trajectory_client->async_send_goal(goal_msg, send_goal_options);

    RCLCPP_INFO(LOGGER, "Trajectory sent to controller with %zu waypoints.", trajectory.points.size());
}

/**
 * Instantiate the pose tracking interface.
 * Send a pose slightly different from the starting pose
 * Then keep updating the target pose a little bit
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("follow_demo");
  // Add a subscription for the start signal
  auto start_signal_sub = node->create_subscription<std_msgs::msg::Bool>("/start_tracking", rclcpp::SystemDefaultsQoS(), startSignalCallback);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node);

  if (servo_parameters == nullptr)
  {
    RCLCPP_FATAL(LOGGER, "Could not get servo parameters!");
    exit(EXIT_FAILURE);
  }

  // Trajectory queue
  std::queue<trajectory_msgs::msg::JointTrajectory> trajectory_queue;
  std::mutex queue_mutex;
   
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr mtc_traj_sub;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_client;

  if (servo_parameters->only_republish){
    RCLCPP_INFO(LOGGER, "Only republishing the trajectory");

    trajectory_outgoing_cmd_pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/left_arm_controller/joint_trajectory", rclcpp::SystemDefaultsQoS());
    trajectory_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
			node, "/left_arm_controller/follow_joint_trajectory");

    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepAll())
                                .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE) // Ensure reliability
                                .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)   // Volatile durability
                                .history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);
    mtc_traj_sub = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
          // "/mtc_joint_trajectory", rclcpp::SystemDefaultsQoS(),
          "mtc_joint_trajectory", qos_profile,
          [&queue_mutex, &trajectory_queue](const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr& msg) { return targetJointTrajectoryCallback(msg, queue_mutex, trajectory_queue); });

    // mtc_traj_sub = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    //     "/mtc_joint_trajectory", rclcpp::SystemDefaultsQoS(),
    //     //   "mtc_joint_trajectory", qos_profile,
    //     [&trajectory_outgoing_cmd_pub](const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr& msg) { return targetJointTrajectoryRepublishCallback(msg, trajectory_outgoing_cmd_pub, 1.0); });

  } else {
    RCLCPP_INFO(LOGGER, "Servoing the trajectory");
  }

  // Load the planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene_monitor->providePlanningSceneService();
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor(servo_parameters->joint_topic);
  planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  // Wait for Planning Scene Monitor to setup
  if (!planning_scene_monitor->waitForCurrentRobotState(node->now(), 5.0 /* seconds */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Error waiting for current robot state in PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  // TF listener
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // Create the pose tracker
  moveit_servo::Follow tracker(node, servo_parameters, planning_scene_monitor);

  // Make a publisher for sending pose commands
  auto target_pose_pub =
      node->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", rclcpp::SystemDefaultsQoS());

  // Subscribe to servo status (and log it when it changes)
  StatusMonitor status_monitor(node, servo_parameters->status_topic);

  Eigen::Vector3d lin_tol{ 0.001, 0.001, 0.001 };
  double rot_tol = 0.01;

  // // Get the current EE transform
  geometry_msgs::msg::TransformStamped current_ee_tf;
  tracker.getCommandFrameTransform(current_ee_tf);

  // Convert it to a Pose
  geometry_msgs::msg::PoseStamped target_pose;

  if(!servo_parameters->only_republish){
  target_pose.header.frame_id = current_ee_tf.header.frame_id;
  target_pose.pose.position.x = current_ee_tf.transform.translation.x;
  target_pose.pose.position.y = current_ee_tf.transform.translation.y;
  target_pose.pose.position.z = current_ee_tf.transform.translation.z;
  target_pose.pose.orientation = current_ee_tf.transform.rotation;

  // Modify it a little bit
  target_pose.pose.position.x += 0.1;

  // resetTargetPose() can be used to clear the target pose and wait for a new one, e.g. when moving between multiple
  // waypoints
  tracker.resetTargetPose();

  // Publish target pose
  target_pose.header.stamp = node->now();
  target_pose_pub->publish(target_pose);
  }

  // Run the pose tracking in a new thread
  std::thread move_to_pose_thread([&tracker, &lin_tol, &rot_tol] {
    moveit_servo::FollowStatusCode follow_status =
        tracker.moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */);
    RCLCPP_INFO_STREAM(LOGGER, "Pose tracker exited with status: "
                                  << moveit_servo::FOLLOW_STATUS_CODE_MAP.at(follow_status));
  });

  rclcpp::WallRate loop_rate(50);

  // for (size_t i = 0; i < 500; ++i)
  // {
  //   // Modify the pose target a little bit each cycle
  //   // This is a dynamic pose target
  //   target_pose.pose.position.z += 0.0004;
  //   target_pose.header.stamp = node->now();
  //   target_pose_pub->publish(target_pose);

  //   loop_rate.sleep();
  // }

  geometry_msgs::msg::TransformStamped transform_stamped;
  trajectory_msgs::msg::JointTrajectory trajectory;
  bool processing_trajectory = false;
  std::vector<std::string> follow_joint_names;
  std::vector<size_t> follow_joint_indices; // To map filtered joints back to their original indices


  while ((rclcpp::ok)) {
    if (servo_parameters->only_republish){
      if (!processing_trajectory && !trajectory_queue.empty()) {
        // Fetch the next trajectory from the queue
        std::lock_guard<std::mutex> lock(queue_mutex);
        if (!trajectory_queue.empty()) {
            trajectory = trajectory_queue.front();
            trajectory_queue.pop();
            processing_trajectory = true; // Mark that we are processing a trajectory
            RCLCPP_INFO(LOGGER, "Processing new trajectory with %zu waypoints.", trajectory.points.size());
        }
      }
      if (processing_trajectory) {
        /* Split only follow arm joints from the trajectory msg*/
        // get desired joint names
        std::vector<std::string> follow_joint_names;
        std::vector<size_t> follow_joint_indices; // To map filtered joints back to their original indices

        for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
            const auto& joint_name = trajectory.joint_names[i];
            if (joint_name.find("left_") != std::string::npos) { // Filter left-side joints
                // Filter out finger joints
                if (joint_name.find("finger") != std::string::npos) {
                    RCLCPP_INFO(LOGGER, "Skipping finger trajectory");
                    processing_trajectory = false;
                    continue;
                }
                follow_joint_names.push_back(joint_name);
                follow_joint_indices.push_back(i);
            }
        }

        if (follow_joint_names.empty()) {
            // RCLCPP_WARN(LOGGER, "No follow arm joints found in trajectory. Skipping...");
            processing_trajectory = false;
            continue;
        }else{
            RCLCPP_INFO(LOGGER, "Joint names: ");
            for (const auto& name : follow_joint_names)
            {
                RCLCPP_INFO_STREAM(LOGGER, name << " ");
            }
            if (!start_tracking.load()){
              start_tracking.store(true);
              RCLCPP_INFO(LOGGER, "Received valid trajectory. Start tracking.");
            }
        }

        // Construct msg with only follow arm joints
        trajectory_msgs::msg::JointTrajectory new_msg;
        new_msg.header = trajectory.header;
        new_msg.joint_names = follow_joint_names;

        for (const auto& point : trajectory.points) {
            trajectory_msgs::msg::JointTrajectoryPoint follow_point;

            // Filter positions
            for (const auto& index : follow_joint_indices) {
                follow_point.positions.push_back(point.positions[index]);
                if (!point.velocities.empty()) {
                    follow_point.velocities.push_back(point.velocities[index]);
                }
                if (!point.accelerations.empty()) {
                    follow_point.accelerations.push_back(point.accelerations[index]);
                }
                if (!point.effort.empty()) {
                    follow_point.effort.push_back(point.effort[index]);
                }
            }

            follow_point.time_from_start = point.time_from_start;
            new_msg.points.push_back(follow_point);
        }

        /*Republish*/
        // trajectory_outgoing_cmd_pub->publish(new_msg);
        // RCLCPP_INFO(LOGGER, "Republished new trajectory with %zu waypoints.", new_msg.points.size());

        // Wait for trajectory execution
        sendTrajectoryToController(new_msg, trajectory_client);

        // Wait for trajectory execution to complete
        std::unique_lock<std::mutex> lock(trajectory_done_mutex);
        trajectory_done_cv.wait(lock, []{ return trajectory_done.load(); });

        processing_trajectory = false;
      }else{
        auto clock = node->get_clock();
        RCLCPP_INFO_THROTTLE(LOGGER, *clock, 5000, "Waiting for trajectories...");
      }
    } else{
    if (start_tracking.load()){ // Servoing and start signal received
      try {
        // Get the transform from master's EE to follower's base
        // transform_stamped = tf_buffer.lookupTransform(servo_parameters->planning_frame, servo_parameters->leading_ee_frame, tf2::TimePointZero);
        // // Target in follower's base frame
        // target_pose.header.frame_id = transform_stamped.header.frame_id;
        // target_pose.pose.position.x = transform_stamped.transform.translation.x;
        // target_pose.pose.position.y = transform_stamped.transform.translation.y - 0.3;
        // target_pose.pose.position.z = transform_stamped.transform.translation.z;
        // target_pose.pose.orientation = transform_stamped.transform.rotation;

        // Get the target in master's EE frame
        target_pose.header.frame_id = servo_parameters->leading_ee_frame;
        target_pose.pose.position.x = -0.15;
        target_pose.pose.position.y = 0.0;
        target_pose.pose.position.z = 0.0;
        target_pose.pose.orientation.x = 0.0;
        target_pose.pose.orientation.y = 0.0;  // 0.3826834
        target_pose.pose.orientation.z = 0.0;
        target_pose.pose.orientation.w = 1.0; // 0.9238795

        // Publish target pose
        target_pose.header.stamp = node->now();
        target_pose_pub->publish(target_pose);
        // RCLCPP_INFO_STREAM(LOGGER, "Published target pose: " << target_pose.pose.position.x << ", " << target_pose.pose.position.y << ", " << target_pose.pose.position.z);
      }
      catch (tf2::TransformException &ex) {
        RCLCPP_ERROR_STREAM(LOGGER, "Could not get transform: " << ex.what());
      }
    }
    else
    {
      auto clock = node->get_clock();
      RCLCPP_INFO_THROTTLE(LOGGER, *clock, 5000, "Waiting for start signal...");
    }
    }

    // Add a small sleep to prevent busy-waiting
    loop_rate.sleep();
  }

  // Make sure the tracker is stopped and clean up
  move_to_pose_thread.join();

  // Kill executor thread before shutdown
  executor.cancel();
  executor_thread.join();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
