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
#include <tf2_ros/buffer.h>

#include <moveit_servo/servo.h>
#include <moveit_servo/lead.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>
#include <std_msgs/msg/bool.hpp>
#include <atomic>
#include <condition_variable>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.lead_demo");
std::atomic<bool> start_tracking(false);  // Shared variable to indicate tracking state
constexpr size_t MAX_QUEUE_SIZE = 10; 

// Shared flag to indicate waypoint completion
std::atomic<bool> waypoint_reached(false);
std::mutex cv_mutex;
std::condition_variable cv;

int ROBOT_JOINT_DIM = 7;

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

void targetJointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr& msg, std::mutex& queue_mutex,
                                         std::queue<trajectory_msgs::msg::JointTrajectory>& trajectory_queue)
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

// Callback function to update waypoint status
void waypointReachedCallback(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
    if (msg->data)
    {
        waypoint_reached = true;
        cv.notify_one(); // Notify waiting threads
    }
}

/**
 * Instantiate the pose tracking interface.
 * Send a pose slightly different from the starting pose
 * Then keep updating the target pose a little bit
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("lead_demo");

  // Trajectory queue
  std::queue<trajectory_msgs::msg::JointTrajectory> trajectory_queue;
  std::mutex queue_mutex;

  rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepAll())
                                .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE) // Ensure reliability
                                .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)   // Volatile durability
                                .history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);         
  auto mtc_traj_sub = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      // "/mtc_joint_trajectory", rclcpp::SystemDefaultsQoS(),
      "mtc_joint_trajectory", qos_profile,
      [&queue_mutex, &trajectory_queue](const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr& msg) { return targetJointTrajectoryCallback(msg, queue_mutex, trajectory_queue); });

  // Subscription for waypoint confirmation
  auto waypoint_status_sub = node->create_subscription<std_msgs::msg::Bool>(
      "waypoint_reached", rclcpp::SystemDefaultsQoS(), waypointReachedCallback);

  auto target_joint_pub = node->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("target_joint", rclcpp::SystemDefaultsQoS());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node);

  if (servo_parameters == nullptr)
  {
    RCLCPP_FATAL(LOGGER, "Could not get servo parameters!");
    exit(EXIT_FAILURE);
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
  moveit_servo::Lead tracker(node, servo_parameters, planning_scene_monitor);

  // Subscribe to servo status (and log it when it changes)
  StatusMonitor status_monitor(node, servo_parameters->status_topic);

  // resetTargetPose() can be used to clear the target pose and wait for a new one, e.g. when moving between multiple
  // waypoints
  // tracker.resetTargetPose();

  // Run the pose tracking in a new thread
  std::thread execute_joint_trajectory_thread([&tracker] {
    moveit_servo::LeadStatusCode lead_status = tracker.moveToJoint(0.1);
    RCLCPP_INFO_STREAM(LOGGER, "Joint trajectory execution exited with status: "
                                   << moveit_servo::LEAD_STATUS_CODE_MAP.at(lead_status));
  });

  trajectory_msgs::msg::JointTrajectory trajectory;
  rclcpp::WallRate loop_rate(10);
  bool processing_trajectory = false;
  bool set_joint_names = false;
  std::vector<std::string> lead_joint_names;
  std::vector<size_t> lead_joint_indices; // To map filtered joints back to their original indices

  while (rclcpp::ok()) {
    if (!processing_trajectory && !trajectory_queue.empty()) {
        // Fetch the next trajectory from the queue
        std::lock_guard<std::mutex> lock(queue_mutex);
        if (!trajectory_queue.empty()) {
            trajectory = trajectory_queue.front();
            trajectory_queue.pop();
            processing_trajectory = true; // Mark that we are processing a trajectory
            set_joint_names = true;       // Mark that we need to set joint names
            RCLCPP_INFO(LOGGER, "Processing new trajectory with %zu waypoints.", trajectory.points.size());
        }
    }

    if (processing_trajectory) {
        // Execute trajectory points one by one
        static size_t point_index = 0; // Keep track of the current point index

        // set Joint names
        if (set_joint_names) {
            lead_joint_names.clear();
            lead_joint_indices.clear();
            for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
                const auto& joint_name = trajectory.joint_names[i];
                if (joint_name.find("right_") != std::string::npos) { // Filter right-side joints
                    lead_joint_names.push_back(joint_name);
                    lead_joint_indices.push_back(i);
                }
            }

            if (lead_joint_names.empty()) {
                RCLCPP_WARN(LOGGER, "No lead joints found in trajectory. Skipping...");
                processing_trajectory = false;
                continue;
            }
            
            tracker.setJointNames(lead_joint_names);
            set_joint_names = false;
        }

        if (point_index < trajectory.points.size() - 1) {
            const auto& current_point = trajectory.points[point_index];
            const auto& next_point = trajectory.points[point_index + 1];
            RCLCPP_INFO(LOGGER, "next waypoint has position size %zu", next_point.positions.size());

            trajectory_msgs::msg::JointTrajectoryPoint target_point;
            // Extract only right-side joints for the target point
            target_point.positions.resize(lead_joint_indices.size());
            for (size_t i = 0; i < lead_joint_indices.size(); ++i) {
                target_point.positions[i] = next_point.positions[lead_joint_indices[i]];
            }

            if (!next_point.velocities.empty()) {
                target_point.velocities.resize(lead_joint_indices.size());
                for (size_t i = 0; i < lead_joint_indices.size(); ++i) {
                    target_point.velocities[i] = next_point.velocities[lead_joint_indices[i]];
                }
            }

            if (!next_point.accelerations.empty()) {
                target_point.accelerations.resize(lead_joint_indices.size());
                for (size_t i = 0; i < lead_joint_indices.size(); ++i) {
                    target_point.accelerations[i] = next_point.accelerations[lead_joint_indices[i]];
                }
            }

            if (!next_point.effort.empty()) {
                target_point.effort.resize(lead_joint_indices.size());
                for (size_t i = 0; i < lead_joint_indices.size(); ++i) {
                    target_point.effort[i] = next_point.effort[lead_joint_indices[i]];
                }
            }

            target_point.time_from_start = next_point.time_from_start;

            // Reset the waypoint_reached flag
            waypoint_reached = false;

            // tracker.executeTrajectorySegment(current_point, next_point, trajectory.joint_names);
            target_joint_pub->publish(target_point);
            RCLCPP_INFO(LOGGER, "Process target point %zu with size %zu", point_index, target_point.positions.size());
            if (target_point.positions.size() == ROBOT_JOINT_DIM) {
                RCLCPP_INFO(LOGGER, "Published next joint position: %f, %f, %f, %f, %f, %f, %f",
                        target_point.positions[0], target_point.positions[1], target_point.positions[2], 
                        target_point.positions[3], target_point.positions[4], target_point.positions[5], target_point.positions[6]);
            }
          
            // Wait for waypoint confirmation
            std::unique_lock<std::mutex> lock(cv_mutex);
            cv.wait(lock, [] { return waypoint_reached.load(); });

            point_index = point_index + 1; // Move to the next point
        } else {
            // Finished processing this trajectory
            RCLCPP_INFO(LOGGER, "Finished executing trajectory.");
            processing_trajectory = false; // Reset state
            point_index = 0;              // Reset the point index
        }
    } else {
        // Log a message if no trajectory is being processed
        auto clock = node->get_clock();
        RCLCPP_INFO_THROTTLE(LOGGER, *clock, 2000, "Waiting for trajectories...");
    }
        
    // // Add a small sleep to prevent busy-waiting
    // loop_rate.sleep();
}

  // Make sure the tracker is stopped and clean up
  execute_joint_trajectory_thread.join();

  // Kill executor thread before shutdown
  executor.cancel();
  executor_thread.join();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
