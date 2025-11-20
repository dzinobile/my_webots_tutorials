// Copyright 2025 Zinobile-Corp LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/**
 * @file WalkerNode.cpp
 * @brief Context class source file
 * @author Daniel Zinobile
 * @date 20-Nov-2025
 */

#include "WalkerNode.hpp"
#include "States.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>
#include "pluginlib/class_list_macros.hpp"

#define HALF_DISTANCE_BETWEEN_WHEELS 0.045
#define WHEEL_RADIUS 0.025
#define MAX_RANGE 0.5 // Sensor detection range

using namespace std::chrono_literals;

/**
 * @brief Computes and applies motor commands at each simulation step
 * 
 * Converts cmd_vel message into differential drive wheel velocities
 * and sends them to the Webots motor service
 */
void WalkerNode::step(){
  auto forward_speed = cmd_vel_msg_.linear.x;
  auto angular_speed = cmd_vel_msg_.angular.z;
  auto command_motor_left = 
    (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / 
    WHEEL_RADIUS;
  auto command_motor_right = 
    (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / 
    WHEEL_RADIUS;

  wb_motor_set_velocity(left_motor, command_motor_left);
  wb_motor_set_velocity(right_motor, command_motor_right);

  
}

/**
 * @brief Initializes robot hardware, ROS interfaces, and state machine
 * 
 *  - Retrieves Webots motor handles
 *  - Sets initial motor status
 *  - Creates publisher for cmd_vel
 *  - Creates subscribers for sensors
 *  - Initializes state machine to FORWARD state
 *  - Starts a timer to drive state machine updates
 * 
 * @param node Pointer to Webots driver node used for ROS interfaces
 * @param parameters Map containing plugin parameters (unused, leftover from modifying example file)
 */
void WalkerNode::init(
  webots_ros2_driver::WebotsNode *node,
  std::unordered_map<std::string, std::string> &parameters) {
    right_motor = wb_robot_get_device("right wheel motor");
    left_motor = wb_robot_get_device("left wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);

    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(right_motor, 0.0);

    curState = &FORWARD_State;
    publisher_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    
    left_sensor_sub_ = node->create_subscription<sensor_msgs::msg::Range>(
      "/left_sensor", 1, 
      std::bind(&WalkerNode::leftSensorCallback, this, 
      std::placeholders::_1));

    right_sensor_sub_ = node->create_subscription<sensor_msgs::msg::Range>(
      "/right_sensor", 1,
      std::bind(&WalkerNode::rightSensorCallback, this,
      std::placeholders::_1));

    timer_ = node->create_wall_timer(
      500ms, std::bind(&WalkerNode::timerCallback, this));
  }

/**
 * @brief Callback function for left distance sensor
 * @param msg Shared pointer to ROS Range message from left sensor
 */
void WalkerNode::leftSensorCallback(
  const sensor_msgs::msg::Range::SharedPtr msg) {
    left_sensor_value = msg->range;
  }

/**
 * @brief Callback function for right distance sensor
 * Determines if obstacle is detected based on sensor readings
 * @param msg Shared pointer to ROS Range message from right sensor
 */
void WalkerNode::rightSensorCallback(
  const sensor_msgs::msg::Range::SharedPtr msg) {
    right_sensor_value = msg->range;

    if(left_sensor_value < 0.9 * MAX_RANGE || 
        right_sensor_value < 0.9 * MAX_RANGE) {
          obstacle_detected_ = true;
        } else {
          obstacle_detected_ = false;
        }

  }

/**
 * @class WalkerNode::state_FORWARD
 * @brief State in which robot drives forward
 */
WalkerNode::state_FORWARD::state_FORWARD(){}

/**
 * @brief Publishes message to cmd_vel for moving forward
 * @param context Reference to the WalkerNode containing robot state
 */
void WalkerNode::state_FORWARD::update(WalkerNode &context) {
  context.cmd_vel_msg_.linear.x = 0.2;
  context.cmd_vel_msg_.angular.z = 0.0;
  context.publisher_->publish(context.cmd_vel_msg_);
}

/**
 * @brief Determines next state based on obstacle detection and previous state
 * If obstacle detected and previous direction was right, turns left
 * If obstacle detected and previous direction was left, turns right
 * If no obstacle detected, forward state
 * @param context The robot context
 * @return Pointer to the next state
 */
States* WalkerNode::state_FORWARD::transition(WalkerNode &context) {
  if (context.obstacle_detected_){
    if (context.prevDirection == "right"){
      return &context.TURNLEFT_State;
    } else {
      return &context.TURNRIGHT_State;
    }
  } else {
    return &context.FORWARD_State;
  }
}

/**
 * @class WalkerNode::state_TURNLEFT
 * @brief State in which robot turns left
 */
WalkerNode::state_TURNLEFT::state_TURNLEFT(){}

/**
 * @brief Publishes message to cmd_vel for turning left
 * @param context Reference to the WalkerNode containing robot state
 */
void WalkerNode::state_TURNLEFT::update(WalkerNode &context) {
  context.cmd_vel_msg_.linear.x = 0.0;
  context.cmd_vel_msg_.angular.z = 0.5;
  context.publisher_->publish(context.cmd_vel_msg_);
}

/**
 * @brief Determines next state based on obstacle detection
 * If obstacle detected, continue turning left
 * If no obstacle detected, move to forward state
 * @param context The robot context
 * @return Pointer to the next state
 */
States* WalkerNode::state_TURNLEFT::transition(WalkerNode &context) {
  context.prevDirection = "left";
  if (context.obstacle_detected_){
    return &context.TURNLEFT_State;
  } else {
    return &context.FORWARD_State;
  }
}

/**
 * @class WalkerNode::state_TURNRIGHT
 * @brief State in which the robot turns right
 */
WalkerNode::state_TURNRIGHT::state_TURNRIGHT(){}

/**
 * @brief Publishes message to cmd_vel for turning right
 * @param context Reference to the WalkerNode containing robot state
 */
void WalkerNode::state_TURNRIGHT::update(WalkerNode &context) {
  context.cmd_vel_msg_.linear.x = 0.0;
  context.cmd_vel_msg_.angular.z = -0.5;
  context.publisher_->publish(context.cmd_vel_msg_);
}

/**
 * @brief Determines next state based on obstacle detection
 * If obstacle detected, continues turning right
 * If no obstacle detected, moves to forward state
 * @param context The robot context
 * @return Pointer to the next state
 */
States* WalkerNode::state_TURNRIGHT::transition(WalkerNode &context) {
  context.prevDirection = "right";
  if (context.obstacle_detected_){
    return &context.TURNRIGHT_State;
  } else {
    return &context.FORWARD_State;
  }
  
}

/**
 * @brief Executes one full state machine cycle:
 *  - Execute current state's update() method 
 *  - Check transitions and switch to next state
 */
void WalkerNode::cmdVelCallback(){
  curState->update(*this);

  States *next = curState->transition(*this);
  curState = next;

}

/**
 * @brief Timer callback to trigger state machine updates
 */
void WalkerNode::timerCallback(){
  cmdVelCallback();
}

PLUGINLIB_EXPORT_CLASS(WalkerNode, 
  webots_ros2_driver::PluginInterface)
