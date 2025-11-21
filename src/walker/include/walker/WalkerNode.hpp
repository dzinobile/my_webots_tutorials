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
 * @file WalkerNode.hpp
 * @brief Header file for State Interface class
 * @author Daniel Zinobile
 * @date 20-Nov-2025
 */
#pragma once

#include <memory>

#include "States.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

/**
 * @class WalkerNode
 * @brief Webots plugin for a custom FSM robot controller
 * Class inherits from Webots PluginInterface and allows Webots to
 * load and run the ROS_integrated behavior.
 * The node:
 *  - publishes velocity commands to cmd_vel
 *  - subscribes to messages from the sensors
 *  - Defines an FSM with 3 states
 */
class WalkerNode : public webots_ros2_driver::PluginInterface {
 public:
  /**
   * @brief Called every Webots simulation step
   * Converts the cmd_vel messages into Webots wheel velocities
   */
  void step() override;

  /**
   * @brief Initializes Webots devices, publishers, subscribers, and initial
   * State
   * @param node Pointer to the Webots Node
   * @param parameters Plugin parameters (unused)
   */
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;

 protected:
  /**
   * @class state_FORWARD
   * @brief FSM state where robot moves forward
   */
  class state_FORWARD : public States {
   public:
    /**
     * @brief Constructor for state_FORWARD class
     */
    state_FORWARD();

    /**
     * @brief Evaluates current conditions and returns next required state
     * Transitions to TURNLEFT_State if obstacle detected and previous turn was
     * right Transitions to TURNRIGHT_State if obstacle detected and previous
     * turn was left Remains in FORWARD_State if no obstacle detected
     * @param context Reference to the WalkerNode containing current state
     */
    States *transition(WalkerNode &context);

    /**
     * @brief Publish forward velocity command to cmd_vel
     * @param context Reference to the WalkerNode containing current state
     */
    void update(WalkerNode &context) override;
  };

  /**
   * @class state_TURNLEFT
   * @brief FSM state where robot turns left until no obstacle detected
   */
  class state_TURNLEFT : public States {
   public:
    /**
     * @brief Constructor for state_TURNLEFT class
     */
    state_TURNLEFT();

    /**
     * @brief Evaluates current conditions and returns next required state
     * Transitions to FORWARD_State if no obstacle detected
     * Remains in TURNLEFT_State if obstacle detected
     * @param context Reference to the WalkerNode containing current state
     */
    States *transition(WalkerNode &context);

    /**
     * @brief Publish turn left command to cmd_vel
     * @param context Reference to WalkerNode containing current state
     */
    void update(WalkerNode &context) override;
  };

  /**
   * @class state_TURNRIGHT
   * @brief FSM state where robot turns right until no obstacle detected
   */
  class state_TURNRIGHT : public States {
   public:
    /**
     * @brief Constructor for state_TURNRIGHT class
     */
    state_TURNRIGHT();

    /**
     * @brief Evaluates current conditions and returns next requried state
     * Transitions to FORWARD_State if no obstacle detected
     * Remains in TURNRIGHT_State if obstacle detected
     * @param context Reference to the WalkerNode containing current state
     */
    States *transition(WalkerNode &context);

    /**
     * @brief Publish turn right command to cmd_vel
     * @param context Reference to WalkerNode containing current state
     */
    void update(WalkerNode &context) override;
  };

  class state_STOP : public States {
    public:
    state_STOP();
    States *transition(WalkerNode &context);
    void update(WalkerNode &context) override;
  };

  /**
   * @brief Pointer to the current state.
   */
  States *curState;

  /**
   * @brief String containing previous turn direction
   */
  std::string prevDirection;

  state_FORWARD FORWARD_State;
  state_TURNLEFT TURNLEFT_State;
  state_TURNRIGHT TURNRIGHT_State;
  state_STOP STOP_State;

 private:
  /**
   * @brief Callback for left sensor messages
   * Stores mesage range value
   * @param msg Shared pointer to left sensor message
   */
  void leftSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg);

  /**
   * @brief Callback for right sensor messages
   * Stores message range value
   * Updates obstacle_detected boolean based on left and right sensor values
   * @param msg Shared pointer to right sensor message
   */
  void rightSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg);

  /**
   * @brief Callback to execute FSM update() and transition()
   */
  void cmdVelCallback();

  /**
   * @brief Timer callback to run FSM regularly
   */
  void timerCallback();

  // cmd_vel publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  // Sensor subscribers
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr left_sensor_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr right_sensor_sub_;

  // Webots motor device handles
  WbDeviceTag right_motor;
  WbDeviceTag left_motor;

  // Boolean for obstacle detected or not
  bool obstacle_detected_;

  // Last sensor readings
  double left_sensor_value{0.0};
  double right_sensor_value{0.0};

  // Timer for timer callback
  rclcpp::TimerBase::SharedPtr timer_;

  // Most recent twist command produced by FSM
  geometry_msgs::msg::Twist cmd_vel_msg_;

  // Sensor timeouts
  rclcpp::Time last_left_msg_time_;
  rclcpp::Time last_right_msg_time_;
  bool sensor_timeout_;

  webots_ros2_driver::WebotsNode* ros_node_;
};
