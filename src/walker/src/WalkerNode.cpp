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
#define MAX_RANGE 0.15
using namespace std::chrono_literals;
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

// void WalkerNode::ProcessInput(std::string input) {
//   prevDirection = input;
//   curState = curState->transition (*this);
// }

void WalkerNode::leftSensorCallback(
  const sensor_msgs::msg::Range::SharedPtr msg) {
    left_sensor_value = msg->range;
  }

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

WalkerNode::state_FORWARD::state_FORWARD(){}
void WalkerNode::state_FORWARD::update(WalkerNode &context) {
  context.cmd_vel_msg_.linear.x = 0.2;
  context.cmd_vel_msg_.angular.z = 0.0;
  context.publisher_->publish(context.cmd_vel_msg_);
}
States* WalkerNode::state_FORWARD::transition(WalkerNode &context) {
  if (context.prevDirection == "right"){
    return &context.TURNLEFT_State;
  } else {
    return &context.TURNRIGHT_State;
  }
}

WalkerNode::state_TURNLEFT::state_TURNLEFT(){}
void WalkerNode::state_TURNLEFT::update(WalkerNode &context) {
  context.cmd_vel_msg_.linear.x = 0.0;
  context.cmd_vel_msg_.angular.z = 0.5;
  context.publisher_->publish(context.cmd_vel_msg_);
}
States* WalkerNode::state_TURNLEFT::transition(WalkerNode &context) {
  context.prevDirection = "left";
  return &context.FORWARD_State;
}

WalkerNode::state_TURNRIGHT::state_TURNRIGHT(){}
void WalkerNode::state_TURNRIGHT::update(WalkerNode &context) {
  context.cmd_vel_msg_.linear.x = 0.0;
  context.cmd_vel_msg_.angular.z = -0.5;
  context.publisher_->publish(context.cmd_vel_msg_);
}
States* WalkerNode::state_TURNRIGHT::transition(WalkerNode &context) {
  context.prevDirection = "right";
  return &context.FORWARD_State;
}


void WalkerNode::cmdVelCallback(){
  curState->update(*this);

  States *next = curState->transition(*this);
  curState = next;

}

void WalkerNode::timerCallback(){
  cmdVelCallback();
}

PLUGINLIB_EXPORT_CLASS(WalkerNode, 
  webots_ros2_driver::PluginInterface)
