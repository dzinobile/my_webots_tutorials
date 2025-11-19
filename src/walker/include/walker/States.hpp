#pragma once

#include <memory>

#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "WalkerNode.hpp"

class States {
    public:
        States();
        virtual ~States();

        virtual void update(WalkerNode &context) = 0;

        virtual States* transition(WalkerNode &context) = 0;

};