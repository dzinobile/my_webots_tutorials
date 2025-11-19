#pragma once

#include <memory>

#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "States.hpp"


class WalkerNode : public webots_ros2_driver::PluginInterface {
    public:

        void step() override;
        void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;
        
        // void ProcessInput(std::string input);
    
    protected:
        class state_FORWARD : public States {
            public:
                state_FORWARD();
                States* transition (WalkerNode &context);
                void update(WalkerNode &context) override;

        };
        
        class state_TURNLEFT : public States {
            public:
                state_TURNLEFT();
                States* transition (WalkerNode &context);
                void update(WalkerNode &context) override;

        };
            
        class state_TURNRIGHT : public States {
            public:
                state_TURNRIGHT();
                States* transition (WalkerNode &context);
                void update(WalkerNode &context) override;

        };

        States *curState;
        std::string prevDirection;
        state_FORWARD FORWARD_State;
        state_TURNLEFT TURNLEFT_State;
        state_TURNRIGHT TURNRIGHT_State;
    
    private:
        void leftSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg);
        void rightSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg);
        void cmdVelCallback();
        void timerCallback();
        
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr left_sensor_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr right_sensor_sub_;
        WbDeviceTag right_motor;
        WbDeviceTag left_motor;
        bool obstacle_detected_;
        double left_sensor_value{0.0};
        double right_sensor_value{0.0};
        rclcpp::TimerBase::SharedPtr timer_;
        geometry_msgs::msg::Twist cmd_vel_msg_;




};
