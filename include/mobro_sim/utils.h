#ifndef UTILS_H
#define UTILS_H

/***********/
/* Include */
/***********/

/* ROS2 C++ */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

/* Libraries */
#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>
#include <tuple>
#include <urdf/model.h>

/* Messages */
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

/*********/
/* Using */
/*********/

/* Eigen */ 
using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::Vector2d;

/* Messages */
using nav_msgs::msg::Path;
using sensor_msgs::msg::JointState;
using std_msgs::msg::Float32MultiArray;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Twist;
using rcl_interfaces::msg::SetParametersResult;

/* Publishers */
using PubFloat32MultiArray = std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32MultiArray, std::allocator<void>>>;

/* Namespaces */
using namespace std;
using namespace std::chrono_literals;

#endif
