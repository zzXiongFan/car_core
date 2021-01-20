//ROS头文件
#include <ros/ros.h>
#include <std_msgs/Float32.h>
// 获取多线程库
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#include <ros/callback_queue.h>
// pgv: zhangjunwang
#include <reader/pos.h>
#include <nav_msgs/Odometry.h>
// #include <geometry_msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// TODO: 重构代码
#include <location.h>
#include <controller.h>

// PGV消息队列
ros::CallbackQueue pgv_queue;
ros::CallbackQueue task_queue;
ros::CallbackQueue odom_queue;
// Location2 loc;
Location& loc = Location::getInstance();
Controller controller;
