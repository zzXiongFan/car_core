//ROS头文件
#ifndef LOCATION_H
#define LOCATION_H
#include <ros/ros.h>
// 获取多线程库
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#include <ros/callback_queue.h>
// vector
#include <vector>
// 引入全局工具函数
#include <utils.h>

// 读写锁
typedef boost::shared_lock<boost::shared_mutex> ReadLock;
typedef boost::lock_guard<boost::shared_mutex> WriteLock;


class Location
{
private:
  // 当前坐标
  GlobalPosition position_;
  // 下一个目标点
  GlobalPosition goal_;
  // 读写锁
  boost::shared_mutex rwMutex_;
  // 到达判断阈值
  double ARRIVE_THRESHOLD;
  Location& operator=(const Location&);
public:
  // 单例模式
  static Location& getInstance() {
    static Location instance;
    return instance;
  };
public:
  // 传入 ros 句柄
  Location();
  // 获取当前的坐标信息
  GlobalPosition getPosition();
  // 设置 launch 参数
  void getLaunchParams(ros::NodeHandle nh);
  // 设置新的坐标信息
  void setPositon(GlobalPosition p);
  // 设置新的目标点
  void setGoal(GlobalPosition g);
  // 判断是否到达检查点
  bool isArrive();
  ~Location();
};

#endif
