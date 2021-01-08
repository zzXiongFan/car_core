#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <location.h>
// vector
#include <vector>
#include <nav_msgs/Odometry.h>
// #include <geometry_msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <fstream>

// 枚举小车状态
enum car_status {
  STOP, FORWARD, TWIST
};

class Controller
{
private:
  // 内部 Location 类
  Location& loc_ = Location::getInstance();
  // 读写控制器
  boost::shared_mutex rwMutex_;
  // 全局任务列表
  std::vector<GlobalPosition> task_;
  // 当前任务
  GlobalPosition goal_;
  int taskIndex_;
  car_status status_;
  // 判断是否看到 二维码的标志
  bool isGetQRCode_;
  // 上次发送的命令
  geometry_msgs::Twist last_twist_;
  // 微调时的增益倍数
  double ANGLE_ADJUST_GAIN = 0;
  // 最大转向速度
  double MAX_TWIST = 0;
  // 转向变化步长
  double TWIST_STEP = 0;
  // 初次调整转向阈值
  double TWIST_THRESHOLD = 0.1;
  // 最大前向速度
  double MAX_FORWARD = 0;
  // 转向步长变化
  double FORWARD_STEP = 0;
  // 测试模式
  bool TEST_MODE = false;

  // 计算 cur 到 goal 的全局目标角度值
  double calculateGoalAngle(GlobalPosition cur, GlobalPosition goal);
  // 计算 角度控制矢量
  double calculateTwist(GlobalPosition cur_, GlobalPosition goal_);
  // 计算 前行控制矢量
  double calculateForward(GlobalPosition cur_, GlobalPosition goal_);
  // 计算 前行时方向微调偏差
  double calculateTwistWithRedundancy(GlobalPosition cur_, GlobalPosition goal_);
  // 计算缓变控制量
  geometry_msgs::Twist getNextTwist(geometry_msgs::Twist last, geometry_msgs::Twist goal);
public:
  Controller();
  ~Controller();
  // 从 params 读取参数
  void init(ros::NodeHandle nh);
  // 切换小车状态
  void switchCarStatus(car_status status);
  // 切换二维码状态
  void switchQRCodeStatus(bool status);
  // update Goal 更新目标点
  void updateGoal();
  // 获取控制参数
  geometry_msgs::Twist getTwist();
};




#endif
