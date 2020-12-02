//ROS头文件
#include <ros/ros.h>
//包含自定义msg产生的头文件
// #include <topic_demo/gps.h>
//ROS标准msg头文件
#include <std_msgs/Float32.h>
// 获取多线程库
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#include <ros/callback_queue.h>
// vector
#include <vector>
// pgv: zhangjunwang
#include <reader/pos.h>
// mapf
// #include <mapf/AgentsTrajectory.h>
// odom 
#include <nav_msgs/Odometry.h>
// #include <geometry_msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
// eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/LU>

// odom 信息
typedef struct {
  float x;
  float y;
  Eigen::Quaterniond quater;
}Odom;

// 全局定位信息
typedef struct {
  float x;
  float y;
  float angle;

  // 上次步进时间
  float last_time;
}Position;
// 全局 task 信息
typedef struct {
  float x;
  float y;
  float angle;
}Goal;


typedef boost::shared_lock<boost::shared_mutex> ReadLock;
typedef boost::lock_guard<boost::shared_mutex> WriteLock;
#define PI       3.14159265358979323846   // pi
// 上次的 odom 信息
Odom last_odom_;
bool init = false;

// 创建共享数据
class Location
{
private:
  Position position_;
  boost::shared_mutex rwMutex_;
  std::vector<Goal> task;
public:
  Location(/* args */);
  Position getLocation() {
    ReadLock lock(rwMutex_);
    return position_;
  }
  Goal getCurGoal() {
    ReadLock lock(rwMutex_);
    // 此时没有具体的任务，直接返回虚假的目标
    Goal temp;
    temp.x = position_.x;
    temp.y = position_.y;
    temp.angle = position_.angle;
    // X轴前进 1m
    temp.x = 1;
    // X轴前进 1m
    // temp.y = 1;
    // 原地旋转90°
    // temp.angle = PI/2;
    return temp;
  }
  geometry_msgs::Twist getTwist() {
    // 创建并返回消息
    ReadLock lock(rwMutex_);
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.angular.z = 0;
    // 获取当前的坐标信息
    Position cur_position = position_;
    // 获取当前的目标信息
    Goal cur_goal = getCurGoal();
    // 通过距离和角度差值判断当前的状态
    
    if( abs(position_.x - cur_goal.x) < 0.3 && abs(position_.y - cur_goal.y) < 0.3 ) {
      // 目前在旋转状态
      if( abs(cur_goal.angle - position_.angle) > PI/5 ) {
        twist.angular.z = cur_goal.angle > position_.angle ? 0.6 : -0.6;
      } else {
        twist.angular.z = cur_goal.angle > position_.angle ? 0.3 : -0.3;
      }
    } else {
      // 目前在直行状态
      if( sqrt(pow(position_.x - cur_goal.x, 2) + pow(position_.y - cur_goal.y, 2)) > 0.5 ) {
        twist.linear.x = 0.6;
      } else {
        twist.linear.x = 0.3;
      }
    }
    std::cout << "liner = "<< twist.linear.x <<  "  angular : " << twist.angular.z << std::endl ;
    return twist;
  }
  void setLocation(float x, float y, float angle, float time) {
    // PGV赢更新，更新所有参数
    WriteLock lock(rwMutex_);
    // 更新当前位置信息
    position_.x = x;
    position_.y = y;
    position_.angle = angle;
    position_.last_time = time;
    // 此处不记录时间
    std::cout << "Recived PGV update: [ " << position_.x << ", " << position_.y << ", " << position_.angle  << "]" << std::endl;
    // ROS_INFO_STREAM("I heard: [ " << time << "] in thread [" << boost::this_thread::get_id() << "]");
    // 是否需要解锁?
  }

  // 差量更新当前位置
  void updateLocation(float x, float y, float angle, float time) {
    WriteLock lock(rwMutex_);
    position_.x += x;
    position_.y += y;
    position_.angle += angle;
    // 去除异常的角度状态
    if(position_.angle > 2 * PI) position_.angle = position_.angle - 2 * PI;
    else if(position_.angle < 0) position_.angle = position_.angle + 2 * PI;
    position_.last_time = time;
    std::cout << "Recived pose update: [ " << position_.x << ", " << position_.y << ", " << position_.angle  << "]" << std::endl;
  }
};

// 构造函数
Location::Location(/* args */)
{
  // 初始化参数
  position_.x = 0;
  position_.y = 0;
  position_.angle = 0;
  // 初始化时标记当前时间为负值
  position_.last_time = -1;

  // last_position_.x = 0;
  // last_position_.y = 0;
  // last_position_.angle = 0;
  // // 初始化时标记当前时间为负值
  // last_position_.last_time = -1;
}

// 创建消息队列
ros::CallbackQueue pgv_queue;
ros::CallbackQueue task_queue;
ros::CallbackQueue odom_queue;
Location loc;

// 旋转计算器
//计算旋转角
double calculateAngle(const Eigen::Vector3d &vectorBefore, const Eigen::Vector3d &vectorAfter)
{
    double ab, a1, b1, cosr;
    ab = vectorBefore.x()*vectorAfter.x() + vectorBefore.y()*vectorAfter.y() + vectorBefore.z()*vectorAfter.z();
    a1 = sqrt(vectorBefore.x()*vectorBefore.x() + vectorBefore.y()*vectorBefore.y() + vectorBefore.z()*vectorBefore.z());
    b1 = sqrt(vectorAfter.x()*vectorAfter.x() + vectorAfter.y()*vectorAfter.y() + vectorAfter.z()*vectorAfter.z());
    cosr = ab / a1 / b1;
    return (acos(cosr) * 180 / PI);
}
//计算旋转轴
inline Eigen::Vector3d calculateRotAxis(const Eigen::Vector3d &vectorBefore, const Eigen::Vector3d &vectorAfter)
{
    return Eigen::Vector3d(vectorBefore.y()*vectorAfter.z() - vectorBefore.z()*vectorAfter.y(), \
        vectorBefore.z()*vectorAfter.y() - vectorBefore.x()*vectorAfter.z(), \
        vectorBefore.x()*vectorAfter.y() - vectorBefore.y()*vectorAfter.x());
}
//计算旋转矩阵
void rotationMatrix(const Eigen::Vector3d &vectorBefore, const Eigen::Vector3d &vectorAfter, Eigen::Matrix3d &rotMatrix)
{
    Eigen::Vector3d vector = calculateRotAxis(vectorBefore, vectorAfter);
    double angle = calculateAngle(vectorBefore, vectorAfter);
    Eigen::AngleAxisd rotationVector(angle, vector.normalized());
    Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
    rotMatrix =  rotationVector.toRotationMatrix();//所求旋转矩阵
}


// 新建线程回调
void pgvCallbackThread() {
  ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());
  
  ros::NodeHandle n;
  while (n.ok())
  {
    // 执行所有回调
    pgv_queue.callAvailable(ros::WallDuration(0));
  }
}

// pgv 回调
void pgvCallback(const reader::pos::ConstPtr &msg) {
  // 直接覆盖当前的位置参数，并清空队列
  loc.setLocation(msg->x, msg->y, msg->angle / 180 * PI, msg->stamp);
}

// odom 处理线程
void odomCallbackThread() {
  ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());
  
  ros::NodeHandle n;
  while (n.ok())
  {
    // 执行所有回调
    odom_queue.callAvailable(ros::WallDuration(0));
  }
}

// double test = 0;

// odom 回调
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  // 提取四元数: 可能有问题
  geometry_msgs::Quaternion cur_orientation = msg->pose.pose.orientation;
  // 转换为 Eigen 四元数
  Eigen::Quaterniond cur_quaternion4(cur_orientation.w, cur_orientation.x, cur_orientation.y, cur_orientation.z);
  // 计算两者差值
  double cur_z = cur_quaternion4.toRotationMatrix().eulerAngles(2, 1, 0)[0];
  if(init == false) {
    last_odom_.x = msg->pose.pose.position.x;
    last_odom_.y = msg->pose.pose.position.y;
    last_odom_.quater = cur_quaternion4;
    init = true;
  }
  double last_z = last_odom_.quater.toRotationMatrix().eulerAngles(2, 1, 0)[0];
  // TODO: 确定此处的正反
  double diff_angle = cur_z - last_z;
  double x = msg->pose.pose.position.x - last_odom_.x;
  double y = msg->pose.pose.position.y - last_odom_.y;
  // 此处计算时: 若二者差距过大：代表越过 PI 界
  if(diff_angle < -2) {
    diff_angle = (cur_z + PI) - last_z;
  }else if(diff_angle > 2) {
    diff_angle = cur_z - (last_z + PI);
  }
  // test += (diff_angle / PI * 180);
  // std::cout << "sum = "<< test << last_z <<  "  diff z: " << cur_z << std::endl ;
  

  // 记录当前的 odom 信息
  last_odom_.x = msg->pose.pose.position.x;
  last_odom_.y = msg->pose.pose.position.y;
  last_odom_.quater = cur_quaternion4;

  // 更新位置信息
  loc.updateLocation(x, y, diff_angle, 1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  // PGV 订阅处理线程
  ros::SubscribeOptions opt_pgv = ros::SubscribeOptions::create<reader::pos>
    ("/position_info", 10, pgvCallback, ros::VoidPtr(), &pgv_queue);
  ros::Subscriber sub_pgv = n.subscribe(opt_pgv);
  boost::thread PGV_thread(pgvCallbackThread); 

  // Odom 订阅处理线程
  ros::SubscribeOptions opt_odom = ros::SubscribeOptions::create<nav_msgs::Odometry>
    ("/odom", 10, odomCallback, ros::VoidPtr(), &odom_queue);
  ros::Subscriber sub_odom = n.subscribe(opt_odom);
  boost::thread ODOM_thread(odomCallbackThread); 

  // 主进程添加消息发布逻辑
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);


  ROS_INFO_STREAM("Main thread id=" << boost::this_thread::get_id());
 
  // boost::thread PUB_thread(PubCallbackThread); 
  ros::Rate r(20);
  while (n.ok())
  {
    // 主程序循环：计算当前位置与目标点的距离，生成控制指令
    // 发布
    // ROS_INFO("Talker: GPS: x = %f, y = %f ",  msg.x ,msg.y);
    //以1Hz的频率发布msg
    // pub.publish();
    pub.publish(loc.getTwist());
    // std::cout<< 1<< std::endl;
    r.sleep();
  }
 
  // 等待对应的线程结束
  PGV_thread.join();
  ODOM_thread.join();
  return 0;
}

