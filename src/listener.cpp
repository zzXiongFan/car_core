//ROS头文件
#include <ros/ros.h>
//包含自定义msg产生的头文件
#include <topic_demo/gps.h>
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
#include <mapf/AgentsTrajectory.h>
// odom 
// #include <nav_msgs/Odometry.h>

typedef boost::shared_lock<boost::shared_mutex> ReadLock;
typedef boost::lock_guard<boost::shared_mutex> WriteLock;

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
  void setLocation(float x, float y, float angle, float time) {
    // 更新所有参数
    WriteLock lock(rwMutex_);
    position_.x = x;
    position_.y = y;
    position_.angle = angle;
    position_.last_time = time;
    ROS_INFO_STREAM("I heard: [ " << time << "] in thread [" << boost::this_thread::get_id() << "]");
    // 是否需要解锁?
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
}

// 创建消息队列
ros::CallbackQueue pgv_queue;
ros::CallbackQueue serial_queue;
ros::CallbackQueue task_queue;
// ros::CallbackQueue odom_queue;
Location loc;


// 主程序回调
void gpsMainQueue(const topic_demo::gps::ConstPtr &msg) {
  std_msgs::Float32 distance;
  distance.data = sqrt(pow(msg->x,2)+pow(msg->y,2));
  Position temp = loc.getLocation();
  ROS_INFO_STREAM("I heard: [ " << distance.data << "] in thread [" << boost::this_thread::get_id() << "] (Main thread)  " << temp.x);
}

// 子线程回调
// void gpsCustomQueue(const topic_demo::gps::ConstPtr &msg) {
//   std_msgs::Float32 distance;
//   distance.data = sqrt(pow(msg->x,2)+pow(msg->y,2));
//   loc.setLocation(msg->x, msg->y);
//   ROS_INFO_STREAM("I heard: [ " << distance.data << "] in thread [" << boost::this_thread::get_id() << "]");
// }

// test 回调
// void gpsCallback(const topic_demo::gps::ConstPtr &msg)
// {  
//     //计算离原点(0,0)的距离
//     std_msgs::Float32 distance;
//     distance.data = sqrt(pow(msg->x,2)+pow(msg->y,2));
//     //float distance = sqrt(pow(msg->x,2)+pow(msg->y,2));
//     ROS_INFO("Listener: Distance to origin = %f, state: %s",distance.data,msg->state.c_str());
// }

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
  loc.setLocation(msg->x, msg->y, msg->angle, msg->stamp);
  // 清空 serial_queue 队列
  serial_queue.clear();
  ROS_INFO_STREAM("Get position from class: [ " << loc.getLocation().last_time);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  
  // // test sub
  // ros::SubscribeOptions opt_test = ros::SubscribeOptions::create<topic_demo::gps>
  //   ("gps_info", 10, gpsCustomQueue, ros::VoidPtr(), &pgv_queue);

  // PGV 订阅处理线程
  ros::SubscribeOptions opt_pgv = ros::SubscribeOptions::create<reader::pos>
    ("/position_info", 10, pgvCallback, ros::VoidPtr(), &pgv_queue);
  ros::Subscriber sub_pgv = n.subscribe(opt_pgv);
  boost::thread PGV_thread(pgvCallbackThread); 

  // 调度信息订阅: 主线程


  ROS_INFO_STREAM("Main thread id=" << boost::this_thread::get_id());
 
  ros::Rate r(20);
  while (n.ok())
  {
    // 主程序循环：计算当前位置与目标点的距离，生成控制指令
    r.sleep(); 
  }
 
  // 等待对应的线程结束
  PGV_thread.join();
 
  return 0;
}

