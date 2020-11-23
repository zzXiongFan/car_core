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

typedef boost::shared_lock<boost::shared_mutex> ReadLock;
typedef boost::lock_guard<boost::shared_mutex> WriteLock;

// 全局定位信息
typedef struct {
  float x;
  float y;
  // float angle;
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
  void setLocation(float x, float y, bool isPGV = false) {
    WriteLock lock(rwMutex_);
    if(isPGV) {
      ROS_INFO_STREAM("recived PGV information" << "[" << x << "," << y << "]");
    }
    position_.x = x;
    position_.y = y;
  }
  // 更新全局目标点
  // void updateTask()
  // 获取下一目标点
  // Goal getNextGoal() {
  //   ReadLock lock(rwMutex_);
  //   return task.pop_back();
  // }
};

Location::Location(/* args */)
{
  position_.x = 0;
  position_.y = 0;
}

// 创建消息队列
ros::CallbackQueue g_queue;
Location loc;


// 主程序回调
void gpsMainQueue(const topic_demo::gps::ConstPtr &msg) {
  std_msgs::Float32 distance;
  distance.data = sqrt(pow(msg->x,2)+pow(msg->y,2));
  Position temp = loc.getLocation();
  ROS_INFO_STREAM("I heard: [ " << distance.data << "] in thread [" << boost::this_thread::get_id() << "] (Main thread)  " << temp.x);
}

// 子线程回调
void gpsCustomQueue(const topic_demo::gps::ConstPtr &msg) {
  std_msgs::Float32 distance;
  distance.data = sqrt(pow(msg->x,2)+pow(msg->y,2));
  loc.setLocation(msg->x, msg->y);
  ROS_INFO_STREAM("I heard: [ " << distance.data << "] in thread [" << boost::this_thread::get_id() << "]");
}

// 线程回调
void callbackThread() {
  ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());
  
  ros::NodeHandle n;
  while (n.ok())
  {
    // 执行所有回调
    g_queue.callAvailable(ros::WallDuration(0));
  }
  
}

// test 回调
void gpsCallback(const topic_demo::gps::ConstPtr &msg)
{  
    //计算离原点(0,0)的距离
    std_msgs::Float32 distance;
    distance.data = sqrt(pow(msg->x,2)+pow(msg->y,2));
    //float distance = sqrt(pow(msg->x,2)+pow(msg->y,2));
    ROS_INFO("Listener: Distance to origin = %f, state: %s",distance.data,msg->state.c_str());
}

// pgv 回调
void pgvCallback(const reader::pos::ConstPtr &msg) {
  loc.setLocation(msg->x, msg->y, true);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  
  // test sub
  ros::SubscribeOptions opt_test = ros::SubscribeOptions::create<topic_demo::gps>
    ("gps_info", 10, gpsCustomQueue, ros::VoidPtr(), &g_queue);

  // PGV sub
  ros::SubscribeOptions opt_pgv = ros::SubscribeOptions::create<reader::pos>
    ("/position_info", 10, pgvCallback, ros::VoidPtr(), &g_queue);
  
  ros::Subscriber sub = n.subscribe(opt_test);
  ros::Subscriber sub2 = n.subscribe(opt_pgv);
  
  // ros::Subscriber sub2 = n.subscribe("gps_info", 1, gpsMainQueue);

  boost::thread PGV_thread(callbackThread); 
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

