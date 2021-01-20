#include <car_core.h>

// 新建线程回调
void pgvCallbackThread() {
  ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());
  
  ros::NodeHandle n;
  while (n.ok())
  {
    pgv_queue.callAvailable(ros::WallDuration(0.1));
  }
}

// pgv 回调
void pgvCallback(const reader::pos::ConstPtr &msg) {
  // std::cout<< "[PGV] recived: [ \t" << msg->x << ",\t"<< msg->y << ",\t"<< msg->angle / 180 * PI << "\t ]" <<std::endl;
  GlobalPosition pos = {.x = msg->x, .y = msg->y, .angle = msg->angle / 180 * PI};
  loc.setPositon(pos);
  controller.switchQRCodeStatus(true);
  // 调用控制器查看是否到达节点
  // [2021.01.20]zzxiongfan: 修改此处逻辑，以odom 为准，定位到二维码上方再进行转向
  if( loc.isArrive() && init ) {
    ROS_INFO_STREAM("arrived");
    // TODO: 参数覆盖
    controller.updateGoal();
  }
  if(!init) {
    init = true;
    controller.updateGoal();
  }
}

// odom 处理线程
void odomCallbackThread() {
  ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());
  
  ros::NodeHandle n;
  while (n.ok())
  {
    // 执行所有回调: 回调等待周期统一为回调一个周期的长度
    odom_queue.callAvailable(ros::WallDuration(0.05));
  }
}

// odom 回调
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  // 提取旋转四元数: 可能有问题
  // 未就绪状态直接跳过，此时 ODOM 不可靠
  if(!init) return;
  geometry_msgs::Quaternion cur_orientation = msg->pose.pose.orientation;
  double cur_z = toEulerAngle(cur_orientation.x, cur_orientation.y, cur_orientation.z, cur_orientation.w).z;

  GlobalPosition pos = {
    .x = msg->pose.pose.position.x,
    .y = msg->pose.pose.position.y,
    .angle = cur_z
  };
  loc.setPositon(pos);
  if( loc.isArrive() ) {
    ROS_INFO_STREAM("arrived");
    // TODO: 参数覆盖
    controller.updateGoal();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "car_core");
  // 局部句柄
  ros::NodeHandle nh("~");
  // 全局句柄
  ros::NodeHandle n;
  
  // 读取启动参数
  loc.getLaunchParams(nh);
  controller.init(nh);

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
    // controller.getTwist();
    pub.publish(controller.getTwist());
    r.sleep();
  }
 
  // 等待对应的线程结束
  PGV_thread.join();
  ODOM_thread.join();
  return 0;
}

