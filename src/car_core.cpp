#include <car_core.h>

// odom 信息
typedef struct {
  float x;
  float y;
  Eigen::Quaterniond quater;
}Odom;

Odom last_odom_;
bool odom_init = false;

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
  if( loc.isArrive() || !init ) {
    init = true;
    ROS_INFO_STREAM("arrived");
    // TODO: 参数覆盖
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
  geometry_msgs::Quaternion cur_orientation = msg->pose.pose.orientation;
  // 转换为 Eigen 四元数
  Eigen::Quaterniond cur_quaternion4(cur_orientation.w, cur_orientation.x, cur_orientation.y, cur_orientation.z);
  // 计算两者差值
  double cur_z = cur_quaternion4.toRotationMatrix().eulerAngles(2, 1, 0)[0];
  if(odom_init == false) {
    last_odom_.x = msg->pose.pose.position.x;
    last_odom_.y = msg->pose.pose.position.y;
    last_odom_.quater = cur_quaternion4;
    odom_init = true;
  }
  double last_z = last_odom_.quater.toRotationMatrix().eulerAngles(2, 1, 0)[0];
  // TODO: 确定此处的正反: 使用更优雅的方式
  double diff_angle = cur_z - last_z;
  // 此处计算时: 若二者差距过大：代表越过 PI 界
  if(diff_angle < -2) {
    diff_angle = (cur_z + PI) - last_z;
  }else if(diff_angle > 2) {
    diff_angle = cur_z - (last_z + PI);
  }

  GlobalPosition pos = {
    .x = msg->pose.pose.position.x,
    .y = msg->pose.pose.position.y,
    .angle = (loc.getPosition().angle + diff_angle),
    // .angle = cur_z
  };
  if(pos.angle > 2*PI) pos.angle = pos.angle - 2*PI;
  if(pos.angle < 0) pos.angle = pos.angle + 2*PI;
  last_odom_.quater = cur_quaternion4;
  loc.setPositon(pos);
  // std::cout<< "[ODOM] recived: [ \t" << pos.x << ",\t"<< pos.y << ",\t"<< pos.angle << "\t ]" <<std::endl;
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

