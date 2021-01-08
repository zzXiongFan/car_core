#include <location.h>

Location::Location() {
  // 初始化参数
  position_ = {
    .x = 0,
    .y = 0,
    .angle = 0
  };

  goal_ = position_;

  ARRIVE_THRESHOLD = 0.5;
}



Location::~Location() {};

void Location::getLaunchParams(ros::NodeHandle nh) {
  // 从参数中初始化到达阈值
  nh.getParam("arrive_threshold", ARRIVE_THRESHOLD);
  std::cout<<"[LOCATION]  SET-PARAMS: arrive_threshold: \t"<< ARRIVE_THRESHOLD << std::endl;
}

GlobalPosition Location::getPosition() {
  ReadLock lock(rwMutex_);
  return position_;
}

void Location::setPositon(GlobalPosition p) {
  WriteLock lock(rwMutex_);
  position_ = p;
  // std::cout<< "[LOCATION] position update: [ " << p.x << ",\t"<< p.y << ",\t" << p.angle << "]" << std::endl;
  return;
}


void Location::setGoal(GlobalPosition g) {
  WriteLock lock(rwMutex_);
  goal_ = g;
  return;
}


bool Location::isArrive() {
  double distance = calculateDistance(position_, goal_);
  // double distance = 0.1;
  return distance <= ARRIVE_THRESHOLD;
}



