#include <controller.h>

Controller::Controller() {
  
  taskIndex_ = -1;
  status_ = car_status::STOP;
  isGetQRCode_ = false;

  // 初始化默认参数
  // ANGLE_ADJUSTGAIN_GAIN = 0;
}

Controller::~Controller() {}

void Controller::init(ros::NodeHandle nh) {
  WriteLock lock(rwMutex_);
  // 设置当前小车状态
  switchCarStatus(car_status::STOP);

  nh.getParam("angle_adjust_gain", ANGLE_ADJUST_GAIN);
  std::cout<<"[CONTROLLER]  SET-PARAMS: angle_adjust_gain: \t"<< ANGLE_ADJUST_GAIN << std::endl;

  nh.getParam("max_twist", MAX_TWIST);
  std::cout<<"[CONTROLLER]  SET-PARAMS: max_twist: \t\t"<< MAX_TWIST << std::endl;

  nh.getParam("twist_step", TWIST_STEP);
  std::cout<<"[CONTROLLER]  SET-PARAMS: twist_step: \t\t"<< TWIST_STEP << std::endl;

  nh.getParam("twist_threshold", TWIST_THRESHOLD);
  std::cout<<"[CONTROLLER]  SET-PARAMS: twist_threshold: \t"<< TWIST_THRESHOLD << std::endl;

  nh.getParam("max_forward", MAX_FORWARD);
  std::cout<<"[CONTROLLER]  SET-PARAMS: max_forward: \t\t"<< MAX_FORWARD << std::endl;

  nh.getParam("forward_step", FORWARD_STEP);
  std::cout<<"[CONTROLLER]  SET-PARAMS: forward_step: \t"<< FORWARD_STEP << std::endl;

  nh.getParam("test_mode", TEST_MODE);
  std::cout<<"[CONTROLLER]  SET-PARAMS: test_mode: \t\t"<< TEST_MODE << std::endl;
  
  // 判断测试模式，若为测试模式，读取 路径参数
  if( TEST_MODE ) {
    std::string filePath;
    nh.getParam("testPath_file", filePath);
    std::cout<<"path: " << filePath << std::endl;
    std::ifstream infile(filePath);
    std::string s;
    while (getline(infile, s))
    {
      /* code */
      std::vector<std::string> checkPoint = split(s, " ");
      GlobalPosition point = {
        .x = atof(checkPoint[0].c_str()),
        .y = atof(checkPoint[1].c_str()),
      };
      task_.push_back(point);
      std::cout<<"file content: " << s << std::endl;
    }
    infile.close();
  }
}

double Controller::calculateGoalAngle(GlobalPosition cur, GlobalPosition goal) {
  double delt_x = goal.x - cur.x;
  double delt_y = goal.y - cur.y;
  double goal_angle = atan(delt_y / (delt_x + BAIS));
  goal_angle = goal.x < cur.x ? goal_angle + PI : goal_angle;
  // 修复负转角的出现
  return goal_angle < 0 ? goal_angle + 2*PI : goal_angle;
}

double Controller::calculateTwist(GlobalPosition cur_, GlobalPosition goal_) {
  double cur = cur_.angle, goal = goal_.angle;
  if( fabs(cur - goal) <= TWIST_THRESHOLD) return 0;
  float theta_n = goal - cur;
  theta_n = theta_n < 0 ? theta_n + 2*PI : theta_n;
  float theta_p = cur - goal;
  theta_p = theta_p < 0 ? theta_p + 2*PI : theta_p;
  if(theta_p < theta_n) return MAX_TWIST;
  return -MAX_TWIST;
}

geometry_msgs::Twist Controller::getNextTwist(geometry_msgs::Twist last, geometry_msgs::Twist goal) {
  geometry_msgs::Twist res;
  res.linear.x = last.linear.x;
  res.angular.z = last.angular.z;
  if(goal.angular.z > last.angular.z) {
    // 加速阶段
    res.angular.z = std::min(goal.angular.z, last.angular.z + TWIST_STEP);
  } else {
    res.angular.z = std::max(goal.angular.z, last.angular.z - TWIST_STEP);
  }
  // 角度调整结束
  if(res.angular.z == goal.angular.z) {
    if(goal.linear.x > last.linear.x) {
      res.linear.x = std::min(goal.linear.x, last.linear.x + FORWARD_STEP);
    } else {
      res.linear.x = std::max(goal.linear.x, last.linear.x - FORWARD_STEP);
    }
  }
  return res;
}

double Controller::calculateForward(GlobalPosition cur_, GlobalPosition goal_) {
  double distance = calculateDistance(cur_, goal_);
  std::cout<< distance <<std::endl;
  // 待返回量
  double forward = MAX_FORWARD;
  // 当且仅当以下连个条件满足时，减速并切换到下一状态
  if( distance < 0.5 && isGetQRCode_) {
    // 减速阶段
    switchQRCodeStatus(false);
    forward = 0;
    return forward;
  }
  return forward;
}

double Controller::calculateTwistWithRedundancy(GlobalPosition cur_, GlobalPosition goal_) {
  // 计算两点距离
  double distance = calculateDistance(cur_, goal_);
  if(distance <= 0.5 && isGetQRCode_) return 0;
  // 计算当前位置的角度冗余量
  double redundancy = atan(0.05 / (distance + BAIS));
  // 计算当前角度的目标转向
  double goal = calculateGoalAngle(cur_, goal_);
  // 排除异常情况
  double cur = cur_.angle;
  // 待返回结果
  float twist = 0;
  float diff = cur - goal;
  if( fabs(diff) > PI ) {
    if(cur > PI) cur = cur - 2 * PI;
    else goal = goal - 2 * PI;
    // 重新计算角度差
    diff = cur - goal;
  }
  // 正常判断方向，并计算参数
  if( diff > redundancy || (-diff) > redundancy) {
    // 角度过大，需要顺时针转向: diff 自带方向性
    twist = diff * ANGLE_ADJUST_GAIN;
    twist = std::min( diff * ANGLE_ADJUST_GAIN, MAX_TWIST * 0.25);
  }
  return twist;
}

void Controller::switchCarStatus(car_status status) {
  status_ = status;
}

void Controller::switchQRCodeStatus(bool status) {
  isGetQRCode_ = status;
}

void Controller::updateGoal() {
  // 小车必须在停止状态： 为缓加减速提供空间
  if(status_ != car_status::STOP) return;
  WriteLock lock(rwMutex_);
  taskIndex_ ++;
  // 检测到测试状态, 循环测试
  if(TEST_MODE) taskIndex_ = taskIndex_ % task_.size();
  GlobalPosition goal;
  goal.x = task_[taskIndex_].x;
  goal.y = task_[taskIndex_].y;
  goal.angle = calculateGoalAngle(loc_.getPosition(), goal);
  goal_ = goal;
  loc_.setGoal(goal_);
  // 切换小车状态
  switchCarStatus(car_status::TWIST);
  std::cout<< "[CONTROLLER] update Goal: [ \t" << goal_.x << ",\t"<<goal_.y<<",\t"<<goal_.angle<<"\t]"<<std::endl;
}

geometry_msgs::Twist Controller::getTwist() {
  WriteLock lock(rwMutex_);
  GlobalPosition cur = loc_.getPosition();
  geometry_msgs::Twist twist;
  switch (status_) {
  case car_status::STOP:
    // 小车处于停止状态，发送速度为0
    twist.linear.x = 0;
    twist.angular.z = 0;
    break;
  
  case car_status::TWIST: {
    // 小车目前进入转向调整阶段
    twist.linear.x = 0;
    // 计算理论旋转矢量
    double twist_z = calculateTwist(cur, goal_);
    // std::cout<<twist_z << " last: "<< last_twist_.angular.z<<std::endl;
    // double twist_z_abs = fabs(twist_z);
    // if(twist_z_abs >= fabs(last_twist_.angular.z)) {
    //   // std::cout<<"twist up "<< last_twist_.angular.z <<std::endl;
    //   twist_z_abs = std::min( MAX_TWIST, fabs(last_twist_.angular.z) + TWIST_STEP );
    // } else {
    //   twist_z_abs = std::max(0.0, fabs(last_twist_.angular.z) - TWIST_STEP);
    // }
    if(twist_z == 0) {
      switchCarStatus(car_status::FORWARD);
    }
    twist.angular.z = twist_z;
    // std::cout<<twist.angular.z<<std::endl;
    break;
  }
  
  case car_status::FORWARD: {
    // 小车进入直行阶段
    // 直行缓加速
    twist.linear.x = calculateForward(cur, goal_);
    // 计算微调角度
    twist.angular.z = calculateTwistWithRedundancy(cur, goal_);
    if( twist.linear.x == 0 ) {
      switchCarStatus(car_status::STOP);
    }
    break;
  }

  default:
    twist.linear.x = 0;
    twist.angular.z = 0;
    break;
  }
  // 记录当前发送的状态
  twist = getNextTwist(last_twist_, twist);
  last_twist_ = twist;
  std::cout<< "[CONTROLLER] Current Position: [ " << cur.x << ",\t"<< cur.y << ",\t" << cur.angle << " ]\tGoal: [ "
           << goal_.x << ",\t" << goal_.y << ",\t" << goal_.angle << " ]"
           << std::endl;
  std::cout<< "[CONTROLLER] Last Controll: \t forward: " << twist.linear.x << ",\tangle: "<<twist.angular.z<<std::endl;
  return twist;
}

