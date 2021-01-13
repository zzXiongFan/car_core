#include <utils.h>
#include <ros/ros.h>
using namespace std;

int main() {
  // 测试欧拉角与四元数的转换
  Quaterniond q1 = toQuaternion(0, 0, 0);
  EulerAngle e1 = toEulerAngle(q1.x, q1.y, q1.z, q1.w);
  cout<< "yaw: 0, ToEuler: x: "<< e1.x<< "  y: "<<e1.y<< "  z: "<<e1.z<<endl;

  Quaterniond q2 = toQuaternion(90, 0, 0);
  EulerAngle e2 = toEulerAngle(q2.x, q2.y, q2.z, q2.w);
  cout<< "yaw: 90, ToEuler: x: "<< e2.x<< "  y: "<<e2.y<< "  z: "<<e2.z<<endl;

  Quaterniond q3 = toQuaternion(179, 0, 0);
  EulerAngle e3 = toEulerAngle(q3.x, q3.y, q3.z, q3.w);
  cout<< "yaw: 179, ToEuler: x: "<< e3.x<< "  y: "<<e3.y<< "  z: "<<e3.z<<endl;

  Quaterniond q4 = toQuaternion(181, 0, 0);
  EulerAngle e4 = toEulerAngle(q4.x, q4.y, q4.z, q4.w);
  cout<< "yaw: 181, ToEuler: x: "<< e4.x<< "  y: "<<e4.y<< "  z: "<<e4.z<<endl;

  Quaterniond q5 = toQuaternion(361, 0, 0);
  EulerAngle e5 = toEulerAngle(q5.x, q5.y, q5.z, q5.w);
  cout<< "yaw: 359, ToEuler: x: "<< e5.x<< "  y: "<<e5.y<< "  z: "<<e5.z<<endl;
  return 0;
}