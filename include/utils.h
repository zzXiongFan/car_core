#ifndef UTILS_H
#define UTILS_H


#include <stdio.h>
#include <math.h>
#include <vector>
#include <iostream>
#include <string.h>
// #include <algorithm>
#define PI       3.14159265358979323846   // pi
#define BAIS     0.00000000000000000001   // 除数偏差

// 全局定位信息
typedef struct {
  double x;
  double y;
  double angle;
} GlobalPosition;

inline float calculateDistance(GlobalPosition p1, GlobalPosition p2) {
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

// 定义欧拉角结构
typedef struct {
	double x;
	double y;
	double z;
} EulerAngle;

// 定义四元数
typedef struct {
	double x;
	double y;
	double z;
	double w;
} Quaterniond;


inline std::vector<std::string> split(const std::string& str, const std::string& delim) {
	std::vector<std::string> res;
	if("" == str) return res;
	//先将要切割的字符串从string类型转换为char*类型
	char * strs = new char[str.length() + 1] ; //不要忘了
	strcpy(strs, str.c_str()); 
 
	char * d = new char[delim.length() + 1];
	strcpy(d, delim.c_str());
 
	char *p = strtok(strs, d);
	while(p) {
		std::string s = p; //分割得到的字符串转换为string类型
		res.push_back(s); //存入结果数组
		p = strtok(NULL, d);
	}
 
	return res;
}

inline double min(double a, double b) {
	return a >= b ? b:a;
}

inline double max(double a, double b) {
	return a >= b ? a:b;
}

// 四元数转欧拉角
inline EulerAngle toEulerAngle(const double x, const double y, const double z, const double w) {
	// roll (x-axis rotation)
    double sinr_cosp = +2.0 * (w * x + y * z);
    double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
    double roll = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
    double sinp = +2.0 * (w * y - z * x);
		double pitch = 0;
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

// yaw (z-axis rotation)
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    double yaw = atan2(siny_cosp, cosy_cosp);
//    return yaw;
		EulerAngle res = {
			.x = roll < 0 ? roll + 2 * PI : roll,
			.y = pitch < 0 ? pitch + 2* PI : pitch,
			.z = yaw < 0 ? yaw + 2* PI : yaw
		};
		return res;
}

// 欧拉角转四元数: 输入角度值
inline Quaterniond toQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    //Degree to radius:
    yaw = yaw * M_PI / 180;
    pitch = pitch * M_PI / 180;
    roll = roll * M_PI / 180;


    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaterniond q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
    return q;
}

#endif