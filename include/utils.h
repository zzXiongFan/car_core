#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <vector>
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

#endif