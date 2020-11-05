#!/usr/bin/env python
#coding=utf-8
# 引入ros 核心api
import rospy
# 导入自定义的数据类型
from topic_demo.msg import gps

def talker():
    #Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
    #Publisher 返回一个对象，操作这个对象进行发布
    #queue_size: None（不建议）  #这将设置为阻塞式同步收发模式！
    #queue_size: 0（不建议）#这将设置为无限缓冲区模式，很危险！
    #queue_size: 10 or more  #一般情况下，设为10 。queue_size太大了会导致数据延迟不同步。
    pub = rospy.Publisher('gps_info', gps , queue_size=10)
    # init_node 第一个参数为节点名称，第二个参数 anonymous=True 表示自动在第一个参数后面添加一个随机数，保证节点的唯一性
    rospy.init_node('pytalker', anonymous=True)
    # 节点频率：单位 Hz
    rate = rospy.Rate(1) # 此处设置的频率不会考虑一个循环运行的时间
    # 定义参数
    x=1.0
    y=2.0
    state='working'
    while not rospy.is_shutdown():
        #计算距离
        rospy.loginfo('Talker: GPS: x=%f ,y= %f',x,y)
        # 使用 gps(state,x,y) 创建需要发布的对象，直接调用 publish 方法进行发布
        pub.publish(gps(state,x,y))
        x=1.03*x
        y=1.01*y
        # 循环间隔
        rate.sleep()

if __name__ == '__main__':
    talker()
 
