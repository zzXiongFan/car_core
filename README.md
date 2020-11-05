# topic_demo

topic_demo软件包，同时包括 c++ 和 python 的内容。

若只看 Python 版本，仅需要关注 ./msg 和 ./script 两个文件夹

## 开发方法

ROS 消息为标准的 发布-订阅模式，后续提到的所有概念均从这个模式出发，在 ROS 的消息模式中，需要关注以下几个点，此为编程的关键：

1. 发布者如何定义自己的 Topic？——> 是用 ROS api 定义名称
2. 发布的数据格式如何定义？ ——>  msg 文件
3. 如何控制发送的频率？——> 循环中使用 Rate

开发方法参考以下步骤：

1. 首先定义消息类型 ./msg/gps.msg 具体格式参看注解
2. 在 ./script 中引入消息类型，并执行对应的方法，具体查看注释，其中
   - pytalker.py 为发布者
   - pylistener.py 为订阅者

## 运行方法

首先启动 roscore

启动发布者

```sh
$ rosrun topic_demo pytalker.py   #Python
# 或者直接启动对应的 .py 文件亦可
# ./script
python pytalker.py
```

启动接收者

```sh
$ rosrun topic_demo pylistener.py   #Python
# 或者直接启动对应的 .py 文件亦可
# ./script
python pylistener.py
```

msg是与编程语言无关的通信协议，因此收发双方无论用哪个语言来实现，都可以实现相互的topic通信。
