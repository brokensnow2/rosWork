#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sstream"

void doMsg(const std_msgs::String::ConstPtr &msg_p)
{
    ROS_INFO("我听见:%s", msg_p->data.c_str());
    // ROS_INFO("我听见:%s",(*msg_p).data.c_str());
}

int main(int argc, char *argv[])
{
    // 设置全局编码
    setlocale(LC_ALL, "");
    // 创建节点
    ros::init(argc, argv, "listener");
    // 创建ros句柄
    ros::NodeHandle nh;
    // 实例化 订阅者 对象
    ros::Subscriber sub = nh.subscribe<std_msgs::String>(
        "chatter", 10, doMsg);
    // 处理订阅的消息(回调函数)

    // 设置循环调用回调函数
    ros::spin(); // 循环读取接收的数据，并调用回调函数处理
    return 0;
}
