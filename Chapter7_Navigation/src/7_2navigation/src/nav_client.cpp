/**
 * @file nav_client.cpp
 * @brief 该文件是一个 ROS 目标导航客户端。
 *
 * 该程序使用 actionlib 连接 move_base 服务器，并发送一个导航目标。
 * 机器人会尝试移动到设定的目标位置。
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// 定义一个 MoveBaseClient 类型的别名，方便后续使用
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
    // 初始化 ROS 节点，命名为 "nav_client"
    ros::init(argc, argv, "nav_client");

    // 创建 action 客户端，连接到 "move_base" 服务器
    MoveBaseClient ac("move_base", true);

    // 等待 move_base 服务器启动，每 5 秒检查一次
    while (ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // 创建一个目标点对象
    move_base_msgs::MoveBaseGoal goal;

    // 目标点的坐标系，设为 "map" 表示使用全局地图坐标
    goal.target_pose.header.frame_id = "map";
    // 设置时间戳，表示当前时间
    goal.target_pose.header.stamp = ros::Time::now();

    // 设定目标位置 (x, y) 坐标
    goal.target_pose.pose.position.x = -3.0;
    goal.target_pose.pose.position.y = 2.0;
    // 设定目标方向，四元数的 w=1.0 表示朝向默认方向（无旋转）
    goal.target_pose.pose.orientation.w = 1.0;

    // 发送目标点给 move_base
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    // 等待执行结果
    ac.waitForResult();

    // 判断是否成功到达目标点
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Mission complete !");
    else
        ROS_INFO("Mission failed ...");
}
