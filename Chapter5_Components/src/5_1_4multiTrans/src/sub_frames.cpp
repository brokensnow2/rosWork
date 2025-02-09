/**
 * 需求描述:
 * 现有坐标系统，父级坐标系统 world,下有两子级系统 son1，son2，son1 相对于 world，以及 son2 相对于 world 的关系是已知的，求 son1原点在 son2中的坐标，又已知在 son1中一点的坐标，要求求出该点在 son2 中的坐标
 *
 * 实现分析:
 * 首先，需要发布 son1 相对于 world，以及 son2 相对于 world 的坐标消息
 * 然后，需要订阅坐标发布消息，并取出订阅的消息，借助于 tf2 实现 son1 和 son2 的转换
 * 最后，还要实现坐标点的转换
 *
 * 实现流程:C++ 与 Python 实现流程一致
 * 新建功能包，添加依赖
 * 创建坐标相对关系发布方(需要发布两个坐标相对关系)
 * 创建坐标相对关系订阅方
 * 执行
 *
 */

// 1. 包含头文件
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"

int main(int argc, char *argv[])
{
    // 设置全局编码
    setlocale(LC_ALL, "");

    // 2. 初始化ros节点
    ros::init(argc, argv, "sub_frames");

    // 3. 创建ros句柄
    ros::NodeHandle nh;

    // 4. 创建tf订阅对象
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    // 5. 解析订阅信息中获取 son1 坐标系原点在 son2 中的坐标
    ros::Rate r(1);
    while (ros::ok())
    {
        try
        {
            // 解析son1中的点相对son2的坐标
            // tfBuffer.lookupTransform 查询两个坐标系之间的变换关系。可以指定查询的源坐标系、目标坐标系和查询时间。 见TF坐标树.md
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("son2", "son1", ros::Time(0));
            ROS_INFO("Son1 相对于 Son2 的坐标关系： 父坐标系ID是: %s", tfs.header.frame_id.c_str());
            ROS_INFO("Son1 相对于 Son2 的坐标关系： 子坐标系ID是: %s", tfs.child_frame_id.c_str());
            ROS_INFO("Son1 相对于 Son2 的坐标关系： x = %.2f, y = %.2f, z = %.2f",
                     tfs.transform.translation.x,
                     tfs.transform.translation.y,
                     tfs.transform.translation.z);
            // 坐标点解析
            geometry_msgs::PointStamped ps;
            ps.header.frame_id = "son1";
            ps.header.stamp = ros::Time::now();
            ps.point.x = 1.0;
            ps.point.y = 2.0;
            ps.point.z = 3.0;

            geometry_msgs::PointStamped psAtSon2;
            psAtSon2 = buffer.transform(ps, "son2");
            ROS_INFO("在 Son2 中的坐标:x =% .2f,y = %.2f,z = %.2f",
                     psAtSon2.point.x,
                     psAtSon2.point.y,
                     psAtSon2.point.z);
        }
        catch (const std::exception &e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("异常信息： %s", e.what());
        }
    }

    return 0;
}
