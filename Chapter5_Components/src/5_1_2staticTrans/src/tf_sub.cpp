/*
    订阅坐标系信息，生成一个相对于 子级坐标系的坐标点数据，转换成父级坐标系中的坐标点

    实现流程:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建 TF 订阅节点
        4.生成一个坐标点(相对于子级坐标系)
        5.转换坐标点(相对于父级坐标系)
        6.spin()
*/
// 1.包含头文件
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    // 2.初始化 ROS 节点
    ros::init(argc, argv, "tf_sub");
    ros::NodeHandle nh;
    // 3.创建 TF 订阅节点, 用于接收和缓存坐标变换信息
    tf2_ros::Buffer buffer;
    /*
    TF 监听器 会自动订阅一个 ROS 话题（通常是 /tf 和 /tf_static），并在后台持续接收和更新坐标变换数据。
    这些变换信息会存储在缓存对象 buffer中，你可以通过调用 buffer.lookupTransform() 查询任意两个坐标系之间的变换关系。
    监听器本身不需要显式接收消息或写订阅回调函数，它自动在后台完成消息解析和缓存维护。
    bbuffer 可以保存最近几秒的变换记录（默认10秒），支持查询任意时间点的变换
    一旦广播器发送了变换，监听器就会接收到，并将数据保存在 tfBuffer 中。
    */
    tf2_ros::TransformListener listener(buffer);

    ros::Rate r(1);
    while (ros::ok())
    {
        // 4.生成一个坐标点(相对于子级坐标系)
        geometry_msgs::PointStamped point_laser;
        point_laser.header.frame_id = "laser";
        point_laser.header.stamp = ros::Time::now();
        point_laser.point.x = 1;
        point_laser.point.y = 2;
        point_laser.point.z = 7.3;
        // 5.转换坐标点(相对于父级坐标系)
        // 新建一个坐标点，用于接收转换结果
        //--------------使用 try 语句或休眠，否则可能由于缓存接收延迟而导致坐标转换失败------------------------
        try
        {
            geometry_msgs::PointStamped point_base;
            // buffer.transform 是用来将一个点或对象从一个坐标系变换到另一个坐标系
            /*
            point_laser：geometry_msgs::PointStamped 类型，表示在 laser 坐标系下的一个点。
            "base_link"：目标坐标系。表示要将这个点变换到 base_link 坐标系下。
            buffer：tf2_ros::Buffer 对象，存储了所有 TF 变换信息。
            返回值：point_base 是变换后的点，位于 base_link 坐标系下。
            */
            point_base = buffer.transform(point_laser, "base_link");
            ROS_INFO("转换后的数据:(%.2f,%.2f,%.2f),参考的坐标系是:(%s)", point_base.point.x, point_base.point.y, point_base.point.z, point_base.header.frame_id.c_str());
        }
        catch (const std::exception &e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("程序异常.....");
        }

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
