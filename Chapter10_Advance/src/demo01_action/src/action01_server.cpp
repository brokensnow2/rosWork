#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "demo01_action/AddIntsAction.h"
/*
    需求:
        创建两个ROS节点，服务器和客户端，
        客户端可以向服务器发送目标数据N（一个整型数据）
        服务器会计算1到N之间所有整数的和，这是一个循环累加的过程，返回给客户端，
        这是基于请求响应模式的，
        又已知服务器从接收到请求到产生响应是一个耗时操作，每累加一次耗时0.1s，
        为了良好的用户体验，需要服务器在计算过程中，
        每累加一次，就给客户端响应一次百分比格式的执行进度，使用action实现。

    流程:
        1.包含头文件;
        2.初始化ROS节点;
        3.创建NodeHandle;
        4.创建action服务对象;
        5.处理请求,产生反馈与响应;  ---callback
        6.spin().

*/

typedef actionlib::SimpleActionServer<demo01_action::AddIntsAction> Server;

// 回调函数，处理接收到的目标请求，并在计算过程中反馈进度和最终结果
void cb(const demo01_action::AddIntsGoalConstPtr &goal, Server *server)
{
    // 从目标消息中获取用户指定的数字（累加至该数字）
    int num = goal->num;
    // 在ROS日志中打印目标值
    ROS_INFO("目标值:%d", num);
    
    // 初始化累加结果为0
    int result = 0;
    // 创建反馈消息对象，用于在计算过程中向客户端发送进度反馈
    demo01_action::AddIntsFeedback feedback; // 连续反馈消息对象
    // 创建ros::Rate对象，以10Hz的频率进行循环睡眠，控制反馈发布的频率
    ros::Rate rate(10);                      // 设置频率为10Hz
    
    // 循环从1累加到目标数字num
    for (int i = 1; i <= num; i++)
    {
        // 将当前数字累加到result中
        result += i;
        // 计算当前进度，转换为浮点数，存入反馈消息中的progress_bar字段
        feedback.progress_bar = i / (double)num;
        // 将反馈消息发布出去，让客户端获取当前执行进度
        server->publishFeedback(feedback);
        // 按照预定的频率暂停，确保反馈消息发布的频率为10Hz
        rate.sleep();
    }
    
    // 创建结果消息对象，用于存储最终累加结果
    demo01_action::AddIntsResult r;
    // 将计算得到的累加结果赋值到结果消息中
    r.result = result;
    // 向客户端发送成功状态及最终结果
    server->setSucceeded(r);
    // 在ROS日志中打印最终计算的结果
    ROS_INFO("最终结果:%d", r.result);
}


int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ROS_INFO("action服务端实现");
    // 2.初始化ROS节点;
    ros::init(argc, argv, "AddInts_server");
    // 3.创建NodeHandle;
    ros::NodeHandle nh;
    // 4.创建action服务对象;

    /* SimpleActionServer构造函数原型：*/
        /*
        * @brief  Constructor for a SimpleActionServer
        * @param n A NodeHandle to create a namespace under
        * @param name A name for the action server
        * @param execute_callback Optional callback that gets called in a separate thread whenever
        *                         a new goal is received, allowing users to have blocking callbacks.
        *                         Adding an execute callback also deactivates the goalCallback.
        * @param  auto_start A boolean value that tells the ActionServer whether or not to start publishing as soon as it comes up. THIS SHOULD ALWAYS BE SET TO FALSE TO AVOID RACE CONDITIONS and start() should be called after construction of the server.
        */
    /*
        SimpleActionServer(ros::NodeHandle n,
                        std::string name,
                        boost::function<void (const demo01_action::AddIntsGoalConstPtr &)> execute_callback,
                        bool auto_start)
    */

    // actionlib::SimpleActionServer<demo01_action::AddIntsAction> server(....);

    // 有些类似话题通信和服务通信的结合体

    /*
    args:         节点    消息名称     回调函数                        启动模式
    */
    Server server(nh, "addInts", boost::bind(&cb, _1, &server), false); //boost::bind用于创建一个函数对象。其中的 _1 是一个占位符，代表将来在调用这个回调函数时由系统传入的第一个参数（即目标指针）。而 &server 则固定地作为第二个参数传入。
    server.start(); // 如果auto_start为false，需要手动调用该函数启动服务
    // 5.处理请求,产生反馈与响应;

    // 6.spin().
    ros::spin();
    return 0;
}
