#include "ros/ros.h"
// #include "dynamic_reconfigure/server.h"
#include "dynamic_reconfigure/client.h"
#include "demo02_dynamic_param/drConfig.h"

int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "dr_client");
    
    ros::NodeHandle nh;

    // 创建 dynamic_reconfigure 客户端，连接到指定的服务端节点（如 /my_node）
    dynamic_reconfigure::Client<demo02_dr::drConfig> client("dr_server");

    demo02_dr::drConfig config;

    // 获取当前参数
    if (client.getCurrentConfiguration(config, ros::Duration(5)))
    {
        ROS_INFO("Current 整型参数: %d  \n", config.int_param);
    }
    else
    {
        ROS_ERROR("Failed to get current configuration.");
    }

    // 修改参数
    config.int_param = 2.5;

    client.setConfiguration(config);
    ROS_INFO("Updated parameters: Current 整型参数: %d  \n", config.int_param);

    return 0;
}
