# Chapter7 机器人导航

- 大部分内容参见[Autolabor第 7 章 机器人导航(仿真)](http://www.autolabor.com.cn/book/ROSTutorials/di-7-zhang-ji-qi-ren-dao-822a28-fang-771f29.html)

## 功能包说明
- 7_2navigation 实现了

        7.2.1 导航实现01_SLAM建图
        7.2.2 导航实现02_地图服务
        7.2.3 导航实现03_定位
        7.2.4 导航实现04_路径规划
        7.2.5 导航与SLAM建图

    通过不同的launch文件启动

---

## 内容补充

### 7.2.1 SLAM建图步骤
1. 先启动 Gazebo 仿真环境:
    ```bash
    cd Chapter6_Simulation/
    source ./devel/setup.bash
    roslaunch 6_6gazebo car.launch
    ```

2. 然后再启动地图绘制的 launch 文件(再开一个终端):
    ```bash
    cd Chapter7_Navigation/
    source ./devel/setup.bash
    roslaunch 7_2navigation nav01_slam.launch
    ```

3. 启动键盘键盘控制节点，用于控制机器人运动建图:
    ```bash
    sudo apt install ros-noetic-teleop-twist-keyboard
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    ```
4. 在 rviz 中添加组件，显示栅格地图:
![alt text](imgs/Slam.png)

### 7.2.3 定位建图步骤
1. 同上

2. 启动键盘控制节点：
    ```bash
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    ```

3. 启动集成地图服务、amcl的 launch 文件：
    ```bash
    cd Chapter7_Navigation/
    source ./devel/setup.bash
    roslaunch 7_2navigation test_amcl.launch
    ```
4. 在启动的 rviz 中，添加RobotModel、Map组件，分别显示机器人模型与地图，添加 posearray 插件，设置topic为particlecloud来显示 amcl 预估的当前机器人的位姿，箭头越是密集，说明当前机器人处于此位置的概率越高；

5. 通过键盘控制机器人运动，会发现 posearray 也随之而改变。
![alt text](imgs/amcl.png)

### 7.2.4 路径规划启动步骤
1. 直接roslaunch 7_2navigation nav06_test.launch

2. 在rviz添加各个组件

3. 通过Rviz工具栏的 2D Nav Goal设置目的地实现导航。

---

## 有关知识点
### 7.1.2 坐标系单继承关系：map->odom->机器人模型中的根坐标系
![alt text](imgs/TF.png)

### 7.2.4 [代价地图](./Costmap.md)

### 7.2.4 [move_base功能包](./move_base.md)

---

## 有关问题
### 7.2.2 map_server保存图片：
1. 你需要自己执行一遍：
    ```bash
    roslaunch 7_2navigation nav02_map_save.launch
    ```
    因为.yaml文件的image路径需要定位到对应的目录
### 7.2.4 单个launch统一启动所有节点
1.  
    1) 问题：
        6_6gazebo使用频率比较高，将他放到nav06_test.launch中，执行
        ```bash
        source ~/rosWork/Chapter6_Simulation/devel/setup.bash
        source ~/rosWork/Chapter7_Navigation/devel/setup.bash
        roslaunch 7_2navigation nav06_test.launch
        会报错：
        Resource not found: 6_6gazebo
        ROS path [0]=/opt/ros/noetic/share/ros
        ROS path [1]=/home/hanyi/rosWork/Chapter7_Navigation/src
        ROS path [2]=/opt/ros/noetic/share
        The traceback for the exception was written to the log file
        ```
        如果你先执行 source 一个工作空间的 setup.bash，然后执行另一个工作空间的 setup.bash，后一个会覆盖前一个的环境变量，导致之前的工作空间路径丢失。

        哪怕
        ```bash
        source ~/rosWork/Chapter6_Simulation/devel/setup.bash >> ~/.bashrc 
        ```
        也是一样的

    2) 解决方法：
        1) 编辑 ~/.bashrc，并添加以下内容：
        ```bash
        export ROS_PACKAGE_PATH=~/rosWork/Chapter6_Simulation/src:~/rosWork/Chapter7_Navigation/src:$ROS_PACKAGE_PATH
        ```
        这样，ROS_PACKAGE_PATH 会包含两个工作空间的 src 路径。

        2) 保存并刷新 .bashrc：
        ```bash
        source ~/.bashrc
        ```

        验证是否正确： 运行以下命令查看 ROS_PACKAGE_PATH 是否包含两个工作空间：
        ```bash
        echo $ROS_PACKAGE_PATH
        ```

        输出应该是：
        ```bash
        /home/hanyi/rosWork/Chapter6_Simulation/src:/home/hanyi/rosWork/Chapter7_Navigatio
        ```

### 7.2.4 nav06_test.launch本地代价地图为空
1. 排查原因：

    1. 传感器数据问题
        - 排查： 检查激光雷达或深度摄像头数据：

        - 查看 `rostopic echo /scan`（如果使用激光雷达）

        ```sh
        rostopic echo /scan
        ```

        - 确保有合理的障碍物数据返回，而不是 `inf` 或 `nan`。
    
        - 使用 RViz 检查传感器数据
    - 添加 `LaserScan` 或 `PointCloud2`，看看障碍物是否正确显示。

    2. 局部代价地图参数问题
        - 排查： 检查 `local_costmap_params.yaml` 配置：
        - 可能的错误：
            - `obstacle_range` 太小：障碍物在代价地图外
            - `raytrace_range` 太大：导致激光射线清除掉障碍物
            - `inflation_radius` 过小：障碍物膨胀不足

        示例：
        ```yaml
        local_costmap:
        global_frame: odom
        robot_base_frame: base_link
        update_frequency: 5.0
        publish_frequency: 2.0
        width: 3.0  # 局部地图宽度（单位：米）
        height: 3.0 # 局部地图高度（单位：米）
        resolution: 0.05
        static_map: false
        rolling_window: true  # 局部地图随机器人移动

        obstacle_layer:
            enabled: true
            obstacle_range: 2.5
            raytrace_range: 3.0
            inflation_radius: 0.5
        ```
        
        - 检查 `obstacle_range` 和 `raytrace_range` 设置是否合理

    3. TF 变换问题

    4. 局部代价地图没有订阅激光雷达

    5. 静态地图 vs. 实时 SLAM
        - 如果使用静态地图（如 `gmapping`），确保 `static_map: false` 否则 `local_costmap` 可能不会更新。

2. 解决方法：
- 是TF变换出现了问题

- 在 `Chapter6_Simulation` 中car_urdf.xacro 文件定义了一个 **带有支架的激光雷达** 机器人模型

- 首先，我们要明白ROS 中小车 URDF 的坐标是怎么来的：
    1) 在 ROS 中，小车的部件坐标主要来源于两部分：

        1. URDF 文件（静态坐标）
            通过 parent-child link 和 joint origin 定义初始相对位置

        `robot_state_publisher` 读取 URDF 并发布 TF 变换
        2. 运行时 TF 变换（动态坐标）
        传感器、里程计（odometry）、SLAM 可能会动态调整 base_link 坐标

    2) 在 ROS 运行时，TF 变换是由 robot_state_publisher 生成的：

    ```bash
    rosrun robot_state_publisher robot_state_publisher
    ```

    这个节点会：

    1. 读取 URDF 里的 joint 关系

    2. 计算每个 link 的 初始坐标（基于 origin）

    3. 通过 tf 广播所有 link 的 静态坐标（如果 joint 是 fixed）

在 `local_costmap` 配置里，`sensor_frame` 被设定为 `base_scan`：  

```yaml
scan:
  sensor_frame: base_scan
  data_type: LaserScan
  topic: scan
  marking: true
  clearing: true
```
这表示 **局部代价地图（local_costmap）期望 `base_scan` 坐标系下的数据**。

但是，在雷达的xacro 文件中，激光雷达的 link 叫 **`laser`**：
```xml
<link name="laser">
```
所以，如果 `laser` 这个 link 在 `tf` 里发布的是 `laser`，但 `local_costmap` 需要 `base_scan`，**两者不匹配**，就会导致 `local_costmap` 无法正确获取障碍物数据。

- 解决方案:
    1. **修改 `sensor_frame` 使其匹配 `laser`**（更推荐）

    ```yaml
    scan:
        sensor_frame: laser  # 改成 "laser" 以匹配 Xacro 里的 link
        data_type: LaserScan
        topic: scan
        marking: true
        clearing: true
    ```

    2. **修改 Xacro 里的 `laser` link 使其变成 `base_scan`**
    ```xml
    <link name="base_scan">
    ```
    **但这样可能影响其他组件，通常不推荐，除非所有 `tf` 变换也一起改。**

---

