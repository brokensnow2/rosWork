[赵虚左老师写的很清除了](http://www.autolabor.com.cn/book/ROSTutorials/chapter1/15-ben-zhang-xiao-jie.html)

# ROS文件系统
ROS（Robot Operating System）的文件系统组织类似于一个分层结构，方便开发者管理和组织机器人项目。它主要由 **工作空间（workspace）** 和 **功能包（package）** 组成。  

---

### **1. 文件系统层级**  
ROS 文件系统的核心层级包括：  

#### **1.1. 工作空间（Workspace）**  
- **工作空间**是 ROS 开发的基础目录，包含多个 ROS 功能包（package）。  
- ROS 1 常用的工作空间构建工具是 `catkin`，ROS 2 则使用 `colcon`。  

**示例：ROS 1 典型工作空间目录结构**  
```bash
~/catkin_ws/         # 工作空间根目录
│
├── src/             # 源代码目录，存放功能包
│   ├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
│   └── my_package/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── src/
│       ├── include/
│       ├── launch/
│       └── msg/
│
├── build/           # 构建文件夹
├── devel/           # 开发空间，存放生成的库和可执行文件
└── install/         # 安装空间（colcon 构建中存在）
```  
- **`src/`**：存放源码的目录，所有功能包放在这里。  
- **`build/`**：存放构建过程中的临时文件。  
- **`devel/`**：构建后生成的二进制文件和库文件，开发时运行程序主要使用这个目录。  
- **`install/`**：安装后的文件，适合部署环境中使用（ROS 2 使用）。  

---

#### **1.2. 功能包（Package）**  
- **功能包**是 ROS 项目的最小单元，包含一个独立的功能模块，比如导航、控制、感知等。  
- 每个功能包都有自己的 `CMakeLists.txt` 和 `package.xml`，用于描述包的依赖关系和编译方式。  

**典型的 ROS 功能包结构**  
```bash
my_package/
├── CMakeLists.txt      # CMake 构建配置文件
├── package.xml         # 功能包的元信息和依赖描述
├── src/                # 源代码目录
│   └── main.cpp
├── include/            # 头文件目录
│   └── my_package/
├── launch/             # 启动文件目录
│   └── my_launch.launch
├── msg/                # 自定义消息类型
├── srv/                # 自定义服务类型
└── config/             # 配置文件
```  
- **`CMakeLists.txt`**：使用 CMake 进行构建，包含包的构建规则。  
- **`package.xml`**：定义包的依赖关系、版本和作者信息。  
- **`src/`**：源码文件夹，包含实现代码。  
- **`include/`**：头文件目录。  
- **`launch/`**：ROS 启动文件，定义节点如何启动和配置。  
- **`msg/`**：自定义消息类型。  
- **`srv/`**：自定义服务类型。  

---

### **2. 关键文件解析**  

#### **2.1. package.xml**  
- 每个功能包都需要有一个 `package.xml` 文件，描述包的信息和依赖关系。  
**示例：`package.xml` 内容**  
```xml
<package format="2">
  <name>my_package</name>
  <version>0.1.0</version>
  <description>A simple ROS package</description>
  <maintainer email="user@example.com">User Name</maintainer>
  <license>BSD</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>std_msgs</depend>
</package>
```
- `name`：功能包名称。  
- `version`：版本号。  
- `depend`：功能包的依赖包。  
- `maintainer`：维护者信息。  

---

#### **2.2. CMakeLists.txt**  
- 每个包的 `CMakeLists.txt` 文件定义了包的构建规则和依赖关系。  
**示例：CMakeLists.txt**  
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(my_package)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(node_name src/main.cpp)
target_link_libraries(node_name ${catkin_LIBRARIES})
```

---

### **3. ROS 1 vs ROS 2 文件系统差异**  
- **ROS 1 使用 catkin**，而 **ROS 2 使用 colcon 和 ament**。  
- **ROS 2 增加了 install/ 目录**，在部署时直接从该目录运行。  
- ROS 2 强调模块化，`src` 目录下可以存放不同语言的包（C++, Python）。  

---

### **4. 启动文件（Launch Files）**  
- 启动文件用于一次性启动多个 ROS 节点，并配置节点参数。  
- ROS 1 使用 **XML 格式**，ROS 2 支持 **XML 和 Python 格式**。  

**ROS 1 启动文件示例**  
```xml
<launch>
  <node name="talker" pkg="my_package" type="talker" output="screen"/>
  <node name="listener" pkg="my_package" type="listener" output="screen"/>
</launch>
```  

---

### **5. 重要命令**  
- **创建功能包**  
```bash
catkin_create_pkg my_package roscpp std_msgs
```  
- **构建工作空间**  
```bash
catkin_make
```  
- **运行功能包**  
```bash
rosrun my_package node_name
```  
- **启动启动文件**  
```bash
roslaunch my_package my_launch.launch
```  

---

### **总结**  
- **ROS 文件系统层次清晰**，通过工作空间和功能包结构组织项目。  
- **功能包是核心单元**，包含源代码、头文件、启动文件等。  
- **CMakeLists.txt 和 package.xml 是包构建和依赖管理的关键文件**。  
- 通过 `catkin` 和 `colcon` 可以轻松管理和构建复杂的 ROS 项目。