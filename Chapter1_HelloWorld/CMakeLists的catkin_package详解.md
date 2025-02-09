`catkin_package` 是 ROS 构建系统 `catkin` 中的一个核心命令，它用于定义和导出 ROS 包的构建信息，使其他 ROS 包能够使用当前包的头文件、库和依赖项。  

在 `CMakeLists.txt` 文件中，这个命令通常出现在中间位置，确保在定义构建目标后导出相关信息。

---

### 语法解析
```cmake
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_robot_lib
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)
```
---

### 作用
- **声明包的输出信息**，供其他 ROS 包在构建时使用。  
- 将包的头文件路径、库文件和依赖项暴露给其他包，使其能通过 `find_package` 导入这些信息。

---

### 参数详解
---

#### 1. `INCLUDE_DIRS`
```cmake
INCLUDE_DIRS include
```
- **作用**：声明当前包的头文件目录路径。  
- **例子**：假设包的头文件位于`include`目录下，例如：  
  ```
  my_robot_package/
  ├── include/
  │   └── my_robot_package/
  │       └── robot_control.h
  └── src/
      └── robot_control.cpp
  ```
  - 其他包在使用该包时，只需在代码中这样引用头文件：
    ```cpp
    #include <my_robot_package/robot_control.h>
    ```
  - 这行配置告诉 CMake，`include` 目录是公共头文件路径，供其他包访问。

- **多个路径示例**：
  ```cmake
  INCLUDE_DIRS include common/include
  ```

---

#### 2. `LIBRARIES`
```cmake
LIBRARIES my_robot_lib
```
- **作用**：声明当前包生成的库文件（静态库或动态库），供其他包链接使用。  
- **例子**：  
  ```cmake
  add_library(my_robot_lib src/robot_control.cpp)
  ```
  - 这将生成名为 `my_robot_lib` 的库，并通过 `catkin_package` 导出。  
  - 其他包可以通过以下方式链接该库：
    ```cmake
    target_link_libraries(other_package_node ${my_robot_lib})
    ```

- **多个库示例**：
  ```cmake
  LIBRARIES my_robot_lib motion_lib sensor_lib
  ```

---

#### 3. `CATKIN_DEPENDS`
```cmake
CATKIN_DEPENDS roscpp std_msgs sensor_msgs
```
- **作用**：声明当前包构建和运行时所需的 ROS 依赖项。  
- **例子**：
  - 依赖于 `roscpp`（C++接口），`std_msgs`（标准消息类型），以及 `sensor_msgs`（传感器消息类型）。
  - 这些依赖在`package.xml`中也必须声明：
    ```xml
    <depend>roscpp</depend>
    <depend>std_msgs</depend>
    <depend>sensor_msgs</depend>
    ```
  - CMake 会自动处理这些依赖，确保编译时能找到对应的头文件和库。  

- **多个依赖示例**：
  ```cmake
  CATKIN_DEPENDS roscpp message_generation tf
  ```
  - 表示依赖于`roscpp`（基础通信），`message_generation`（生成自定义消息），`tf`（坐标变换）。

---

### 其他可选参数
---

#### 1. `DEPENDS`
```cmake
DEPENDS Eigen Boost
```
- **作用**：声明非 ROS 第三方库依赖。  
- **例子**：  
  - `Eigen` 是 C++ 数学库，`Boost` 是 C++ 通用库。  
  - 这表示当前包依赖于系统级的 `Eigen` 和 `Boost` 库。

---

#### 2. `CFG_EXTRAS`
```cmake
CFG_EXTRAS extra_config.cmake
```
- **作用**：指定额外的 CMake 配置文件，通常用于复杂的自定义构建规则或宏定义。  

---

### 示例解析
```cmake
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_robot_lib
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
  DEPENDS Eigen
)
```
- 头文件位于`include`目录下。  
- 构建出名为`my_robot_lib`的库供其他包使用。  
- 依赖于`roscpp`、`std_msgs`和`sensor_msgs`，以及第三方库`Eigen`。  
- 其他包可以通过以下方式使用该包的功能：
  ```cmake
  find_package(catkin REQUIRED COMPONENTS my_robot_package)
  include_directories(${my_robot_package_INCLUDE_DIRS})
  target_link_libraries(my_node ${my_robot_package_LIBRARIES})
  ```

---

### 工作流程
1. **定义库和头文件**  
   在当前包中定义库或头文件路径。  

2. **导出构建信息**  
   通过`catkin_package`导出库和头文件路径，使其他包能够访问。  

3. **其他包使用**  
   在依赖包中通过`find_package`找到这个包，并链接其库文件和头文件。  

---

### `catkin_package`和`package.xml`的关系
- `catkin_package` 导出构建信息，`package.xml` 声明依赖关系。  
- 两者需要保持一致，`package.xml` 中的依赖也必须在 `CMakeLists.txt` 中声明。  

示例：
```xml
<depend>roscpp</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
```
对应的 `CMakeLists.txt`：
```cmake
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)
```

---

### 常见问题
1. **忘记导出头文件或库**  
   如果在`catkin_package`中省略`INCLUDE_DIRS`或`LIBRARIES`，其他包可能找不到头文件或无法链接库。  

2. **依赖声明不完整**  
   如果`CATKIN_DEPENDS`缺失所需依赖，构建时会报找不到符号或头文件。  

3. **系统级依赖未声明**  
   忘记在`DEPENDS`中声明第三方库，可能导致链接失败。