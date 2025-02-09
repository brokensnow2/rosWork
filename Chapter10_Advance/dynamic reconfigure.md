# dynamic reconfigure
在 **ROS Noetic** 中，`cfg` 文件和 **Dynamic Reconfigure** 主要用于动态调整参数，而无需重新启动节点。Dynamic Reconfigure 允许你在程序运行时修改参数，使得调试和参数优化更加高效。

---

## **1. `cfg` 文件（配置文件）**
在 **dynamic_reconfigure** 包中，`cfg` 文件用于定义可以动态调整的参数。`cfg` 文件本质上是一个 Python 文件，定义了一系列参数，并使用 **dynamic_reconfigure** 提供的 API 进行注册。

### **1.1 `cfg` 文件的结构**
通常，`cfg` 文件放在 **`cfg/`** 目录下，文件名通常以 `.py` 或者 `.cfg` 结尾。例如，创建一个 `cfg/TutorialConfig.cfg` 文件：

```python
#!/usr/bin/env python
from dynamic_reconfigure.parameter_generator_catkin import *

# 创建参数
gen = ParameterGenerator()
gen.add("int_param", int_t, 0, "An integer parameter", 10, 0, 100)  # 整数参数，默认10，范围[0, 100]
gen.add("double_param", double_t, 0, "A double parameter", 0.5, 0.0, 1.0)  # 浮点数参数
gen.add("bool_param", bool_t, 0, "A boolean parameter", True)  # 布尔参数
gen.add("string_param", str_t, 0, "A string parameter", "default")  # 字符串参数
gen.add("level_param", int_t, 0, "Level-based parameter", 0, 0, 5)

# 生成配置文件
"""
gen.generate():
1. **第一个参数（Python 命名空间）**
   - 影响 **Python 端** 的 `cfg` 文件存放位置:
   生成的 Python 配置文件会放在 `devel/lib/python3/dist-packages/my_package/cfg` 目录下。 

   - 所以，Python 代码要使用 `from my_package.cfg import TutorialConfig`。

2. **第二个参数（C++ 头文件命名空间）**
   - 影响 **C++ 端** 头文件的生成路径：
   生成的 C++ 头文件通常在(devel/include/my_package/TutorialConfig.h）。

   - 所以在 C++ 代码中使用 `dynamic_reconfigure::Server<my_package::TutorialConfig>` 

3. **第三个参数（配置名称）**
   - 影响 `TutorialConfig.cfg` 生成的 Python 和 C++ 代码文件：
     - Python 端：`TutorialConfig.py`
     - C++ 端：`TutorialConfig.h`
   - 运行时，ROS 参数服务器中的 `dynamic_reconfigure` 相关参数也会带有这个名称。

- **第一个参数**：Python 命名空间（包名）
- **第二个参数**：C++ 头文件的命名空间
- **第三个参数**：配置名称（影响 `cfg` 文件生成的 `.py` 和 `.h` 文件）
- 这个函数的参数 **没有直接影响 ROS 节点名称**，只是生成的动态配置文件的命名。

如果想改变 **ROS 节点名称**，那要在 `ros::init(argc, argv, "node_name")` 或者 `roslaunch` 里调整，而不是在 `cfg` 里设置。
"""
exit(gen.generate("my_package", "my_package", "TutorialConfig"))
```

### **1.2 生成 `.h` 头文件**
运行以下命令会自动生成 C++ 头文件：
```bash
catkin_make
```
它会在 `devel/include/my_package` 目录下生成 `TutorialConfig.h` 头文件，供 C++ 代码使用。

---

## **2. 使用 Dynamic Reconfigure**
### **2.1 在 C++ 中使用**
在 C++ 代码中，导入 **dynamic_reconfigure** 相关头文件，并使用 `DynamicReconfigureServer` 监听参数变化：
```cpp
#include <dynamic_reconfigure/server.h>
#include <my_package/TutorialConfig.h>

void callback(my_package::TutorialConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %d %f %s %s",
             config.int_param,
             config.double_param,
             config.string_param.c_str(),
             config.bool_param ? "True" : "False");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamic_reconfigure_example");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<my_package::TutorialConfig> server;
    dynamic_reconfigure::Server<my_package::TutorialConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin();
    return 0;
}
```

编译后，运行：
```bash
rosrun my_package dynamic_reconfigure_example
```

---

## **3. `dynamic_reconfigure` 使用命令行或者编程方式修改参数。**

### **3.1 使用 GUI (`reconfigure_gui`)**
使用 `reconfigure_gui` 可以图形化调整参数：
```bash
rosrun dynamic_reconfigure reconfigure_gui
```
会显示所有支持 `dynamic_reconfigure` 的节点，并提供可视化调节选项。

### **3.2 使用命令行 (`rosparam set` 和 `rosservice call`)**
可以使用命令行工具 **`rosparam`** 或 **`rosservice`** 进行动态修改：

#### **方式 1：使用 `rosparam set`**
```bash
rosparam set /dynamic_reconfigure_example/int_param 50
```

但 `rosparam` 修改后不会立即触发回调，需要重新获取参数：
```bash
rosparam get /dynamic_reconfigure_example/int_param
```

#### **方式 2：使用 `rosservice call`**
```bash
rosservice call /dynamic_reconfigure_example/set_parameters "{int_param: 50, bool_param: true}"
```
这样会立即调用回调函数，更新参数。

### 编程实现
- [见demo02_dynamic_param包下的demo02_dr_client.cpp](./src/demo02_dynamic_param/src/demo02_dr_client.cpp)

#### 执行步骤：
1. roscore

2. 
   ```bash
   source ./devel/setup.bash
   rosrun demo02_dynamic_param demo01_dr_server
   ```

3. 
   ```bash
   source ./devel/setup.bash
   rosrun demo02_dynamic_param demo02_dr_client
   ```