# msg文件详解
- 在ROS中，`.msg`文件用于定义消息的结构。ROS的消息分为两类：标准消息（如`std_msgs`）和自定义消息（用户自己定义的）。

在2_1_5customMsg的例子中，`Person.msg`和`Money.msg`都是自定义消息，分别定义了一个`Person`类型和一个`Money`类型的消息。

---

## `Person.msg`内容解释

`Person.msg`是一个自定义消息，它包含了三个字段，分别是`headers`、`money`、`name`、`age`和`height`。这些字段类型分别为：

### `std_msgs/Header`
首先，`std_msgs/Header`是一个标准消息类型，通常用于存储消息的元数据。它包含时间戳和帧ID等信息。ROS中有很多标准消息类型（`std_msgs`是其中之一），可以直接引用这些消息类型。`Header`的定义如下：

```plaintext
# std_msgs/Header
uint32 seq        # 序列号，表示消息的顺序
time stamp        # 时间戳，表示消息生成的时间
string frame_id   # 坐标系ID，表示这个消息的数据在哪个坐标系中有效
```

消息的内容可以包含基本的ROS数据类型（如`string`、`uint16`、`float64`等），还可以包含其他消息类型。例如，`money`字段使用了另一个自定义消息`Money`。

### `Money.msg`
`Money.msg`是另一个自定义消息类型，它定义了一个字段`money`，类型是`uint16`，表示金钱的数量。

```plaintext
# Money.msg
uint16 money
```

### 基本类型
`uint16 age`: 无符号16位整数，表示人的年龄。
`float64 height`: 64位浮动类型，表示人的身高。

---

### 如何使用这些消息
1. **定义自定义消息**：你在功能包`msg`目录下定义了`Person.msg`和`Money.msg`，这些消息文件的格式是ROS的标准消息格式（每个字段使用不同的数据类型定义）。

2. **编译消息文件**：在ROS中，每次定义或修改消息文件后，必须在`CMakeLists.txt`中添加相关设置，然后通过`catkin_make`来编译消息。编译过程会自动生成相应的C++、Python代码，使你能够在程序中直接使用这些消息类型。

3. **在代码中使用自定义消息**：
    - 在C++中，你可以通过`#include <your_package/Person.h>`来使用自定义消息。
    - 在Python中，你可以通过`from your_package.msg import Person`来导入并使用。

4. **发布和订阅**：使用自定义消息后，你可以在ROS节点中发布和订阅这些消息。例如，在一个节点中发布`Person`类型的消息：

   ```cpp
   #include <ros/ros.h>
   #include <your_package/Person.h>
   
   int main(int argc, char** argv) {
       ros::init(argc, argv, "person_publisher");
       ros::NodeHandle nh;
       
       ros::Publisher pub = nh.advertise<your_package::Person>("person_topic", 10);
       
       your_package::Person msg;
       msg.name = "John";
       msg.age = 30;
       msg.height = 1.75;
       msg.money.money = 500; // 重要!
       
       ros::Rate loop_rate(1);
       while (ros::ok()) {
           pub.publish(msg);
           ros::spinOnce();
           loop_rate.sleep();
       }
       return 0;
   }
   ```

在这个代码示例中，定义了一个`Person`消息，并发布了它，其中`money`字段是通过`Money`消息类型进行赋值的。

总结一下：
- `msg`文件用于定义消息结构，包括内建消息类型（如`std_msgs/Header`）和自定义消息类型（如`Person`和`Money`）。
- 可以在ROS节点中发布和订阅这些消息。
- 自定义消息可以包含其他消息类型，这使得ROS中的数据交换更加灵活。