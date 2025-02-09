# Chapter2 ROS通信机制

- 大部分内容参见[Autolabor文档第二章](http://www.autolabor.com.cn/book/ROSTutorials/di-2-zhang-ros-jia-gou-she-ji.html)

## 内容补充
### 2.1.2 执行步骤
1. 
    使用isolated构建

        ```bash
        catkin_make_isolated --directory /home/hanyi/rosWork/Chapter2_Communications -DCMAKE_BUILD_TYPE=RelWithDebInfo 
        ```
    而不是catkin_make,因为 2_1_5customMsg 包无法编译,所以要独立编译
2. 
    设置环境

    ```bash
    source ./devel_isolated/2_1_2topicCommun/setup.bash 
    ```
    而不是
    ```bash
    source ./devel/setup.bash 
    ```

3. 启动节点

    ```bash
    roscore
    rosrun 2_1_2topicCommun demo01_pub
    rosrun 2_1_2topicCommun demo02_sub
    ```

### 2.2.3 服务通信自定义srv调用A(C++) 执行步骤
1. 
    同上,使用isolated构建：
    ```bash
    catkin_make_isolated --only-pkg-with-deps demo02_paramCommun --install --directory /home/hanyi/rosWork/Chapter2_Communications -DCMAKE_BUILD_TYPE=RelWithDebInfo
    ```
    因为isolated编译到第二个失败后就自动停止了。虽然第一个编译成功了，但无法编译到第三个。
    所以使用使用 --only-pkg-with-deps 参数来指定只编译哪些包。

2. 
    同上
    ```bash
    source ./devel_isolated/demo02_paramCommun/setup.bash
    ```

3. 
    同上
    ```bash
    rosrun demo02_paramCommun AddInts_Client
    rosrun demo02_paramCommun AddInts_Server
    ```

---

## 有关知识点
### 2.1.5 [自定义消息](./msg文件详解.md)

---

## 有关问题
### 2.1.5 [2_1_5customMsg包编译失败](./src/2_1_5customMsg/该为失败案例.md)