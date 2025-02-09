# Question List

## 1. 

### Q：RVis报错：Unknown frame map
### A：
1. 问题解析：RViz 报错 "Unknown frame map" 表示 RViz 无法找到名为 map 的坐标系（TF frame）。这通常意味着 TF 发布器（Transform Broadcaster）没有正确发布 map 这个坐标系，或者 TF 关系链不完整。
2. 排查：但URDF文件配置没有问题，也已经在.launch文件配置发布节点。先检查 TF 树状态：
使用以下命令查看 TF 坐标系关系，结果发生报错：
    ```bash
    rosrun tf view_frames
    evince frames.pdf
    ```

    ```bash
    rosrun tf view_frames
    Listening to /tf for 5.0 seconds
    Done Listening
    b'dot - graphviz version 2.43.0 (0)\n'
    Traceback (most recent call last):
    File "/opt/ros/noetic/lib/tf/view_frames", line 119, in <module>
        generate(dot_graph)
    File "/opt/ros/noetic/lib/tf/view_frames", line 89, in generate
        m = r.search(vstr)
    TypeError: cannot use a string pattern on a bytes-like object
    ```
3. 解决方法：这是 ROS Noetic 中 view_frames 与 graphviz 新版本的兼容性问题。所以我们要先修复修改 view_frames 源代码
    - 打开 view_frames 文件：
        ```bash
        sudo gedit /opt/ros/noetic/lib/tf/view_frames
        ```
    - 找到以下代码（大约在第 89 行）：
        ```python
        m = r.search(vstr)
        ```
    - 修改为：
        ```python
        # 解释：decode('utf-8') 将字节流转换为字符串，避免类型不匹配问题。
        m = r.search(vstr.decode('utf-8'))
        ```
    - 保存并重新运行命令：
        ```bash
        rosrun tf view_frames
        evince frames.pdf
        ```
    - 成功输出PDF。再次启动launch文件无报错.
4. PS: 有可能只是命令行没有关闭造成的