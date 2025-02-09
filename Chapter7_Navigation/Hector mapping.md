# hector-mapping
GMapping 和 Hector Mapping 都是基于激光雷达（LiDAR）的 **SLAM（Simultaneous Localization and Mapping）** 算法，在 ROS（Robot Operating System）中广泛用于 **2D 地图构建**。它们的主要区别在于 **位姿估计方式**、**运动模型依赖性**、**计算复杂度** 和 **适用场景**。  

---

### 1. **相同点**
**目标相同**：  
- 都用于 **2D 激光雷达 SLAM**，能够在未知环境中生成占据栅格地图（Occupancy Grid Map）。  
- 适用于 **室内环境**，比如机器人导航、AGV（自动导引车）等。  

**输入数据相同**：
- 主要输入：**激光雷达数据（LaserScan）**  
- 可选输入：**IMU、里程计（Odometry）**（Hector Mapping 不需要里程计）。  

**都基于粒子滤波（Particle Filter）**  
- 两者都使用 **RBPF（Rao-Blackwellized Particle Filter）** 进行 SLAM 计算。

---

### 2. **不同点**
|  | **GMapping** | **Hector Mapping** |
|------|------------|----------------|
| **位姿估计** | 依赖 **里程计（Odometry）** 作为先验，结合激光雷达数据进行校正 | **仅依赖激光雷达数据**，不需要里程计 |
| **运动模型** | 需要一个良好的运动模型（比如轮式里程计） | 不依赖运动模型，适用于**无轮式里程计**的机器人（如四旋翼无人机） |
| **匹配方式** | 采用 **scan-matching + 粒子滤波** 进行优化 | 采用 **高精度 scan-matching（Gauss-Newton 优化）** |
| **计算复杂度** | 计算量较大，粒子数越多，计算越慢 | 计算量较低，适用于高频更新 |
| **适用场景** | 适用于 **低速、规则运动** 的机器人（如 AGV、服务机器人） | 适用于 **快速、高动态** 运动的机器人（如无人机、UGV） |
| **地图质量** | 对里程计依赖较强，里程计误差大时可能产生漂移 | 由于直接基于激光雷达匹配，通常能构建更精确的地图 |
| **对计算资源的需求** | 需要较多计算资源，尤其是粒子数较多时 | 计算高效，适用于实时 SLAM |

---

### 3. **适用场景分析**
- **GMapping**：适用于 **移动机器人（AGV、服务机器人）**，对里程计质量要求较高，适合 **低速、平稳运动** 的场景。  
- **Hector Mapping**：适用于 **无人机、腿式机器人、无轮里程计的移动机器人**，适合 **高速、动态运动**，对 IMU 依赖较强。

---

### 4. **总结**
- 如果 **机器人有高质量的里程计**，用 **GMapping**，可以结合运动模型提供更稳定的位姿估计。  
- 如果 **机器人没有里程计或者运动较快**，用 **Hector Mapping**，它直接使用激光雷达数据进行位姿估计，适应性更强。  
- Hector Mapping 对 **高分辨率激光雷达** 更友好，而 GMapping 对 **低速平稳运动** 更合适。  

---

## 安装
sudo apt install ros-noetic-hector-mapping

## 实验

## Hecotr-mapping的缺点：
1. https://www.bilibili.com/video/BV1ih4y177QK?spm_id_from=333.788.player.switch&vd_source=ac9e977aebe7eff3c527a35dde43de6d