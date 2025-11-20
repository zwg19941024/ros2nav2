# 仅导航功能启动文件使用说明

## 概述

`navigation_only.launch.py` 是一个专门的启动文件，仅包含Nav2导航功能，不包含AMCL定位模块。这适用于已经有外部定位系统提供 `map->odom` 变换的情况。

## 主要特点

- **不包含AMCL定位**：假设外部定位系统已经提供了 `map->odom` 变换
- **包含完整导航功能**：路径规划、路径跟踪、局部避障等
- **地图服务器**：加载静态地图
- **可视化支持**：启动RViz2进行导航可视化

## 启动的组件

1. **地图服务器** (`map_server`): 加载静态地图
2. **导航堆栈** (`navigation_launch.py`): 包含以下组件：
   - 路径规划服务器 (`planner_server`)
   - 控制器服务器 (`controller_server`) 
   - 行为树导航器 (`bt_navigator`)
   - 行为服务器 (`behavior_server`)
   - 路径平滑服务器 (`smoother_server`)
   - 航点跟随器 (`waypoint_follower`)
   - 速度平滑器 (`velocity_smoother`)
   - 全局和局部代价地图 (`global_costmap`, `local_costmap`)
3. **RViz2**: 导航可视化界面
4. **生命周期管理器**: 管理节点生命周期

## 使用方法

### 基本启动
```bash
ros2 launch isaac_nav2 navigation_only.launch.py
```

### 自定义参数启动
```bash
ros2 launch isaac_nav2 navigation_only.launch.py \
    map:=/path/to/your/map.yaml \
    use_sim_time:=false \
    params_file:=/path/to/your/nav2_params.yaml
```

## 启动参数

- `use_sim_time` (默认: false): 是否使用仿真时间
- `map` (默认: isaac_nav2/maps/home.yaml): 地图文件路径
- `params_file` (默认: isaac_nav2/config/nav2_navigation_only_params.yaml): Nav2参数文件
- `use_lifecycle_mgr` (默认: true): 是否使用生命周期管理器
- `autostart` (默认: true): 是否自动启动导航堆栈

## 前提条件

在使用此启动文件之前，请确保：

1. **外部定位系统正在运行**：必须有系统发布 `map->odom` 变换
2. **传感器数据可用**：确保激光雷达或点云数据正在发布到正确的话题
3. **机器人模型已加载**：确保robot_state_publisher正在运行并发布机器人的TF树

## TF树要求

系统期望的TF树结构：
```
map -> odom (由外部定位系统提供)
odom -> base_link (由里程计提供)
base_link -> laser_frame (由robot_state_publisher提供)
```

## 常见问题

### 1. 导航不工作
- 检查 `map->odom` 变换是否正在发布
- 确认传感器数据话题名称是否与配置文件中一致
- 验证代价地图是否正确更新

### 2. 路径规划失败
- 检查地图是否正确加载
- 确认起始位置和目标位置在地图的自由空间内
- 查看全局代价地图是否有有效路径

### 3. 局部避障问题
- 检查局部代价地图配置
- 确认传感器数据正在正确发布到局部代价地图
- 调整DWB控制器参数

## 相关文件

- `config/nav2_navigation_only_params.yaml`: 导航专用参数配置
- `launch/navigation2.launch.py`: 包含AMCL的完整导航启动文件
- `config/nav2_params.yaml`: 包含AMCL的完整参数配置
