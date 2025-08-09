# FtgData.msg
# 这个消息包含了从FTG主节点发送给 RVIZ可视化节点 所需的所有数据

# 消息头，包含时间戳和坐标系信息
std_msgs/Header header

# FTG算法处理后的雷达数据（已包含安全气泡）
float32[] proc_ranges

# 创建安全气泡之前的雷达数据（用于正确显示被覆盖的点）
float32[] proc_ranges_before_bubble

# 每个雷达数据点之间的弧度增量，这对于坐标转换至关重要
float32 radians_per_elem

# 最近障碍物的索引
int32 closest_index

# 最大间隙的起始和结束索引
int32 gap_start
int32 gap_end

# 选定的最佳目标点的索引
int32 best_point_index