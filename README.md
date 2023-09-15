需要安装
numpy
matplotlib
scipy
ffmpeg-python

Rebuildmulti.py  -----多智能体 sipp 算法实现
Rebuildsipp.py ----- SIPP 算法实现
RebuildGraph.py -----地图网格初始化
RebuildVis.py -----可视化


算法运行
python3 Rebuildmulti.py generated_map.yaml generated_map_output.yaml

generated_map.yaml 地图信息
generated_map_output.yaml 算法输出文件

可视化运行
python3 RebuildVis.py generated_map.yaml generated_map_output.yaml

generated_map.yaml 地图信息
generated_map_output.yaml 算法输出文件

已完成
sipp 算法实现
多智能体任意大小路径规划

发现问题
算法无法优化进入一秒内
    尝试修改开放列表改为优先队列,优先队列确保了每次都是首先考虑估算总代价最低的节点,提升 3 被速度
    修改启发式函数---未果
    修改领居检测条件---未果
    修改 A* 算法---未果
    怀疑底层初始化问题, 由于会初始化地图遍历地图方格中的时间间隔,算法会因为地图增大而消耗更多时间

...
    液滴
    topleft botttomright 两个坐标可以确定液滴大小以及在地图中的位置
...
agents:
- name: agent0
  start_top_left: [32, 20]
  start_bottom_right: [33, 19]
  goal_top_left: [5, 55]
  goal_bottom_right: [6, 54]
- name: agent1
  start_top_left: [35, 20]
  start_bottom_right: [36, 19]
  goal_top_left: [8, 55]
  goal_bottom_right: [9, 54]

...
    obstacles 可以用于表示地图上的障碍物(坏点)
...
map:
  dimensions: [10, 10]
  obstacles: []
dyn_obstacles: {}
