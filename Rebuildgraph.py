
# Define a class to simulate the given environment import bisect
import numpy as np
import yaml
import argparse
import bisect


# 定义状态类
class State(object):
    def __init__(self, top_left=(-1, -1), bottom_right=(-1, -1), time=0, interval=(0, float('inf')), agent_id=None):
        self.agent_id = agent_id  # 代理的唯一ID
        self.top_left = top_left  # 代理的左上角坐标
        self.bottom_right = bottom_right  # 代理的右下角坐标
        self.time = time  # 与此状态关联的时间
        self.interval = interval  # 与此状态关联的时间间隔
        # 代理的大小
        self.size = (bottom_right[0] - top_left[0] + 1, bottom_right[1] - top_left[1] + 1)

    def __lt__(self, other):
        return self.time < other.time  # 基于'time'属性进行比较

# 定义SippGrid类
class SippGrid(object):
    def __init__(self):
        self.f = float('inf')  # f值，用于A*搜索
        self.g = float('inf')  # g值，用于A*搜索
        self.parent_state = State()  # 父状态
        self.time_intervals = [(0, float('inf'))]  # 时间间隔列表

    # 分割时间间隔
    def split_interval(self, t, last_t=False):
        """
        Update the time intervals based on a given time 't' and dynamic obstacles.
        """
        for interval in self.time_intervals:
            if last_t:
                if t > interval[1]:
                    continue
                elif t<=interval[0]:
                    self.time_intervals.remove(interval)
                else:
                    self.time_intervals.remove(interval)
                    self.time_intervals.append((interval[0],t-1))
            else:
                if t == interval[0]: 
                    self.time_intervals.remove(interval)
                    if t+1 <=interval[1]:
                        self.time_intervals.append((t+1,interval[1]))
                elif t == interval[1]:
                    self.time_intervals.remove(interval)
                    if t-1 >=interval[0]:
                        self.time_intervals.append((interval[0],t-1))
                elif bisect.bisect_left(interval,t) == 1:
                    self.time_intervals.remove(interval)
                    self.time_intervals.append((interval[0],t-1))
                    self.time_intervals.append((t+1,interval[1]))
            self.time_intervals.sort()

    def __str__(self):
        return f"SippGrid(f={self.f}, g={self.g}, parent_state={self.parent_state}, time_intervals={self.time_intervals})"

# 定义SippGraph类
class SippGraph:
    def __init__(self, yaml_file):
        self.map = yaml_file  # 地图信息
        self.dimensions = yaml_file['map']['dimensions']  # 地图维度
        self.obstacles = [tuple(v) for v in yaml_file['map']['obstacles']]  # 静态障碍物
        # 代理的目标信息
        self.agent_goals = [
            {
                'goal_top_left': tuple(agent['goal_top_left']),
                'goal_bottom_right': tuple(agent['goal_bottom_right'])
            } for agent in yaml_file['agents']
        ]
        self.dyn_obstacles = yaml_file['dyn_obstacles']  # 动态障碍物
        self.sipp_graph = {}  # Sipp图
        self.init_graph()  # 初始化图
        self.init_intervals()  # 初始化时间间隔
    
    # 初始化图
    def init_graph(self):
        for i in range(self.dimensions[0]):
            for j in range(self.dimensions[1]):
                self.sipp_graph[(i, j)] = SippGrid()

    # 初始化时间间隔
    def init_intervals(self):
        if not self.dyn_obstacles:
            return
        for schedule in self.dyn_obstacles.values():
            for i in range(len(schedule)):
                location = schedule[i]

                top_left_x = location['top_left_x']
                top_left_y = location['top_left_y']
                bottom_right_x = location['bottom_right_x']
                bottom_right_y = location['bottom_right_y']
                t = location["t"]
                
                last_t = i == len(schedule) - 1  # 判断这是否是最后一个时间间隔

                # 更新矩阵内的所有格子的时间间隔
                for x in range(top_left_x, bottom_right_x):
                    for y in range(bottom_right_y,top_left_y):
                        self.sipp_graph[(x, y)].split_interval(t, last_t)

    # 检查位置是否有效    
    def is_valid_position(self, position, agent_id):
        dim_check = position[0] in range(self.dimensions[0]) and  position[1] in range(self.dimensions[1])
        obs_check = position not in self.obstacles 
        #goal_check = position not in [goal for i, goal in enumerate(self.agent_goals) if i != agent_id]
        return dim_check and obs_check #and goal_check

    # 获取有效的邻居   
    def get_valid_neighbors(self, top_left, bottom_right, agent_id):
        direction_neighbors = {
            "up": [],
            "down": [],
            "left": [],
            "right": []
        }

        top_right = (bottom_right[0], top_left[1])
        bottom_left = (top_left[0], bottom_right[1])

        # Edges
        top_edge = [(x, top_left[1]) for x in range(top_left[0], top_right[0] + 1)]
        bottom_edge = [(x, bottom_left[1]) for x in range(bottom_left[0], top_right[0] + 1)]
        left_edge = [(top_left[0], y) for y in range(bottom_left[1], top_left[1] + 1)]
        right_edge = [(top_right[0], y) for y in range(bottom_right[1], top_right[1] + 1)]

        # Neighbors
        up_neighbors = [(x, y + 1) for x, y in top_edge]
        down_neighbors = [(x, y - 1) for x, y in bottom_edge] 
        left_neighbors = [(x - 1, y) for x, y in left_edge]
        right_neighbors = [(x + 1, y) for x, y in right_edge]

        # Validate up neighbors
        if all(self.is_valid_position(neighbor, agent_id) for neighbor in up_neighbors):
            direction_neighbors["up"] = up_neighbors

        # Validate down neighbors
        if all(self.is_valid_position(neighbor, agent_id) for neighbor in down_neighbors):
            direction_neighbors["down"] = down_neighbors

        # Validate left neighbors 
        if all(self.is_valid_position(neighbor, agent_id) for neighbor in left_neighbors):
            direction_neighbors["left"] = left_neighbors

        # Validate right neighbors
        if all(self.is_valid_position(neighbor, agent_id) for neighbor in right_neighbors):
            direction_neighbors["right"] = right_neighbors

        return direction_neighbors
    
    def __str__(self):
        return f"SippGrid(time_intervals={self.time_intervals})"






        """
        遍历现有时间间隔: 该方法遍历与给定单元格相关联的现有时间间隔。对于每个间隔，它都检查特定时间t是否在该间隔内。

        处理动态障碍物的最后一个时间点: 如果last_t参数为True（表示这是动态障碍物的最后一个时间点），则方法有三种可能的操作：

        如果t大于间隔的结束时间，则保持间隔不变。
        如果t小于或等于间隔的开始时间，则移除该间隔。
        如果t在间隔内，则将间隔分割为两部分，第一部分从原始开始时间到t-1，第二部分被移除。
        处理间隔中的其他时间点: 如果last_t为False，并且t在间隔内，则根据t的位置有三种可能的操作：

        如果t等于间隔的开始时间，则移除间隔，并在t+1和原始结束时间之间创建新的间隔。
        如果t等于间隔的结束时间，则移除间隔，并在原始开始时间和t-1之间创建新的间隔。
        如果t在间隔的中间，则将间隔分割为两部分，第一部分从原始开始时间到t-1，第二部分从t+1到原始结束时间。
        排序时间间隔: 在更新所有间隔后，该方法重新排序时间间隔，以确保它们按时间顺序排列。


yaml_file = {
    'map': {
        'dimensions': [10, 10],
        'obstacles': [(2, 2), (3, 3), (4, 4)]
    },
    'agents': [{'goal_top_left': (9, 9), 'goal_bottom_right': (9, 9)}],
    'dyn_obstacles': {}
}

# Initialize SippGraph
graph = SippGraph(yaml_file)

# Define the agent's top-left and bottom-right coordinates
top_left = (5, 5)
bottom_right = (7,3)
agent_id = 0

# Get the valid neighbors
valid_neighbors = graph.get_valid_neighbours(top_left, bottom_right, agent_id)

# Print the valid neighbors

print(neighbour_list = [n for neighbors in direction_neighbors.values() for n in neighbors])



# Expected valid neighbors are positions that don't collide with obstacles or map boundaries




    test_yaml_file = {
    'map': {
        'dimensions': [5, 5],
        'obstacles': [(1, 1), (2, 2)]
    },
    'agents': [
        {'goal_top_left': (4, 4), 'goal_bottom_right': (4, 4)}
    ],
    'dyn_obstacles': {
        'obstacle_1': [
            {'x': 0, 'y': 0, 't': 1},
            {'x': 0, 'y': 1, 't': 2}
        ]
    }
}



# Testing the SippGrid class
sipp_grid_example = SippGrid()
print("Before splitting:", sipp_grid_example)
sipp_grid_example.split_interval(5)
print("After splitting at t=5:", sipp_grid_example)

graph = SippGraph(test_yaml_file)
print(graph.dimensions)  # Expected: [5, 5]
print(graph.obstacles)   # Expected: [(1, 1), (2, 2)]
print(graph.agent_goals) # Expected: [{'goal_top_left': (4, 4), 'goal_bottom_right': (4, 4)}]
print(graph.sipp_graph)  # Expected: A 5x5 grid of SippGrid objects
print(graph.sipp_graph[(1, 1)])
# Output: SippGrid(f=inf, g=inf, time_intervals=[(0, inf)])

"""