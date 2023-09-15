#!/usr/bin/env python3
# 导入必要的库和模块
import yaml

from matplotlib.patches import Circle, Rectangle, Arrow
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import argparse
import math

# 定义颜色列表
Colors = ['orange', 'blue', 'green']

# 定义动画类，主要负责显示智能体在地图上的移动
class Animation:
    def __init__(self, map, schedule):
        self.map = map
        self.schedule = schedule
        self.obstacle_schedule = map.get("dyn_obstacles", {})
        self.combined_schedule = {}
        self.combined_schedule.update(self.schedule.get("schedule", {}))
        self.combined_schedule.update(self.obstacle_schedule)



        aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]
        self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
        self.ax = self.fig.add_subplot(111, aspect='equal')

        # Hide the tick labels
        self.ax.set_xticklabels([])
        self.ax.set_yticklabels([])

        self.ax.grid(True, which='both', color='black', linewidth=0.5)  # 设置网格

        # 设置x和y轴的刻度位置
        x_ticks = np.arange(0, self.map["map"]["dimensions"][0] + 1, 1)
        y_ticks = np.arange(0, self.map["map"]["dimensions"][1] + 1, 1)
        self.ax.set_xticks(x_ticks)
        self.ax.set_yticks(y_ticks)

        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)

        self.patches = []
        self.artists = []
        self.agents = dict()

        # create boundary patch
        xmin = -0.5
        ymin = -0.5
        xmax = map["map"]["dimensions"][0] - 0.5
        ymax = map["map"]["dimensions"][1] - 0.5

        plt.xlim(xmin, xmax)
        plt.ylim(ymin, ymax)
        self.patches.append(CustomRectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='red'))
        
        for o in map["map"]["obstacles"]:
            x, y = o[0], o[1]
            self.patches.append(CustomRectangle((x, y), 1, 1, facecolor='red', edgecolor='red'))

        self.T = 0
        for d in map["agents"]:
            goal = d["goal_top_left"]
            #self.patches.append(Rectangle((goal[0], goal[1]), 3, 3, facecolor=Colors[0], edgecolor='black', alpha=0.5))
            # 然后在您的代码中使用 CustomRectangle 而不是 Rectangle
            self.patches.append(CustomRectangle((goal[0], goal[1]), 1, 1, facecolor=Colors[0], edgecolor='black', alpha=0.5))

        for d in map["agents"]:
            name = d["name"]
            # Change agent from Circle to Rectangle
            self.agents[name] = CustomRectangle((d["start_top_left"][0], d["start_top_left"][1]), 1, 1, facecolor=Colors[0], edgecolor='black')


            self.agents[name].original_face_color = Colors[0]
            self.patches.append(self.agents[name])
            self.T = max(self.T, schedule["schedule"][name][-1]["t"])
            
        for name, data in self.obstacle_schedule.items():
            self.agents[name] = Rectangle((data[0]["start"][0], data[0]["start"][1]), 1, 1, facecolor=Colors[2], edgecolor='black')
            self.agents[name].original_face_color = Colors[2]
            self.patches.append(self.agents[name])
            self.T = max(self.T, data[-1]["t"])

        self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                                            init_func=self.init_func,
                                            frames=int(self.T + 1) * 10,
                                            interval=100,
                                            blit=True)


    # 保存动画为视频文件
    def save(self, file_name, speed):
        self.anim.save(
            file_name,
            "ffmpeg",
            fps=25 * speed,
            dpi=200)

    # 显示动画
    def show(self):
        plt.show()

    # 初始化动画的图形元素
    def init_func(self):
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        return self.patches + self.artists

    # 定义每一帧的动画效果
    def animate_func(self, i):
        for agent_name, agent in self.combined_schedule.items():
            pos = self.getState(i / 10, agent)
            # 使用新方法更新智能体的位置
            self.agents[agent_name].set_top_left(pos)

        # reset all colors
        for _, agent in self.agents.items():
            agent.set_facecolor(agent.original_face_color)

        # check drive-drive collisions
        agents_array = [agent for _, agent in self.agents.items()]
        for i in range(0, len(agents_array)):
            for j in range(i + 1, len(agents_array)):
                d1 = agents_array[i]
                d2 = agents_array[j]
                pos1 = np.array(d1.get_xy()) + 0.3  # Getting center position
                pos2 = np.array(d2.get_xy()) + 0.3  # Getting center position
                if np.linalg.norm(pos1 - pos2) < 0.7:
                    d1.set_facecolor('red')
                    d2.set_facecolor('red')
                    print("COLLISION! (agent-agent) ({}, {})".format(i, j))

        return self.patches + self.artists

    # 根据给定的时间和智能体调度，返回智能体的位置
    def getState(self, t, d):
        idx = 0
        while idx < len(d) and d[idx]["t"] < t:
            idx += 1
        if idx == 0:
            return np.array([float(d[0]["top_left_x"]), float(d[0]["top_left_y"])])
        elif idx < len(d):
            posLast = np.array([float(d[idx-1]["top_left_x"]), float(d[idx-1]["top_left_y"])])
            posNext = np.array([float(d[idx]["top_left_x"]), float(d[idx]["top_left_y"])])
        else:
            return np.array([float(d[-1]["top_left_x"]), float(d[-1]["top_left_y"])])
        dt = d[idx]["t"] - d[idx-1]["t"]
        t = (t - d[idx-1]["t"]) / dt
        pos = (posNext - posLast) * t + posLast
        return pos

# 自定义的Rectangle类
class CustomRectangle(Rectangle):
    def __init__(self, xy, width, height, **kwargs):
        super().__init__((xy[0], xy[1]-height), width, height, **kwargs)
    
    def set_top_left(self, xy):
        self.set_xy((xy[0], xy[1]-self.get_height()))





if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="input file containing map (YAML)")
    parser.add_argument("schedule", help="schedule for agents (YAML)")
    args = parser.parse_args()

    with open(args.map) as map_file:
        map_data = yaml.load(map_file, Loader=yaml.FullLoader)

    with open(args.schedule) as states_file:
        schedule = yaml.load(states_file, Loader=yaml.FullLoader)

    # 创建动画实例并展示
    anim = Animation(map_data, schedule)
    anim.show()
