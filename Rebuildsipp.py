from math import fabs
import argparse
from typing import Any
from Rebuildgraph import SippGraph, State
import yaml

import heapq

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]
    

class SippPlanner:
    def __init__(self,map,agent_id):
        self.graph = SippGraph(map)
        self.start_top_left = tuple(map["agents"][agent_id]["start_top_left"])
        self.start_bottom_right = tuple(map["agents"][agent_id]["start_bottom_right"])
        self.goal_top_left = tuple(map["agents"][agent_id]["goal_top_left"])
        self.goal_bottom_right = tuple(map["agents"][agent_id]["goal_bottom_right"])
        self.name = map["agents"][agent_id]["name"]
        self.size=self.start_top_left[1]-self.start_bottom_right[1]+1
        self.open = PriorityQueue()
        self.id = agent_id
        self.starttime =agent_id*3

    def get_successors(self, state):
        successors = []
        m_time = 1
        direction_neighbors = self.graph.get_valid_neighbors(state.top_left, state.bottom_right, self.id)
        """
        print("")
        print("位置：", state.top_left, "的所有邻居：", direction_neighbors)
        """
        for direction, neighbors in direction_neighbors.items():
            # 如果邻居列表为空，跳过这个方向
            if not neighbors:
                continue

            # 判断该方向的所有邻居是否都在安全区间
            all_neighbors_safe = True
            for neighbour in neighbors:
                # 获取该位置的时间区间
                time_intervals = self.graph.sipp_graph[neighbour].time_intervals
                # 检查代理在该位置的当前时间和下一个时间步是否安全
                if not any(interval[0] <= state.time + m_time <= interval[1] for interval in time_intervals):
                    all_neighbors_safe = False
                    break

            # 如果该方向所有邻居都安全，记录这个方向的移动
            if all_neighbors_safe:
                if direction == "up":
                    next_position = (state.top_left[0], state.top_left[1] + 1)
                elif direction == "down":
                    next_position = (state.top_left[0], state.top_left[1] - 1)
                elif direction == "left":
                    next_position = (state.top_left[0] - 1, state.top_left[1])
                else:  # direction == "right"
                    next_position = (state.top_left[0] + 1, state.top_left[1])

                time = state.time + m_time
                s = State(top_left=next_position, bottom_right=(next_position[0] + self.size - 1, next_position[1] - self.size + 1), time=time, agent_id=self.id)
                successors.append(s)
        """
        # 打印所有后继坐标
        print("\n所有后继坐标：", time)
        for suc in successors:
            print(suc.top_left, suc.bottom_right)
        """  
        return successors





    def get_heuristic(self, position):
        return fabs(position[0] - self.goal_top_left[0]) + fabs(position[1]-self.goal_top_left[1])
    
    def compute_plan(self):
        goal_reached = False
        cost = 1

        s_start = State(top_left=self.start_top_left, bottom_right=self.start_bottom_right, time=0, agent_id=self.id)

        self.graph.sipp_graph[s_start.top_left].g = 0
        f_start = self.get_heuristic(self.start_top_left)
        self.graph.sipp_graph[s_start.top_left].f = f_start

        self.open.put(s_start, f_start)

        while (not goal_reached):
            if self.open == []:
                return 0
            
            s = self.open.get()

            successor = self.get_successors(s)

            for successor in successor:
                if self.graph.sipp_graph[successor.top_left].g > self.graph.sipp_graph[s.top_left].g + cost:
                    self.graph.sipp_graph[successor.top_left].g = self.graph.sipp_graph[s.top_left].g + cost
                    self.graph.sipp_graph[successor.top_left].parent_state = s

                    if successor.top_left == self.goal_top_left:
                        goal_reached = True
                        break
                    self.graph.sipp_graph[successor.top_left].f = self.graph.sipp_graph[successor.top_left].g + self.get_heuristic(successor.top_left)
                    self.open.put(successor, self.graph.sipp_graph[successor.top_left].f)

        
        start_reached = False
        self.plan = []
        current = successor


        while not start_reached:
            self.plan.insert(0, current)
            if current.top_left == self.start_top_left:
                start_reached = True
            current = self.graph.sipp_graph[current.top_left].parent_state
        return 1


    def get_plan(self):# 获取计划
        path_list = []

        # first setpoint
        setpoint = self.plan[0]
        temp_dict = {
            "top_left_x": setpoint.top_left[0], 
            "top_left_y": setpoint.top_left[1], 
            "bottom_right_x": setpoint.bottom_right[0], 
            "bottom_right_y": setpoint.bottom_right[1], 
            "t": setpoint.time
        }
        path_list.append(temp_dict)

        for i in range(len(self.plan)-1):
            for j in range(self.plan[i+1].time - self.plan[i].time-1):
                x_tl = self.plan[i].top_left[0]
                y_tl = self.plan[i].top_left[1]
                x_br = self.plan[i].bottom_right[0]
                y_br = self.plan[i].bottom_right[1]
                t = self.plan[i].time
                temp_dict = {
                    "top_left_x": x_tl, 
                    "top_left_y": y_tl, 
                    "bottom_right_x": x_br, 
                    "bottom_right_y": y_br, 
                    "t": t+j+1
                }
                path_list.append(temp_dict)

            setpoint = self.plan[i+1]
            temp_dict = {
                "top_left_x": setpoint.top_left[0], 
                "top_left_y": setpoint.top_left[1], 
                "bottom_right_x": setpoint.bottom_right[0], 
                "bottom_right_y": setpoint.bottom_right[1], 

                "t": setpoint.time
            }
            path_list.append(temp_dict)

        data = {self.name: path_list}
        return data



