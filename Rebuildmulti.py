"""
实现多智能体sipp算法
"""
from Rebuildsipp import SippPlanner
import argparse
import yaml
import time  # 1. 导入time模块

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="包含地图和动态障碍物的输入文件")
    parser.add_argument("output", help="带有调度信息的输出文件")
    args = parser.parse_args()
    
    # 1. 加载地图文件
    try:
        with open(args.map, 'r') as map_file:
            map_data = yaml.load(map_file, Loader=yaml.FullLoader)
    except yaml.YAMLError as exc:
        print("Error loading the YAML file:", exc)
        return

    # 初始化输出字典
    output = {"schedule": {}}

    start_time = time.time()  # 2. 记录开始时间
    
    # 为每个智能体计算路径
    for i in range(len(map_data["agents"])):
        planner = SippPlanner(map_data, i)
        plan_found = planner.compute_plan()
        

        if plan_found:
            plan = planner.get_plan()
            output["schedule"].update(plan)
            map_data["dyn_obstacles"].update(plan)
        else: 
            print(f"Plan not found for agent {i}")
    end_time = time.time()  # 3. 记录结束时间

    # 将计划写入输出文件
    with open(args.output, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)  

    print(f"Total execution time: {end_time - start_time} seconds")  # 4. 打印算法的执行时间

if __name__ == '__main__':
    main()
