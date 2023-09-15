import yaml
import random
import math

# Add a custom representer for tuples to display them as inline lists
def represent_tuple(dumper, data):
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data)

# Add the representer to the Dumper
yaml.add_representer(tuple, represent_tuple)

# Function to generate random obstacles
def generate_obstacles(map_dim, num_obstacles):
    obstacles = []
    for _ in range(num_obstacles):
        x = random.randint(0, map_dim-1)
        y = random.randint(0, map_dim-1)
        if [x, y] not in obstacles:
            obstacles.append([x, y])
    return obstacles

class MapGenerator:
    def __init__(self, map_dim, agents_info, obstacles):
        self.map_dim = map_dim
        self.agents_info = agents_info
        self.obstacles = obstacles

    def generate_map(self):
        # Prepare the agent configurations
        agents = [{"name": f"agent{i}",
                   "start_top_left": agent_info["start_top_left"], 
                   "start_bottom_right": [agent_info["start_top_left"][0]+size-1, agent_info["start_top_left"][1]-size+1],
                   "goal_top_left": agent_info["goal_top_left"],
                   "goal_bottom_right": [agent_info["goal_top_left"][0]+size-1, agent_info["goal_top_left"][1]-size+1]}
                  for i, agent_info in enumerate(self.agents_info)]

        # Prepare the map dictionary
        map_dict = {
            "agents": agents,
            "map": {
                "dimensions": [self.map_dim, self.map_dim],
                "obstacles": self.obstacles
            },
            "dyn_obstacles": {}
        }

        return map_dict

    def save_map(self, filename, map_dict):
        # Save the map to a YAML file
        with open(filename, 'w') as file:
            yaml.dump(map_dict, file, default_flow_style=None, sort_keys=False)
        print(f"The map has been successfully created and saved in '{filename}'.")

# Function to generate agent positions based on grid
def generate_agent_positions(map_dim, num_agent, size, safe_distance, obstacles, start, goal_start):
    # Generate start points
    matrix_start = generate_points(start, num_agent, safe_distance)
    
    # Generate goal points
    matrix_goal = generate_points(goal_start, num_agent, safe_distance)
    
    #random.shuffle(matrix_goal)  # Shuffle the goal positions

    agent_positions = []
    for i in range(num_agent):
        agent_positions.append({
            "start_top_left": list(matrix_start[i]),
            "goal_top_left": list(matrix_goal[i])
        })
    return agent_positions

# Function to generate points based on start position, total agents and safe distance
def generate_points(start, total_agents, safe_distance):
    x, y = start[0][0], start[0][1]
    agents_per_row = math.ceil(math.sqrt(total_agents))
    return [(x + j * (safe_distance + 1), y - i * (safe_distance + 1)) 
            for i in range(agents_per_row) 
            for j in range(agents_per_row)][:total_agents]



dim = 10
num_agent = 2  # Total number of agents
size = 2
safe = 2

num_obstacles = 0
# Generate random obstacles
obstacles = generate_obstacles(dim, num_obstacles)

agent_start = [[1, 1]]
goal_start = [[5, 5]]

# Generate agent positions
agents_info = generate_agent_positions(dim, num_agent, size, safe, obstacles, agent_start, goal_start)

# Now, use agents_info in your MapGenerator class
map_gen = MapGenerator(dim, agents_info, obstacles)
map_dict = map_gen.generate_map()
map_gen.save_map('generated_map.yaml', map_dict)