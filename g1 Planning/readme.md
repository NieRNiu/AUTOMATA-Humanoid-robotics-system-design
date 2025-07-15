# Planning

## Introduction
This project integrates navigation maps, an A* global path planning algorithm, and a Dynamic Window Approach (DWA) local planning algorithm to enable fully autonomous humanoid robot navigation.

---

## Navigation Map Generation

### 1. Point Cloud Map Processing
- Use tools such as CloudCompare to manually edit the point cloud map and align it horizontally with the ground plane.
- Save the processed map as a `.pcd` file.

### 2. Generate 2D Grid Map
- Use the script `pcd2pgm.py` to convert the `.pcd` file into a `.pgm` format 2D occupancy grid map for global path planning.

---

## Global Path Planning — A*

### Heuristic Function
- Use Euclidean distance (L2 norm) as the heuristic function `h(n)`.

### Neighbor Node Expansion
- Expand neighboring nodes and calculate the cost for each, iteratively constructing the optimal global path.

#### 💻 Code Module: 
```python
import heapq
import math

def heuristic(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])

def astar(grid, start, goal):
    neighbors = [(0,1),(1,0),(0,-1),(-1,0)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data[::-1]

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j            
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < grid.shape[0]:
                if 0 <= neighbor[1] < grid.shape[1]:
                    if grid[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    continue
            else:
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if  tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))

    return False
```
---
## Local Path Planning — DWA (Dynamic Window Approach)

### Principle
- The Dynamic Window Approach (DWA) generates a set of possible trajectories based on the robot's dynamic and kinematic constraints, and evaluates each trajectory.

### Scoring Mechanism
- Inspired by course references, the scoring function combines multiple weighted criteria, such as distance to the goal, clearance from obstacles, and velocity. The trajectory with the highest score is selected as the local path.

#### 💻 Code Module: 

---

## ROS Integration
This system can be integrated within the ROS (Robot Operating System) framework to execute perception, localization, global planning, and control in a modular and scalable way.

---

## License
This project is licensed under the MIT License.

---

## Contact
If you have questions or would like to contribute, please create an issue or contact the maintainers directly.
