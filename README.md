# A* Search Algorithm Implementation

A clean, efficient Python implementation of the **A* (A-Star)** pathfinding algorithm. This repository includes the core logic and a suite of benchmarks to evaluate performance across different scenarios.

## 🛠 Features
* **Core A\* Logic:** Implemented in `Astar.py` with support for custom heuristics.
* **Benchmarking:** Multiple scripts to test execution time and memory usage.
* **Extensible:** Easy to integrate into grid-based games or robotic path-planning simulations.

## 🚀 Getting Started

### Prerequisites
* Python 3.x

### Installation
1. Clone the repository:
   ```bash
   git clone [https://github.com/Abhinav8899/AStar.git](https://github.com/Abhinav8899/AStar.git)
   cd AStar

## 💡 Usage

You can easily integrate the A* algorithm into your own projects. Here is a basic example of how to define a grid and find a path:

```python
from Astar import astar

# Define a 2D grid: 0 is empty, 1 is a wall (obstacle)
grid = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 1, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0]
]

start = (0, 0) # Top-left
goal = (4, 4)  # Bottom-right

# Find the shortest path
path = astar(grid, start, goal)

if path:
    print(f"Path found! Length: {len(path)} steps")
    print(path)
else:
    print("No path found.")
```

 📊 Performance Benchmarking

This repository includes three pre-configured benchmark scenarios to evaluate the algorithm's performance in 3D environments. These scripts measure **execution time**, **node expansion**, and **path optimality**.

### How to Run
Execute the benchmarks directly from your terminal:

```bash
# Scenario 1: Basic environment with low obstacle density
python benchmarksingle1.py

# Scenario 2: Moderate complexity with intersecting obstacles
python benchmarksingle2.py

# Scenario 3: High-complexity stress test with dense 3D obstacle fields
python benchmarksingle3.py
```

## 🤝 Contributing
Feel free to fork this repository and submit pull requests. For major changes, please open an issue first to discuss what you would like to change.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
