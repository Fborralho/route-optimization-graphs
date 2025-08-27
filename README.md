# Route Optimization Tool

This project implements a **route planning system** using graph algorithms, designed to optimize travel across an urban network. The tool supports multimodal paths (driving + walking), environmentally friendly routing, and dynamic exclusion of roads.

## Features
- Import locations and road networks from `.csv` or `.txt` files.
- Graph representation with nodes (locations) and edges (roads).
- **Dijkstra’s Algorithm** for shortest paths.
- Environmentally friendly routes: combination of walking and driving.
- Support for blocked roads and dynamic exclusion without rebuilding the graph.
- Multimodal routes using parking nodes (drive + walk).
- Alternative path suggestions.

##  Tech Stack
- Language: **C++**
- Data Structures: Graphs, Priority Queues
- Algorithms: Dijkstra, Pathfinding, Multimodal Routing

## Input & Output
- Reads input constraints from `input.txt`  
- Writes optimal path(s) to `output.txt`

## Complexity
- Dijkstra: `O((N + M) log N)`  
- Environmentally friendly routing: `O(N * (N + M) log N)`

##  User Interface
- Simple menu-driven interaction via text file input/output:
  - Normal planning
  - Planning with restrictions
  - Driving + walking planning

## Authors
- Francisco Rafael dos Santos Borralho  
- Gonçalo Tavares de Pinho Mendes Calvo
