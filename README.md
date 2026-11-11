# Route Optimization — Graph Algorithms in C++

Academic C++ project for finding routes through a road network, including driving restrictions and combined driving/walking journeys.

## Features

- Represents locations and bidirectional roads as a graph with driving and walking costs.
- Uses **Dijkstra's shortest-path algorithm** for route planning.
- Supports excluded nodes and road segments in restricted-route queries.
- Computes **drive-and-walk** routes through eligible parking locations subject to a maximum walking time, with alternative suggestions when constraints cannot be met.
- Reads network data and route constraints from text files and writes route results to `output.txt`.

## Implementation

| File | Responsibility |
| --- | --- |
| [Graph.h](Graph.h) / [Graph.cpp](Graph.cpp) | Graph data structures, Dijkstra and combined driving/walking routing |
| [FileManager.h](FileManager.h) / [FileManager.cpp](FileManager.cpp) | Import road-network data, parse route requests and export results |
| [Menu.cpp](Menu.cpp) | Command-line menu and application entry point |
| [CMakeLists.txt](CMakeLists.txt) | C++17 build definition |

**Core concepts:** weighted graphs, adjacency lists, shortest paths, priority queues, constrained routing, algorithmic complexity.

The source documents typical Dijkstra complexity as `O((V + E) log V)` and the combined routing approach as `O(V (V + E) log V)`.

## Build

Requirements: **C++17 compiler** and **CMake 3.30 or newer**.

```bash
cmake -S . -B build
cmake --build build
```

The configured executable is named `MainProject`. The program currently loads `LocSample.txt` and `DisSample.txt` from its working directory and reads `input.txt` to process requests.

**These input files are not included in this public snapshot**, so building the code is not sufficient to run a route example out of the box.

## Limitations

- Local road-network datasets and route input examples are not included in this repository.
- There is no published automated test suite in this snapshot.
- Build and runtime execution have not been independently verified as part of this README update.

## Authors

- Francisco Rafael dos Santos Borralho
- Gonçalo Tavares de Pinho Mendes Calvo
