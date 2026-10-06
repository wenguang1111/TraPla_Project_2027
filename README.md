# TraPla Project 2027

This repository provides a scalar single-thread Frenet Optimal Planning baseline and a CommonRoad test environment.

## Requirements

- Ubuntu 22.04 or 24.04
- Python 3.10 or 3.11
- Poetry
- CMake 3.16 or newer
- C++17 compiler

## Clone

```bash
git clone --recurse-submodules https://github.com/wenguang1111/TraPla_Project_2027.git
cd TraPla_Project_2027
```

For an existing clone:

```bash
git submodule update --init --recursive
```

## Python environment

```bash
poetry env use python3.10
poetry install
```

## C++ build

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

## Run CommonRoad scenarios

```bash
python scripts/run_commonroad.py
```

The configuration file controls the scenario list, sampling resolution, vehicle parameters, target speed, and output directory. When `SAVE_PLOT` is `true`, the runner generates an animated `result.gif` for each scenario.

The CommonRoad runner loads each XML scenario, computes a global reference route, projects the initial state into the Frenet frame, prepares obstacle polygons over the planning horizon, and executes the scalar C++ planner.

Results are written to:

```text
data/output/<scenario_name>/
```

Each result directory contains:

```text
scenario_input.txt
trajectories.csv
best_trajectory.csv
result.gif
```

## Project structure

```text
cfgs/
data/scenarios/
include/fop/
scripts/
src/
third_party/commonroad-io/
```

`third_party/commonroad-io` is pinned as a Git submodule to the CommonRoad 2024.1 source version. The runtime Python dependencies are installed through Poetry.
