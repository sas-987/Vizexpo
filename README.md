# AGV Task 4: Autonomous Navigation with LiDAR-Style DFS Exploration (ViZDoom)

An autonomous navigation module that explores an unknown maze in [ViZDoom](https://vizdoom.cs.put.edu.pl/) and finds a goal object using only on-board sensing: the agent's pose and the depth buffer. No map is given in advance. The agent builds its own topological map while it explores.

## Highlights

- **Simulated LiDAR from a depth buffer.** The agent rotates to exact compass headings and samples the centre depth column, giving a 360° scan at 10° resolution (36 rays per scan).
- **Depth-first exploration over a node graph.** Each scan creates a node holding its ray depths and the headings already tried. The agent always tries the deepest untried heading first.
- **Wall probing.** Before committing to a heading, the agent pushes forward a few tics and measures real displacement, so thin gaps and false openings are rejected.
- **Junction, side-opening and room-centre detection.** While driving, the agent samples left and right depth every 80 world units to catch side corridors and rooms, and stops at the centre of open rooms to scan.
- **Loop guard and dead-end handling.** Nodes too close to existing ones are skipped, and cramped alcoves are rejected as dead rooms.
- **A\* backtracking.** At a dead end, the agent plans the shortest path back through the explored node graph instead of retracing blindly.
- **Goal detection.** Objects reported by the engine are checked for the blue key objects (`BlueSkull`, `BlueCard`, `BlueSkulltag`), and exploration stops once the agent is within 100 world units of the goal.

## How it works

```
spawn
  │
  ▼
360° LiDAR scan ──► create root node (depths per heading)
  │
  ▼
DFS loop
  ├─ pick deepest untried heading at the current node
  ├─ wall probe ─── wall? ─► mark tried, return, try next heading
  ├─ drive_to_max: drive along the heading, sampling depth
  │     ├─ side opening  → record a checkpoint (scanned lazily on visit)
  │     ├─ open room     → stop at the room centre
  │     ├─ junction      → stop when forward depth stays low
  │     └─ wall          → stop when displacement collapses
  ├─ scan at the terminal position
  ├─ cramped? ─► treat as a dead room, return
  ├─ add checkpoint and terminal nodes to the graph, push onto the DFS stack
  └─ no untried headings ─► dead end ─► A* back to the parent node
  │
  ▼
goal within GOAL_RADIUS ─► done
```

### Key components

| Component | Purpose |
|---|---|
| `lidar_scan` | Turns to absolute headings 0°, 10°, … 350° and reads the forward depth at each. |
| `Node` / `new_node` | Graph node holding position, per-heading depths, tried headings and parent link. |
| `probe_wall` | Pushes forward and measures displacement to confirm the heading is passable. |
| `drive_to_max` | Drives along a heading with heading correction, side sampling, junction and wall detection. |
| `navigate_to` / `fast_navigate_to` | Waypoint following with heading alignment and an unstick wiggle. |
| `astar_nodes` | A\* over the node graph for backtracking. |
| `explore` | The main DFS loop tying everything together. |

## Repository structure

| File | Description |
|---|---|
| `viz2_3(ok).py` | Main LiDAR-DFS explorer (the version this README describes). |
| `viz.py` | Level 1 baseline: plans a path with RRT\* on an occupancy grid built from the map's sector data, then follows it. |
| `MAP01.wad` | The maze scenario (map `MAP01`). |
| `_vizdoom.ini` | ViZDoom configuration. |

## Two approaches in this repo

| | `viz.py` (known map) | `viz2_3(ok).py` (unknown map) |
|---|---|---|
| Map knowledge | Builds an occupancy grid from the engine's sector and line data | None. Builds a node graph from its own scans |
| Planning | RRT\* with goal bias, then line-of-sight path smoothing | DFS exploration with A\* backtracking |
| Sensing | Pose and map geometry | Pose and depth buffer (simulated LiDAR) |
| Control | Waypoint following with heading alignment and stuck recovery | Wall probing, junction detection and waypoint following |

`viz.py` gives a planned-path baseline when the map is available. `viz2_3(ok).py` is the harder case, where the agent has to find the goal without any map.

## Setup

Requirements: Python 3.8+, `numpy` and `vizdoom`. `opencv-python` or `scipy` are optional and only speed up obstacle inflation in `viz.py`.

```bash
pip install vizdoom numpy
```

## Run

```bash
# Unknown-map explorer (LiDAR-DFS)
python "viz2_3(ok).py" MAP01.wad

# Known-map baseline (RRT* planner)
python viz.py MAP01.wad
```

If no path is given, the script looks for `MAP01.wad` in the current directory. A ViZDoom window opens and the agent explores on its own. The console logs each decision (`[SCAN]`, `[GO]`, `[WALL]`, `[SIDE]`, `[ROOM]`, `[NODE]`, `[DEAD END]`, `[A*]`) and prints a summary on completion: result, elapsed time and number of nodes created.

## Tuning

All parameters are constants at the top of the script.

| Parameter | Default | Meaning |
|---|---|---|
| `SCAN_STEP_DEG` | 10 | Angular resolution of each LiDAR sweep. |
| `DEPTH_DROP_THRESH` | 60 | Stop when forward depth falls below this. |
| `MIN_DEPTH_TO_EXPLORE` | 35 | Minimum depth for a heading to count as a candidate. |
| `MIN_NODE_SEPARATION` | 250 | Minimum travel before a terminal node may be created. |
| `CHECKPOINT_INTERVAL_WU` | 80 | Distance between side-depth samples while driving. |
| `ROOM_SIDE_THRESH` | 100 | Both sides deeper than this means an open room. |
| `REVISIT_RADIUS` | 130 | Loop guard radius around existing nodes. |
| `CRAMPED_DEPTH_THRESH` | 60 | Max depth below this means a dead alcove. |
| `GOAL_RADIUS` | 100 | Success radius around the goal. |
| `RENDER_DELAY` | 0.05 | Seconds per action (controls playback speed). |

Keep `MIN_DEPTH_TO_EXPLORE` at or below `DEPTH_DROP_THRESH`. The agent stops when forward depth drops under `DEPTH_DROP_THRESH`, so a higher explore threshold would discard the forward heading as a candidate right after stopping.

## Limitations and future work

- Exploration is depth-first, so it finds a goal but not necessarily the shortest route to it.
- Each LiDAR sweep needs the agent to rotate in place, which is slower than a true spinning sensor.
- Wall detection depends on displacement thresholds tuned for this map.
- Possible extensions: frontier-based exploration, an occupancy grid, and path smoothing.
