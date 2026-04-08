# anhc Benchmark Report

**Generated:** 2026-04-08T10:16:37  

**Source CSV:** `/home/anhuynh/anhc_botrl/bench_results/benchmark_20260408_101341.csv`  

**Total trials:** 26  


## Algorithm Comparison

| Metric | astar | dijkstra |
|--------|--------|--------|
| **Planning Time (ms)** | 27.339 ± 11.891 | 26.525 ± 13.198 |
| **Path Length (m)** | 1.875 ± 3.905 | 0.005 ± 0.006 |
| **Path Smoothness (rad)** | 0.000 ± 0.000 | 0.000 ± 0.000 |
| **Nodes Expanded** | 508.692 ± 222.498 | 497.615 ± 271.429 |
| **Execution Time (s)** | 1.974 ± 4.049 | 0.000 ± 0.000 |
| **Avg Clearance (m)** | -1.000 ± 0.000 | -1.000 ± 0.000 |
| **CPU Usage (%)** | 0.000 ± 0.000 | 2.562 ± 9.236 |
| **Memory (MB)** | 82.875 ± 1.333 | 83.482 ± 0.452 |
| **Success Rate** | 92.31% | 100.00% |
| **Collision Rate** | 0.00% | 0.00% |


## Per-Scenario Summary

### long_path_with_obstacles

Trials: 10 | Algorithms: astar, dijkstra


| Metric | astar | dijkstra |
|--------|--------|--------|
| **Planning Time (ms)** | 22.720 ± 12.339 | 27.160 ± 15.079 |
| **Path Length (m)** | 1.625 ± 3.608 | 0.012 ± 0.000 |
| **Path Smoothness (rad)** | 0.000 ± 0.000 | 0.000 ± 0.000 |
| **Nodes Expanded** | 617.200 ± 283.955 | 601.400 ± 233.100 |
| **Execution Time (s)** | 1.788 ± 3.997 | 0.000 ± 0.000 |
| **Avg Clearance (m)** | -1.000 ± 0.000 | -1.000 ± 0.000 |
| **CPU Usage (%)** | 0.000 ± 0.000 | 0.000 ± 0.000 |
| **Memory (MB)** | 83.464 ± 0.173 | 83.688 ± 0.070 |


### narrow_corridor

Trials: 6 | Algorithms: astar, dijkstra


| Metric | astar | dijkstra |
|--------|--------|--------|
| **Planning Time (ms)** | 27.913 ± 14.936 | 30.687 ± 16.933 |
| **Path Length (m)** | 4.030 ± 6.981 | 0.000 ± 0.000 |
| **Path Smoothness (rad)** | 0.000 ± 0.000 | 0.000 ± 0.000 |
| **Nodes Expanded** | 416.667 ± 157.240 | 331.000 ± 135.632 |
| **Execution Time (s)** | 4.027 ± 6.975 | 0.000 ± 0.000 |
| **Avg Clearance (m)** | -1.000 ± 0.000 | -1.000 ± 0.000 |
| **CPU Usage (%)** | 0.000 ± 0.000 | 0.000 ± 0.000 |
| **Memory (MB)** | 83.969 ± 0.000 | 83.969 ± 0.000 |


### short_path_clear

Trials: 10 | Algorithms: astar, dijkstra


| Metric | astar | dijkstra |
|--------|--------|--------|
| **Planning Time (ms)** | 31.614 ± 10.520 | 23.394 ± 11.229 |
| **Path Length (m)** | 0.831 ± 1.857 | 0.000 ± 0.000 |
| **Path Smoothness (rad)** | 0.000 ± 0.000 | 0.000 ± 0.000 |
| **Nodes Expanded** | 455.400 ± 175.796 | 493.800 ± 350.977 |
| **Execution Time (s)** | 0.928 ± 2.075 | 0.000 ± 0.000 |
| **Avg Clearance (m)** | -1.000 ± 0.000 | -1.000 ± 0.000 |
| **CPU Usage (%)** | 0.000 ± 0.000 | 6.660 ± 14.892 |
| **Memory (MB)** | 81.630 ± 1.426 | 82.984 ± 0.260 |


## Raw Data Sample (first 10 rows)

```

algorithm_name         map_name  start_x  start_y  goal_x  goal_y  planning_time_ms  path_length_m  path_smoothness  nodes_expanded  success  collision  execution_time_s  clearance_avg_m  cpu_usage_percent  memory_mb
         astar short_path_clear      0.0      0.0     4.0     0.0             22.99          4.153              0.0             220     True      False          4.639460             -1.0                0.0  79.246094
         astar short_path_clear      0.0      0.0     4.0     0.0             36.51          0.000              0.0             632     True      False          0.000008             -1.0                0.0  81.460938
         astar short_path_clear      0.0      0.0     4.0     0.0             40.19          0.000              0.0             351     True      False          0.000003             -1.0                0.0  82.070312
         astar short_path_clear      0.0      0.0     4.0     0.0             40.57          0.000              0.0             457     True      False          0.000023             -1.0                0.0  82.679688
         astar short_path_clear      0.0      0.0     4.0     0.0             17.81          0.000              0.0             617     True      False          0.000009             -1.0                0.0  82.691406
      dijkstra short_path_clear      0.0      0.0     4.0     0.0             15.06          0.000              0.0             153     True      False          0.000038             -1.0                0.0  82.691406
      dijkstra short_path_clear      0.0      0.0     4.0     0.0              9.31          0.000              0.0             891     True      False          0.000039             -1.0                0.0  82.707031
      dijkstra short_path_clear      0.0      0.0     4.0     0.0             27.81          0.000              0.0             288     True      False          0.000003             -1.0               33.3  83.171875
      dijkstra short_path_clear      0.0      0.0     4.0     0.0             37.64          0.000              0.0             856     True      False          0.000009             -1.0                0.0  83.171875
      dijkstra short_path_clear      0.0      0.0     4.0     0.0             27.15          0.000              0.0             281     True      False          0.000004             -1.0                0.0  83.175781

```
