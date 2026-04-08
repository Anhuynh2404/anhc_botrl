# anhc Benchmark Report

**Generated:** 2026-04-08T09:51:39  

**Source CSV:** `/home/anhuynh/anhc_botrl/bench_results/benchmark_20260408_094910.csv`  

**Total trials:** 4  


## Algorithm Comparison

| Metric | astar | dijkstra |
|--------|--------|--------|
| **Planning Time (ms)** | 18.745 ± 8.803 | 14.795 ± 9.143 |
| **Path Length (m)** | 2.212 ± 3.128 | 0.000 ± 0.000 |
| **Path Smoothness (rad)** | 0.000 ± 0.000 | 0.000 ± 0.000 |
| **Nodes Expanded** | 572.000 ± 428.507 | 418.500 ± 102.531 |
| **Execution Time (s)** | 2.485 ± 3.514 | 0.000 ± 0.000 |
| **Avg Clearance (m)** | -1.000 ± 0.000 | -1.000 ± 0.000 |
| **CPU Usage (%)** | 0.000 ± 0.000 | 0.000 ± 0.000 |
| **Memory (MB)** | 80.777 ± 1.944 | 83.461 ± 0.326 |
| **Success Rate** | 100.00% | 100.00% |
| **Collision Rate** | 0.00% | 0.00% |


## Per-Scenario Summary

### short_path_clear

Trials: 4 | Algorithms: astar, dijkstra


| Metric | astar | dijkstra |
|--------|--------|--------|
| **Planning Time (ms)** | 18.745 ± 8.803 | 14.795 ± 9.143 |
| **Path Length (m)** | 2.212 ± 3.128 | 0.000 ± 0.000 |
| **Path Smoothness (rad)** | 0.000 ± 0.000 | 0.000 ± 0.000 |
| **Nodes Expanded** | 572.000 ± 428.507 | 418.500 ± 102.531 |
| **Execution Time (s)** | 2.485 ± 3.514 | 0.000 ± 0.000 |
| **Avg Clearance (m)** | -1.000 ± 0.000 | -1.000 ± 0.000 |
| **CPU Usage (%)** | 0.000 ± 0.000 | 0.000 ± 0.000 |
| **Memory (MB)** | 80.777 ± 1.944 | 83.461 ± 0.326 |


## Raw Data Sample (first 10 rows)

```

algorithm_name         map_name  start_x  start_y  goal_x  goal_y  planning_time_ms  path_length_m  path_smoothness  nodes_expanded  success  collision  execution_time_s  clearance_avg_m  cpu_usage_percent  memory_mb
         astar short_path_clear      0.0      0.0     4.0     0.0             24.97          4.424              0.0             269     True      False          4.969430             -1.0                0.0  79.402344
         astar short_path_clear      0.0      0.0     4.0     0.0             12.52          0.000              0.0             875     True      False          0.000004             -1.0                0.0  82.152344
      dijkstra short_path_clear      0.0      0.0     4.0     0.0              8.33          0.000              0.0             491     True      False          0.000003             -1.0                0.0  83.230469
      dijkstra short_path_clear      0.0      0.0     4.0     0.0             21.26          0.000              0.0             346     True      False          0.000006             -1.0                0.0  83.691406

```
