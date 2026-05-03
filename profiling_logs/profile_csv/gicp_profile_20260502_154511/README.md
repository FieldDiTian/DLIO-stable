# GICP Localization Profiling Report

Profiler output directory:

`profiling_logs/profile_csv/gicp_profile_20260502_154511`

This report summarizes the CSV outputs and generated PNG plots from the GICP localization resource profiler. The run captured localization, rosbag playback, robot state publisher, system resources, per-core CPU activity, and ROS scan/debug metrics.

## Run Coverage

| Item | Value |
| --- | ---: |
| Wall-clock start | `2026-05-02T15:45:11.427` |
| Wall-clock end | `2026-05-02T16:11:41.321` |
| Profile duration | `1589.894 s` (`26.50 min`) |
| System samples | `3058` |
| Mean system sample period | `0.520 s` |
| CPU cores observed | `32` |
| Process resource rows | `9137` |
| ROS scan metric rows | `29627` |
| Scan metric active duration | `1567.470 s` (`26.12 min`) |
| Approx. scan metric rate | `18.90 Hz` |
| Lap event rows | `0` |

The raw files are:

- [system_resources.csv](system_resources.csv): host CPU, memory, swap, load, disk, network, CPU frequency, and temperature.
- [cpu_cores.csv](cpu_cores.csv): per-core CPU utilization.
- [process_resources.csv](process_resources.csv): tracked process CPU, memory, threads, context switches, and I/O.
- [ros_scan_metrics.csv](ros_scan_metrics.csv): GICP timing, fitness, convergence, point counts, correspondences, correction/jump metrics, and IMU timing.
- [lap_events.csv](lap_events.csv): no lap rows were captured for this run.
- [summary.csv](summary.csv): profiler-generated min/mean/p50/p95/p99/max table.

## Plot Index

Primary overview plots:

- [system_overview.png](plots/system_overview.png): host CPU, memory, disk, network, CPU frequency, and temperature.
- [process_overview.png](plots/process_overview.png): per-role process CPU, memory, I/O, and thread count.
- [ros_scan_metrics_overview.png](plots/ros_scan_metrics_overview.png): GICP latency, fitness, point counts, correspondence ratio, correction, jump, pose, and IMU timing.
- [cpu_cores_heatmap.png](plots/cpu_cores_heatmap.png): per-core CPU utilization heatmap over elapsed time.
- [summary_statistics.png](plots/summary_statistics.png): top summary metrics by absolute max, with min/mean/p50/p95/p99/max.

Detailed all-column plots:

- [system_resources_all_metrics_01.png](plots/system_resources_all_metrics_01.png), [system_resources_all_metrics_02.png](plots/system_resources_all_metrics_02.png), [system_resources_all_metrics_03.png](plots/system_resources_all_metrics_03.png)
- [process_resources_all_metrics_01.png](plots/process_resources_all_metrics_01.png), [process_resources_all_metrics_02.png](plots/process_resources_all_metrics_02.png)
- [ros_scan_metrics_all_metrics_01.png](plots/ros_scan_metrics_all_metrics_01.png), [ros_scan_metrics_all_metrics_02.png](plots/ros_scan_metrics_all_metrics_02.png), [ros_scan_metrics_all_metrics_03.png](plots/ros_scan_metrics_all_metrics_03.png)
- [cpu_cores_all_metrics.png](plots/cpu_cores_all_metrics.png)

There is no `lap_events.png` because [lap_events.csv](lap_events.csv) has only the header row.

## Executive Summary

The localization stack completed a long profiling capture with stable host-level resource use. The dominant compute consumer was the GICP localization process, which averaged about `365.75%` process CPU, meaning roughly `3.66` full CPU cores. Its p95 CPU was `649.63%` (`6.50` cores), p99 was `1004.23%` (`10.04` cores), and peak was `1395.52%` (`13.96` cores). See [process_overview.png](plots/process_overview.png) and [process_resources_all_metrics_01.png](plots/process_resources_all_metrics_01.png).

Host CPU utilization stayed moderate relative to the 32-core machine: mean `13.09%`, p95 `22.07%`, p99 `33.34%`, and max `45.60%`. CPU temperature rose from the low 40s C into a typical operating band, with mean `62.49 C`, p95 `71.15 C`, p99 `74.00 C`, and max `83.00 C`. See [system_overview.png](plots/system_overview.png), [system_resources_all_metrics_01.png](plots/system_resources_all_metrics_01.png), and [cpu_cores_heatmap.png](plots/cpu_cores_heatmap.png).

GICP scan latency was generally low after startup. The mean `gicp_elapsed_ms` was `8.28 ms`, median `7.21 ms`, p95 `14.56 ms`, and p99 `29.79 ms`. The max was `2180.12 ms`, which appears to be the first measured GICP alignment and should be treated as a startup outlier. Excluding that first GICP timing row, the max was `75.07 ms`, mean `8.20 ms`, p95 `14.56 ms`, and p99 `29.79 ms`. See [ros_scan_metrics_overview.png](plots/ros_scan_metrics_overview.png) and [ros_scan_metrics_all_metrics_01.png](plots/ros_scan_metrics_all_metrics_01.png).

Memory usage was stable. Host memory use averaged `9773.75 MB` (`7.60%` of total memory), with p95 `9878.85 MB` and max `10090.02 MB`. The localization process RSS averaged `3250.34 MB`, p95 `3327.19 MB`, and max `3362.20 MB`. Localization PSS averaged `3237.30 MB`, p95 `3314.18 MB`, and max `3349.12 MB`. See [system_overview.png](plots/system_overview.png), [process_overview.png](plots/process_overview.png), and [process_resources_all_metrics_02.png](plots/process_resources_all_metrics_02.png).

The rosbag player was not CPU-heavy, but it was the main disk reader. It averaged `16.85%` process CPU, or about `0.17` CPU cores, with p95 `20.07%` and max `49.90%`. Its read throughput averaged `121.49 MB/s`, p95 `139.99 MB/s`, and max `173.37 MB/s`. System disk read throughput averaged `240.06 MB/s`, p95 `278.18 MB/s`, and max `339.97 MB/s`. See [process_overview.png](plots/process_overview.png), [system_overview.png](plots/system_overview.png), and [system_resources_all_metrics_02.png](plots/system_resources_all_metrics_02.png).

## Process-Level Results

| Role | Rows | PID | CPU mean | CPU p95 | CPU p99 | CPU max | RSS mean | RSS p95 | RSS max | Threads max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| localization | `3058` | `119020` | `365.87%` | `649.67%` | `1004.24%` | `1395.52%` | `3250.34 MB` | `3327.19 MB` | `3362.20 MB` | `1038` |
| rosbag_play | `3021` | `119669` | `16.85%` | `20.07%` | `22.28%` | `49.90%` | `310.25 MB` | `348.61 MB` | `354.47 MB` | `16` |
| robot_state_publisher | `3058` | `119018` | `0.01%` | `0.00%` | `0.00%` | `2.00%` | `20.79 MB` | `21.18 MB` | `21.24 MB` | `12` |

The localization process is the main sizing target. The measured CPU demand means the localization workload should be budgeted for approximately `4` CPU cores on average, with burst capacity closer to `7-10` cores for p95/p99 behavior. The absolute maximum reached nearly `14` cores. The high thread count is also visible in [process_overview.png](plots/process_overview.png), and should be considered if deploying on machines with strict thread limits or CPU affinity rules.

## System Resource Results

| Metric | Mean | p50 | p95 | p99 | Max |
| --- | ---: | ---: | ---: | ---: | ---: |
| Host CPU | `13.09%` | `12.51%` | `22.07%` | `33.34%` | `45.60%` |
| Host memory used | `9773.75 MB` | `9798.86 MB` | `9878.85 MB` | `9909.60 MB` | `10090.02 MB` |
| Host memory used percent | `7.60%` | `7.62%` | `7.68%` | `7.71%` | `7.85%` |
| Swap used percent | `0.02%` | `0.00%` | `0.06%` | `0.06%` | `0.06%` |
| Disk read | `240.06 MB/s` | `244.38 MB/s` | `278.18 MB/s` | `281.75 MB/s` | `339.97 MB/s` |
| Disk write | `0.17 MB/s` | `0.00 MB/s` | `1.28 MB/s` | `1.96 MB/s` | `3.22 MB/s` |
| Network RX | `0.001 MB/s` | `0.001 MB/s` | `0.003 MB/s` | `0.012 MB/s` | `0.181 MB/s` |
| Network TX | `0.021 MB/s` | `0.015 MB/s` | `0.030 MB/s` | `0.171 MB/s` | `0.524 MB/s` |
| CPU temperature | `62.49 C` | `64.00 C` | `71.15 C` | `74.00 C` | `83.00 C` |

The system-level plots show that this run was not memory constrained and did not use swap meaningfully. Disk read traffic was substantial because the rosbag was being streamed from storage. Network traffic was negligible. The per-core heatmap shows the workload spread across cores rather than pinning a single core. The busiest cores by mean utilization were `cpu10` (`19.37%` mean), `cpu8` (`18.83%` mean), `cpu14` (`15.45%` mean), `cpu12` (`15.28%` mean), and `cpu6` (`14.49%` mean). See [cpu_cores_heatmap.png](plots/cpu_cores_heatmap.png) and [cpu_cores_all_metrics.png](plots/cpu_cores_all_metrics.png).

## ROS/GICP Scan Metrics

| Metric | Count | Mean | p50 | p95 | p99 | Max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `gicp_elapsed_ms` | `29626` | `8.28 ms` | `7.21 ms` | `14.56 ms` | `29.79 ms` | `2180.12 ms` |
| `fitness` | `29627` | `0.240` | `0.024` | `0.178` | `9.379` | `23.047` |
| `scan_dt_s` | `29626` | `0.0529 s` | `0.0500 s` | `0.0500 s` | `0.1000 s` | `2.5000 s` |
| `raw_points` | `29626` | `106266` | `108681` | `132435` | `140233` | `143896` |
| `preprocessed_points` | `29626` | `12169` | `12055` | `18628` | `19930` | `21841` |
| `num_correspondences` | `29626` | `12163` | `12060` | `18603` | `19901` | `21841` |
| `correspondence_ratio` | `29626` | `0.9994` | `1.0000` | `1.0000` | `1.0000` | `1.0000` |
| `final_error` | `29626` | `8427.01` | `5456.23` | `21476.93` | `48226.96` | `368403.54` |
| `guess_to_solution_trans_m` | `29626` | `0.141 m` | `0.074 m` | `0.492 m` | `1.172 m` | `12.241 m` |
| `guess_to_solution_rot_deg` | `29626` | `0.184 deg` | `0.143 deg` | `0.428 deg` | `0.847 deg` | `6.427 deg` |
| `jump_trans_m` | `29626` | `0.141 m` | `0.074 m` | `0.492 m` | `1.171 m` | `12.241 m` |
| `jump_rot_deg` | `29626` | `0.184 deg` | `0.143 deg` | `0.429 deg` | `0.850 deg` | `6.427 deg` |
| `imu_buffer_span_s` | `29626` | `15.941 s` | `15.999 s` | `16.023 s` | `16.032 s` | `16.046 s` |
| `scan_to_latest_imu_lag_s` | `29626` | `-0.0201 s` | `-0.0173 s` | `-0.0105 s` | `-0.0066 s` | `0.0827 s` |

The scan metrics indicate that normal GICP runtime stayed well under the nominal `50 ms` scan interval for most of the run. p95 latency was `14.56 ms`, leaving substantial margin at a 20 Hz scan cadence. The p99 latency of `29.79 ms` is still below `50 ms`. The one `2180.12 ms` max event is a startup outlier; the next-worst GICP timing after removing the first timing row was `75.07 ms`.

Fitness stayed below the earlier `25.0` rejection threshold throughout this CSV capture. There were `986` scans with fitness greater than `1.0`, `271` scans greater than `10.0`, and `0` scans greater than `25.0`. The `converged` field was `true` for `29626` rows; one initial row had no convergence value. See [ros_scan_metrics_overview.png](plots/ros_scan_metrics_overview.png), [ros_scan_metrics_all_metrics_01.png](plots/ros_scan_metrics_all_metrics_01.png), [ros_scan_metrics_all_metrics_02.png](plots/ros_scan_metrics_all_metrics_02.png), and [ros_scan_metrics_all_metrics_03.png](plots/ros_scan_metrics_all_metrics_03.png).

## Data Quality Notes

1. `ros_time_s` is empty in all CSVs. The profiler likely did not receive `/clock` because of the ROS clock QoS mismatch seen in earlier logs. The plots therefore use `elapsed_s` as the x-axis.
2. [lap_events.csv](lap_events.csv) has no lap rows. This means the profiler did not record lap returns during this run, even though scan and process data were captured.
3. The first GICP timing row with a non-empty `gicp_elapsed_ms` is a large startup outlier (`2180.12 ms`). For steady-state latency, use p95/p99 or the "excluding first timing row" values above.
4. Process CPU percentages are Linux process percentages where `100%` is roughly one fully used CPU core. For example, localization mean `365.87%` means about `3.66` CPU cores.
5. Host CPU percentages are normalized against the full 32-core system. A host CPU value of `13%` on this machine is compatible with one process using several cores.

## How To Regenerate Plots

From the repository root:

```bash
source install/setup.bash
ros2 run gicp_localization plot_localization_profile.py \
  /home/roar-blacktank/DLIO_plusplus/profiling_logs/profile_csv/gicp_profile_20260502_154511
```

Or point the script at the parent directory to plot the latest profile run:

```bash
source install/setup.bash
ros2 run gicp_localization plot_localization_profile.py \
  /home/roar-blacktank/DLIO_plusplus/profiling_logs/profile_csv
```
