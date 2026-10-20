# CPU and memory of the estimator process, sampled by seb_online/resource_monitor.py
# (psutil, separate process).
#
#   header.stamp   ROS time of the sample
#   cpu_percent    CPU time of the process (all its threads) over the last period,
#                  in % of ONE core: on a multicore system it can exceed 100
#                  (maximum 100 * num_cpus)
#   rss_mb         resident set size [MiB]

std_msgs/Header header
int32 pid
string process_name
float64 cpu_percent
uint32 num_cpus           # logical CPUs, to normalise cpu_percent if needed
float64 rss_mb
uint32 num_threads
