#!/usr/bin/env python3
"""CPU and memory of the estimator process, published on /benchmark/resources.

Runs as its own process and samples the estimator process (found by executable
name, never this monitor) with psutil every `period_s` (default 0.5 s = 2 Hz).

cpu_percent is psutil's Process.cpu_percent(): CPU time of all the threads of the
process over the last period, in % of ONE core. On a multicore system it can
exceed 100 (up to 100 * num_cpus); divide by num_cpus for the share of the machine.

Nothing is published until the process is found and a first CPU interval has
elapsed (the first psutil reading has no reference and is meaningless).

  ros2 run seb_online resource_monitor.py
  ros2 run seb_online resource_monitor.py --ros-args -p process_name:=online_estimator_node -p period_s:=0.5
"""

import os

import psutil
import rclpy
from rclpy.node import Node
from seb_interfaces.msg import ResourceUsage


class ResourceMonitor(Node):
    def __init__(self):
        super().__init__("seb_resource_monitor")
        self.process_name = self.declare_parameter("process_name", "online_estimator_node").value
        topic = self.declare_parameter("topic", "/benchmark/resources").value
        period = float(self.declare_parameter("period_s", 0.5).value)
        self.pub = self.create_publisher(ResourceUsage, topic, 10)
        self.proc = None
        self.num_cpus = psutil.cpu_count(logical=True) or 0
        self.timer = self.create_timer(period, self.sample)
        self.get_logger().info(f"Monitoring '{self.process_name}' on {topic} every {period} s")

    def find_process(self):
        """The estimator process: its executable (argv[0]) is named process_name."""
        me = os.getpid()
        found = []
        for p in psutil.process_iter(["pid", "cmdline"]):
            cmd = p.info["cmdline"] or []
            if p.info["pid"] != me and cmd and os.path.basename(cmd[0]) == self.process_name:
                found.append(p)
        if len(found) > 1:
            self.get_logger().warn(
                f"{len(found)} processes named '{self.process_name}' "
                f"(pids {[p.pid for p in found]}): monitoring none", throttle_duration_sec=5.0)
            return None
        return found[0] if found else None

    def sample(self):
        if self.proc is None:
            self.proc = self.find_process()
            if self.proc is None:
                self.get_logger().warn(f"'{self.process_name}' not running", throttle_duration_sec=5.0)
                return
            self.proc.cpu_percent(None)  # reference point; the next call measures this period
            self.get_logger().info(f"Found '{self.process_name}' (pid {self.proc.pid})")
            return
        try:
            with self.proc.oneshot():
                cpu = self.proc.cpu_percent(None)
                rss = self.proc.memory_info().rss
                threads = self.proc.num_threads()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            self.get_logger().warn(f"'{self.process_name}' (pid {self.proc.pid}) ended")
            self.proc = None
            return
        msg = ResourceUsage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pid = self.proc.pid
        msg.process_name = self.process_name
        msg.cpu_percent = float(cpu)
        msg.num_cpus = self.num_cpus
        msg.rss_mb = rss / (1024.0 * 1024.0)
        msg.num_threads = threads
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ResourceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
