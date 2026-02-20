#!/usr/bin/env python3
import math
import time
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

def wrap_to_pi(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    # Make -pi map to +pi for consistency if desired
    if a <= -math.pi:
        a += 2.0 * math.pi
    return a


class ScanFovInfer(Node):
    """
    Subscribes to a LaserScan and infers which angular bins are ever observed (valid range).
    Reports contiguous observed segments and a suggested (angle_min, angle_max) interval with margin.
    """

    def __init__(self):
        super().__init__('scan_fov_infer')

        # Parameters
        self.declare_parameter('scan_topic', '/mid70/merged_scan')
        self.declare_parameter('min_hits_per_bin', 3)      # robustness: require N hits to mark a bin as "observed"
        self.declare_parameter('report_period_s', 1.0)     # how often to print
        self.declare_parameter('margin_rad', 0.10)         # expand inferred limits by this margin (radians)
        self.declare_parameter('min_segment_bins', 5)      # discard tiny segments (noise)
        self.declare_parameter('valid_max_fraction', 0.999)  # treat r >= valid_max_fraction*range_max as invalid
        self.declare_parameter('qos_depth', 10)

        topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        depth = self.get_parameter('qos_depth').get_parameter_value().integer_value

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(LaserScan, topic, self.cb_scan, sensor_qos)

        # State
        self.initialized = False
        self.angle_min = 0.0
        self.angle_inc = 0.0
        self.n = 0
        self.range_min = 0.0
        self.range_max = 0.0

        self.hit_counts: List[int] = []
        self.observed: List[bool] = []

        self.last_report = time.time()

        self.get_logger().info(f"Listening on {topic}")

    def cb_scan(self, msg: LaserScan) -> None:
        # Initialize on first message (or if geometry changes)
        if (not self.initialized) or (self.n != len(msg.ranges)) or (abs(self.angle_inc - msg.angle_increment) > 1e-9) or (abs(self.angle_min - msg.angle_min) > 1e-9):
            self.angle_min = msg.angle_min
            self.angle_inc = msg.angle_increment
            self.n = len(msg.ranges)
            self.range_min = msg.range_min
            self.range_max = msg.range_max

            self.hit_counts = [0] * self.n
            self.observed = [False] * self.n
            self.initialized = True

            self.get_logger().info(
                f"Initialized scan geometry: angle_min={self.angle_min:.4f}, "
                f"angle_max={msg.angle_max:.4f}, inc={self.angle_inc:.6f}, bins={self.n}, "
                f"range_min={self.range_min:.3f}, range_max={self.range_max:.3f}"
            )

        # Validity thresholds
        min_hits = int(self.get_parameter('min_hits_per_bin').value)
        valid_max_frac = float(self.get_parameter('valid_max_fraction').value)
        valid_max = valid_max_frac * self.range_max

        # Update per-bin hit counters
        for i, r in enumerate(msg.ranges):
            # Reject NaNs and infs explicitly
            if not math.isfinite(r):
                continue
            # Valid return: strictly within sensor range
            if (r > self.range_min) and (r < valid_max):
                self.hit_counts[i] += 1
                if (not self.observed[i]) and (self.hit_counts[i] >= min_hits):
                    self.observed[i] = True

        # Periodic reporting
        now = time.time()
        if now - self.last_report >= float(self.get_parameter('report_period_s').value):
            self.last_report = now
            self.report()

    def report(self) -> None:
        if not self.initialized:
            return

        # Extract contiguous observed segments (in index space)
        min_seg_bins = int(self.get_parameter('min_segment_bins').value)
        segments = self._find_segments(self.observed, min_seg_bins)

        if not segments:
            self.get_logger().info("No observed segments yet (drive around near obstacles).")
            return

        # Convert segments to angle ranges
        seg_angles = []
        for (i0, i1) in segments:
            a0 = self.angle_min + i0 * self.angle_inc
            a1 = self.angle_min + i1 * self.angle_inc
            seg_angles.append((a0, a1, i0, i1))

        # Suggested single interval spanning all segments
        global_i0 = min(s[0] for s in segments)
        global_i1 = max(s[1] for s in segments)
        a_min = self.angle_min + global_i0 * self.angle_inc
        a_max = self.angle_min + global_i1 * self.angle_inc

        margin = float(self.get_parameter('margin_rad').value)
        a_min_m = max(self.angle_min, a_min - margin)
        a_max_m = min(self.angle_min + (self.n - 1) * self.angle_inc, a_max + margin)

        # Coverage stats
        observed_bins = sum(1 for x in self.observed if x)
        coverage_pct = 100.0 * observed_bins / float(self.n)

        # Print
        self.get_logger().info(
            f"Observed bins: {observed_bins}/{self.n} ({coverage_pct:.1f}%). "
            f"Segments: {len(seg_angles)}. "
            f"Single-interval suggestion: [{a_min_m:.3f}, {a_max_m:.3f}] rad "
            f"(raw [{a_min:.3f}, {a_max:.3f}], margin={margin:.3f})."
        )

        # Print segments (most useful for diagnosing disjoint coverage)
        for idx, (s0, s1, i0, i1) in enumerate(seg_angles):
            self.get_logger().info(
                f"  segment[{idx}]: idx[{i0},{i1}] angles[{s0:.3f},{s1:.3f}] rad "
                f"({(i1 - i0 + 1)} bins)"
            )

    @staticmethod
    def _find_segments(mask: List[bool], min_len: int) -> List[Tuple[int, int]]:
        """Return contiguous True segments as (start_idx, end_idx), inclusive, with minimum length."""
        segments: List[Tuple[int, int]] = []
        in_seg = False
        start = 0
        for i, v in enumerate(mask):
            if v and not in_seg:
                in_seg = True
                start = i
            elif (not v) and in_seg:
                end = i - 1
                if end - start + 1 >= min_len:
                    segments.append((start, end))
                in_seg = False
        if in_seg:
            end = len(mask) - 1
            if end - start + 1 >= min_len:
                segments.append((start, end))
        return segments


def main():
    rclpy.init()
    node = ScanFovInfer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()