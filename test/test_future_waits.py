import re
import threading
import time
from pathlib import Path

from rclpy.executors import MultiThreadedExecutor


def test_no_internal_spin_once():
    source = (Path(__file__).parent.parent / "pymoveit2" / "moveit2.py").read_text()
    assert not re.search(r"rclpy\.spin_once\(", source)


def test_compute_fk_with_external_executor_times_out_cleanly(rclpy_node, moveit2):
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(rclpy_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        start = time.monotonic()
        result = moveit2.compute_fk(joint_state=[0.0] * 7, timeout_sec=2.0)
        elapsed = time.monotonic() - start
        assert result is None
        assert elapsed < 10.0
    finally:
        executor.shutdown(timeout_sec=2.0)
        executor.remove_node(rclpy_node)
