"""Run manually after sourcing ROS/install: python3 node_smoke_test.py /path/to/sapphire.

Uses a separate DDS domain and synthetic identity calibration in a temp directory.
Checks actual subscription selection, cross-frame callback fusion, and timeout.
"""
import os
import re
import signal
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

os.environ["ROS_DOMAIN_ID"] = "215"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


def cloud(times, rear=False):
    msg = PointCloud2()
    msg.header.frame_id = "rear" if rear else "front"
    msg.height, msg.width = 1, len(times)
    msg.point_step, msg.row_step = 24, 24 * len(times)
    msg.fields = [PointField(name=n, offset=i * 4, datatype=8 if i == 4 else 7, count=1)
                  for i, n in enumerate(["x", "y", "z", "intensity", "timestamp"])]
    msg.data = b"".join(struct.pack("<ffffd", 2.0, 0.0, 0.0, 1.0, t) for t in times)
    return msg


def run(binary, mode, lidar_type, tmp, algorithm, rate=10.0, ros_override=True):
    label = mode + "_" + lidar_type + "_" + str(int(rate)) + ("" if ros_override else "_toml")
    type_setting = f"    lidar_type: {lidar_type}" if ros_override else ""
    config = tmp / (label + ".yaml")
    config.write_text(f"""/**:
  ros__parameters:
    lidar_mode: {mode}
{type_setting}
    obs_mode: lio
    algorithm_config: {algorithm}
    topics.lidar: /smoke/front
    topics.rear_lidar: /smoke/rear
    topics.imu: /smoke/imu
    topics.image: /smoke/image
    lidar.extrinsic_tran: [0.0, 0.0, 0.0]
    lidar.extrinsic_rota: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    lidar.max_wait: 0.5
    lidar.lidar_merge_hz: {rate}
    lidar.imu_rate_hz: 200.0
""")
    logfile = tmp / (label + ".log")
    with logfile.open("w") as log:
        process = subprocess.Popen([binary, "--ros-args", "--params-file", str(config),
                                    "-r", "__node:=sapphire_" + mode + "_smoke"], stdout=log, stderr=log)
        node = rclpy.create_node("probe_" + mode)
        try:
            deadline = time.monotonic() + 10
            topics = set()
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(logfile.read_text())
                try:
                    topics = {name for name, _ in node.get_subscriber_names_and_types_by_node("sapphire_" + mode + "_smoke", "/")}
                except Exception:
                    topics = set()
                if "/smoke/front" in topics and "/smoke/imu" in topics:
                    break
                rclpy.spin_once(node, timeout_sec=0.05)
            assert "/smoke/front" in topics and "/smoke/imu" in topics, topics
            assert ("/smoke/rear" in topics) == (mode == "dual"), topics
            assert "/smoke/image" not in topics, "LIO must not subscribe to images"
            if mode == "single" and lidar_type == "hesai":
                front = node.create_publisher(PointCloud2, "/smoke/front", qos_profile_sensor_data)
                deadline = time.monotonic() + 5
                while front.get_subscription_count() == 0 and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                assert front.get_subscription_count()
                front.publish(cloud([1.04, 1.0, 1.09]))
                until = time.monotonic() + .3
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=.02)
                content = logfile.read_text()
                assert "malformed PointCloud2" not in content and "Rejected invalid" not in content, content
                bad = cloud([2.0, 2.05])
                bad.fields[-1].datatype = PointField.FLOAT32
                front.publish(bad)
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline and "malformed PointCloud2" not in logfile.read_text():
                    rclpy.spin_once(node, timeout_sec=.05)
                assert "malformed PointCloud2" in logfile.read_text(), logfile.read_text()
            if mode == "dual":
                front = node.create_publisher(PointCloud2, "/smoke/front", qos_profile_sensor_data)
                rear = node.create_publisher(PointCloud2, "/smoke/rear", qos_profile_sensor_data)
                deadline = time.monotonic() + 5
                while min(front.get_subscription_count(), rear.get_subscription_count()) == 0 and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                assert front.get_subscription_count() and rear.get_subscription_count()
                front.publish(cloud([1.0, 1.05, 1.1]))
                rear.publish(cloud([0.95, 1.0, 1.05], True))
                rear.publish(cloud([1.06, 1.1, 1.15], True))
                # Wait for the first window, then exercise the timer without rear arrivals.
                until = time.monotonic() + 0.15
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=0.01)
                front.publish(cloud([2.0, 2.05, 2.1]))
                expected_emitted = "emitted=2" if rate == 20.0 else "emitted=1"
                # Drop diagnostics are throttled to five seconds. At 20 Hz the
                # artificial one-second gap can log coverage before timeout.
                deadline = time.monotonic() + 8
                while time.monotonic() < deadline:
                    content = logfile.read_text()
                    if expected_emitted in content and re.search(r"timeout=[1-9][0-9]*", content):
                        break
                    rclpy.spin_once(node, timeout_sec=0.03)
                assert expected_emitted in content and re.search(r"timeout=[1-9][0-9]*", content), content
        finally:
            node.destroy_node()
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                raise RuntimeError("Sapphire did not shut down")
        assert process.returncode == 0, logfile.read_text()
        assert f"lidar_mode={mode}, lidar_type={lidar_type}, obs_mode=lio" in logfile.read_text(), logfile.read_text()
    print(label + ": subscriptions and clean shutdown OK" +
          ("; callback checks passed" if mode == "dual" or lidar_type == "hesai" else ""))


def main():
    binary = str(Path(sys.argv[1]).resolve())
    with tempfile.TemporaryDirectory(prefix="sapphire-sensor-smoke-") as directory:
        tmp = Path(directory)
        os.environ["ROS_LOG_DIR"] = str(tmp / "ros_logs")
        source = Path(__file__).resolve().parents[2] / "sapphire/config/dual.toml"
        text = source.read_text().replace("extrinsic_tran = []", "extrinsic_tran = [0.0, 0.0, 0.0]")
        text = text.replace("extrinsic_rota = []", "extrinsic_rota = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]")
        text = text.replace("enabled = true", "enabled = false")
        algorithm = tmp / "synthetic.toml"
        algorithm.write_text(text)
        rclpy.init()
        try:
            for mode, lidar_type in [("single", "livox"), ("single", "airy"), ("single", "hesai"), ("dual", "airy")]:
                run(binary, mode, lidar_type, tmp, algorithm)
            run(binary, "dual", "airy", tmp, algorithm, 20.0)
            hesai_algorithm = tmp / "hesai.toml"
            hesai_algorithm.write_text(text.replace('lidar_type = "airy"', 'lidar_type = "hesai"'))
            run(binary, "single", "hesai", tmp, hesai_algorithm, ros_override=False)
            for overrides, error in [
                (["lidar_mode:=dual", "lidar_type:=livox"], "dual currently supports lidar_type=airy"),
                (["lidar_mode:=dual", "lidar_type:=hesai"], "dual currently supports lidar_type=airy"),
                (["obs_mode:=livo"], "visual observations are not implemented yet"),
                (["lidar_mode:=invalid"], "lidar_mode must be single or dual"),
                (["lidar_type:=invalid"], "lidar_type must be livox, airy or hesai"),
                (["obs_mode:=invalid"], "obs_mode must be lio or livo"),
            ]:
                args = [binary, "--ros-args", "-p", "algorithm_config:=" + str(algorithm)]
                for value in overrides:
                    args.extend(["-p", value])
                result = subprocess.run(args, capture_output=True, text=True, timeout=10)
                assert result.returncode != 0 and error in result.stdout + result.stderr, result.stdout + result.stderr
            print("Unsupported and invalid modes rejected with explicit diagnostics")
        finally:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
