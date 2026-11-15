"""ROS 2 truth-odometry feedback controller for the independent single-boat demo."""
import csv
import json
import signal
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

from crk_auv_demo.core import ControlLaw, WaypointSequence, rpy


class Tracker(Node):
    def __init__(self):
        super().__init__('crk_singleboat_tracker')
        share = Path(get_package_share_directory('crk_auv_demo'))
        self.declare_parameter('vehicle_file', str(share/'config/vehicle.json'))
        self.declare_parameter('waypoints_file', str(share/'config/waypoints.json'))
        self.declare_parameter('output_dir', '/tmp/crk_auv_results')
        self.config = json.loads(Path(self.get_parameter('vehicle_file').value).read_text())
        self.plan = json.loads(Path(self.get_parameter('waypoints_file').value).read_text())
        self.control = ControlLaw(self.config)
        self.sequence = WaypointSequence(self.plan['points'], self.plan['arrival_radius_m'],
                                         self.plan['arrival_speed_m_s'], self.plan['dwell_s'])
        self.output = Path(self.get_parameter('output_dir').value)
        self.output.mkdir(parents=True, exist_ok=True)
        self.stream = (self.output/'trajectory.csv').open('x', newline='')
        self.csv = csv.writer(self.stream)
        self.csv.writerow(['sim_s', 'waypoint_index', 'x', 'y', 'z', 'target_x', 'target_y',
                           'target_z', 'distance_m', 'speed_m_s', 'roll_rad', 'pitch_rad',
                           'yaw_rad', 'max_abs_thrust_N', 'allocation_residual'])
        self.publishers_ = [self.create_publisher(Float64, f'/auv_0/thruster_{i}/cmd_thrust', 10)
                            for i in range(8)]
        self.subscription = self.create_subscription(Odometry, '/auv_0/odom', self.on_state,
                                                     qos_profile_sensor_data)
        self.timer = self.create_timer(0.02, self.tick)
        self.state = None
        self.start_sim = None
        self.start_wall = time.monotonic()
        self.last_sim = None
        self.last_logged = -1.
        self.last_printed = -1.
        self.last_position = None
        self.path_length = 0.
        self.completed_at = None
        self.final_errors = []
        self.max_roll_pitch = 0.
        self.max_force = 0.
        self.samples = 0
        self.saturated = 0
        self.reason = 'interrupted'
        self.finished = False
        self.get_logger().info('Waiting for /clock and /auv_0/odom. Truth feedback; zero current.')

    def on_state(self, msg):
        if msg.header.frame_id != 'map' or msg.child_frame_id != 'auv_0/base_link':
            self.reason = 'unexpected_odometry_frame'
            self.finished = True
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        stamp = msg.header.stamp.sec + 1e-9*msg.header.stamp.nanosec
        self.state = (stamp, np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w]),
                      np.array([v.x, v.y, v.z]), np.array([w.x, w.y, w.z]))

    def publish(self, forces):
        for publisher, force in zip(self.publishers_, forces):
            publisher.publish(Float64(data=float(force)))

    def tick(self):
        if self.finished or self.state is None:
            return
        stamp, position, q, velocity, rates = self.state
        if not np.isfinite(np.r_[stamp, position, q, velocity, rates]).all():
            self.reason = 'nonfinite_state'
            self.finished = True
            return
        if self.start_sim is None:
            self.start_sim = self.last_sim = stamp
            self.last_position = position.copy()
            return
        dt = stamp-self.last_sim
        if dt == 0:
            if self.get_clock().now().nanoseconds*1e-9-stamp > 0.5:
                self.reason = 'odometry_stale'
                self.finished = True
            return
        if not 0 < dt <= 0.2:
            self.reason = 'time_reset_or_odometry_gap'
            self.finished = True
            return
        self.last_sim = stamp
        elapsed = stamp-self.start_sim
        if elapsed > self.plan['timeout_sim_s']:
            self.reason = 'simulation_timeout'
            self.finished = True
            return
        if abs(position[0]) > 8 or abs(position[1]) > 8 or not -5.5 < position[2] < -0.3:
            self.reason = 'outside_test_volume'
            self.finished = True
            return
        angles = rpy(q)
        self.max_roll_pitch = max(self.max_roll_pitch, float(np.max(np.abs(angles[:2]))))
        if self.max_roll_pitch > np.deg2rad(45):
            self.reason = 'attitude_limit_exceeded'
            self.finished = True
            return
        self.path_length += float(np.linalg.norm(position-self.last_position))
        self.last_position = position.copy()
        speed = float(np.linalg.norm(velocity))
        if self.sequence.update(position, speed, dt):
            self.get_logger().info(f'Accepted waypoint {self.sequence.accepted}/{len(self.sequence.points)}')
        if self.sequence.completed and self.completed_at is None:
            self.completed_at = elapsed
            self.get_logger().info('All waypoints reached. Beginning 10 s final station keeping.')
        target = self.sequence.target
        distance = float(np.linalg.norm(target-position))
        forces, residual = self.control.step(position, q, velocity, rates, target, dt)
        self.publish(forces)
        maximum = float(np.max(np.abs(forces)))
        self.max_force = max(self.max_force, maximum)
        self.samples += 1
        self.saturated += int(maximum >= self.control.limit-1e-3)
        if self.completed_at is not None:
            self.final_errors.append(distance)
            if elapsed-self.completed_at >= self.plan['final_hold_s']:
                self.reason = 'completed'
                self.finished = True
        if elapsed-self.last_logged >= 0.1:
            self.csv.writerow([elapsed, self.sequence.index, *position, *target, distance, speed,
                               *angles, maximum, residual])
            self.stream.flush()
            self.last_logged = elapsed
        if elapsed-self.last_printed >= 1.:
            self.get_logger().info(f't={elapsed:.1f}s waypoint={self.sequence.index+1} '
                                   f'error={distance:.3f}m speed={speed:.3f}m/s '
                                   f'position={position.round(2).tolist()}')
            self.last_printed = elapsed

    def finish(self):
        # The main process keeps the ROS context alive until zero commands are sent.
        for _ in range(10):
            self.publish(np.zeros(8))
            time.sleep(0.02)
        self.stream.close()
        rms = float(np.sqrt(np.mean(np.square(self.final_errors)))) if self.final_errors else None
        summary = {
            'schema': 'crk.singleboat.runtime.v1', 'termination_reason': self.reason,
            'all_waypoints_accepted': self.sequence.completed,
            'waypoints_accepted': self.sequence.accepted, 'waypoints_total': len(self.sequence.points),
            'simulation_elapsed_s': None if self.start_sim is None else self.last_sim-self.start_sim,
            'executed_path_m': self.path_length, 'final_hold_rmse_m': rms,
            'max_roll_pitch_deg': float(np.rad2deg(self.max_roll_pitch)),
            'max_abs_thrust_N': self.max_force,
            'saturation_fraction': self.saturated/max(1, self.samples),
            'engineering_acceptance_passed': bool(self.reason == 'completed' and rms is not None
                                                  and rms < 0.2 and self.max_roll_pitch < np.deg2rad(5)),
            'feedback': 'Gazebo truth odometry, world pose, body-frame twist',
            'model_status': self.config['model_status'],
            'not_a_thesis_mission_success_metric': True,
            'vehicle_config': self.config, 'waypoint_config': self.plan}
        (self.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        self.get_logger().info(f'Result: {self.reason}; saved to {self.output}')


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopped = [False]
    def stop(signum, frame):
        stopped[0] = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = None
    code = 1
    try:
        node = Tracker()
        while rclpy.ok() and not stopped[0] and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.start_sim is None and time.monotonic()-node.start_wall > 45:
                node.reason = 'no_clock_or_odometry_within_45_wall_seconds'
                break
        code = 0 if node.reason == 'completed' else 1
    finally:
        if node is not None:
            node.finish()
            node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
