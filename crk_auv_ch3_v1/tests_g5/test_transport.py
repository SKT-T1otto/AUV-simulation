from dataclasses import fields, replace
import unittest
import numpy as np
from crk_ch3.mission_transport import MotionExchange, MotionFeedback, MotionRequest, feedback_from_gazebo
from crk_ch3.motion_protocol import BodyState
from crk_ch3.scenario import Frame


class TransportTests(unittest.TestCase):
    def request(self):
        return MotionRequest.create(1, np.zeros((4, 3)), np.zeros((4, 3)), np.ones((4, 3)), [False]*4)

    def test_request_is_read_only_and_carries_no_map_or_target_truth(self):
        request = self.request()
        self.assertFalse(any('target' in f.name or 'map' in f.name for f in fields(request)))
        with self.assertRaises(ValueError):
            request.acceleration_world[0, 0] = 2.
        with self.assertRaises(ValueError):
            replace(request, positions=np.zeros((3, 3)))
        with self.assertRaises(ValueError):
            replace(request, velocities_world=np.full((4, 3), np.nan))

    def test_wrong_identity_timestamp_and_frame_fail_and_poison_exchange(self):
        request = self.request()
        good = MotionFeedback.for_request(request, request.positions, request.velocities_world)
        for bad in (replace(good, token='wrong'), replace(good, task_step=2),
                    replace(good, stamp_ns=request.end_ns-3_000_000),
                    replace(good, stamp_ns=request.end_ns+1), replace(good, frame='NED')):
            exchange = MotionExchange(lambda _: bad)
            with self.assertRaises(ValueError):
                exchange.exchange(request)
            with self.assertRaises(RuntimeError):
                exchange.exchange(request)

    def test_duplicate_transaction_is_rejected(self):
        request = self.request()
        exchange = MotionExchange(lambda r: MotionFeedback.for_request(r, r.positions, r.velocities_world))
        exchange.exchange(request)
        with self.assertRaises(ValueError):
            exchange.exchange(request)

    def test_gazebo_body_velocity_is_rotated_before_native_frame_conversion(self):
        request = self.request()
        frame = Frame(yaw=np.pi/2)
        epoch = 40_000_000
        # Body x points along Gazebo world y, also native world x here.
        q = (0, 0, np.sin(np.pi/4), np.cos(np.pi/4))
        states = [BodyState(epoch+request.end_ns, frame.position((i, 2, 3)), q, (1, 0, 0), (0, 0, 0))
                  for i in range(4)]
        result = feedback_from_gazebo(request, states, epoch_ns=epoch, frame=frame)
        np.testing.assert_allclose(result.positions, [[i, 2, 3] for i in range(4)], atol=1e-12)
        np.testing.assert_allclose(result.velocities_world, [[1, 0, 0]]*4, atol=1e-12)
        self.assertEqual(result.stamp_ns, request.end_ns)
        with self.assertRaises(ValueError):
            feedback_from_gazebo(request, [replace(states[0], stamp_ns=1), *states[1:]], epoch_ns=epoch)
