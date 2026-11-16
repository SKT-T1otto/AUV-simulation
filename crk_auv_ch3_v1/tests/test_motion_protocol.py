import unittest
import numpy as np
from crk_ch3.motion_protocol import BodyState, TeamStateBuffer, build_program, evaluate_phase


def state(t, x=0):
    return BodyState(t, (x, 0, 0), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))


class ProtocolTests(unittest.TestCase):
    def test_barrier_requires_all_four_at_one_timestamp(self):
        buffer = TeamStateBuffer()
        for i in range(3):
            buffer.push(i, state(20), child_frame=f'auv_{i}/base_link')
        buffer.push(3, state(10), child_frame='auv_3/base_link')
        self.assertIsNone(buffer.take(20, 2))
        buffer.push(3, state(20), child_frame='auv_3/base_link')
        self.assertEqual(len(buffer.take(20, 2)), 4)
        self.assertIsNone(buffer.take(20, 2))

    def test_wrong_frames_conflicts_and_invalid_state_are_rejected(self):
        buffer = TeamStateBuffer()
        with self.assertRaises(ValueError):
            buffer.push(1, state(1), child_frame='auv_0/base_link')
        buffer.push(0, state(1), child_frame='auv_0/base_link')
        with self.assertRaises(ValueError):
            buffer.push(0, state(1, x=1), child_frame='auv_0/base_link')
        with self.assertRaises(ValueError):
            BodyState(1, (float('nan'), 0, 0), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))

    def test_only_one_vehicle_changes_target_in_each_isolation_phase(self):
        program = build_program(np.zeros((4, 3)))
        for i in range(4):
            previous, phase = program[i], program[i+1]
            changed = np.linalg.norm(phase.targets-previous.targets, axis=1) > 0
            self.assertEqual(np.flatnonzero(changed).tolist(), [i])
        self.assertEqual(sum(p.ticks for p in program)*.02, 37)
        self.assertEqual(program[-1].moving, ())

    def test_motion_and_isolation_are_both_required(self):
        phase = build_program(np.zeros((4, 3)))[1]
        self.assertTrue(evaluate_phase(phase, np.zeros((4, 3)), phase.targets, np.zeros(4))['passed'])
        self.assertFalse(evaluate_phase(phase, np.zeros((4, 3)), np.zeros((4, 3)), np.zeros(4))['passed'])
        self.assertFalse(evaluate_phase(phase, np.zeros((4, 3)), phase.targets, [0, .3, 0, 0])['passed'])
