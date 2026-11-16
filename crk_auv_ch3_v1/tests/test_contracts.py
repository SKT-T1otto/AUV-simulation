import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from crk_ch3.contracts import canonical_hash, validate_experiment, verify_source_lock

ROOT = Path(__file__).resolve().parents[1]


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((ROOT/'config/experiment.json').read_text(encoding='utf-8'))

    def test_original_contract_and_time_partition(self):
        c = validate_experiment(self.config)
        self.assertEqual(c['task_dt_s']/c['physics_dt_s'], 100)
        self.assertEqual(c['task_dt_s']/c['control_dt_s'], 10)
        self.assertEqual(self.config, c)

    def test_refuse_unsupported_algorithm_and_hidden_semantic_changes(self):
        for key, value in [('method', 'B3'), ('checkpoint', 'old.pt'), ('map_version', 'bounded_v3'),
                           ('observation_dim', 29), ('roles', ['executor']*4), ('task_dt_s', .201),
                           ('control_dt_s', 0), ('physics_dt_s', float('nan')), ('training_enabled', True),
                           ('handoff', 'instant'), ('obstacle_sensor_noise_std', .1), ('max_steps', True),
                           ('motion_authority', 'python')]:
            c = copy.deepcopy(self.config)
            c[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_experiment(c)

    def test_source_tampering_and_path_escape_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root/'source.py'
            source.write_bytes(b'locked source\n')
            files = {'source.py': hashlib.sha256(source.read_bytes()).hexdigest()}
            lock = {'schema': 'crk.ch3.source_lock.v1', 'files': files, 'files_sha256': canonical_hash(files)}
            self.assertEqual(verify_source_lock(lock, root), 1)
            source.write_bytes(b'changed\n')
            with self.assertRaises(ValueError):
                verify_source_lock(lock, root)
            for path in ('../outside', '/outside', 'C:/outside', 'a/../../x', 'a\\x'):
                files = {path: '0'*64}
                lock.update(files=files, files_sha256=canonical_hash(files))
                with self.subTest(path=path), self.assertRaises(ValueError):
                    verify_source_lock(lock, root)
