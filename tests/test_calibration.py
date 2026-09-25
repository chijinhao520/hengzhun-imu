# Copyright (c) 2026 池金壕
# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
import contextlib
import hashlib
import importlib.util
import io
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'tools/imu_calib_fit.py'
spec = importlib.util.spec_from_file_location('calibration', SCRIPT)
cal = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cal)


class CalibrationTests(unittest.TestCase):
    def run_cli(self, *args):
        return subprocess.run([sys.executable, '-B', '-X', 'utf8', str(SCRIPT), *args],
                              cwd=ROOT, capture_output=True, text=True, encoding='utf-8')

    def temp_log(self, text):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        path = Path(tmp.name) / 'sensor.csv'
        path.write_text(text, encoding='utf-8')
        return path

    def test_six_face_real_sample(self):
        result = self.run_cli('sixface', 'examples/data/six-face.csv')
        self.assertEqual(result.returncode, 0, result.stderr)
        for expected in ('-8.9f', '-40.5f', '-78.1f', '1.00784f', '1.00061f', '1.00490f', '0.46%'):
            self.assertIn(expected, result.stdout)

    def test_yaw_real_sample(self):
        result = self.run_cli('yawscale', 'examples/data/yaw-three-turns.csv:+3,-3')
        self.assertEqual(result.returncode, 0, result.stderr)
        for expected in ('1.00230f', '0.096%', '1.00182', '1.00278'):
            self.assertIn(expected, result.stdout)

    def test_sample_hashes_and_cadence(self):
        manifest = json.loads((ROOT / 'examples/data/manifest.json').read_text(encoding='utf-8'))
        for item in manifest['files']:
            path = ROOT / item['file']
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), item['sha256'])
            rows, dropped = cal.parse_records(path)
            self.assertEqual((len(rows), dropped), (item['records'], 0))
            self.assertTrue(all(b[0] - a[0] == 20 for a, b in zip(rows, rows[1:])))

    def test_split_record_and_serial_prefix(self):
        path = self.temp_log('[12:34:56.789] Rx: 10000,0,0,4098,0,0,-\n'
                             '2,10,0,0,230\n10020,0,0,4098,0,0,-2,10,0,0,230\n')
        rows, dropped = cal.parse_records(path)
        self.assertEqual((len(rows), dropped), (2, 0))
        self.assertEqual(rows[0][6], -2)

    def test_missing_face_rejected(self):
        lines = (ROOT / 'examples/data/six-face.csv').read_text().splitlines()[:50]
        result = self.run_cli('sixface', str(self.temp_log('\n'.join(lines) + '\n')))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('缺少朝向', result.stderr)

    def test_discontinuous_timestamps_rejected(self):
        lines = (ROOT / 'examples/data/six-face.csv').read_text().splitlines()
        variants = {
            'missing': lines[:10] + lines[11:],
            'duplicate': lines[:10] + [lines[9]] + lines[10:],
            'reset': lines[:10] + ['0,' + lines[10].split(',', 1)[1]] + lines[11:],
        }
        for label, values in variants.items():
            with self.subTest(label=label):
                path = self.temp_log('\n'.join(values) + '\n')
                result = self.run_cli('sixface', str(path))
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('时间戳必须连续递增', result.stderr)

    def test_wrong_turn_count_rejected(self):
        result = self.run_cli('yawscale', 'examples/data/yaw-three-turns.csv:+2,-2')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('与标称', result.stderr)

    def test_wrong_platform_count_rejected(self):
        result = self.run_cli('yawscale', 'examples/data/yaw-three-turns.csv:+3')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('静止平台', result.stderr)

    def test_invalid_turn_values_rejected(self):
        for value in ('0', 'nan', 'inf', '-inf', 'bad'):
            with self.subTest(value=value), self.assertRaises(SystemExit):
                cal._parse_spec('C:/logs/run.csv:' + value, False)

    def test_windows_path_preserved(self):
        self.assertEqual(cal._parse_spec('C:/logs/run.csv:+3,-3', False),
                         ('C:/logs/run.csv', None, [3.0, -3.0]))

    def test_zero_gyro_integral_rejected(self):
        text = ''.join(f'{10000 + i * 20},0,0,4098,0,0,0,0,0,0,230\n' for i in range(200))
        result = self.run_cli('gyroscale', str(self.temp_log(text)) + ':x:+3')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('积分角度为零', result.stderr)

    def test_help_success(self):
        result = self.run_cli('--help')
        self.assertEqual(result.returncode, 0)
        self.assertIn('衡准 IMU', result.stdout)


if __name__ == '__main__':
    unittest.main()
