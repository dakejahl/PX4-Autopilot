import unittest
from unittest.mock import patch
from types import SimpleNamespace
from tempfile import TemporaryDirectory
from pathlib import Path
import json

import numpy as np
from compare_raw_flow import interpolate, fit_model, rotation, analyze


class RawFlowAnalysisTest(unittest.TestCase):
    def test_invalid_brackets_and_no_extrapolation(self):
        result = interpolate(np.array([-1., .5, 1.5, 3.5, 5.]),
                             np.array([0., 1., 2., 4.]),
                             np.array([[0.], [1.], [2.], [4.]]),
                             np.array([True, True, False, True]), 1.1)
        self.assertEqual(result[1, 0], .5)
        self.assertTrue(np.all(np.isnan(result[[0, 2, 3, 4]])))

    def test_known_axis_mapping(self):
        rng = np.random.default_rng(3905)
        expected = rng.normal(size=(1000, 2))
        model = np.array([[470., -20.], [12., -490.]])
        counts = expected @ model.T
        fitted = fit_model(expected, counts, np.ones(1000, dtype=bool))
        np.testing.assert_allclose(fitted, model, atol=1e-8)
        with self.assertRaises(ValueError):
            fit_model(np.ones((1000, 2)), counts, np.ones(1000, dtype=bool))

    def test_quaternion_direction(self):
        q = np.array([[np.sqrt(.5), 0, 0, np.sqrt(.5)]])
        np.testing.assert_allclose(rotation(q)[0] @ [1., 0., 0.], [0., 1., 0.], atol=1e-12)

    def test_end_to_end_known_delay_and_integer_counts(self):
        t = np.arange(0, 20, .001)
        stamp = (1e6 + t*1e6).astype(np.uint64)
        vel = np.column_stack([5*np.sin(3*t), 4*np.cos(4*t), np.zeros(len(t))])
        gnss = dict(timestamp=stamp, timestamp_sample=stamp, vel_n_m_s=vel[:, 0],
                    vel_e_m_s=vel[:, 1], vel_d_m_s=vel[:, 2],
                    vel_ned_valid=np.ones(len(t)), fix_type=np.full(len(t), 3),
                    s_variance_m_s=np.full(len(t), .1))
        att = dict(timestamp_sample=stamp, **{f'q[{i}]': np.ones(len(t)) if i == 0 else np.zeros(len(t)) for i in range(4)})
        distance = dict(timestamp=stamp, current_distance=np.full(len(t), 2.),
                        min_distance=np.full(len(t), .1), max_distance=np.full(len(t), 20.),
                        orientation=np.full(len(t), 25), signal_quality=np.full(len(t), 100))
        end = np.arange(.1, 19.9, .01)
        dt = .01
        shift = .007
        def translation(query):
            return np.column_stack([-np.interp(query, t, vel[:, 1])/2, np.interp(query, t, vel[:, 0])/2])
        expected = (translation(end-dt+shift) + 4*translation(end-dt/2+shift) + translation(end+shift)) * dt/6
        model = np.array([[470., -20.], [12., 490.]])
        counts = np.round(expected @ model.T).astype(np.int16)
        raw = dict(timestamp_sample=(1e6+end*1e6).astype(np.uint64), interval_us=np.full(len(end), 10000),
                   delta_x=counts[:, 0], delta_y=counts[:, 1], gyro_samples=np.full(len(end), 10),
                   timestamp_sample_valid=np.ones(len(end)), observation=np.full(len(end), 63, dtype=np.uint8),
                   squal_raw=np.full(len(end), 100), raw_data_sum=np.full(len(end), 50),
                   shutter=np.full(len(end), 1000), motion=np.full(len(end), 128, dtype=np.uint8),
                   frame_counter=np.arange(len(end)),
                   **{f'gyro_integral[{i}]': np.zeros(len(end)) for i in range(3)})
        data = [('flow_raw', raw), ('sensor_gps', gnss), ('vehicle_attitude', att), ('distance_sensor', distance)]
        log = SimpleNamespace(data_list=[SimpleNamespace(name=name, multi_id=0, data=d) for name, d in data])
        with TemporaryDirectory() as temp, patch('pyulog.ULog', return_value=log), patch('builtins.print'):
            args = SimpleNamespace(log=Path('synthetic.ulg'), output=Path(temp), instance=0,
                                   gnss_instance=0, range_instance=0, node_yaw_deg=180., raw_yaw_deg=0.,
                                   flow_minus_gnss=[0., 0., 0.], gnss_receive_time=False, gnss_delay_ms=0.,
                                   max_speed_error=.5, max_gnss_gap=.3, max_range_gap=.1,
                                   include_ground=True, squal_min=1, scan_ms=15)
            analyze(args)
            result = json.loads((Path(temp)/'fit.json').read_text())
            self.assertLessEqual(abs(result['reference_shift_ms'] - 7), 1)
            np.testing.assert_allclose(result['counts_per_radian_matrix'], model, atol=2)
            self.assertLess(result['held_out_fitted_velocity_rms_m_s'], .2)
            self.assertTrue((Path(temp)/'comparison.png').exists())
            self.assertEqual(len((Path(temp)/'frames.csv').read_text().splitlines()), len(end)+1)


if __name__ == '__main__':
    unittest.main()
