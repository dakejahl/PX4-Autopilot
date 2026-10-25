import math
from pathlib import Path
import sys
import unittest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'src/drivers/uavcan/libdronecan/libuavcan/dsdl_compiler/pydronecan'))
import dronecan


class RawFlowDsdlTest(unittest.TestCase):
    def test_round_trip(self):
        dronecan.load_dsdl(str(REPO / 'src/drivers/uavcan/dsdl/ark'))
        dtype = dronecan.TYPENAMES['ark.flow.RawFlow']
        message = dtype()
        values = dict(frame_counter=0xffffffff, delta_x=-32768, delta_y=32767,
                      motion=0x81, observation=0xbf, squal_raw=255, raw_data_sum=152,
                      shutter=0x7fffff, interval_us=8000, gyro_samples=8,
                      node_timestamp_us=1234567890123, bus_timestamp_us=9876543210123,
                      gyro_device_id=0xffffffff)
        for field, value in values.items():
            setattr(message, field, value)
        message.gyro_integral = [.001, -.002, float('nan')]
        packed = message._pack()
        self.assertEqual(len(packed), 46*8)
        decoded = dtype()
        decoded._unpack(packed)
        for field, value in values.items():
            self.assertEqual(getattr(decoded, field), value, field)
        self.assertAlmostEqual(decoded.gyro_integral[0], .001, delta=1e-6)
        self.assertAlmostEqual(decoded.gyro_integral[1], -.002, delta=1e-6)
        self.assertTrue(math.isnan(decoded.gyro_integral[2]))


if __name__ == '__main__':
    unittest.main()
