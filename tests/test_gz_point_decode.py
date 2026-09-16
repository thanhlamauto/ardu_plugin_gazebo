import struct
import unittest
from types import SimpleNamespace
from scripts.lidar_to_mavlink_avoidance import _read_gz_points


class PackedDecodeTests(unittest.TestCase):
    def test_endian_padding_nan_limit_and_single_buffer_read(self):
        for endian in ('<', '>'):
            class Message:
                width, height, point_step, row_step = 2, 2, 16, 36
                is_bigendian = endian == '>'
                field = [SimpleNamespace(name=n, datatype=6, offset=i*4) for i,n in enumerate('xyz')]
                reads = 0
                @property
                def data(self):
                    self.reads += 1
                    return payload
            rows = [(1,2,3), (4,5,6), (float('nan'),0,0), (7,8,9)]
            payload = b''.join(struct.pack(endian+'ffff', *p, 0) + (b'0000' if i%2 else b'')
                               for i,p in enumerate(rows))
            message = Message()
            self.assertEqual(_read_gz_points(message, 4), [(1,2,3),(4,5,6),(7,8,9)])
            self.assertEqual(message.reads, 1)
            self.assertEqual(_read_gz_points(message, 1), [(1,2,3)])


if __name__ == '__main__':
    unittest.main()
