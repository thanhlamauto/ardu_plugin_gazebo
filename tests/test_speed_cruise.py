import unittest

from scripts.analyze_speed_cruise import cruise_windows


class CruiseMeasurementTests(unittest.TestCase):
    def rows(self, speed):
        return [{'sim_time_s': i*.1, 'wall_monotonic_s': i*.1,
                 'position_enu': [speed*i*.1, 0, 5]} for i in range(81)]

    def test_sustained_and_overspeed(self):
        windows = cruise_windows(self.rows(5), 5)
        self.assertGreater(windows[0]['duration_sim_s'], 7)
        self.assertEqual(cruise_windows(self.rows(5.5), 5), [])

    def test_wall_stall_breaks_apparent_cruise(self):
        rows = self.rows(5)
        for row in rows[40:]:
            row['wall_monotonic_s'] += 2
        windows = cruise_windows(rows, 5)
        self.assertEqual(len(windows), 2)
        self.assertLess(max(w['duration_sim_s'] for w in windows), 5)

    def test_sim_gap_is_not_interpolated_as_cruise(self):
        rows = self.rows(5)
        windows = cruise_windows(rows[:30]+rows[50:], 5)
        self.assertEqual(len(windows), 2)
        self.assertLess(max(w['duration_sim_s'] for w in windows), 5)
