import unittest
from types import SimpleNamespace
from mppi_ardupilot.mavlink_interface import ArduPilotInterface


class MavlinkDrainTests(unittest.TestCase):
    def interface(self, receive):
        interface = ArduPilotInterface.__new__(ArduPilotInterface)
        interface.master = SimpleNamespace(recv_match=receive)
        interface.mavutil = SimpleNamespace(mavlink=SimpleNamespace(
            MAVLINK_MSG_ID_LOCAL_POSITION_NED=32, MAVLINK_MSG_ID_ATTITUDE=30))
        return interface

    def test_continuous_stream_cannot_starve_control_loop(self):
        calls = []
        def receive(**kwargs):
            self.assertFalse(kwargs['blocking'])
            calls.append(1)
            return SimpleNamespace(get_msgId=lambda: 0)
        interface = self.interface(receive)
        interface.spin_once(max_messages=10)
        self.assertEqual(len(calls), 10)

    def test_drain_keeps_latest_state_and_stops_at_empty(self):
        first = SimpleNamespace(get_msgId=lambda: 32)
        latest = SimpleNamespace(get_msgId=lambda: 32)
        messages = iter([first, latest, None])
        interface = self.interface(lambda **kwargs: next(messages))
        interface.spin_once()
        self.assertIs(interface._lned, latest)
