#!/usr/bin/env python3
"""
test_acceleration_control.py
============================
Unit tests for acceleration_control.py

Tests cover:
- type_mask encoding (bit 12, acc fields, yaw handling)
- AccelerationNed dataclass
- hover() sends zero acceleration
- yaw=None vs explicit yaw type_mask selection
- Disconnected state guard

Run with:
    python3 -m pytest test_acceleration_control.py -v
"""

import math
import unittest
from unittest.mock import MagicMock, patch, call

# ── import the module under test ─────────────────────────────────────────────
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

# Patch pymavlink before importing so the module loads without real hardware
mavutil_mock = MagicMock()
mavutil_mock.mavlink.MAV_FRAME_LOCAL_NED = 1
mavutil_mock.mavlink.MAV_CMD_COMPONENT_ARM_DISARM = 400
mavutil_mock.mavlink.MAV_CMD_NAV_LAND = 21
mavutil_mock.mavlink.MAV_CMD_DO_SET_MODE = 176
mavutil_mock.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

with patch.dict('sys.modules', {'pymavlink': MagicMock(), 'pymavlink.mavutil': mavutil_mock}):
    from acceleration_control import (
        AccelerationControl,
        AccelerationNed,
        _ACC_SP_EXTERNAL_FLAG,
        _TYPE_MASK_ACC_ONLY,
        _TYPE_MASK_ACC_WITH_YAW,
        _MAV_FRAME_LOCAL_NED,
    )


# ─────────────────────────────────────────────────────────────────────────────
# Test: constants
# ─────────────────────────────────────────────────────────────────────────────

class TestConstants(unittest.TestCase):

    def test_bit12_value(self):
        self.assertEqual(_ACC_SP_EXTERNAL_FLAG, 0x1000)

    def test_type_mask_acc_only_has_bit12(self):
        self.assertTrue(_TYPE_MASK_ACC_ONLY & _ACC_SP_EXTERNAL_FLAG)

    def test_type_mask_acc_only_ignores_position(self):
        # bits 0,1,2 set = ignore x,y,z position
        self.assertTrue(_TYPE_MASK_ACC_ONLY & (1 << 0))
        self.assertTrue(_TYPE_MASK_ACC_ONLY & (1 << 1))
        self.assertTrue(_TYPE_MASK_ACC_ONLY & (1 << 2))

    def test_type_mask_acc_only_ignores_velocity(self):
        # bits 3,4,5 set = ignore vx,vy,vz
        self.assertTrue(_TYPE_MASK_ACC_ONLY & (1 << 3))
        self.assertTrue(_TYPE_MASK_ACC_ONLY & (1 << 4))
        self.assertTrue(_TYPE_MASK_ACC_ONLY & (1 << 5))

    def test_type_mask_acc_only_uses_acceleration(self):
        # bits 6,7,8 clear = use afx,afy,afz
        self.assertFalse(_TYPE_MASK_ACC_ONLY & (1 << 6))
        self.assertFalse(_TYPE_MASK_ACC_ONLY & (1 << 7))
        self.assertFalse(_TYPE_MASK_ACC_ONLY & (1 << 8))

    def test_type_mask_acc_only_ignores_yaw(self):
        # bit 9 set = ignore yaw
        self.assertTrue(_TYPE_MASK_ACC_ONLY & (1 << 9))

    def test_type_mask_with_yaw_does_not_ignore_yaw(self):
        # bit 9 clear = use yaw
        self.assertFalse(_TYPE_MASK_ACC_WITH_YAW & (1 << 9))

    def test_type_mask_with_yaw_has_bit12(self):
        self.assertTrue(_TYPE_MASK_ACC_WITH_YAW & _ACC_SP_EXTERNAL_FLAG)


# ─────────────────────────────────────────────────────────────────────────────
# Test: AccelerationNed dataclass
# ─────────────────────────────────────────────────────────────────────────────

class TestAccelerationNed(unittest.TestCase):

    def test_default_values(self):
        cmd = AccelerationNed()
        self.assertEqual(cmd.north_m_s2, 0.0)
        self.assertEqual(cmd.east_m_s2,  0.0)
        self.assertEqual(cmd.down_m_s2,  0.0)
        self.assertIsNone(cmd.yaw_rad)

    def test_custom_values(self):
        cmd = AccelerationNed(north_m_s2=2.0, east_m_s2=-1.0,
                              down_m_s2=0.0, yaw_rad=1.57)
        self.assertAlmostEqual(cmd.north_m_s2, 2.0)
        self.assertAlmostEqual(cmd.east_m_s2, -1.0)
        self.assertAlmostEqual(cmd.yaw_rad, 1.57)


# ─────────────────────────────────────────────────────────────────────────────
# Test: send_acceleration_ned type_mask selection
# ─────────────────────────────────────────────────────────────────────────────

class TestSendAcceleration(unittest.TestCase):

    def _make_connected_ac(self):
        ac = AccelerationControl()
        ac._mav = MagicMock()
        ac._mav.target_system    = 1
        ac._mav.target_component = 1
        return ac

    def test_no_yaw_uses_acc_only_mask(self):
        ac = self._make_connected_ac()
        ac.send_acceleration_ned(1.0, 0.0, 0.0, yaw_rad=None)

        call_kwargs = ac._mav.mav.set_position_target_local_ned_send.call_args
        self.assertEqual(call_kwargs.kwargs['type_mask'], _TYPE_MASK_ACC_ONLY)

    def test_explicit_yaw_uses_with_yaw_mask(self):
        ac = self._make_connected_ac()
        ac.send_acceleration_ned(1.0, 0.0, 0.0, yaw_rad=1.57)

        call_kwargs = ac._mav.mav.set_position_target_local_ned_send.call_args
        self.assertEqual(call_kwargs.kwargs['type_mask'], _TYPE_MASK_ACC_WITH_YAW)
        self.assertAlmostEqual(call_kwargs.kwargs['yaw'], 1.57)

    def test_acceleration_fields_correct(self):
        ac = self._make_connected_ac()
        ac.send_acceleration_ned(ax=2.0, ay=-1.5, az=0.3)

        kw = ac._mav.mav.set_position_target_local_ned_send.call_args.kwargs
        self.assertAlmostEqual(kw['afx'],  2.0)
        self.assertAlmostEqual(kw['afy'], -1.5)
        self.assertAlmostEqual(kw['afz'],  0.3)

    def test_position_fields_zero(self):
        ac = self._make_connected_ac()
        ac.send_acceleration_ned(1.0, 0.0, 0.0)

        kw = ac._mav.mav.set_position_target_local_ned_send.call_args.kwargs
        self.assertEqual(kw['x'], 0.0)
        self.assertEqual(kw['y'], 0.0)
        self.assertEqual(kw['z'], 0.0)

    def test_velocity_fields_zero(self):
        ac = self._make_connected_ac()
        ac.send_acceleration_ned(1.0, 0.0, 0.0)

        kw = ac._mav.mav.set_position_target_local_ned_send.call_args.kwargs
        self.assertEqual(kw['vx'], 0.0)
        self.assertEqual(kw['vy'], 0.0)
        self.assertEqual(kw['vz'], 0.0)

    def test_frame_is_local_ned(self):
        ac = self._make_connected_ac()
        ac.send_acceleration_ned(0.0, 0.0, 0.0)

        kw = ac._mav.mav.set_position_target_local_ned_send.call_args.kwargs
        self.assertEqual(kw['coordinate_frame'], _MAV_FRAME_LOCAL_NED)

    def test_hover_sends_zero_acceleration(self):
        ac = self._make_connected_ac()
        ac.hover()

        kw = ac._mav.mav.set_position_target_local_ned_send.call_args.kwargs
        self.assertEqual(kw['afx'], 0.0)
        self.assertEqual(kw['afy'], 0.0)
        self.assertEqual(kw['afz'], 0.0)

    def test_send_acceleration_dataclass(self):
        ac = self._make_connected_ac()
        cmd = AccelerationNed(north_m_s2=3.0, east_m_s2=1.0,
                              down_m_s2=0.0, yaw_rad=None)
        ac.send_acceleration(cmd)

        kw = ac._mav.mav.set_position_target_local_ned_send.call_args.kwargs
        self.assertAlmostEqual(kw['afx'], 3.0)
        self.assertAlmostEqual(kw['afy'], 1.0)


# ─────────────────────────────────────────────────────────────────────────────
# Test: disconnected guard
# ─────────────────────────────────────────────────────────────────────────────

class TestDisconnectedGuard(unittest.TestCase):

    def test_send_without_connect_raises(self):
        ac = AccelerationControl()
        with self.assertRaises(RuntimeError):
            ac.send_acceleration_ned(0.0, 0.0, 0.0)

    def test_hover_without_connect_raises(self):
        ac = AccelerationControl()
        with self.assertRaises(RuntimeError):
            ac.hover()

    def test_arm_without_connect_raises(self):
        ac = AccelerationControl()
        with self.assertRaises(RuntimeError):
            ac.arm()


# ─────────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    unittest.main(verbosity=2)
