#!/usr/bin/env python3
import unittest
import numpy as np
from panda import Panda
from panda.tests.libpanda import libpanda_py
import panda.tests.safety.common as common
from panda.tests.safety.common import make_msg

MS_TO_KPH = 3.6
SAMPLING_FREQ = 100 #Hz

ANGLE_MAX_BP = [5., 15., 25.]  #m/s
ANGLE_MAX = [228.6, 42.3, 15.2]  #deg

ANGLE_RATE_BP = [0., 5., 25.]      # m/s
ANGLE_RATE_WINDUP = [500., 80., 15.]     #deg/s windup rate limit
ANGLE_RATE_UNWIND = [500., 350., 40.]  #deg/s unwind rate limit

TORQUE_RATE_BP = [0., 5., 15.]      # m/s
TORQUE_RATE_MAX = [16., 8., 1.]     #Nm/10ms

TX_MSGS = [[0x194, 0],[0x194, 1], [0xFA, 2]]

CAN_BMW_SPEED_FAC = 0.1
CAN_BMW_ANGLE_FAC = 0.04395
CAN_ACTUATOR_POS_FAC = 0.125
CAN_ACTUATOR_TQ_FAC = 0.125

MODE_OFF = 0
MODE_TORQUE = 1
MODE_ANGLE = 2

def twos_comp(val, bits):
  if val >= 0:
    return val
  else:
    return (2**bits) + val

def sign(a):
  if a > 0:
    return 1
  else:
    return -1



class TestBmwSafety(unittest.TestCase):
  def setUp(self):
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(Panda.SAFETY_BMW, 0)
    self.safety.init_tests()

  def _angle_meas_msg(self, angle, angle_rate):
    to_send = make_msg(0, 0xc4, 7)

    angle_int = int(angle / CAN_BMW_ANGLE_FAC)
    angle_t = twos_comp(angle_int, 16) # signed

    angle_rate_int = int(angle_rate / CAN_BMW_ANGLE_FAC)
    angle_rate_t = twos_comp(angle_rate_int, 16) # signed

    to_send[0].RDLR = (angle_t & 0xFFFF ) | ((angle_rate_t & 0x00FF) << 24)
    to_send[0].RDHR = (angle_rate_t & 0xFF00) >> 8

    return to_send

  def _set_prev_angle(self, t):
    t = int(t * -SAMPLING_FREQ)
    self.safety.set_bmw_desired_angle_last(t)


  def _actuator_angle_cmd_msg(self, mode, torque_req, angle_delta):

    to_send = make_msg(2, 558)

    cnt = 0

    steer_angle = int(twos_comp(angle_delta / CAN_ACTUATOR_POS_FAC, 16)) # signed angle_delta
    steer_tq = int(twos_comp(torque_req / CAN_ACTUATOR_TQ_FAC, 11))

    checksum = (cnt + mode + steer_angle + steer_tq)
    checksum = checksum >> 8 + checksum & 0xFF
    checksum = checksum & 0xFF

    to_send[0].RDLR = (checksum & 0xFF) | ((cnt & 0xF) << 8) | ((mode & 0x3) << 12) | ((steer_angle & 0xFFFF) << 16)
    to_send[0].RDHR = (steer_tq & 0xFF) << 0
    return to_send


  def _speed_msg(self, speed):
    to_send = make_msg(0, 0x1a0)
    speed = int(speed / CAN_BMW_SPEED_FAC)
    to_send[0].RDLR = (speed & 0xFFF)

    return to_send

  def _brake_msg(self, brake):
    to_send = make_msg(0, 168)
    to_send[0].RDHR = (brake * 0x3) << (61-32)

    return to_send

  def _cruise_button_msg(self, buttons_bitwise): #todo: read creuisesate
    to_send = make_msg(0, 404, 4)
    const_0xFC = 0xFC
    buttons_bitwise = buttons_bitwise & 0xFF
    if (buttons_bitwise != 0): #if any button pressed
      request_0xF = 0xF
    else:
      request_0xF = 0x0

    if (buttons_bitwise & (1<<7 | 1<<4)): #if any cancel pressed
      notCancel = 0x0
    else:
      notCancel = 0xF

    to_send[0].RDLR = (buttons_bitwise << 16) | (request_0xF << 12) | (notCancel << 4) | (const_0xFC << 24)
    return to_send

  def test_angle_cmd_when_enabled(self): #todo add faulty BMW angle sensor (step angle)
    # when controls are allowed, angle cmd rate limit is enforced
    speeds = [ 5, 10, 15, 50, 100] #kph
    for s in speeds:
      max_angle      = np.interp(int(s/CAN_BMW_SPEED_FAC) * CAN_BMW_SPEED_FAC / MS_TO_KPH, ANGLE_MAX_BP, ANGLE_MAX) #deg
      max_delta_up   = np.interp(int(s/CAN_BMW_SPEED_FAC) * CAN_BMW_SPEED_FAC / MS_TO_KPH, ANGLE_RATE_BP, ANGLE_RATE_WINDUP) #deg
      max_delta_down = np.interp(int(s/CAN_BMW_SPEED_FAC) * CAN_BMW_SPEED_FAC / MS_TO_KPH, ANGLE_RATE_BP, ANGLE_RATE_UNWIND) #deg
      max_tq_rate    = np.interp(int(s/CAN_BMW_SPEED_FAC) * CAN_BMW_SPEED_FAC / MS_TO_KPH, TORQUE_RATE_BP, TORQUE_RATE_MAX) #Nm/10ms

      # use integer rounded value for interpolation ^^, same as what panda will receive

      self.safety.set_controls_allowed(1)
      self.assertTrue(self.safety.get_controls_allowed())
      self.safety.safety_rx_hook(self._speed_msg(s)) #receive speed which triggers angle limits to be updated to be later used by tx

      # Stay within limits
      # Up
      self.safety.safety_rx_hook(self._angle_meas_msg(max_angle, max_delta_up))
      self.assertTrue(self.safety.get_controls_allowed())

      self.assertEqual(1, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(MODE_ANGLE, max_tq_rate, min(max_angle, max_delta_up))),\
          'Speed: %f, Angle: %f, Delta: %f' % (s, max_angle, max_delta_up))
      self.assertTrue(self.safety.get_controls_allowed())

      # Stay within limits
      # Down
      self.safety.safety_rx_hook(self._angle_meas_msg(-max_angle, -max_delta_down))
      self.assertTrue(self.safety.get_controls_allowed())

      self.assertEqual(1, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(MODE_ANGLE, -max_tq_rate, -min(max_angle, max_delta_down))),\
          'Speed: %f, Angle: %f, Delta: %f' % (s, max_angle, max_delta_down))
      self.assertTrue(self.safety.get_controls_allowed())

      # Reset to 0 angle
      self.safety.set_controls_allowed(1)
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertEqual(1, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(MODE_OFF, 0, 0)))
      self.assertTrue(self.safety.get_controls_allowed())

      # Up
      # # Inject too large measured angle
      self.safety.set_controls_allowed(1)
      self.safety.safety_rx_hook(self._angle_meas_msg(max_angle+1, max_delta_up))
      self.assertFalse(self.safety.get_controls_allowed())

      # Reset to 0 angle
      self.safety.set_controls_allowed(1)
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertEqual(1, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(MODE_OFF, 0, 0)))
      self.assertTrue(self.safety.get_controls_allowed())

      # Up
      # Inject too high measured rate
      self.safety.set_controls_allowed(1)
      self.safety.safety_rx_hook(self._angle_meas_msg(max_angle, max_delta_up+1))
      self.assertFalse(self.safety.get_controls_allowed(),\
          'Speed: %f, Angle: %f, Delta: %f' % (s, max_angle, max_delta_down))

      # Reset to 0 angle
      self.safety.set_controls_allowed(1)
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertEqual(1, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(MODE_OFF, 0, 0)))
      self.assertTrue(self.safety.get_controls_allowed())

      # Up
      # Inject too high command angle rate - since last angle value is 0, sending angle value represents angle rate
      self.safety.set_controls_allowed(1)
      self.assertEqual(0, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(MODE_ANGLE, 0, min(max_angle, max_delta_up) + 1.)),\
          'Speed: %f, Angle: %f, Delta: %f' % (s, max_angle, max_delta_up))

      # Up
      # Inject too high command torque rate - since last value of torque is 0, sending torque value represents torque rate
      self.safety.set_controls_allowed(1)
      self.assertEqual(0, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(MODE_TORQUE, max_tq_rate + 1., 0)),\
          'Speed: %f, Torque: %f' % (s, max_tq_rate))

      # Reset to 0 angle
      self.safety.set_controls_allowed(1)
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertEqual(1, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(MODE_OFF, 0, 0)))
      self.assertTrue(self.safety.get_controls_allowed())


      # Down
      # Inject too large measured angle
      self.safety.set_controls_allowed(1)
      self.safety.safety_rx_hook(self._angle_meas_msg(-max_angle-1, -max_delta_down))
      self.assertFalse(self.safety.get_controls_allowed())

      # Reset to 0 angle
      self.safety.set_controls_allowed(1)
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertEqual(1, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(MODE_OFF, 0, 0)))
      self.assertTrue(self.safety.get_controls_allowed())

      #Down
      # Inject too high measured rate
      self.safety.set_controls_allowed(1)
      self.safety.safety_rx_hook(self._angle_meas_msg(-max_angle, -max_delta_down - 1))
      self.assertFalse(self.safety.get_controls_allowed())

      #Down
      # Inject too high command rate
      self.safety.set_controls_allowed(1)
      self.assertEqual(0, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(MODE_ANGLE, 0, -min(max_angle, max_delta_down)-1.)),\
          'Speed: %f, Angle: %f, Delta: %f' % (s, max_angle, max_delta_down))

      #Down
      # Inject too high command torque rate - since last value of torque is 0, sending torque value represents torque rate
      self.safety.set_controls_allowed(1)
      self.assertEqual(0, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(MODE_TORQUE, -max_tq_rate - 1., 0)),\
          'Speed: %f, Torque: %f' % (s, max_tq_rate))

      # Check desired steer should be the same as steer angle when controls are off
      self.safety.set_controls_allowed(0)
      self.assertEqual(0, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(MODE_OFF, 0, 0)),\
          'Speed: %f, Angle: %f, Delta: %f' % (s, max_angle, max_delta_down))

  def test_angle_cmd_when_disabled(self):
    self.safety.set_controls_allowed(0)

    self._set_prev_angle(0)
    self.assertFalse(self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(MODE_OFF, 0, 0)))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_brake_disengage(self):
    self.safety.set_controls_allowed(1)
    self.safety.safety_rx_hook(self._brake_msg(0))
    self.assertTrue(self.safety.get_controls_allowed())


    self.safety.safety_rx_hook(self._speed_msg(10)) #ALLOW_DEBUG keeps the actuator active even at 0 speed
    self.safety.safety_rx_hook(self._brake_msg(1))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_cruise_buttons(self):
    self.safety.set_controls_allowed(1)
    self.assertTrue(self.safety.get_controls_allowed())

    self.safety.safety_rx_hook(self._cruise_button_msg(0x0)) # No button pressed
    self.assertTrue(self.safety.get_controls_allowed())

    self.safety.safety_rx_hook(self._speed_msg(10)) #ALLOW_DEBUG keeps the actuator active even at 0 speed
    self.safety.safety_rx_hook(self._cruise_button_msg(0x10)) # Cancel button
    self.assertFalse(self.safety.get_controls_allowed())

    self.safety.safety_rx_hook(self._cruise_button_msg(0x0)) # No button pressed
    self.assertFalse(self.safety.get_controls_allowed())

if __name__ == "__main__":
  unittest.main()
