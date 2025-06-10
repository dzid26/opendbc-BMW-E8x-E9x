#!/usr/bin/env python3
import numpy as np
from opendbc.car import structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car import get_safety_config
from opendbc.car.bmw.values import CanBus, BmwFlags, CarControllerParams
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.bmw.carcontroller import CarController
from opendbc.car.bmw.carstate import CarState

TransmissionType = structs.CarParams.TransmissionType

# certain driver intervention can be distinguished from maximum estimated wheel turning force
def detect_stepper_override(steer_cmd, steer_act, v_ego, centering_coeff, steer_friction_torque):
  # when steering released (or lost steps), what angle will it return to
  # if we are above that angle, we can detect things
  release_angle = steer_friction_torque / (max(v_ego, 1) ** 2 * centering_coeff)

  override = False
  margin_value = 1
  if abs(steer_cmd) > release_angle:  # for higher angles we steering will not move outward by itself with stepper on
    if steer_cmd > 0:
      override |= steer_act - steer_cmd > margin_value  # driver overrode from right to more right
      override |= steer_act < 0  # releaseAngle -3  # driver overrode from right to opposite direction
    else:
      override |= steer_act - steer_cmd < -margin_value  # driver overrode from left to more left
      override |= steer_act > 0  # -releaseAngle +3 # driver overrode from left to opposite direction
  # else:
    # override |= abs(steerAct) > releaseAngle + marginVal  # driver overrode to an angle where steering will not go by itself
  return override


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  # def __init__(self, CP, CarController, CarState):
  #   super().__init__(CP, CarController, CarState)

  #   self.cp_F = self.CS.get_F_can_parser(CP)
  #   self.can_parsers.append(self.cp_F)
  #   self.cp_aux = self.CS.get_actuator_can_parser(CP)
  #   self.can_parsers.append(self.cp_aux)

  @staticmethod
  # servotronic is a bit more lighter in general and especially at low speeds https://www.spoolstreet.com/threads/servotronic-on-a-335i.1400/page-13#post-117705
  def get_steer_feedforward_servotronic(desired_angle, v_ego): # accounts for steering rack ratio and/or caster nonlinearities https://www.spoolstreet.com/threads/servotronic-on-a-335i.1400/page-15#post-131271
    angle_bp =       [-40.0, -6.0, -4.0, -3.0, -2.0, -1.0, -0.5,  0.5,  1.0,  2.0,  3.0,  4.0,  6.0, 40.0] # deg
    hold_torque_v  = [-6, -2.85, -2.5, -2.25, -2, -1.65, -1, 1, 1.65, 2, 2.25, 2.5, 2.85, 6] # Nm
    hold_torque = np.interp(desired_angle, angle_bp, hold_torque_v)
    return hold_torque # todo add speed component

  @staticmethod
  def get_steer_feedforward(desired_angle, v_ego):
    angle_bp =       [-40.0, -6.0, -4.0, -3.0, -2.0, -1.0, -0.5,  0.5,  1.0,  2.0,  3.0,  4.0,  6.0, 40.0] # deg
    hold_torque_v  = [-6, -2.85, -2.5, -2.25, -2, -1.65, -1, 1, 1.65, 2, 2.25, 2.5, 2.85, 6] # Nm
    hold_torque = np.interp(desired_angle, angle_bp, hold_torque_v)
    return hold_torque # todo add speed component

  def get_steer_feedforward_function(self):
    if self.CP.flags & BmwFlags.SERVOTRONIC:
      return self.get_steer_feedforward_servotronic
    else:
      return self.get_steer_feedforward

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.brand = "bmw"
    if 0x22F in fingerprint[CanBus.SERVO_CAN]:   # Enigne controls speed and reports cruise control status
      ret.flags |= BmwFlags.STEPPER_SERVO_CAN.value

    ret.openpilotLongitudinalControl = True
    ret.radarUnavailable = True
    ret.pcmCruise = False # use OP speed tracking because we control speed using stock cruise speed setpoint or stock cruise is disabled

    ret.autoResumeSng = False
    if 0x200 in fingerprint[CanBus.PT_CAN]:   # Enigne controls speed and reports cruise control status
      ret.flags |= BmwFlags.NORMAL_CRUISE_CONTROL.value # openpilot will inject cruise stalk +/- requests
    elif 0x193 in fingerprint[CanBus.PT_CAN]:   # either DSC or LDM reports cruise control status
      if 0x0D5 not in fingerprint[CanBus.PT_CAN]:                                   # DSC itself applies brakes
        ret.flags |= BmwFlags.DYNAMIC_CRUISE_CONTROL.value # openpilot will inject cruise stalk +/- requests on F-CAN
      else: # LDM sends brake commands
        ret.flags |= BmwFlags.ACTIVE_CRUISE_CONTROL_NO_ACC.value # openpilot will switch between OP and LDM
        ret.autoResumeSng = True #! hopefully
    else: # DSC/DME not sending cruise status and LDM not present - openpilot will be the only requester
      ret.flags |= BmwFlags.ACTIVE_CRUISE_CONTROL_NO_LDM.value
      ret.autoResumeSng = True #! hopefully

    if 0xb8 in fingerprint[CanBus.PT_CAN] or 0xb5 in fingerprint[CanBus.PT_CAN]: # transmission: engine torque requests
      ret.transmissionType = TransmissionType.automatic
    else:
      ret.transmissionType = TransmissionType.manual

    # Detect all wheel drive BMW E90 XI
    if 0xbc in fingerprint[CanBus.PT_CAN]: # XI has a transfer case
      ret.steerRatio = 18.5 # XI has slower steering rack

    if ret.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:  # DCC imperial has higher threshold
      ret.minEnableSpeed = 30. * CV.KPH_TO_MS # if self.CS.is_metric else 20. * CV.MPH_TO_MS
    if ret.flags & BmwFlags.NORMAL_CRUISE_CONTROL:
      ret.minEnableSpeed = 30. * CV.KPH_TO_MS

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.bmw)]
    ret.safetyConfigs[0].safetyParam = 0

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.steerActuatorDelay = 0.4
    ret.steerLimitTimer = 0.4

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    ret.lateralTuning.torque.kp = 1.5 / CarControllerParams.STEER_MAX
    ret.lateralTuning.torque.ki = 0.5 / CarControllerParams.STEER_MAX
    ret.lateralTuning.torque.kf = 5.0 / CarControllerParams.STEER_MAX
    ret.lateralTuning.torque.friction = 0.23 #live parameters
    ret.lateralTuning.torque.latAccelFactor = 1.41 #live parameters
    ret.lateralTuning.torque.useSteeringAngle = False
    ret.lateralTuning.torque.steeringAngleDeadzoneDeg = 0.0 # backlash of stepper?

    ret.longitudinalActuatorDelay  = 1.0 #s, Gas/Brake actuator delay
    ret.longitudinalTuning.kpBP = [0.]
    ret.longitudinalTuning.kpV = [.1]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [0.]

    ret.centerToFront = ret.wheelbase * 0.44

    ret.startAccel = 0.0
    print("Controller: " + ret.lateralTuning.which())

    # has_servotronic = False
    # for fw in car_fw:  # todo check JBBF firmware for $216A
    #   if fw.ecu == "eps" and b"," in fw.fwVersion:
    #     has_servotronic = True

    return ret