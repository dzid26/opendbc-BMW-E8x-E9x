import numpy as np
import math
from opendbc.can.packer import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, AngleSteeringLimits, DT_CTRL, rate_limit
from opendbc.car.interfaces import CarControllerBase, ISO_LATERAL_ACCEL
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.values import CarControllerParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.common.conversions import Conversions as CV

# limit angle rate to both prevent a fault and for low speed comfort (~12 mph rate down to 0 mph)
MAX_ANGLE_RATE = 5  # deg/20ms frame, EPS faults at 12 at a standstill

# Add extra tolerance for average banked road since safety doesn't have the roll
AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation. higher actual roll lowers lateral acceleration
MAX_LATERAL_ACCEL = ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL)  # ~3.6 m/s^2
MAX_LATERAL_JERK = 3.0 + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL)  # ~3.6 m/s^3

STEER_BIAS_MAX = 0.2 # Nm
STEER_VIRTUAL_SPRING_COEFF = 4.0


def get_max_angle_delta(v_ego_raw: float, VM: VehicleModel):
  max_curvature_rate_sec = MAX_LATERAL_JERK / (v_ego_raw ** 2)  # (1/m)/s
  max_angle_rate_sec = math.degrees(VM.get_steer_from_curvature(max_curvature_rate_sec, v_ego_raw, 0))  # deg/s
  return max_angle_rate_sec * (DT_CTRL * CarControllerParams.STEER_STEP)


def get_max_angle(v_ego_raw: float, VM: VehicleModel):
  max_curvature = MAX_LATERAL_ACCEL / (v_ego_raw ** 2)  # 1/m
  return math.degrees(VM.get_steer_from_curvature(max_curvature, v_ego_raw, 0))  # deg


def apply_tesla_steer_angle_limits(apply_angle: float, apply_angle_last: float, v_ego_raw: float, steering_angle: float,
                                   lat_active: bool, limits: AngleSteeringLimits, VM: VehicleModel) -> float:
  v_ego_raw = max(v_ego_raw, 1)

  # *** max lateral jerk limit ***
  max_angle_delta = get_max_angle_delta(v_ego_raw, VM)

  # prevent fault
  max_angle_delta = min(max_angle_delta, MAX_ANGLE_RATE)
  new_apply_angle = rate_limit(apply_angle, apply_angle_last, -max_angle_delta, max_angle_delta)

  # *** max lateral accel limit ***
  max_angle = get_max_angle(v_ego_raw, VM)
  new_apply_angle = np.clip(new_apply_angle, -max_angle, max_angle)

  # angle is current angle when inactive
  if not lat_active:
    new_apply_angle = steering_angle

  # prevent fault
  return float(np.clip(new_apply_angle, -limits.STEER_ANGLE_MAX, limits.STEER_ANGLE_MAX))


def get_safety_CP():
  # We use the TESLA_MODEL_Y platform for lateral limiting to match safety
  # A Model 3 at 40 m/s using the Model Y limits sees a <0.3% difference in max angle (from curvature factor)
  from opendbc.car.tesla.interface import CarInterface
  return CarInterface.get_non_essential_params("TESLA_MODEL_Y")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0
    self.steeringRateDeg_last = 0
    
    self.driver_override_angle_last = 0
    self.driver_override_angle_delta_last = 0
    
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCAN(self.packer)

    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # Tesla EPS enforces disabling steering on heavy lateral override force.
    # When enabling in a tight curve, we wait until user reduces steering force to start steering.
    # Canceling is done on rising edge and is handled generically with CC.cruiseControl.cancel
    lat_active = CC.latActive and CS.hands_on_level < 3
    
    steeringAccDeg = (CS.out.steeringRateDeg - self.steeringRateDeg_last) / DT_CTRL # todo replace with CAN timestamps delta
    self.steeringRateDeg_last = CS.out.steeringRateDeg # this signal should be EPS motor speed, but it's not available
    STEERING_RIM_WEIGHT = 2 # kg
    STEERING_RIM_RADIUS = 0.15 # m
    FOREARM_WEIGHT = 1.5 #kg
    STEERING_MOMENT = STEERING_RIM_WEIGHT * STEERING_RIM_RADIUS ** 2 # kg*m^2
    STEERING_FOREARM_MOMENT = FOREARM_WEIGHT * STEERING_RIM_RADIUS ** 2 # kg*m^2
    steering_inertia = steeringAccDeg * CV.DEG_TO_RAD * STEERING_MOMENT
    steering_torque_comp = CS.out.steeringTorque + steering_inertia

    if self.frame % 2 == 0:
      ## create virtual spring effect
      # add deadzone to avoid steer torque offset and torque due to gravity and inertia
      steering_torque_deadzone = steering_torque_comp - np.clip(steering_torque_comp, -STEER_BIAS_MAX, STEER_BIAS_MAX)
      driver_torque_to_angle = min(get_max_angle(max(1, CS.out.vEgoRaw), self.VM) / STEER_VIRTUAL_SPRING_COEFF, 90)

      driver_override_angle = steering_torque_deadzone * driver_torque_to_angle

      # limit angle acceleration to allow arm+steering to keep up with the rotation and avoid oscillations
      driver_override_angle_delta = driver_override_angle - self.driver_override_angle_last
      steering_driver_acc_ff = steering_torque_comp / (STEERING_MOMENT + STEERING_FOREARM_MOMENT) * CV.RAD_TO_DEG * (DT_CTRL * 2) # deg/s^2
      self.driver_override_angle_delta_last = rate_limit(driver_override_angle_delta, self.driver_override_angle_delta_last, -steering_driver_acc_ff, steering_driver_acc_ff)
      self.driver_override_angle_last = driver_torque_to_angle + self.driver_override_angle_delta_last

      # Angular rate limit based on speed
      self.apply_angle_last = apply_tesla_steer_angle_limits(actuators.steeringAngleDeg + self.driver_override_angle_last, self.apply_angle_last,
                                                             CS.out.vEgoRaw, CS.out.steeringAngleDeg, lat_active,
                                                             CarControllerParams.ANGLE_LIMITS, self.VM)
      

      can_sends.append(self.tesla_can.create_steering_control(self.apply_angle_last, lat_active))

    if self.frame % 10 == 0:
      can_sends.append(self.tesla_can.create_steering_allowed())

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      if self.frame % 4 == 0:
        state = 13 if CC.cruiseControl.cancel else 4  # 4=ACC_ON, 13=ACC_CANCEL_GENERIC_SILENT
        accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        cntr = (self.frame // 4) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive))

    else:
      # Increment counter so cancel is prioritized even without openpilot longitudinal
      if CC.cruiseControl.cancel:
        cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False))

    # TODO: HUD control
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    new_actuators.torque = steering_torque_comp # debugging
    self.frame += 1
    return new_actuators, can_sends

 
  
if __name__ == "__main__":
  
  VM = VehicleModel(get_safety_CP())
  
  for v_ego_kph in [10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120]:
    v_ego_raw = v_ego_kph * 0.277778
    print(f"v_ego: {v_ego_kph:.0f} kph, max_angle_delta: {get_max_angle_delta(v_ego_raw, VM):.2f} deg/20ms, max_angle: {get_max_angle(v_ego_raw, VM):.2f} deg")
