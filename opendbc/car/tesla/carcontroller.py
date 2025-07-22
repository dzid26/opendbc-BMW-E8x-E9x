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
STEER_MAX_OVERRIDE_TORQUE = 2.0 # Nm before disengages for sure
STEER_OVERRRIDE_MAX_LAT_ACCEL = 1.0

STEERING_RIM_WEIGHT = 2 # kg
STEERING_RIM_RADIUS = 0.15 # m
FOREARM_WEIGHT = 1.5 #kg
STEERING_MOMENT = STEERING_RIM_WEIGHT * STEERING_RIM_RADIUS ** 2 # kg*m^2
STEERING_FOREARM_MOMENT = FOREARM_WEIGHT * STEERING_RIM_RADIUS ** 2 # kg*m^2
MAX_TORQUE_DUE_TO_ACCEL = 0.2 # Nm


def get_max_angle_delta(v_ego_raw: float, VM: VehicleModel):
  max_curvature_rate_sec = MAX_LATERAL_JERK / (v_ego_raw ** 2)  # (1/m)/s
  max_angle_rate_sec = math.degrees(VM.get_steer_from_curvature(max_curvature_rate_sec, v_ego_raw, 0))  # deg/s
  return max_angle_rate_sec * (DT_CTRL * CarControllerParams.STEER_STEP)


def get_max_angle(v_ego_raw: float, VM: VehicleModel, lat_accel: float = MAX_LATERAL_ACCEL):
  max_curvature = lat_accel / (v_ego_raw ** 2)  # 1/m
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

STABLE_TORQUE_TO_ANGLE = 6
def applyOverrideAngle(driverTorque: float, vEgo: float, apply_angle: float, apply_angle_last: float, 
                     apply_angle_delta_last: float, VM: VehicleModel, sample_time: float = DT_CTRL) -> tuple[float, float]:

    # virtual spring from lateral acceleration
    steering_torque_deadzone = driverTorque - np.clip(driverTorque, -STEER_BIAS_MAX, STEER_BIAS_MAX)
    # todo maybe saturate target lateral acc based on safety limit minus actual lateral acc
    torque_to_angle = min(get_max_angle(max(1, vEgo), VM, STEER_OVERRRIDE_MAX_LAT_ACCEL) / (STEER_MAX_OVERRIDE_TORQUE - STEER_BIAS_MAX), STABLE_TORQUE_TO_ANGLE)
    override_angle_target = steering_torque_deadzone * torque_to_angle
    
    
    # # desired system characteristics
    # desired_steering_weight = STEERING_MOMENT /4   # kg·m²
    # k_spring = 1 / (torque_to_angle * CV.DEG_TO_RAD)
    # b_crit = 2 * np.sqrt(desired_steering_weight * k_spring)  # critically damped - no overshoots

    # velocity = apply_angle_delta_last / sample_time * CV.DEG_TO_RAD
    # position = (apply_angle_last - apply_angle) * CV.DEG_TO_RAD
    # input_torque = steering_torque_deadzone
    
    # desired_alpha = (input_torque - b_crit * velocity - k_spring * position) / desired_steering_weight  # rad/s²
    
    # # limit acceleration to make sure torsion bar torque doesn't change much during acceleration
    # max_alpha = MAX_TORQUE_DUE_TO_ACCEL / STEERING_MOMENT  # rad/s²
    # desired_alpha = np.clip(desired_alpha, -max_alpha, max_alpha)
    
    # new_velocity = (velocity + desired_alpha * sample_time)
    # new_angle = apply_angle_last + new_velocity * CV.RAD_TO_DEG * sample_time
    new_angle = override_angle_target
    
    return new_angle, override_angle_target
  
class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0
    self.apply_angle_delta_last = 0
    self.steeringRateDeg_last = 0
    
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
    steering_inertia = steeringAccDeg * CV.DEG_TO_RAD * STEERING_MOMENT
    steering_torque_comp = CS.out.steeringTorque + steering_inertia

    if self.frame % 2 == 0:
      steering_angle_with_override, _ = applyOverrideAngle(CS.out.steeringTorque, CS.out.vEgoRaw, actuators.steeringAngleDeg, self.apply_angle_last, self.apply_angle_delta_last, self.VM, DT_CTRL * 2)
      
      # Angular rate limit based on speed
      apply_angle_rate_limited = apply_tesla_steer_angle_limits(steering_angle_with_override, self.apply_angle_last,
                                                             CS.out.vEgoRaw, CS.out.steeringAngleDeg, lat_active,
                                                             CarControllerParams.ANGLE_LIMITS, self.VM)
      self.apply_angle_delta_last = apply_angle_rate_limited - self.apply_angle_last
      self.apply_angle_last = apply_angle_rate_limited

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

  import matplotlib.pyplot as plt
  import numpy as np
  from matplotlib.widgets import MultiCursor

  # User-friendly specification (just key points)
  time_keypoints = [  0, .1,  1,  1.2, 1.7,   3.5, 4, 9, 10]      # Time in seconds
  torque_keypoints = [0, 0.5, 0.5, 0.9, 0.4, 0.6, 0, -1, 0]   # Torque at keypoints (Nm)
  v_ego = 4
  
  # Generate high-resolution time array (for smooth calculations)
  time = np.linspace(time_keypoints[0], time_keypoints[-1],int(time_keypoints[-1] / DT_CTRL))
  driver_torque = np.interp(time, time_keypoints, torque_keypoints)

  angle_target_last = 0
  angle_target_delta_last = 0
  override_angles, target_override_angles, override_angle_delta = [], [], []
  for tq in driver_torque:
      angle_target, target_angle = applyOverrideAngle(tq, v_ego, 0, angle_target_last, angle_target_delta_last, VM)
      angle_target_delta_last = angle_target - angle_target_last
      angle_target_last = angle_target
      override_angles.append(angle_target)
      target_override_angles.append(target_angle)
      override_angle_delta.append(angle_target_delta_last)
  
  # Plotting
  fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

  multi = MultiCursor(fig.canvas, (ax1, ax2), color='r', lw=1, horizOn=False, vertOn=True)

  # Torque profile
  ax1.plot(time, driver_torque, 'g-', linewidth=2, label='Torque (Nm)')
  ax1.set_ylabel('Torque [Nm]')
  ax1.legend(loc='upper right')
  ax1.grid(True)

  # Angle response
  ax2.plot(time, override_angles, 'b-', label='Override Angle (deg)')
  ax2.plot(time, target_override_angles, 'r--', label='Override Angle target (deg)')
  ax2.plot(time, override_angle_delta, 'k:', label='Override Angle Delta (deg)')
  ax2.set_xlabel('Time [s]')
  ax2.set_ylabel('Angle [deg]')
  ax2.legend(loc='upper right')
  ax2.grid(True)

  plt.tight_layout()
  plt.show()
