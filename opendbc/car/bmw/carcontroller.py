from cereal import car
from opendbc.car import DT_CTRL, apply_dist_to_meas_limits, apply_hysteresis
from opendbc.car.bmw import bmwcan
from opendbc.car.bmw.bmwcan import SteeringModes, CruiseStalk
from opendbc.car.bmw.values import CarControllerParams, CanBus, BmwFlags
from opendbc.car.interfaces import CarControllerBase
from opendbc.can.packer import CANPacker
from opendbc.car.common.conversions import Conversions as CV

VisualAlert = car.CarControl.HUDControl.VisualAlert

# DO NOT CHANGE: Cruise control step size
CC_STEP = 1 # cruise single click jump - always 1 - interpreted as km or miles depending on DSC or DME set units
CRUISE_STALK_IDLE_TICK_STOCK = 0.2 # stock cruise stalk CAN frequency when stalk is not pressed is 5Hz
CRUISE_STALK_HOLD_TICK_STOCK = 0.05 # stock cruise stalk CAN frequency when stalk is pressed is 20Hz

CRUISE_STALK_SINGLE_TICK = CRUISE_STALK_IDLE_TICK_STOCK # we will send also at 5Hz in between stock messages to emulate single presses
CRUISE_STALK_HOLD_TICK = 0.01 # emulate held stalk, 100Hz makes stock messages be ignored

CRUISE_SPEED_HYST_GAP = CC_STEP * 0.6  # between >0.5 and <1 to avoid cruise speed toggling. More than 0.5 to add some phase lead
ACCEL_HYST_GAP = 0.05 # m/s^2

ACCEL_HOLD_MEDIUM = 0.4
DECEL_HOLD_MEDIUM = -0.6
ACCEL_HOLD_STRONG = 1.2
DECEL_HOLD_STRONG = -1.2

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    super().__init__(dbc_name, CP)
    self.flags = CP.flags
    self.min_cruise_speed = CP.minEnableSpeed
    self.cruise_units = None

    self.cruise_cancel = False  # local cruise control cancel
    self.cruise_enabled_prev = False
    # redundant safety check with the board
    self.apply_steer_last = 0
    self.last_cruise_rx_timestamp = 0 # stock cruise buttons
    self.last_cruise_tx_timestamp = 0 # openpilot commands
    self.tx_cruise_stalk_counter_last = 0
    self.rx_cruise_stalk_counter_last = -1
    self.cruise_speed_with_hyst = 0
    self.accel_with_hyst = 0
    self.accel_with_hyst_last = 0
    self.calc_desired_speed = 0

    self.cruise_bus = CanBus.PT_CAN
    if CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      self.cruise_bus = CanBus.F_CAN


    self.packer = CANPacker(dbc_name)


  def update(self, CC, CS, now_nanos):

    actuators = CC.actuators
    can_sends = []

    self.cruise_units = (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)


    # *** hysteresis - trend is your friend ***
    # avoids cruise speed toggling and biases next request toward the direction of the previous one
    self.cruise_speed_with_hyst = apply_hysteresis(CS.out.cruiseState.speed, self.cruise_speed_with_hyst, CRUISE_SPEED_HYST_GAP / self.cruise_units)
    if not CS.out.cruiseState.enabled:
      self.cruise_speed_with_hyst = CS.out.vEgoCluster

    # acceleration target hysteresis - avoids entering / leaving hold stalk emulation to frequently, etc
    self.accel_with_hyst = apply_hysteresis(actuators.accel, self.accel_with_hyst, ACCEL_HYST_GAP)


    # *** desired speed model ***
    # detect filtered acceleration sign change and reset speed calc on change
    accel_zero_cross = self.accel_with_hyst * self.accel_with_hyst_last < 0
    self.accel_with_hyst_last = self.accel_with_hyst
    if accel_zero_cross or not CC.enabled or CS.out.gasPressed:
      self.calc_desired_speed = CS.out.vEgoCluster
    self.calc_desired_speed = self.calc_desired_speed + actuators.accel * DT_CTRL
    speed_err_req = (self.calc_desired_speed - self.cruise_speed_with_hyst) * self.cruise_units
    speed_err_act = self.calc_desired_speed - CS.out.vEgoCluster

    # detect incoming CruiseControlStalk message by observing counter change (message arrives at only 5Hz when nothing pressed)
    if CS.cruise_stalk_counter != self.rx_cruise_stalk_counter_last:
      self.tx_cruise_stalk_counter_last = CS.cruise_stalk_counter
      # stock message was sent some time in between control samples:
      self.last_cruise_rx_timestamp = now_nanos
    self.rx_cruise_stalk_counter_last = CS.cruise_stalk_counter





    # *** send cruise control stalk message at different rates and manage counters ***
    def cruise_cmd(cmd, hold=False):
      time_since_cruise_sent =      (now_nanos - self.last_cruise_tx_timestamp) / 1e9 + DT_CTRL / 10 # add half task sample time to account for latency
      time_since_cruise_received =  (now_nanos - self.last_cruise_rx_timestamp) / 1e9 + DT_CTRL / 10 # add half task sample time to account for latency
      # send single cmd with an effective rate slower than held stalk rate
      if not hold:
        send = time_since_cruise_sent > CRUISE_STALK_SINGLE_TICK \
          and time_since_cruise_received > CRUISE_STALK_HOLD_TICK_STOCK/2 - DT_CTRL \
          and time_since_cruise_received < CRUISE_STALK_IDLE_TICK_STOCK/2 + DT_CTRL
      else:
        # use faster rate to emulate held stalk. Time first message such that subsequent one will nullify stock message:
        send = hold and time_since_cruise_sent > CRUISE_STALK_HOLD_TICK
      if send:
        tx_cruise_stalk_counter = self.tx_cruise_stalk_counter_last + 1
        # avoid counter clash with a potential upcoming message from stock cruise
        if tx_cruise_stalk_counter == CS.cruise_stalk_counter + 1:
          # avoid clashing with upcoming stock message
          # sometimes upcoming stock message is overshadowed by us, so also avoid clashing with one after that
          tx_cruise_stalk_counter = tx_cruise_stalk_counter + 2
        tx_cruise_stalk_counter = tx_cruise_stalk_counter % 0xF
        can_sends.append(bmwcan.create_accel_command(self.packer, cmd, self.cruise_bus, tx_cruise_stalk_counter))
        self.tx_cruise_stalk_counter_last = tx_cruise_stalk_counter
        self.last_cruise_tx_timestamp = now_nanos

    # *** cruise control cancel signal ***
    # CC.cruiseControl.cancel can't be used because it is always false because pcmCruise = False because we need OP speed tracker
    # CC.enabled appears after cruiseState.enabled, so we need to check rising edge to prevent instantaneous cancel after cruise is enabled
    # This is because CC.enabled comes from controld and CS.out.cruiseState.enabled is from card threads
    if not CC.enabled and self.cruise_enabled_prev:
      self.cruise_cancel = True
    # if we need to go below cruise speed, request cancel and coast while steering turns off softly
    if (CS.out.cruiseState.speedCluster - self.min_cruise_speed) < 0.1 and actuators.accel < -0.1 \
      and speed_err_act < -1 and CS.out.vEgoCluster - self.min_cruise_speed < 0.4:
      self.cruise_cancel = True
    # keep requesting cancel until the cruise is disabled
    if not CS.out.cruiseState.enabled:
      self.cruise_cancel = False

    cruise_stalk_human_pressing = CS.cruise_stalk_resume or CS.cruise_stalk_cancel or CS.cruise_stalk_speed != 0

    if not cruise_stalk_human_pressing and CS.out.cruiseState.enabled:
      if self.cruise_cancel:
        cruise_cmd(CruiseStalk.cancel)
        print("cancel")
      elif CC.enabled:
        if (self.accel_with_hyst > ACCEL_HOLD_STRONG or (self.accel_with_hyst > ACCEL_HOLD_MEDIUM and speed_err_act > 3.0)) \
            and not speed_err_req < -10*CV.KPH_TO_MS*self.cruise_units:
          cruise_cmd(CruiseStalk.plus5, hold=True) # produces up to 1.2 m/s2
        elif (self.accel_with_hyst < DECEL_HOLD_STRONG or (self.accel_with_hyst < DECEL_HOLD_MEDIUM and speed_err_act < -3.0)) \
            and not speed_err_req > 10*CV.KPH_TO_MS*self.cruise_units and not CS.out.gasPressed:
          cruise_cmd(CruiseStalk.minus5, hold=True) # produces down to -1.4 m/s2
        elif (self.accel_with_hyst > ACCEL_HOLD_MEDIUM or speed_err_act > 2.0) \
            and not speed_err_req < -5*CV.KPH_TO_MS*self.cruise_units:
          cruise_cmd(CruiseStalk.plus1, hold=True) # produces up to 0.8 m/s2
        elif (self.accel_with_hyst < DECEL_HOLD_MEDIUM or speed_err_act < -2.0) \
            and not speed_err_req > 5*CV.KPH_TO_MS*self.cruise_units and not CS.out.gasPressed:
          cruise_cmd(CruiseStalk.minus1, hold=True) # produces down to -0.8 m/s2
        elif speed_err_req > max(CC_STEP/2, 0.9*CV.KPH_TO_MS*self.cruise_units) and (self.accel_with_hyst > 0.0 or CS.out.gasPressed):
          cruise_cmd(CruiseStalk.plus1)
        elif speed_err_req < -max(CC_STEP/2, 0.9*CV.KPH_TO_MS*self.cruise_units) and self.accel_with_hyst < 0.0 and not CS.out.gasPressed:
          cruise_cmd(CruiseStalk.minus1)



    if self.flags & BmwFlags.STEPPER_SERVO_CAN:
      steer_error =  not CC.latActive and CC.enabled
      if not steer_error: # don't send steer CAN tx if steering is unavailable
        # *** apply steering torque ***
        if CC.enabled:
          new_steer = actuators.steer * CarControllerParams.STEER_MAX
          # explicitly clip torque before sending on CAN
          apply_steer = apply_dist_to_meas_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps,
                                             CarControllerParams.STEER_DELTA_UP, CarControllerParams.STEER_DELTA_DOWN,
                                             CarControllerParams.STEER_ERROR_MAX, CarControllerParams.STEER_MAX)
          can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.TorqueControl, apply_steer))
        elif not CS.cruise_stalk_cancel and not CS.out.brakePressed and not CS.out.gasPressed and self.apply_steer_last != 0:
          can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.SoftOff, self.apply_steer_last))
          apply_steer = CS.out.steeringTorqueEps
        else:
          apply_steer = 0
          can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.Off))
        self.apply_steer_last = apply_steer

    # debug
    if CC.enabled and (self.frame % 10) == 0: #slow print
      frame_number = self.frame
      print(f"Steering req: {actuators.steer}, Speed: {CS.out.vEgoCluster}, Frame number: {frame_number}")

    self.cruise_enabled_prev = CC.enabled

    new_actuators = actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / CarControllerParams.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    new_actuators.speed = self.calc_desired_speed
    new_actuators.accel = speed_err_req

    self.frame += 1
    return new_actuators, can_sends
