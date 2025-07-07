from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs, create_button_events
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.bmw.values import DBC, CanBus, BmwFlags, CruiseSettings

ButtonType = structs.CarState.ButtonEvent.Type

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["TransmissionDataDisplay"]['ShiftLeverPosition']
    self.gas_kickdown = False

    self.cluster_min_speed = CruiseSettings.CLUSTER_OFFSET

    self.is_metric = None
    self.cruise_stalk_speed = 0
    self.cruise_stalk_resume = False
    self.cruise_stalk_cancel = False
    self.cruise_stalk_cancel_up = False
    self.cruise_stalk_cancel_dn = False
    self.cruise_stalk_counter = 0
    self.prev_cruise_stalk_speed = 0
    self.prev_cruise_stalk_resume = self.cruise_stalk_resume
    self.prev_cruise_stalk_cancel = self.cruise_stalk_cancel

    self.right_blinker_pressed = False
    self.left_blinker_pressed = False
    self.other_buttons = False
    self.prev_gas_pressed = False
    self.dtc_mode = False

  def update(self, can_parsers) -> structs.CarState:
    cp_PT = can_parsers[Bus.pt]
    cp_F = can_parsers[Bus.body]
    cp_aux = can_parsers[Bus.alt]

    ret = structs.CarState()

    # set these prev states at the beginning because they are used outside the update()
    self.prev_cruise_stalk_speed = self.cruise_stalk_speed
    self.prev_cruise_stalk_resume = self.cruise_stalk_resume
    self.prev_cruise_stalk_cancel = self.cruise_stalk_cancel

    ret.doorOpen = False # not any([cp.vl["SEATS_DOORS"]['DOOR_OPEN_FL'], cp.vl["SEATS_DOORS"]['DOOR_OPEN_FR']
    ret.seatbeltUnlatched = False # not cp.vl["SEATS_DOORS"]['SEATBELT_DRIVER_UNLATCHED']

    ret.brakePressed = cp_PT.vl["EngineAndBrake"]['BrakePressed'] != 0
    ret.parkingBrake = cp_PT.vl["Status_contact_handbrake"]["Handbrake_pulled_up"] != 0
    ret.gas = cp_PT.vl['AccPedal']["AcceleratorPedalPercentage"]
    # on some cars, when cruise is engaged, half pressed pedal becomes "KickDownPressed", even without pressing kickdown end stop
    ret.gasPressed = cp_PT.vl['AccPedal']["AcceleratorPedalPressed"] != 0 or cp_PT.vl['AccPedal']["KickDownPressed"] != 0
    self.gas_kickdown = cp_PT.vl['AccPedal']["KickDownPressed"] != 0 #BMW has kickdown button at the bottom of the pedal

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp_PT.vl["WheelSpeeds"]["Wheel_FL"],
      cp_PT.vl["WheelSpeeds"]["Wheel_FR"],
      cp_PT.vl["WheelSpeeds"]["Wheel_RL"],
      cp_PT.vl["WheelSpeeds"]["Wheel_RR"],
    )
    ret.vEgoRaw = cp_PT.vl['Speed']["VehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = ret.vEgo + CruiseSettings.CLUSTER_OFFSET * CV.KPH_TO_MS
    ret.standstill = not cp_PT.vl['Speed']["MovingForward"] and not cp_PT.vl['Speed']["MovingReverse"]
    ret.yawRate = cp_PT.vl['Speed']["YawRate"] * CV.DEG_TO_RAD
    ret.steeringRateDeg = cp_PT.vl["SteeringWheelAngle"]['SteeringSpeed']
    can_gear = int(cp_PT.vl["TransmissionDataDisplay"]['ShiftLeverPosition'])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    blinker_on = cp_PT.vl["TurnSignals"]['TurnSignalActive'] != 0 and cp_PT.vl["TurnSignals"]['TurnSignalIdle'] == 0
    ret.leftBlinker = blinker_on and cp_PT.vl["TurnSignals"]['LeftTurn'] !=0   # blinking
    ret.rightBlinker = blinker_on and cp_PT.vl["TurnSignals"]['RightTurn'] !=0   # blinking
    self.right_blinker_pressed = not blinker_on and cp_PT.vl["TurnSignals"]['RightTurn'] != 0
    self.left_blinker_pressed = not blinker_on and cp_PT.vl["TurnSignals"]['LeftTurn'] != 0

    self.dtc_mode = cp_PT.vl['StatusDSC_KCAN']['DTC_on'] != 0 # drifty traction control ;)

    # other buttons help determine driver is paying attention in case the face is not visible
    self.other_buttons = \
      cp_PT.vl["SteeringButtons"]['Volume_DOWN'] !=0  or cp_PT.vl["SteeringButtons"]['Volume_UP'] !=0  or \
      cp_PT.vl["SteeringButtons"]['Previous_down'] !=0  or cp_PT.vl["SteeringButtons"]['Next_up'] !=0 or \
      cp_PT.vl["SteeringButtons"]['VoiceControl'] !=0 or \
      self.prev_gas_pressed and not ret.gasPressed # treat gas pedal tap as a button - button events indicate driver engagement - useful if face not visible

    # E-series doesn't have torque sensor
    # use Voice button or gas pedal to fake steeringPressed to confirm a lane change
    ret.steeringPressed = cp_PT.vl["SteeringButtons"]['VoiceControl'] !=0 or ret.gasPressed
    if ret.steeringPressed and ret.leftBlinker:
      ret.steeringTorque = 1
    elif ret.steeringPressed and  ret.rightBlinker:
      ret.steeringTorque = -1
    else:
      ret.steeringTorque = 0

    ret.espDisabled = cp_PT.vl['StatusDSC_KCAN']['DSC_full_off'] != 0
    ret.cruiseState.available = not ret.espDisabled  #cruise not available when DSC fully off
    ret.cruiseState.nonAdaptive = False # bmw doesn't have a switch


    cruise_control_stal_msg = cp_PT.vl["CruiseControlStalk"]
    if self.CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      ret.steeringAngleDeg = cp_F.vl['SteeringWheelAngle_DSC']['SteeringPosition']  # slightly quicker on F-CAN TODO find the factor and put in DBC
      ret.cruiseState.speed = cp_PT.vl["DynamicCruiseControlStatus"]['CruiseControlSetpointSpeed'] * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS)
      ret.cruiseState.enabled = cp_PT.vl["DynamicCruiseControlStatus"]['CruiseActive'] != 0
      # DCC implies that cruise control is done on F-CAN
      # If we are sending on F-can, we also need to read on F-can to differentiate our messages from car messages
      cruise_control_stal_msg = cp_F.vl["CruiseControlStalk"]
    elif self.CP.flags & BmwFlags.NORMAL_CRUISE_CONTROL:
      ret.steeringAngleDeg = cp_PT.vl['SteeringWheelAngle']['SteeringPosition']
      ret.cruiseState.speed = cp_PT.vl["CruiseControlStatus"]['CruiseControlSetpointSpeed'] * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS)
      ret.cruiseState.enabled = cp_PT.vl["CruiseControlStatus"]['CruiseCoontrolActiveFlag'] != 0
    ret.cruiseState.speedCluster = ret.cruiseState.speed + CruiseSettings.CLUSTER_OFFSET * CV.KPH_TO_MS #For logging. Doesn't do anything with pcmCruise = False
    if cruise_control_stal_msg['plus1'] != 0:
      self.cruise_stalk_speed = 1
    elif cruise_control_stal_msg['minus1'] != 0:
      self.cruise_stalk_speed = -1
    elif cruise_control_stal_msg['plus5'] != 0:
      self.cruise_stalk_speed = 5
    elif cruise_control_stal_msg['minus5'] != 0:
      self.cruise_stalk_speed = -5
    else:
      self.cruise_stalk_speed = 0
    self.cruise_stalk_resume = cruise_control_stal_msg['resume'] != 0
    self.cruise_stalk_cancel = cruise_control_stal_msg['cancel'] != 0
    self.cruise_stalk_cancel_up = cruise_control_stal_msg['cancel_lever_up'] != 0
    self.cruise_stalk_counter = cruise_control_stal_msg['Counter_0x194']
    self.cruise_stalk_cancel_dn = self.cruise_stalk_cancel and not self.cruise_stalk_cancel_up


    # *** cruise control units one-time detection ***
    # when cruise is enabled the car sets cruiseState.speed = vEgo, so we can detect the ratio
    # with resume this wouldn't work, but op will not engage on first resume anyway
    if self.is_metric is None and ret.cruiseState.enabled and ret.vEgo > 5:
      # note, when is_metric is None, cruiseState.speed is already scaled by CV.MPH_TO_MS by default
      speed_ratio = ret.cruiseState.speed / ret.vEgo  # 1 if imperial, 1.6 if metric
      if 0.8 < speed_ratio < 1.2:
        self.is_metric = False
      elif 0.8 * CV.MPH_TO_KPH < speed_ratio < 1.2 * CV.MPH_TO_KPH:
        self.is_metric = True
        # update speed if metric
        ret.cruiseState.speed = ret.cruiseState.speed * CV.KPH_TO_MS
      else:
        ret.accFaulted = True


    ret.genericToggle = self.dtc_mode

    if self.CP.flags & BmwFlags.STEPPER_SERVO_CAN:
      ret.steeringTorqueEps =  cp_aux.vl['STEERING_STATUS']['STEERING_TORQUE']
      ret.steeringAngleOffsetDeg = ret.steeringAngleDeg - cp_aux.vl['STEERING_STATUS']['STEERING_ANGLE']
      ret.steerFaultTemporary = int(cp_aux.vl['STEERING_STATUS']['DEBUG_STATES']) & 0x20 != 0 # Comm error
      ret.steerFaultTemporary |= int(cp_aux.vl['STEERING_STATUS']['DEBUG_STATES']) & 0x40 != 0 # motion task overrun
      ret.steerFaultTemporary |= int(cp_aux.vl['STEERING_STATUS']['DEBUG_STATES']) & 0x80 != 0 # service task overrun
      ret.steerFaultTemporary = int(cp_aux.vl['STEERING_STATUS']['CONTROL_STATUS']) & 0x4 != 0 # SOFT_OFF lockout

    self.prev_gas_pressed = ret.gasPressed


    ret.buttonEvents = [
      *create_button_events(self.cruise_stalk_speed > 0, self.prev_cruise_stalk_speed > 0, {1: ButtonType.accelCruise}),
      *create_button_events(self.cruise_stalk_speed < 0, self.prev_cruise_stalk_speed < 0, {1: ButtonType.decelCruise}),
      *create_button_events(self.cruise_stalk_cancel, self.prev_cruise_stalk_cancel, {1: ButtonType.cancel}),
      *create_button_events(self.other_buttons, not self.other_buttons, {1: ButtonType.altButton2}),
      *create_button_events(self.cruise_stalk_resume, self.prev_cruise_stalk_resume, {
        # repurpose resume button to adjust driver personality when engaged, else just resume
        1: ButtonType.resumeCruise if not ret.cruiseState.enabled else ButtonType.gapAdjustCruise})
      ]

    self.cruise_state_enabled = ret.cruiseState.enabled
    return ret

  # this is only to satisfy non pcmCruise test in test_panda_safety_carstate that requires button_enable
  #
  def update_button_enable(self, buttonEvents: list[structs.CarState.ButtonEvent]):
    if self.cruise_state_enabled and not self.out.cruiseState.enabled:
      return True
    return False


  @staticmethod
  def get_can_parsers(CP):
    pt_messages = [ # message, expected frequency
      ("EngineAndBrake", 100),
      ("TransmissionDataDisplay", 5),
      ("AccPedal", 100),
      ("Speed", 50),
      ("SteeringWheelAngle", 100),
      ("TurnSignals", 0),
      ("SteeringButtons", 0),
      ("WheelSpeeds", 50), # 100 on F-CAN
      ("CruiseControlStalk", 5),
      ("StatusDSC_KCAN", 50),
      ("Status_contact_handbrake", 0),
      ("TerminalStatus", 10),
    ]

    if CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      pt_messages += [
        ("DynamicCruiseControlStatus", 5),
      ]
    if CP.flags & BmwFlags.NORMAL_CRUISE_CONTROL:
      pt_messages += [
        ("CruiseControlStatus", 5),
      ]


    # $540 vehicle option could use just PT_CAN, but $544 requires sending and receiving cruise commands on F-CAN. Use F-can. Works for both options
    fcan_messages = []
    if CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      fcan_messages += [ # message, expected frequency
        ("CruiseControlStalk", 5),
        ("SteeringWheelAngle_DSC", 100),
      ]

    # if the car is equipped with custom actuator
    servo_can_messages = []
    if CP.flags & BmwFlags.STEPPER_SERVO_CAN:
      servo_can_messages += [ # message, expected frequency
      ("STEERING_STATUS", 100),
      ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus.PT_CAN),
      Bus.body: CANParser(DBC[CP.carFingerprint][Bus.pt], fcan_messages, CanBus.F_CAN),
      Bus.alt: CANParser('ocelot_controls', servo_can_messages, CanBus.SERVO_CAN),
    }
