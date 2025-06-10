from dataclasses import dataclass, field
from enum import Enum, IntFlag
from opendbc.car import Bus, Platforms, CarSpecs, PlatformConfig, DbcDict, STD_CARGO_KG
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column
from opendbc.car.common.conversions import Conversions as CV

# Steer torque limits
class CarControllerParams: #controls running @ 100hz
  STEER_STEP = 1 # 100Hz
  STEER_MAX = 12  # Nm
  STEER_DELTA_UP = 10 / 100       # 10Nm/s
  STEER_DELTA_DOWN = 1000 / 100     # 10Nm/sample - no limit
  STEER_ERROR_MAX = 999     # max delta between torque cmd and torque motor

  # STEER_BACKLASH = 1 #deg
  def __init__(self, CP):
    pass

class BmwFlags(IntFlag):
  # Detected Flags
  STEPPER_SERVO_CAN = 2 ** 0
  NORMAL_CRUISE_CONTROL = 2 ** 1          # CC  $540
  DYNAMIC_CRUISE_CONTROL = 2 ** 2         # DCC $544
  ACTIVE_CRUISE_CONTROL = 2 ** 3          # ACC $541 - LDM and ACC sensor - #! not supported
  ACTIVE_CRUISE_CONTROL_NO_ACC = 2 ** 4   # no ACC module - DSC, DME, KOMBI coded to $541, LDM coded to $544
  ACTIVE_CRUISE_CONTROL_NO_LDM = 2 ** 5   # no LDM/ACC - DSC, DME, KOMBI coded to $541
  SERVOTRONIC = 2 ** 6                    # ServoTonic $216A - TODO: needs firmware query

class CruiseSettings:
  CLUSTER_OFFSET = 2 # kph

class CanBus:
  PT_CAN =    0
  SERVO_CAN = 1 # required for steering
  F_CAN =     1 # required for DYNAMIC_CRUISE_CONTROL or optional for logging
  K_CAN =     2 # not used - only logging


class Footnote(Enum):
  StepperServoCAN = CarFootnote(
    "Requires StepperServoCAN",
    Column.FSR_STEERING)
  DCC = CarFootnote(
    "Minimum speed with CC or DCC is 30 kph",
    Column.FSR_LONGITUDINAL)
  CC = CarFootnote(
    "Normal cruise control should work but was not tested in a while. Code in DCC instead or provide a fix",
    Column.PACKAGE)
  ACC = CarFootnote(
    "ACC is required. Also LDM module to take over when OP is off.",
    Column.AUTO_RESUME)
  DIY = CarFootnote(
    "For CC and DCC only a diy USB-C and a resistor is required or a harness box DIY connector",
    Column.HARDWARE)

@dataclass
class BmwCarDocs(CarDocs):
  package: str = "Cruise Control - VO540, VO544, VO541"
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.StepperServoCAN, Footnote.DCC, Footnote.CC, Footnote.ACC, Footnote.DIY])

  def init_make(self, CP: CarParams):
      self.car_parts = CarParts.common([CarHarness.custom])

@dataclass
class BmwPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'bmw_e9x_e8x',
    Bus.chassis: 'bmw_e9x_e8x',
    Bus.body: 'bmw_e9x_e8x',
    })

class CAR(Platforms):
  BMW_E82 = BmwPlatformConfig(
    [BmwCarDocs("BMW E82 2004-13")],
    CarSpecs(mass=3145. * CV.LB_TO_KG + STD_CARGO_KG, wheelbase=2.66, steerRatio=16.00, tireStiffnessFactor=0.8)
  )
  BMW_E90 = BmwPlatformConfig(
    [BmwCarDocs("BMW E90 2005-11")],
    CarSpecs(mass=3300. * CV.LB_TO_KG + STD_CARGO_KG, wheelbase=2.76, steerRatio=16.00, tireStiffnessFactor=0.8)
  )


DBC = CAR.create_dbc_map()
