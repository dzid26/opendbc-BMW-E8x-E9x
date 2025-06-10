from enum import Enum
from opendbc.can.packer import CANPacker
from opendbc.car.bmw.values import CanBus

class SteeringModes(Enum):
  Off = 0
  TorqueControl = 1
  AngleControl = 2
  SoftOff = 3

class CruiseStalk(Enum):
  plus1 = "plus1"
  plus5 = "plus5"
  minus1 = "minus1"
  minus5 = "minus5"
  cancel = "cancel"
  resume = "resume"
  cancel_lever_up = "cancel_lever_up"

# *** StepperServoCAN ***
def create_steer_command(frame: int, mode: SteeringModes, steer_tq: float = 0, steer_delta: float = 0):
    """Creates a CAN message for the actuator STEERING_COMMAND"""
    packer = CANPacker('ocelot_controls')
    values = {
        "COUNTER": frame % 16,
        "STEER_MODE": mode.value,
        "STEER_ANGLE": steer_delta,
        "STEER_TORQUE": steer_tq,
    }
    msg = packer.make_can_msg("STEERING_COMMAND", 0, values)
    addr = msg[0]
    dat  = msg[1]
    values["CHECKSUM"] = calc_checksum_8bit(dat, addr)

    return packer.make_can_msg("STEERING_COMMAND", CanBus.SERVO_CAN, values)


def calc_checksum_4bit(work_data: bytearray, msg_id: int): # 0x130
  checksum = msg_id
  for byte in work_data: #checksum is stripped from the dat
    checksum += byte     #add up all the bytes

  checksum = (checksum & 0xFF) + (checksum >> 8) #add upper and lower Bytes
  checksum &= 0xFF #throw away anything in upper Byte

  checksum = (checksum & 0xF) + (checksum >> 4) #add first and second nibble
  checksum &= 0xF #throw away anything in upper nibble
  return checksum

def calc_checksum_8bit(work_data: bytearray, msg_id: int): # 0xb8 0x1a0 0x19e 0xaa 0xbf
  checksum = msg_id
  for byte in work_data: #checksum is stripped from the data
    checksum += byte     #add up all the bytes

  checksum = (checksum & 0xFF) + (checksum >> 8) #add upper and lower Bytes
  checksum &= 0xFF #throw away anything in upper Byte
  return checksum

def calc_checksum_cruise(work_data: bytearray):# 0x194 this checksum is special - initialized with 0
  return calc_checksum_8bit(work_data, 0)


def create_accel_command(packer, action: CruiseStalk, bus: int, cnt):
    values = {
        "setMe_0xFC": 0xFC,
        "requests_0xF": 0xF,
        "Counter_0x194": cnt % 0xF # counts from 0 to 14
        }
    values[action.value] = 1

    dat = packer.make_can_msg("CruiseControlStalk", bus, values)[1]
    values["Checksum_0x194"] = calc_checksum_cruise(dat)

    return packer.make_can_msg("CruiseControlStalk", bus, values)

