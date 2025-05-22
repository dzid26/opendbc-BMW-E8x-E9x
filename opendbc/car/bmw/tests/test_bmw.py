from parameterized import parameterized

from opendbc.car.bmw.fingerprints import FINGERPRINTS

N55_ENGINE_MSG = {899: 4}
N52_ENGINE_MSG = {899: 2}
CRUISE_STATUS_MSG = {0x200: 8}
DYNAMIC_CRUISE_STATUS_MSG = {0x193: 8}
STEPPER_MSG = {0x22F: 8}




class TestBMWFingerprint:
  @parameterized.expand(FINGERPRINTS.items())
  def test_can_fingerprints(self, car_model, fingerprints):
    assert len(fingerprints) > 0

    assert all(len(finger) for finger in fingerprints)

    for car_config in ((STEPPER_MSG, N55_ENGINE_MSG, DYNAMIC_CRUISE_STATUS_MSG),
                     (STEPPER_MSG, N52_ENGINE_MSG, DYNAMIC_CRUISE_STATUS_MSG),
                     (N55_ENGINE_MSG, CRUISE_STATUS_MSG, DYNAMIC_CRUISE_STATUS_MSG),
                     ):
      failed_fingers = {}
      for i, finger in enumerate(fingerprints):
        failed_addrs = []
        for msg in (car_config):
          for addr, length in msg.items():
            found_length = finger.get(addr)
            if found_length != length:
              failed_addrs.append((addr, length, found_length))
        if failed_addrs:
              failed_fingers[i] = failed_addrs

      if len(failed_fingers) == len(fingerprints):
        raise AssertionError(f"All {len(fingerprints)} fingerprints failed: {failed_fingers}")
