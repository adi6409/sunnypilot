import pytest

from openpilot.selfdrive.controls.lib.volvo_brake_test import VolvoBrakeTest, VolvoBrakeTestConfig, VolvoBrakeTestMode


def test_requires_trigger_and_eligibility():
  test = VolvoBrakeTest()
  assert test.update(0.0, trigger=None, eligible=True) is None
  assert test.update(0.0, trigger=VolvoBrakeTestMode.pulse, eligible=False) is None
  assert not test.used


def test_pulse_profile_is_one_shot():
  config = VolvoBrakeTestConfig(peak_decel=-1.5, ramp_time=1.0, hold_time=1.0, release_time=1.0)
  test = VolvoBrakeTest(config)

  assert test.update(10.0, trigger=VolvoBrakeTestMode.pulse, eligible=True) == pytest.approx(0.0)
  assert test.update(10.5, trigger=None, eligible=True) == pytest.approx(-0.75)
  assert test.update(11.5, trigger=None, eligible=True) == pytest.approx(-1.5)
  assert test.update(12.5, trigger=None, eligible=True) == pytest.approx(-0.75)
  assert test.update(13.0, trigger=None, eligible=True) is None
  assert test.used

  # Keeping or pressing the phone button again cannot retrigger this drive.
  assert test.update(14.0, trigger=VolvoBrakeTestMode.pulse, eligible=True) is None


def test_loss_of_eligibility_cancels_without_retrigger():
  test = VolvoBrakeTest()
  test.update(1.0, trigger=VolvoBrakeTestMode.pulse, eligible=True)
  assert test.active
  assert test.update(1.1, trigger=None, eligible=False) is None
  assert not test.active
  assert test.update(2.0, trigger=VolvoBrakeTestMode.pulse, eligible=True) is None


def test_stop_holds_at_standstill_until_driver_takeover():
  config = VolvoBrakeTestConfig(peak_decel=-1.5, ramp_time=1.0, stop_timeout=12.0)
  test = VolvoBrakeTest(config)

  assert test.update(0.0, VolvoBrakeTestMode.stop, True, v_ego=10.0) == pytest.approx(0.0)
  assert test.update(0.5, None, True, v_ego=9.5) == pytest.approx(-0.75)
  assert test.update(5.0, None, True, v_ego=3.0) == pytest.approx(-1.5)
  assert test.update(8.0, None, True, v_ego=0.0, standstill=True) == pytest.approx(-1.5)
  assert test.update(20.0, None, True, v_ego=0.0, standstill=True) == pytest.approx(-1.5)
  assert test.active
  assert test.stop_reached

  # A pedal/disengagement drops eligibility and releases the diagnostic hold.
  assert test.update(20.1, None, False, v_ego=0.0, standstill=True) is None
  assert not test.active


def test_stop_timeout_requests_cruise_cancel():
  test = VolvoBrakeTest(VolvoBrakeTestConfig(stop_timeout=12.0, cancel_pulse_time=0.5))
  test.update(0.0, VolvoBrakeTestMode.stop, True, v_ego=10.0)
  assert test.update(12.0, None, True, v_ego=5.0) is None
  assert not test.active
  assert test.cancel_cruise
  assert test.update(12.4, None, True, v_ego=5.0) is None
  assert test.cancel_cruise
  assert test.update(12.5, None, True, v_ego=5.0) is None
  assert not test.cancel_cruise


def test_resume_releases_hold_until_vehicle_moves():
  test = VolvoBrakeTest()
  test.update(0.0, VolvoBrakeTestMode.stop, True, v_ego=10.0)
  test.update(7.0, None, True, v_ego=0.0, standstill=True)

  assert test.update(8.0, None, True, v_ego=0.0, standstill=True, resume_trigger=True) is None
  assert test.resume_active
  assert not test.active
  assert test.update(8.5, None, True, v_ego=0.1, standstill=False) is None
  assert test.resume_active
  assert test.update(9.0, None, True, v_ego=0.5, standstill=False) is None
  assert not test.resume_active
  assert test.resume_completed


def test_failed_resume_restores_stop_hold():
  config = VolvoBrakeTestConfig(resume_timeout=3.0)
  test = VolvoBrakeTest(config)
  test.update(0.0, VolvoBrakeTestMode.stop, True, v_ego=10.0)
  test.update(7.0, None, True, v_ego=0.0, standstill=True)
  test.update(8.0, None, True, v_ego=0.0, standstill=True, resume_trigger=True)

  assert test.update(11.0, None, True, v_ego=0.0, standstill=True) == pytest.approx(-1.5)
  assert not test.resume_active
  assert test.active
