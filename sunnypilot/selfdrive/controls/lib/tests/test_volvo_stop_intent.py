from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import (
  VolvoStopIntentDebouncer,
  volvo_model_stop_intent,
)


def test_normal_model_easing_does_not_trigger_full_stop():
  # 17:40 cancellation: this exact class of -0.11 to -0.16 m/s^2 easing
  # previously forced vTarget=0 and an immediate -2.0 m/s^2 command.
  assert not volvo_model_stop_intent(9.4, -0.16, False, 76.0, 6.2)


def test_1359_borderline_model_sample_is_not_stop_evidence():
  # Route 00000027 segment 2 armed at 13:59:41 on this single sample.
  assert not volvo_model_stop_intent(10.78, -0.5041, False, 76.4, 4.51)


def test_strong_model_braking_triggers_stop():
  assert volvo_model_stop_intent(9.4, -1.4, False, 76.0, 6.2)


def test_explicit_model_stop_triggers_stop():
  assert volvo_model_stop_intent(9.4, 0.0, True, 76.0, 6.2)


def test_short_path_requires_low_terminal_speed():
  assert not volvo_model_stop_intent(9.4, 0.0, False, 40.0, 7.0)
  assert volvo_model_stop_intent(9.4, 0.0, False, 40.0, 1.0)


def test_speed_envelope_is_preserved():
  assert not volvo_model_stop_intent(0.0, -1.0, False, 0.0, 0.0)
  assert not volvo_model_stop_intent(14.0, -1.0, False, 30.0, 0.0)


def test_single_stop_evidence_frame_cannot_arm_vehicle_facing_assist():
  debouncer = VolvoStopIntentDebouncer()
  assert not debouncer.update(True, True)
  for _ in range(10):
    assert not debouncer.update(True, False)


def test_sustained_stop_evidence_arms_after_400_ms():
  debouncer = VolvoStopIntentDebouncer()
  for _ in range(debouncer.ACTIVATION_FRAMES - 1):
    assert not debouncer.update(True, True)
  assert debouncer.update(True, True)


def test_ineligible_state_resets_stop_intent_immediately():
  debouncer = VolvoStopIntentDebouncer()
  for _ in range(debouncer.ACTIVATION_FRAMES):
    assert debouncer.update(True, True) == (_ == debouncer.ACTIVATION_FRAMES - 1)
  assert not debouncer.update(False, True)
  assert debouncer.evidence_frames == 0


def test_active_stop_intent_tolerates_one_missing_model_frame():
  debouncer = VolvoStopIntentDebouncer()
  for _ in range(debouncer.ACTIVATION_FRAMES):
    debouncer.update(True, True)
  assert debouncer.update(True, False)
