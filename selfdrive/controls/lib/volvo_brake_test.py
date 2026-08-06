from dataclasses import dataclass
from enum import IntEnum


class VolvoBrakeTestMode(IntEnum):
  pulse = 1
  stop = 2


@dataclass(frozen=True)
class VolvoBrakeTestConfig:
  min_speed: float = 30.0 / 3.6
  max_speed: float = 40.0 / 3.6
  cancel_speed: float = 5.0 / 3.6
  peak_decel: float = -1.5
  ramp_time: float = 0.75
  hold_time: float = 0.75
  release_time: float = 0.75
  stop_timeout: float = 12.0
  resume_timeout: float = 3.0
  cancel_pulse_time: float = 0.5

  @property
  def duration(self) -> float:
    return self.ramp_time + self.hold_time + self.release_time


class VolvoBrakeTest:
  """One-shot, low-speed brake pulse used to validate Volvo longitudinal actuation."""

  def __init__(self, config: VolvoBrakeTestConfig | None = None):
    self.config = config or VolvoBrakeTestConfig()
    self.active = False
    self.used = False
    self.started_at = 0.0
    self.mode: VolvoBrakeTestMode | None = None
    self.cancel_cruise = False
    self.cancel_until = 0.0
    self.stop_reached = False
    self.resume_active = False
    self.resume_started_at = 0.0
    self.resume_completed = False

  def update(self, now: float, trigger: VolvoBrakeTestMode | None, eligible: bool,
             v_ego: float = 0.0, standstill: bool = False, resume_trigger: bool = False) -> float | None:
    self.cancel_cruise = now < self.cancel_until

    if trigger is not None and eligible and not self.used:
      self.active = True
      self.used = True
      self.started_at = now
      self.mode = trigger

    if resume_trigger and eligible and self.active and self.mode == VolvoBrakeTestMode.stop and self.stop_reached:
      self.active = False
      self.resume_active = True
      self.resume_started_at = now

    if self.resume_active:
      if not eligible:
        self.resume_active = False
      elif not standstill and v_ego > 0.3:
        self.resume_active = False
        self.resume_completed = True
      elif now - self.resume_started_at >= self.config.resume_timeout:
        # Resume failed: restore the standstill hold instead of silently
        # falling back to a planner that wants to accelerate on a clear road.
        self.resume_active = False
        self.active = True
        return self.config.peak_decel
      return None

    if not self.active:
      return None

    if not eligible:
      self.active = False
      return None

    elapsed = max(0.0, now - self.started_at)

    if self.mode == VolvoBrakeTestMode.stop:
      if standstill or v_ego < 0.3:
        self.stop_reached = True
        return self.config.peak_decel

      if elapsed >= self.config.stop_timeout:
        self.active = False
        self.cancel_until = now + self.config.cancel_pulse_time
        self.cancel_cruise = True
        return None

      if elapsed < self.config.ramp_time:
        return self.config.peak_decel * elapsed / self.config.ramp_time
      return self.config.peak_decel

    if elapsed >= self.config.duration:
      self.active = False
      return None

    if elapsed < self.config.ramp_time:
      return self.config.peak_decel * elapsed / self.config.ramp_time

    elapsed -= self.config.ramp_time
    if elapsed < self.config.hold_time:
      return self.config.peak_decel

    elapsed -= self.config.hold_time
    return self.config.peak_decel * (1.0 - elapsed / self.config.release_time)
