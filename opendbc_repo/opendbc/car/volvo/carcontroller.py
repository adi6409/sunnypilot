import numpy as np
from opendbc.can import CANPacker
from openpilot.common.realtime import DT_CTRL
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.values import CANBUS, CarControllerParams, SteerDirection


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.CP = CP
    self.CCP = CarControllerParams(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.frame = 0

    self.apply_steer_prev = 0
    self.apply_steer_dir_prev = SteerDirection.NONE

    self.latActive_prev = False
    self.steer_blocked = False
    self.steer_blocked_cnt = 0
    self.steer_dir_bf_block = SteerDirection.NONE

    # SNG
    self.last_resume_frame = 0
    self.distance = 0
    self.waiting = False
    self.sng_count = 0
    # Frame when the most recent resume-blast cycle started. Used to pass
    # through stock's FSM3 accel during the take-off window so the ECM's
    # actual response matches stock ACC's internal expectation — OP's
    # planner under-commands and stock cancels (drives 3e seg 7, 3f seg 11).
    self.takeoff_start_frame = -1_000_000

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    accel = 0.0  # always defined so SNG block never NameErrors

    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel

    # Cancel ACC if engaged when OP is not, but only above minimum steering speed.
    # TODO: is this check needed? it might trying to fix broken standstill behavior
    if pcm_cancel_cmd and CS.out.vEgo > self.CP.minSteerSpeed:
      can_sends.append(volvocan.create_button_msg(self.packer_pt, cancel=True))

    # run at 50hz
    if self.frame % 2 == 0:
      if CC.latActive and CS.out.vEgo > self.CP.minSteerSpeed:
        #apply_steer = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_steer_prev, CS.out.vEgoRaw, CarControllerParams)
        apply_steer = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_steer_prev, CS.out.vEgoRaw, CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)
        apply_steer_dir = SteerDirection.LEFT if apply_steer > 0 else SteerDirection.RIGHT

        error = CS.out.steeringAngleDeg - apply_steer
        error_with_deadzone = 0 if abs(error) < CarControllerParams.DEADZONE else error

        # Update prev with desired if just enabled.
        if not self.latActive_prev:
          self.apply_steer_dir_prev = apply_steer_dir

        if self.steer_blocked:
          if (apply_steer_dir == self.steer_dir_bf_block) or (self.steer_blocked_cnt <= 0) or (error_with_deadzone == 0):
            self.steer_blocked = False
        else:
          if apply_steer_dir != self.apply_steer_dir_prev and error_with_deadzone != 0:
            self.steer_blocked = True
            self.steer_blocked_cnt = CarControllerParams.BLOCK_LEN
            self.steer_dir_bf_block = self.apply_steer_dir_prev

        if self.steer_blocked:
          self.steer_blocked_cnt -= 1
          apply_steer_dir = SteerDirection.NONE
        elif error_with_deadzone == 0:
          # Set old request when inside deadzone
          apply_steer_dir = self.apply_steer_dir_prev

      else:
        apply_steer = 0
        apply_steer_dir = SteerDirection.NONE

      can_sends.append(volvocan.create_lka_msg(self.packer_pt, apply_steer, int(apply_steer_dir)))

      self.apply_steer_prev = apply_steer
      self.apply_steer_dir_prev = apply_steer_dir
      self.latActive_prev = CC.latActive

      # Manipulate data from servo to FSM
      # Avoids faults that will stop servo from accepting steering commands.
      can_sends.append(volvocan.create_lkas_state_msg(self.packer_pt, CS.out.steeringAngleDeg, CS.pscm_stock_values))

    at_standstill = (CS.out.cruiseState.enabled and CS.out.cruiseState.standstill
                     and CS.out.vEgo < 0.01)

    # SNG — evaluated BEFORE the long-control TX block so the take-off window
    # flag set here is visible when we pick OP-vs-stock accel below.
    # wait 100 cycles since last resume sent
    if (self.frame - self.last_resume_frame) * DT_CTRL > 1.00:
      if at_standstill and not self.waiting:
        self.distance = CS.acc_distance
        self.waiting = True
        self.sng_count = 0

      # Trigger resume only on lead moving. The prior "op_wants_go" debounce
      # (0.5s of planner accel > 0.3) was a workaround for OP not having a
      # radar feed — it caused the car to resume before the lead actually
      # pulled away and stock ACC cancelled mid-take-off (drive 0000003e seg 7).
      # With the Delphi ESR wired up, the planner sees real leads and
      # ACC_Distance flags stock's take-off the same instant stock sees it.
      lead_moved = CS.acc_distance > self.distance

      if at_standstill and self.waiting and lead_moved:
        # send 25 messages at a time to increases the likelihood of resume being accepted
        can_sends.extend([volvocan.create_button_msg(self.packer_pt, resume=True)] * 25)
        if self.CP.openpilotLongitudinalControl:
          # Already sending FSM3 every frame above; SNG just needs the resume button blast.
          pass
        else:
          can_sends.extend([volvocan.create_acc_state_msg(self.packer_pt)] * 25)
        # Mark the start of the take-off window on the first blast of this
        # resume cycle so the long block below can defer to stock's accel.
        if self.sng_count == 0:
          self.takeoff_start_frame = self.frame
        self.sng_count += 1
      # disable sending resume after 5 cycles sent or if no more in standstill
      if self.waiting and (self.sng_count >= 5 or not CS.out.cruiseState.standstill):
        self.waiting = False
        self.last_resume_frame = self.frame

    # Longitudinal: only TX when OP is actively controlling (longActive).
    # Panda's fwd hook now unblocks stock FSM1/FSM3 when !gas_pressed, so
    # during gas overrides stock flows through naturally and there's no
    # starvation even though OP isn't TXing. When longActive flickers False
    # due to gas press, OP withdraws from the bus entirely and the car
    # sees stock's real-time FSM3 — matching pre-OP-long behavior.
    if self.CP.openpilotLongitudinalControl and CC.longActive and self.frame % 2 == 0:
      op_accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

      # Take-off passthrough: during the ~3s after a SNG resume blast and
      # while still rolling slowly, use stock's own ACC_AccelerationRequest
      # value instead of OP's. Stock ACC decides its take-off ramp based on
      # lead range/closing rate and commanded +1.68 m/s² in drive 3f seg 11
      # while OP only wanted +1.25; the car fell behind stock's expected
      # profile and stock cancelled. Mirroring stock's accel during take-off
      # keeps stock's state machine happy. OP takes back over once cruising.
      # Only passes through POSITIVE accel — if stock wants to brake we
      # still use OP's value (stock's brake authority is weak on this car).
      takeoff_elapsed = (self.frame - self.takeoff_start_frame) * DT_CTRL
      stock_accel = float(CS.stock_FSM3["ACC_AccelerationRequest"])
      if takeoff_elapsed < 3.0 and CS.out.vEgo < 5.0 and stock_accel > 0:
        accel = stock_accel
      else:
        accel = op_accel

      can_sends.append(volvocan.create_longitudinal(self.packer_pt, CS.stock_FSM3, accel, CS.ACC_Check))
      can_sends.append(volvocan.create_radar(self.packer_pt, CS.stock_FSM1, True))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_steer_prev

    self.frame += 1
    return new_actuators, can_sends
