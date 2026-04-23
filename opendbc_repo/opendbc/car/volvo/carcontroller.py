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
    self.op_go_frames = 0  # counts consecutive frames OP wants to accelerate at standstill

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

    # Longitudinal: TX continuously whenever stock CC is engaged, regardless
    # of OP's longActive state. This is the key insight from drive 38's
    # cancellation analysis — during gas-pedal overrides OP's longActive flips
    # False (deferring to driver). The panda's fwd hook keeps blocking stock's
    # cam->main FSM3 because controls_allowed stays True (stock CC engaged).
    # If we also stop TXing FSM3 during longActive=False, the bus goes silent
    # on FSM3 for the duration of the gas press, stock ACC detects the
    # starvation, and cancels hard.
    #
    # Fix: always TX FSM3 at 50Hz while cc.enabled. Content:
    #   - longActive=True: OP's planner accel (our override)
    #   - longActive=False: stock's own commanded accel passthrough
    # Byte-for-byte everything else matches stock. From stock ACC's
    # perspective its commanded values are what's on the bus — no divergence
    # detection, no cancellation during gas overrides.
    if self.CP.openpilotLongitudinalControl and CS.out.cruiseState.enabled and self.frame % 2 == 0:
      if CC.longActive:
        accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      else:
        accel = float(CS.stock_FSM3["ACC_AccelerationRequest"])
      can_sends.append(volvocan.create_longitudinal(self.packer_pt, CS.stock_FSM3, accel, CS.ACC_Check))
      can_sends.append(volvocan.create_radar(self.packer_pt, CS.stock_FSM1, CC.longActive))

    # Track sustained "OP wants to accelerate" signal while at standstill.
    # Used as an alternative resume trigger for no-lead scenarios (empty road
    # red light → green light). Debounced over ~0.5s to avoid spurious triggers
    # from momentary planner output transients.
    at_standstill = (CS.out.cruiseState.enabled and CS.out.cruiseState.standstill
                     and CS.out.vEgo < 0.01)
    if at_standstill and self.CP.openpilotLongitudinalControl and CC.longActive and actuators.accel > 0.3:
      self.op_go_frames += 1
    else:
      self.op_go_frames = 0

    # SNG
    # wait 100 cycles since last resume sent
    if (self.frame - self.last_resume_frame) * DT_CTRL > 1.00:
      if at_standstill and not self.waiting:
        self.distance = CS.acc_distance
        self.waiting = True
        self.sng_count = 0

      # Trigger resume if EITHER the lead moved (original behavior) OR OP's
      # planner sustained a want-to-go command for >0.5s (new: handles
      # empty-road green-light resume with no lead to follow).
      lead_moved = CS.acc_distance > self.distance
      op_wants_go = self.op_go_frames > 50  # 50 frames × 10ms = ~0.5s

      if at_standstill and self.waiting and (lead_moved or op_wants_go):
        # send 25 messages at a time to increases the likelihood of resume being accepted
        can_sends.extend([volvocan.create_button_msg(self.packer_pt, resume=True)] * 25)
        if self.CP.openpilotLongitudinalControl:
          # Already sending FSM3 every frame above; SNG just needs the resume button blast.
          pass
        else:
          can_sends.extend([volvocan.create_acc_state_msg(self.packer_pt)] * 25)
        self.sng_count += 1
      # disable sending resume after 5 cycles sent or if no more in standstill
      if self.waiting and (self.sng_count >= 5 or not CS.out.cruiseState.standstill):
        self.waiting = False
        self.last_resume_frame = self.frame

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_steer_prev

    self.frame += 1
    return new_actuators, can_sends
