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

    # op_go_frames: count of consecutive controlsd cycles where OP's planner
    # is sustaining a "go" command at standstill. Used as an alternate SNG
    # resume trigger for green-light-no-lead scenarios where ACC_Distance
    # never changes (no lead to "move"). Tighter thresholds than the
    # original (0.3 / 0.5s) which fired prematurely in drive 0000003e seg 7
    # before we had radar/take-off-passthrough/ACC_Check fixes.
    self.op_go_frames = 0

    # Next wall-clock nanosecond at which FSM3 / FSM1 TX is allowed.
    # controlsd scheduling jitter means self.frame % 2 == 0 gives us 10–30ms
    # intervals instead of a clean 20ms (drive 41 seg 4: OP TX stdev 2.6ms
    # vs stock 0.5ms, with 9.8ms bursts and 30ms gaps). The ECM validates
    # cadence of stock ACC messages — our bursty pattern looks like a fault
    # and likely contributes to the self-cancels. Gate TX on wall-clock.
    self.next_long_tx_nanos = 0
    # Period between FSM3 TXs (20ms = 50Hz, matching stock).
    self.LONG_TX_PERIOD_NANOS = 20_000_000

    # ACC_Check is the resume-button acknowledgement bit in FSM3. Per
    # leomonde, it must be forced to 1 specifically during the resume
    # blast (not copied from stock's FSM3, which sits at 0 almost always).
    # Count of remaining FSM3 TXs that should carry ACC_Check=1. Set to 25
    # (~0.5s of 50Hz TX) when SNG fires a resume blast; decrements to 0.
    self.sng_ack_frames = 0

    # FSM1 ACC_Distance spoof state. We use a single smoothed value that
    # ramps toward whichever target makes sense given the current scenario:
    #   - Stop-at-red: 8 m (fake close lead so stock ACC keeps its
    #     standstill-no-lead disengage rule from firing while OP brakes
    #     to a stop)
    #   - Engagement-assist: 30 m (fake comfortable lead so stock ACC
    #     allows CC engagement at vEgo < 30 km/h, which it normally
    #     refuses without a real radar lead)
    #   - Otherwise: passes through stock's real ACC_Distance value
    # Ramping is done at the meter level (RAMP_M m per 20 ms TX) so the
    # ECM never sees a sudden jump in lead distance. State is initialized
    # to 255 (no-lead) and re-synced from stock on first TX of any new
    # session to avoid an initial "lead approaching" artifact.
    self.tx_acc_distance_state = 255.0
    self._fsm1_was_txing = False
    # FSM1 TX cadence: 50 Hz, wall-clock gated like FSM3. FSM1 TX runs
    # independently of longActive (also fires for engagement-assist).
    self.next_fsm1_tx_nanos = 0

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

    # Track sustained "OP wants to accelerate" while at standstill. Counter
    # increments when planner is asking for non-trivial positive accel,
    # resets otherwise. Used as alternate SNG trigger for green-light-no-
    # lead (where lead_moved never fires).
    if at_standstill and CC.longActive and actuators.accel > 0.5:
      self.op_go_frames += 1
    else:
      self.op_go_frames = 0

    # SNG — evaluated BEFORE the long-control TX block so the take-off window
    # flag set here is visible when we pick OP-vs-stock accel below.
    # wait 100 cycles since last resume sent
    if (self.frame - self.last_resume_frame) * DT_CTRL > 1.00:
      if at_standstill and not self.waiting:
        self.distance = CS.acc_distance
        self.waiting = True
        self.sng_count = 0

      # Trigger resume on EITHER:
      #   (a) lead moving — ACC_Distance increases (existing behavior, fires
      #       when stock radar sees a lead pull away)
      #   (b) op_go_frames > 100 — planner has sustained accel > 0.5 m/s² for
      #       1 second straight (green-light-no-lead case where the user
      #       implicitly wants to go, but we have no radar lead to track)
      # Tighter than the original (0.3 / 0.5s) version which fired before
      # the radar lead actually moved in drive 3e seg 7.
      lead_moved = CS.acc_distance > self.distance
      op_wants_go = self.op_go_frames > 100  # 100 frames × 10 ms = 1.0 s

      if at_standstill and self.waiting and (lead_moved or op_wants_go):
        # Send 25 resume buttons + 25 FSM3-ACC_Check=1 acks in the same TX
        # batch. Drive 0000004a seg 11 showed the resume button blast alone
        # (with our 50Hz ACC_Check=1 stream over 0.5s) was insufficient: the
        # car never exited standstill hold (ACC_Standstill stayed 1, vEgo=0
        # for 5s) and stock self-cancelled. Leomonde's original create_acc_
        # state_msg burst pattern — ACC_Check=1 messages co-arriving with the
        # button presses on the bus — is what the ECM actually requires to
        # register the resume. Send it regardless of oplong; the long-control
        # FSM3 TX continues at 50Hz separately.
        can_sends.extend([volvocan.create_button_msg(self.packer_pt, resume=True)] * 25)
        can_sends.extend([volvocan.create_acc_state_msg(self.packer_pt)] * 25)
        # Mark the start of the take-off window on the first blast of this
        # resume cycle so the long block below can defer to stock's accel.
        # Also arm the ACC_Check=1 acknowledgement window for the next
        # ~0.5s of FSM3 TXs — the car needs that ack to actually honor the
        # resume button (per leomonde: "force 1, not copy from FSM").
        if self.sng_count == 0:
          self.takeoff_start_frame = self.frame
          self.sng_ack_frames = 25
        self.sng_count += 1
      # disable sending resume after 5 cycles sent or if no more in standstill
      if self.waiting and (self.sng_count >= 5 or not CS.out.cruiseState.standstill):
        self.waiting = False
        self.last_resume_frame = self.frame

    # Compute spoof conditions up-front. They're evaluated whether or not
    # OP is in long-control because the FSM1 spoof can fire before
    # longActive (engagement-assist).
    stock_acc_dist = int(CS.stock_FSM1["ACC_Distance"])
    no_real_lead = stock_acc_dist == 255
    op_accel_planned = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

    # Stop-at-red: while OP is actively braking below ~43 km/h with no
    # real lead, fake a close lead so stock ACC's no-lead-low-speed
    # disengage rule doesn't fire mid-stop.
    stop_at_red_active = (
      CC.longActive
      and op_accel_planned < -0.5
      and CS.out.vEgo < 12.0
      and no_real_lead
    )

    # Engagement-assist: stock Volvo ACC refuses to engage at 1 < vEgo <
    # ~30 km/h unless there's a real radar lead. Fake a comfortable lead
    # while CC system is on, we're slow, and there's no real lead — so
    # the user can press SET at any speed and stock will accept.
    # CAVEAT: only effective if stock FSM uses bus-visible FSM1 (not just
    # internal radar) for its engagement-allow gating. Pre-engagement, panda
    # fwd_hook does NOT block stock cam→main FSM1, so stock's real FSM1
    # (with ACC_Distance=255) reaches the ECM in parallel with our TX —
    # last-wins on the bus. If host-only spoof doesn't unlock engagement,
    # the next step is changing fwd_hook to always block FSM1.
    engagement_spoof_active = (
      CS.out.cruiseState.available
      and 1.0 < CS.out.vEgo < 9.0  # above near-stop, below stock's ~30 km/h floor
      and no_real_lead
    )

    # Determine desired ACC_Distance to TX, then ramp toward it.
    if stop_at_red_active:
      desired_dist = 8.0
    elif engagement_spoof_active:
      desired_dist = 30.0
    else:
      desired_dist = float(stock_acc_dist)

    RAMP_M = 4.0  # max change per FSM1 TX (50 Hz × 4 m = 200 m/s — fast enough that brief transients don't spike)
    diff = desired_dist - self.tx_acc_distance_state
    self.tx_acc_distance_state += max(min(diff, RAMP_M), -RAMP_M)
    self.tx_acc_distance_state = max(0.0, min(255.0, self.tx_acc_distance_state))

    # Longitudinal: only TX FSM3 when OP is actively controlling
    # (longActive). FSM1 TX runs independently below for engagement-assist.
    # Panda's fwd hook unblocks stock FSM1/FSM3 when !gas_pressed, so
    # during gas overrides stock flows through naturally and there's no
    # starvation even though OP isn't TXing.
    #
    # Rate gating: wall-clock 20ms minimum between TXs, so controlsd jitter
    # can't produce the 10ms bursts leomonde spotted. If we're late we still
    # TX once and slide the next-allowed forward by one period (no catch-up
    # burst).
    long_tx_due = self.CP.openpilotLongitudinalControl and CC.longActive and now_nanos >= self.next_long_tx_nanos
    if long_tx_due:
      # Advance target by exactly one period. If we'd already be past the
      # advanced time (first TX after longActive gap, or controlsd stalled
      # for >1 period), resync to avoid a burst of catch-up TXs.
      next_tx = self.next_long_tx_nanos + self.LONG_TX_PERIOD_NANOS
      if self.next_long_tx_nanos == 0 or next_tx <= now_nanos:
        next_tx = now_nanos + self.LONG_TX_PERIOD_NANOS
      self.next_long_tx_nanos = next_tx

      op_accel = op_accel_planned

      # Take-off passthrough: after a SNG resume blast, while still rolling
      # below 5 m/s, defer to stock's ACC_AccelerationRequest when stock
      # wants MORE positive accel than OP. Stock's take-off ramp was
      # commanding +0.88 m/s² in drive 0000042 seg 3 while OP only wanted
      # +0.62; the car lagged stock's expected response profile and stock
      # cancelled 0.3s into the takeoff. Mirroring stock's value (when
      # higher) keeps stock's state machine satisfied. OP takes back over
      # once cruising above 5 m/s.
      #
      # Previously we gated on a 3s time window starting from the resume
      # blast, but that window expired mid-takeoff when the resume hit near
      # a segment boundary (drive 0000042 seg 2→3). vEgo threshold is a
      # more reliable trigger.
      #
      # Direction guard: only passes stock's POSITIVE accel. If OP wants to
      # brake during the takeoff (e.g. lead suddenly stopped), OP's value
      # drives — OP sees the lead via radar, stock's brake authority via
      # FSM3 is weak on this car anyway.
      takeoff_elapsed = (self.frame - self.takeoff_start_frame) * DT_CTRL
      stock_accel = float(CS.stock_FSM3["ACC_AccelerationRequest"])
      # 15s ceiling is a safety belt in case vEgo never crosses 5 (crawl
      # traffic) — eventually snap back to OP so runaway stock commands
      # can't persist indefinitely.
      in_takeoff_window = takeoff_elapsed < 15.0 and CS.out.vEgo < 5.0
      if in_takeoff_window and stock_accel > op_accel and stock_accel > 0:
        accel = stock_accel
      else:
        accel = op_accel

      # Standstill firm-hold: when stopped with cruise engaged and not
      # taking off, force a negative ACC_AccelerationRequest so the ECM
      # sees active brake intent. OP's planner outputs op_accel=0 at
      # full standstill (passive hold), but stock ACC interprets that as
      # "no active brake" and (per drive 0000004c at user-reported
      # 18:48:30) engages EPB ~2.3s later as a fallback hold — which
      # cancels cruise and locks the user out until they manually
      # release the EPB. -1.0 m/s² is firm enough that ECM won't ask
      # BCM for EPB, and the car is already at v=0 so the value isn't
      # actuated as additional decel.
      if (CS.out.cruiseState.enabled and CS.out.vEgo < 0.05
          and not in_takeoff_window and accel > -0.5):
        accel = -1.0

      # ACC_Check: 1 only during the post-resume acknowledgement window
      # (set by the SNG block above), else 0. Copying stock's ACC_Check —
      # as we were doing — left it at 0 almost always, so SNG resumes
      # worked only when the stock cam-bus value happened to flip in time.
      acc_check = 1 if self.sng_ack_frames > 0 else 0
      if self.sng_ack_frames > 0:
        self.sng_ack_frames -= 1

      # FSM3 only — FSM1 TX is handled below in its own block (it can fire
      # for engagement-assist independent of longActive).
      can_sends.append(volvocan.create_longitudinal(self.packer_pt, CS.stock_FSM3, accel, acc_check))

    # FSM1 TX (50 Hz, wall-clock gated). Fires whenever:
    #   - OP is in long-control (existing behavior — pass through or spoof)
    #   - OR engagement-assist is active (TX a fake lead so stock allows
    #     CC engagement at low speed without a real radar lead)
    fsm1_tx_active = CC.longActive or engagement_spoof_active
    fsm1_tx_due = fsm1_tx_active and now_nanos >= self.next_fsm1_tx_nanos
    if fsm1_tx_due:
      next_tx = self.next_fsm1_tx_nanos + self.LONG_TX_PERIOD_NANOS
      if self.next_fsm1_tx_nanos == 0 or next_tx <= now_nanos:
        next_tx = now_nanos + self.LONG_TX_PERIOD_NANOS
      self.next_fsm1_tx_nanos = next_tx

      # First TX after a quiet period: re-sync the smoothed state to stock's
      # current value so we don't ramp from a stale/wrong starting point.
      if not self._fsm1_was_txing:
        self.tx_acc_distance_state = float(stock_acc_dist)
      self._fsm1_was_txing = True

      can_sends.append(volvocan.create_radar(
        self.packer_pt, CS.stock_FSM1, True,
        override_distance=int(self.tx_acc_distance_state)
      ))
    elif not fsm1_tx_active:
      # Mark TX as inactive so next session re-syncs from stock value.
      self._fsm1_was_txing = False


    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_steer_prev

    self.frame += 1
    return new_actuators, can_sends
