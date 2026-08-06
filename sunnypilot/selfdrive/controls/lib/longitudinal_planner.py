"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from dataclasses import dataclass

from cereal import messaging, custom
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle

DecState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource


def volvo_model_stop_intent(v_ego: float, model_decel: float, model_should_stop: bool,
                            path_end: float, terminal_speed: float) -> bool:
  """Return raw model evidence consistent with a real stop.

  This is intentionally only the evidence classifier. VolvoStopIntentDebouncer
  below decides whether the evidence has persisted long enough to arm the
  vehicle-facing virtual lead.
  """
  strong_model_brake = model_decel < -0.75
  short_stopping_path = (v_ego > 3.0 and path_end < v_ego * 5.0 and
                         terminal_speed < max(2.0, v_ego * 0.5))
  return 0.0 < v_ego < 14.0 and (model_should_stop or strong_model_brake or short_stopping_path)


@dataclass
class VolvoStopIntentDebouncer:
  """Require sustained stop evidence before arming the Volvo virtual lead."""

  evidence_frames: int = 0
  active: bool = False

  # plannerd updates at DT_MDL (50 ms). Route 00000027 segment 2 armed on
  # one -0.504 m/s² sample; 400 ms rejects that transient while still
  # activating early in the sustained 1.7-2.0 s stop windows in routes
  # 00000069 and 00000052.
  ACTIVATION_FRAMES = round(0.4 / DT_MDL)
  RELEASE_FRAMES = round(0.15 / DT_MDL)
  EVIDENCE_DECAY = 2

  def reset(self) -> None:
    self.evidence_frames = 0
    self.active = False

  def update(self, eligible: bool, evidence: bool) -> bool:
    if not eligible:
      self.reset()
      return False

    if evidence:
      self.evidence_frames = min(self.ACTIVATION_FRAMES, self.evidence_frames + 1)
    else:
      self.evidence_frames = max(0, self.evidence_frames - self.EVIDENCE_DECAY)

    if self.active:
      self.active = self.evidence_frames > self.RELEASE_FRAMES
    else:
      self.active = self.evidence_frames >= self.ACTIVATION_FRAMES
    return self.active


class LongitudinalPlannerSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, mpc):
    self.events_sp = EventsSP()
    self.resolver = SpeedLimitResolver()
    self.dec = DynamicExperimentalController(CP, mpc)
    self.scc = SmartCruiseControl()
    self.resolver = SpeedLimitResolver()
    self.sla = SpeedLimitAssist(CP, CP_SP)
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts_helper = E2EAlertsHelper()

    self.output_v_target = 0.
    self.output_a_target = 0.
    self.volvo_stop_assist_enabled = CP.brand == "volvo" and CP.openpilotLongitudinalControl
    self.volvo_stop_assist_active = False
    self.volvo_stop_intent_debouncer = VolvoStopIntentDebouncer()

  def is_e2e(self, sm: messaging.SubMaster) -> bool:
    experimental_mode = sm['selfdriveState'].experimentalMode
    if not self.dec.active():
      return experimental_mode

    return experimental_mode and self.dec.mode() == "blended"

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    self.volvo_stop_assist_active = False
    CS = sm['carState']
    v_cruise_cluster_kph = min(CS.vCruiseCluster, V_CRUISE_MAX)
    v_cruise_cluster = v_cruise_cluster_kph * CV.KPH_TO_MS

    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override

    # Smart Cruise Control
    self.scc.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)

    # Speed Limit Resolver
    self.resolver.update(v_ego, sm)

    # Speed Limit Assist
    has_speed_limit = self.resolver.speed_limit_valid or self.resolver.speed_limit_last_valid
    self.sla.update(long_enabled, long_override, v_ego, a_ego, v_cruise_cluster, self.resolver.speed_limit,
                    self.resolver.speed_limit_final_last, has_speed_limit, self.resolver.distance, self.events_sp)

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
      LongitudinalPlanSource.speedLimitAssist: (self.sla.output_v_target, self.sla.output_a_target),
    }

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]

    # Stop-assist: when in experimental mode, the e2e model is sustaining a
    # brake command (desiredAccel < -0.5), there is no real radar lead, and
    # we are already below cruising speed, override v_target to 0 so the
    # MPC plans a full stop and OP commits to hard brake via FSM3.
    #
    # Why this is needed (drive 0000004a seg 8): TCPMV3 saw the red-light
    # context and ramped its desiredAcceleration to -1.4 m/s², but never
    # set shouldStop=True. The MPC's v_cruise was still at the user's
    # setpoint (~30 km/h, the Volvo stock-ACC floor), so the plan
    # plateaued at v=8 m/s instead of stopping. Driver had to brake.
    #
    # Brake authority is fine on this platform when OP commits — drive
    # 0000049 seg 5 showed -2.0 m/s² commanded, -2.0 m/s² actual. The
    # ACC_Distance spoof in carcontroller prevents stock from cancelling
    # below its no-lead-low-speed floor while we drive the car to a stop.
    try:
      exp_mode = sm['selfdriveState'].experimentalMode
      model_decel = sm['modelV2'].action.desiredAcceleration
      model_should_stop = bool(sm['modelV2'].action.shouldStop)
      lead_present = sm['radarState'].leadOne.status
      # Drive 0000052 user observation: after braking to a stop at an
      # intersection, modelV2's planned x-position trajectory truncates
      # at the stopping line. So model HAS spatial perception of the
      # intersection; it just doesn't translate that into a strong
      # desiredAcceleration. Use path_end as an alternative trigger —
      # if the model's planned position trajectory ends within ~50m
      # while we're moving, the model expects to stop ahead.
      try:
        pos_x = sm['modelV2'].position.x
        path_end = float(pos_x[-1]) if len(pos_x) else 1000.0
        velocity_x = sm['modelV2'].velocity.x
        terminal_speed = float(velocity_x[-1]) if len(velocity_x) else 1000.0
      except Exception:
        path_end = 1000.0
        terminal_speed = 1000.0
      # path_end < ~5 seconds of forward distance means model is planning
      # a hard stop within the horizon. Drive 0000054 23:40:40 brake event
      # showed path_end shrinking from 96m to 65m over 2.5s while v stayed
      # ~19 m/s — at v*3.5 the trigger only fired 0.2s before user braked,
      # at v*5.0 it fires ~3s earlier and gives the firm-brake clamp time
      # to actually take effect. v_ego > 3 to avoid triggering at near-
      # standstill where path naturally truncates.
      path_indicates_stop = (v_ego > 3.0 and path_end < v_ego * 5.0 and
                             terminal_speed < max(2.0, v_ego * 0.5))
      raw_stop_intent = volvo_model_stop_intent(v_ego, model_decel, model_should_stop,
                                                 path_end, terminal_speed)
      stop_assist_eligible = (self.volvo_stop_assist_enabled and exp_mode and long_enabled
                              and CS.cruiseState.enabled and not lead_present)
      stop_assist_active = self.volvo_stop_intent_debouncer.update(
        bool(stop_assist_eligible), bool(raw_stop_intent),
      )
      # NumPy comparisons can produce numpy.bool_. Cap'n Proto only accepts a
      # native bool; leaking numpy.bool_ here crashed plannerd on engagement.
      self.volvo_stop_assist_active = bool(stop_assist_active)
      if stop_assist_active and self.output_v_target > 0.1:
        self.output_v_target = 0.0
        # Use the more aggressive of model_decel or our default -1.0.
        # If path_indicates_stop fired but model_decel is tepid, we still
        # want firm brake because the path tells us a stop is needed.
        forced_a = min(model_decel, -1.0) if path_indicates_stop else model_decel
        self.output_a_target = min(self.output_a_target, forced_a)
        self.source = LongitudinalPlanSource.cruise

      # Takeoff-assist (green light, no lead): drive 0000052 seg 4 t=20.3
      # showed the model knew to go (shouldStop=False, dAccel=+0.17,
      # path_end=44m) for ~1s before the user gas-tapped, but actuators.accel
      # stayed at 0 because longcontrol was still in stopping state (planner
      # output was tiny positive, not enough to exit). Force v_target up
      # when at standstill and model clearly wants to move so MPC plans a
      # takeoff and longcontrol exits stopping; the SNG path's op_go_frames
      # trigger then sees actuators.accel > 0.5 and fires the resume blast.
      takeoff_wants_go = (v_ego < 0.5 and not lead_present
                          and (model_decel > 0.1 or path_end > 30.0))
      if takeoff_wants_go and self.output_v_target < 0.1:
        # Honor cruise setpoint as the takeoff target so we don't accelerate
        # past what the user set. v_cruise is the parameter passed in.
        self.output_v_target = max(self.output_v_target, v_cruise)
    except (KeyError, AttributeError):
      self.volvo_stop_intent_debouncer.reset()

    return self.output_v_target, self.output_a_target

  def update(self, sm: messaging.SubMaster) -> None:
    self.events_sp.clear()
    self.dec.update(sm)
    self.e2e_alerts_helper.update(sm, self.events_sp)

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.source
    longitudinalPlanSP.vTarget = float(self.output_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.volvoStopAssistActive = bool(self.volvo_stop_assist_active)
    longitudinalPlanSP.events = self.events_sp.to_msg()

    # Dynamic Experimental Control
    dec = longitudinalPlanSP.dec
    dec.state = DecState.blended if self.dec.mode() == 'blended' else DecState.acc
    dec.enabled = self.dec.enabled()
    dec.active = self.dec.active()

    # Smart Cruise Control
    smartCruiseControl = longitudinalPlanSP.smartCruiseControl
    # Vision Control
    sccVision = smartCruiseControl.vision
    sccVision.state = self.scc.vision.state
    sccVision.vTarget = float(self.scc.vision.output_v_target)
    sccVision.aTarget = float(self.scc.vision.output_a_target)
    sccVision.currentLateralAccel = float(self.scc.vision.current_lat_acc)
    sccVision.maxPredictedLateralAccel = float(self.scc.vision.max_pred_lat_acc)
    sccVision.enabled = self.scc.vision.is_enabled
    sccVision.active = self.scc.vision.is_active
    # Map Control
    sccMap = smartCruiseControl.map
    sccMap.state = self.scc.map.state
    sccMap.vTarget = float(self.scc.map.output_v_target)
    sccMap.aTarget = float(self.scc.map.output_a_target)
    sccMap.enabled = self.scc.map.is_enabled
    sccMap.active = self.scc.map.is_active

    # Speed Limit
    speedLimit = longitudinalPlanSP.speedLimit
    resolver = speedLimit.resolver
    resolver.speedLimit = float(self.resolver.speed_limit)
    resolver.speedLimitLast = float(self.resolver.speed_limit_last)
    resolver.speedLimitFinal = float(self.resolver.speed_limit_final)
    resolver.speedLimitFinalLast = float(self.resolver.speed_limit_final_last)
    resolver.speedLimitValid = self.resolver.speed_limit_valid
    resolver.speedLimitLastValid = self.resolver.speed_limit_last_valid
    resolver.speedLimitOffset = float(self.resolver.speed_limit_offset)
    resolver.distToSpeedLimit = float(self.resolver.distance)
    resolver.source = self.resolver.source
    assist = speedLimit.assist
    assist.state = self.sla.state
    assist.enabled = self.sla.is_enabled
    assist.active = self.sla.is_active
    assist.vTarget = float(self.sla.output_v_target)
    assist.aTarget = float(self.sla.output_a_target)

    # E2E Alerts
    e2eAlerts = longitudinalPlanSP.e2eAlerts
    e2eAlerts.greenLightAlert = self.e2e_alerts_helper.green_light_alert
    e2eAlerts.leadDepartAlert = self.e2e_alerts_helper.lead_depart_alert

    pm.send('longitudinalPlanSP', plan_sp_send)
