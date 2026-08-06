from pathlib import Path

from cereal import log

from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl, BigMultiParamToggle, BigToggle
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.settings.common import restart_needed_callback
from openpilot.selfdrive.ui.ui_state import ui_state

PERSONALITY_TO_INT = log.LongitudinalPersonality.schema.enumerants
VOLVO_BRAKE_TEST_ARM_FILE = Path("/data/volvo_brake_test_armed")


class TogglesLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()

    self._personality_toggle = BigMultiParamToggle("driving personality", "LongitudinalPersonality", ["aggressive", "standard", "relaxed"])
    self._experimental_btn = BigParamControl("experimental mode", "ExperimentalMode")
    self._volvo_brake_test_toggle = BigToggle("arm Volvo brake test",
                                              initial_state=VOLVO_BRAKE_TEST_ARM_FILE.exists(),
                                              toggle_callback=self._on_volvo_brake_test_mode)
    is_metric_toggle = BigParamControl("use metric units", "IsMetric")
    ldw_toggle = BigParamControl("lane departure warnings", "IsLdwEnabled")
    always_on_dm_toggle = BigParamControl("always-on driver monitor", "AlwaysOnDM")
    record_front = BigParamControl("record & upload driver camera", "RecordFront", toggle_callback=restart_needed_callback)
    record_mic = BigParamControl("record & upload mic audio", "RecordAudio", toggle_callback=restart_needed_callback)
    enable_openpilot = BigParamControl("enable sunnypilot", "OpenpilotEnabledToggle", toggle_callback=restart_needed_callback)

    self._scroller.add_widgets([
      self._personality_toggle,
      self._experimental_btn,
      self._volvo_brake_test_toggle,
      is_metric_toggle,
      ldw_toggle,
      always_on_dm_toggle,
      record_front,
      record_mic,
      enable_openpilot,
    ])

    # Toggle lists
    self._refresh_toggles = (
      ("ExperimentalMode", self._experimental_btn),
      ("IsMetric", is_metric_toggle),
      ("IsLdwEnabled", ldw_toggle),
      ("AlwaysOnDM", always_on_dm_toggle),
      ("RecordFront", record_front),
      ("RecordAudio", record_mic),
      ("OpenpilotEnabledToggle", enable_openpilot),
    )

    enable_openpilot.set_enabled(lambda: not ui_state.engaged)
    self._volvo_brake_test_toggle.set_enabled(self._can_arm_volvo_brake_test)
    record_front.set_enabled(False if ui_state.params.get_bool("RecordFrontLock") else (lambda: not ui_state.engaged))
    record_mic.set_enabled(lambda: not ui_state.engaged)

    if ui_state.params.get_bool("ShowDebugInfo"):
      gui_app.set_show_touches(True)
      gui_app.set_show_fps(True)

    ui_state.add_engaged_transition_callback(self._update_toggles)

  def _update_state(self):
    super()._update_state()

    if ui_state.sm.updated["selfdriveState"]:
      personality = PERSONALITY_TO_INT[ui_state.sm["selfdriveState"].personality]
      if personality != ui_state.personality and ui_state.started:
        self._personality_toggle.set_value(self._personality_toggle._options[personality])
      ui_state.personality = personality

  def show_event(self):
    super().show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()

    # CP gating for experimental mode
    if ui_state.CP is not None:
      volvo_brake_test_available = (ui_state.CP.brand == "volvo" and ui_state.CP.openpilotLongitudinalControl and
                                    not ui_state.is_release)
      self._volvo_brake_test_toggle.set_visible(volvo_brake_test_available)
      if not volvo_brake_test_available:
        VOLVO_BRAKE_TEST_ARM_FILE.unlink(missing_ok=True)

      if ui_state.has_longitudinal_control:
        self._experimental_btn.set_visible(True)
        self._personality_toggle.set_visible(True)
      else:
        # no long for now
        self._experimental_btn.set_visible(False)
        self._experimental_btn.set_checked(False)
        self._personality_toggle.set_visible(False)
        ui_state.params.remove("ExperimentalMode")
    else:
      self._volvo_brake_test_toggle.set_visible(False)

    # Refresh toggles from params to mirror external changes
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))
    self._volvo_brake_test_toggle.set_checked(VOLVO_BRAKE_TEST_ARM_FILE.exists())

  @staticmethod
  def _can_arm_volvo_brake_test() -> bool:
    if ui_state.engaged:
      return False
    if not ui_state.started:
      return True
    CS = ui_state.sm["carState"]
    return ui_state.sm.valid["carState"] and abs(CS.vEgo) < 0.3

  def _on_volvo_brake_test_mode(self, state: bool):
    if state:
      self._volvo_brake_test_toggle.set_checked(False)
      if not self._can_arm_volvo_brake_test():
        return

      def confirm_callback():
        if not self._can_arm_volvo_brake_test():
          return
        VOLVO_BRAKE_TEST_ARM_FILE.touch()
        self._volvo_brake_test_toggle.set_checked(True)
        for param in ("JoystickDebugMode", "LongitudinalManeuverMode", "LateralManeuverMode"):
          ui_state.params.put_bool(param, False)

      icon = gui_app.texture("icons_mici/setup/red_warning.png", 64, 64)
      gui_app.push_widget(BigConfirmationDialog("slide to arm\nVolvo brake test", icon, confirm_callback, red=True))
    else:
      VOLVO_BRAKE_TEST_ARM_FILE.unlink(missing_ok=True)
