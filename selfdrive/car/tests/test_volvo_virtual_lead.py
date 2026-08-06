from openpilot.selfdrive.car.volvo_virtual_lead import VolvoVirtualLeadMonitor


def test_requires_both_virtual_lead_frames_and_acceptance():
  monitor = VolvoVirtualLeadMonitor()
  fsm0 = (0x51, bytes([0, 0, 0x07, 0, 0, 0, 0, 0]), 0)
  fsm1 = (0x260, bytes([8, 0, 0, 0, 0, 0, 0, 0]), 0)

  monitor.observe_tx([fsm0, fsm1], 1.0, real_lead=False)
  assert monitor.state(1.0) == (True, False)

  returned_fsm0 = (fsm0[0], fsm0[1], 0x80)
  returned_fsm1 = (fsm1[0], fsm1[1], 0x80)
  monitor.observe_rx([(0, [returned_fsm0, returned_fsm1])], 1.01)
  assert monitor.state(1.01) == (True, True)


def test_rejected_or_stale_frames_do_not_report_car_acceptance():
  monitor = VolvoVirtualLeadMonitor()
  fsm0 = (0x51, bytes([0, 0, 0x07, 0, 0, 0, 0, 0]), 0)
  fsm1 = (0x260, bytes([8, 0, 0, 0, 0, 0, 0, 0]), 0)
  monitor.observe_tx([fsm0, fsm1], 1.0, real_lead=False)
  monitor.observe_rx([(0, [(fsm0[0], fsm0[1], 0xC0),
                           (fsm1[0], fsm1[1], 0x80)])], 1.01)
  assert monitor.state(1.01) == (True, False)
  assert monitor.state(1.26) == (False, False)


def test_stop_transform_requires_returned_timing_exact_fsm4():
  monitor = VolvoVirtualLeadMonitor()
  fsm0 = (0x51, bytes([0, 0, 0x07, 0, 0, 0, 0, 0]), 0)
  fsm1 = (0x260, bytes([55, 0, 0, 0, 0, 0, 0, 0]), 0)
  config = (0x31A, bytes.fromhex("00564c340907e7b3"), 0)
  monitor.observe_tx([fsm0, fsm1, config], 1.0, real_lead=False)

  returned_fsm0 = (fsm0[0], fsm0[1], 0x80)
  returned_fsm1 = (fsm1[0], fsm1[1], 0x80)
  monitor.observe_rx([(0, [returned_fsm0, returned_fsm1])], 1.01)
  assert monitor.state(1.01) == (True, False)

  transformed_fsm4 = (0x31A, bytes.fromhex("aaf109078bf2f400"), 0x80)
  monitor.observe_rx([(0, [transformed_fsm4])], 1.02)
  assert monitor.state(1.02) == (True, True)


def test_stop_transform_monitor_tolerates_one_configuration_update_of_motion():
  monitor = VolvoVirtualLeadMonitor()
  fsm0 = (0x51, bytes([0, 0, 0x07, 0, 0, 0, 0, 0]), 0)
  fsm1 = (0x260, bytes([55, 0, 0, 0, 0, 0, 0, 0]), 0)
  # The most recent config has advanced by 1 km/h from the native frame that
  # Panda transformed just before userspace drained it.
  config = (0x31A, bytes.fromhex("00564c340806e6b3"), 0)
  monitor.observe_tx([fsm0, fsm1, config], 1.0, real_lead=False)

  returned = [
    (fsm0[0], fsm0[1], 0x80),
    (fsm1[0], fsm1[1], 0x80),
    (0x31A, bytes.fromhex("aaf109078bf2f400"), 0x80),
  ]
  monitor.observe_rx([(0, returned)], 1.01)
  assert monitor.state(1.01) == (True, True)


def test_raw_radar_stop_requires_stock_fsm_to_detect_the_virtual_target():
  monitor = VolvoVirtualLeadMonitor()
  fsm0 = (0x51, bytes([0, 0, 0x07, 0, 0, 0, 0, 0]), 0)
  fsm1 = (0x260, bytes([55, 0, 0, 0, 0, 0, 0, 0]), 0)
  config = (0x31A, bytes.fromhex("00564c340907e7b3"), 0)
  esr = (0x517, bytes.fromhex("006002261000c000"), 1)
  monitor.observe_tx([fsm0, fsm1, config, esr], 1.0, real_lead=False)
  assert monitor.state(1.0) == (True, False)

  returned = [
    (fsm0[0], fsm0[1], 0x80),
    (fsm1[0], fsm1[1], 0x80),
    (0x31A, bytes.fromhex("aaf109078bf2f400"), 0x80),
    (esr[0], esr[1], 0x81),
  ]
  monitor.observe_rx([(0, returned)], 1.01)
  assert monitor.state(1.01) == (True, False)

  stock_detection = [
    (0x51, bytes([0, 0, 0x07, 0, 0, 0, 0, 0]), 2),
    (0x260, bytes([55, 0, 0, 0, 0, 0, 0, 0]), 2),
    (0x31A, bytes.fromhex("aaf109078bf2f400"), 2),
  ]
  monitor.observe_rx([(0, stock_detection)], 1.02)
  assert monitor.state(1.02) == (True, True)


def test_real_lead_does_not_count_as_virtual_simulation():
  monitor = VolvoVirtualLeadMonitor()
  fsm0 = (0x51, bytes([0, 0, 0x07, 0, 0, 0, 0, 0]), 0)
  fsm1 = (0x260, bytes([8, 0, 0, 0, 0, 0, 0, 0]), 0)
  monitor.observe_tx([fsm0, fsm1], 1.0, real_lead=True)
  assert monitor.state(1.0) == (False, False)


def test_pre_engagement_front_car_spoof_reports_simulated():
  monitor = VolvoVirtualLeadMonitor()
  # FrontCar=1, Available=1, Enabled=0 is the intentional pre-SET state.
  fsm0 = (0x51, bytes([0, 0, 0x03, 0, 0, 0, 0, 0]), 0)
  fsm1 = (0x260, bytes([30, 0, 0, 0, 0, 0, 0, 0]), 0)
  monitor.observe_tx([fsm0, fsm1], 1.0, real_lead=False)
  assert monitor.state(1.0) == (True, False)
