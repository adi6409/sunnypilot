class VolvoVirtualLeadMonitor:
  """Tracks virtual-lead TX intent and Panda acceptance onto vehicle CAN."""

  FSM0 = 0x51
  FSM1 = 0x260
  FSM4 = 0x31A
  ESR_TRACK24 = 0x517
  MAIN_BUS = 0
  AUX_BUS = 1
  STOCK_CAM_BUS = 2
  RETURNED_MAIN_BUS = 0x80
  RETURNED_AUX_BUS = 0x81
  FRESH_SECONDS = 0.25
  FSM4_CONFIG_MAGIC = bytes((0x00, 0x56, 0x4C, 0x34))

  def __init__(self) -> None:
    self.tx_fsm0_at = 0.0
    self.tx_fsm1_at = 0.0
    self.tx_fsm4_config_at = 0.0
    self.tx_esr_at = 0.0
    self.accepted_fsm0_at = 0.0
    self.accepted_fsm1_at = 0.0
    self.accepted_fsm4_at = 0.0
    self.accepted_esr_at = 0.0
    self.stock_fsm0_lead_at = 0.0
    self.stock_fsm1_lead_at = 0.0
    self.stock_fsm4_lead_at = 0.0
    self.fsm4_required = False
    self.esr_required = False
    self.fsm4_secondary_speed = 0xFF
    self.fsm4_primary_speed = 0
    self.fsm4_relative_speed = 0
    self.fsm4_main_track_valid = False

  @staticmethod
  def _virtual_fsm0(dat: bytes) -> bool:
    # ACC_FrontCar is bit 0 of byte 2. Pre-engagement assist intentionally
    # leaves ACC_Enabled clear until the driver presses SET, so requiring all
    # three status bits would hide an actively simulated dashboard lead.
    return len(dat) == 8 and (dat[2] & 0x01) != 0

  @staticmethod
  def _virtual_fsm1(dat: bytes) -> bool:
    # 255 is Volvo's no-lead distance sentinel.
    return len(dat) == 8 and dat[0] < 255

  @staticmethod
  def _virtual_esr(dat: bytes) -> bool:
    return (len(dat) == 8 and dat[0] == 0x00 and dat[1] == 0x60 and
            (dat[2] & 0xF8) == 0 and (dat[4] & 0xFC) == 0x10 and
            (dat[6] & 0xC0) == 0xC0)

  @staticmethod
  def _relative_speed_raw(relative_speed_kph: int) -> int:
    scaled = relative_speed_kph * 100
    delta = (scaled + 4) // 9 if scaled >= 0 else -((-scaled + 4) // 9)
    return max(0, min(0x3FFF, 0x340A + delta))

  def _transformed_fsm4(self, dat: bytes) -> bool:
    if len(dat) != 8 or (dat[4] & 0x0F) != 0x0B:
      return False
    # Configuration is refreshed at 100 Hz while native FSM4 arrives at
    # roughly 33 Hz. Accept one update of motion between the transformed
    # frame and userspace observation so the live indicator does not flicker.
    if abs(dat[3] - self.fsm4_primary_speed) > 1:
      return False
    if self.fsm4_secondary_speed != 0xFF and abs(dat[2] - self.fsm4_secondary_speed) > 1:
      return False
    if bool(dat[5] & 0x40) != self.fsm4_main_track_valid:
      return False
    relative_speed_raw = ((dat[5] & 0x3F) << 8) | dat[6]
    return abs(relative_speed_raw - self._relative_speed_raw(self.fsm4_relative_speed)) <= 12

  def observe_tx(self, can_sends, now: float, real_lead: bool) -> None:
    if real_lead:
      return

    for address, dat, bus in can_sends:
      if bus == self.AUX_BUS and address == self.ESR_TRACK24 and self._virtual_esr(dat):
        self.esr_required = True
        self.tx_esr_at = now
      elif bus != self.MAIN_BUS:
        continue
      elif address == self.FSM0 and self._virtual_fsm0(dat):
        self.tx_fsm0_at = now
      elif address == self.FSM1 and self._virtual_fsm1(dat):
        self.tx_fsm1_at = now
      elif address == self.FSM4 and len(dat) == 8 and dat[:4] == self.FSM4_CONFIG_MAGIC:
        active = bool(dat[7] & 0x01)
        self.fsm4_required = active
        if not active:
          self.esr_required = False
        if active:
          self.tx_fsm4_config_at = now
          self.fsm4_secondary_speed = dat[4]
          self.fsm4_primary_speed = dat[5]
          self.fsm4_relative_speed = dat[6] if dat[6] < 0x80 else dat[6] - 0x100
          self.fsm4_main_track_valid = bool(dat[7] & 0x02)

  def observe_rx(self, can_batches, now: float) -> None:
    for _, frames in can_batches:
      for address, dat, src in frames:
        if src == self.RETURNED_MAIN_BUS:
          if address == self.FSM0 and self._virtual_fsm0(dat):
            self.accepted_fsm0_at = now
          elif address == self.FSM1 and self._virtual_fsm1(dat):
            self.accepted_fsm1_at = now
          elif address == self.FSM4 and self.fsm4_required and self._transformed_fsm4(dat):
            self.accepted_fsm4_at = now
        elif src == self.RETURNED_AUX_BUS and address == self.ESR_TRACK24 and self._virtual_esr(dat):
          self.accepted_esr_at = now
        elif src == self.STOCK_CAM_BUS:
          # These are the stock FSM's own, unmodified camera-bus outputs.
          # Requiring all three distinguishes actual upstream radar fusion
          # from merely confirming that our downstream overlays reached CAN.
          if address == self.FSM0 and self._virtual_fsm0(dat):
            self.stock_fsm0_lead_at = now
          elif address == self.FSM1 and self._virtual_fsm1(dat):
            self.stock_fsm1_lead_at = now
          elif address == self.FSM4 and len(dat) == 8 and (dat[4] & 0x0F) == 0x0B:
            self.stock_fsm4_lead_at = now

  def state(self, now: float) -> tuple[bool, bool]:
    fsm4_config_fresh = now - self.tx_fsm4_config_at < self.FRESH_SECONDS
    esr_tx_fresh = now - self.tx_esr_at < self.FRESH_SECONDS
    simulated = (now - self.tx_fsm0_at < self.FRESH_SECONDS and
                 now - self.tx_fsm1_at < self.FRESH_SECONDS and
                 (not self.fsm4_required or fsm4_config_fresh) and
                 (not self.esr_required or esr_tx_fresh))
    stock_fsm_detected = (now - self.stock_fsm0_lead_at < self.FRESH_SECONDS and
                          now - self.stock_fsm1_lead_at < self.FRESH_SECONDS and
                          now - self.stock_fsm4_lead_at < self.FRESH_SECONDS)
    accepted = (simulated and
                now - self.accepted_fsm0_at < self.FRESH_SECONDS and
                now - self.accepted_fsm1_at < self.FRESH_SECONDS and
                (not self.fsm4_required or now - self.accepted_fsm4_at < self.FRESH_SECONDS) and
                (not self.esr_required or
                 (now - self.accepted_esr_at < self.FRESH_SECONDS and stock_fsm_detected)))
    return simulated, accepted
