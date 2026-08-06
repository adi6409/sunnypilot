#!/usr/bin/env python3
import asyncio
import html
import json
import secrets
import time
from pathlib import Path

from aiohttp import web

from cereal import messaging
from openpilot.common.swaglog import cloudlog


PORT = 8089
ARM_FILE = Path("/data/volvo_brake_test_armed")
TRIGGER_REPEATS = 8
TRIGGER_INTERVAL = 0.05


def joystick_message(pressed: bool, mode: str):
  msg = messaging.new_message("testJoystick")
  msg.valid = True
  msg.testJoystick.axes = [0.0, 0.0]
  msg.testJoystick.buttons = [pressed and mode == "pulse", pressed and mode == "stop", pressed and mode == "resume"]
  return msg


class VolvoBrakeTestWeb:
  def __init__(self):
    self.pm = messaging.PubMaster(["testJoystick"])
    self.sm = messaging.SubMaster(["volvoBrakeTestStateSP"])
    self.token = secrets.token_urlsafe(24)
    self.trigger_lock = asyncio.Lock()
    self.stream_lock = asyncio.Lock()

  def status(self) -> dict:
    self.sm.update(0)
    service = "volvoBrakeTestStateSP"
    fresh = (self.sm.recv_frame[service] > 0 and self.sm.valid[service] and
             time.monotonic() - self.sm.recv_time[service] < 1.25)
    state = self.sm[service]
    return {
      "stale": not fresh,
      "armed": fresh and state.armed,
      "enabled": fresh and state.enabled,
      "active": fresh and state.active,
      "longActive": fresh and state.longActive,
      "noLead": fresh and state.noLead,
      "pedalsClear": fresh and state.pedalsClear,
      "speedKph": round(state.speedKph, 1) if fresh else 0.0,
      "ready": fresh and state.ready,
      "standstill": fresh and state.standstill,
      "resumeReady": fresh and state.resumeReady,
      "virtualLeadSimulated": fresh and state.virtualLeadSimulated,
      "virtualLeadAccepted": fresh and state.virtualLeadAccepted,
      "used": fresh and state.used,
    }

  async def index(self, request: web.Request):
    token = html.escape(self.token, quote=True)
    page = f"""<!doctype html>
<html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Volvo brake test</title>
<style>
body {{ background:#111; color:#eee; font:20px system-ui; margin:0; padding:24px; text-align:center; }}
#status {{ margin:24px auto; padding:18px; border-radius:12px; background:#292929; max-width:520px; white-space:pre-line; }}
button {{ display:block; width:min(92vw,520px); height:130px; margin:18px auto; border:0; border-radius:18px; background:#a00;
color:white; font-size:26px; font-weight:700; }}
#pulse {{ background:#8a4b00; }}
#resume {{ background:#087a35; }}
button:disabled {{ background:#555; color:#aaa; }}
.small {{ color:#bbb; font-size:16px; max-width:520px; margin:20px auto; }}
</style></head><body>
<h2>Volvo brake test</h2>
<div id="status">Connecting...</div>
<button id="pulse" disabled>HOLD 1 SECOND: BRAKE PULSE</button>
<button id="stop" disabled>HOLD 2 SECONDS: STOP AT RED</button>
<button id="resume" disabled>HOLD 1 SECOND: SIMULATE GREEN</button>
<div class="small">30–40 km/h only. sunnypilot longitudinal must already be active,
no lead may be present, and both pedals must be released. Gas, brake, or disengagement
cancels either test. STOP AT RED continues braking through zero and holds until you press
a pedal or disengage. SIMULATE GREEN becomes available only after that test reaches standstill.</div>
<div class="small">* “Car detects” requires Panda-confirmed FSM0/FSM1/FSM4 output, a confirmed virtual
Delphi radar frame, and the stock camera's own FSM0/FSM1/FSM4 changing to a lead state.</div>
<script>
const token = {token!r};
const status = document.getElementById('status');
const pulseButton = document.getElementById('pulse');
const stopButton = document.getElementById('stop');
const resumeButton = document.getElementById('resume');
const driveButtons = [pulseButton, stopButton];
let ready = false, fired = false, resumeFired = false;
function applyStatus(s) {{
  ready = s.ready;
  driveButtons.forEach(button => button.disabled = !ready || fired);
  resumeButton.disabled = !s.resumeReady || resumeFired;
  status.textContent = `${{s.stale ? 'STATUS STALE' : (s.ready ? 'READY' : 'NOT READY')}}\n${{s.speedKph}} km/h · ` +
    `long: ${{s.longActive ? 'active' : 'inactive'}}\n` +
    `real lead: ${{s.noLead ? 'none' : 'detected'}}\n` +
    `virtual lead simulated: ${{s.virtualLeadSimulated ? 'yes' : 'no'}}\n` +
    `car detects virtual lead*: ${{s.virtualLeadAccepted ? 'yes' : 'no'}}\n` +
    `one-shot test: ${{s.used ? 'already used' : 'available'}}\n` +
    `ACC: ${{s.enabled ? 'enabled' : 'disabled'}} · pedals: ${{s.pedalsClear ? 'clear' : 'pressed'}}`;
}}
const events = new EventSource('/events');
events.onmessage = event => applyStatus(JSON.parse(event.data));
events.onerror = () => {{
  ready = false;
  driveButtons.forEach(button => button.disabled = true);
  resumeButton.disabled = true;
  status.textContent = 'Connection lost';
}};
function setupButton(button, mode, holdMs, label) {{
  let holdTimer = null;
  function cancelHold() {{
    if (holdTimer) clearTimeout(holdTimer);
    holdTimer = null;
    if (!fired) button.textContent = label;
  }}
  button.addEventListener('pointerdown', e => {{
    e.preventDefault();
    if (!ready || fired) return;
    button.setPointerCapture(e.pointerId);
    button.textContent = 'KEEP HOLDING…';
    holdTimer = setTimeout(async () => {{
      holdTimer = null;
      fired = true;
      driveButtons.forEach(b => b.disabled = true);
      button.textContent = mode === 'stop' ? 'RED-LIGHT STOP REQUESTED' : 'PULSE REQUESTED';
      const r = await fetch(`/trigger/${{mode}}`, {{method:'POST', headers:{{'X-Brake-Test-Token':token}}}});
      if (!r.ok) {{ fired = false; button.textContent = 'REQUEST REJECTED'; }}
    }}, holdMs);
  }});
  button.addEventListener('pointerup', cancelHold);
  button.addEventListener('pointercancel', cancelHold);
}}
setupButton(pulseButton, 'pulse', 1000, 'HOLD 1 SECOND: BRAKE PULSE');
setupButton(stopButton, 'stop', 2000, 'HOLD 2 SECONDS: STOP AT RED');
let resumeTimer = null;
function cancelResumeHold() {{
  if (resumeTimer) clearTimeout(resumeTimer);
  resumeTimer = null;
  if (!resumeFired) resumeButton.textContent = 'HOLD 1 SECOND: SIMULATE GREEN';
}}
resumeButton.addEventListener('pointerdown', e => {{
  e.preventDefault();
  if (resumeButton.disabled || resumeFired) return;
  resumeButton.setPointerCapture(e.pointerId);
  resumeButton.textContent = 'KEEP HOLDING…';
  resumeTimer = setTimeout(async () => {{
    resumeTimer = null;
    resumeFired = true;
    resumeButton.disabled = true;
    resumeButton.textContent = 'GREEN-LIGHT RESUME REQUESTED';
    const r = await fetch('/resume', {{method:'POST', headers:{{'X-Brake-Test-Token':token}}}});
    if (!r.ok) {{ resumeFired = false; resumeButton.textContent = 'RESUME REJECTED'; }}
  }}, 1000);
}});
resumeButton.addEventListener('pointerup', cancelResumeHold);
resumeButton.addEventListener('pointercancel', cancelResumeHold);
</script></body></html>"""
    return web.Response(text=page, content_type="text/html", headers={"Cache-Control": "no-store"})

  async def get_status(self, request: web.Request):
    return web.json_response(self.status(), headers={"Cache-Control": "no-store"})

  async def events(self, request: web.Request):
    if self.stream_lock.locked():
      raise web.HTTPConflict(text="a status client is already connected")

    response = web.StreamResponse(headers={
      "Cache-Control": "no-store",
      "Content-Type": "text/event-stream",
      "Connection": "keep-alive",
      "X-Accel-Buffering": "no",
    })
    await response.prepare(request)

    try:
      async with self.stream_lock:
        while True:
          payload = json.dumps(self.status(), separators=(",", ":"))
          await response.write(f"data:{payload}\n\n".encode())
          await asyncio.sleep(0.5)
    except (ConnectionResetError, BrokenPipeError):
      pass
    return response

  async def trigger(self, request: web.Request):
    mode = request.match_info["mode"]
    if mode not in ("pulse", "stop"):
      raise web.HTTPNotFound()
    if request.headers.get("X-Brake-Test-Token") != self.token:
      raise web.HTTPForbidden()
    if not self.status()["ready"]:
      raise web.HTTPConflict(text="test preconditions are not satisfied")
    if self.trigger_lock.locked():
      raise web.HTTPConflict(text="trigger already in progress")

    async with self.trigger_lock:
      cloudlog.warning(f"Volvo brake {mode} test requested from local web control")
      for _ in range(TRIGGER_REPEATS):
        self.pm.send("testJoystick", joystick_message(True, mode))
        await asyncio.sleep(TRIGGER_INTERVAL)
      self.pm.send("testJoystick", joystick_message(False, mode))
    return web.json_response({"ok": True})

  async def resume(self, request: web.Request):
    if request.headers.get("X-Brake-Test-Token") != self.token:
      raise web.HTTPForbidden()
    if not self.status()["resumeReady"]:
      raise web.HTTPConflict(text="resume preconditions are not satisfied")
    if self.trigger_lock.locked():
      raise web.HTTPConflict(text="trigger already in progress")

    async with self.trigger_lock:
      cloudlog.warning("Volvo green-light resume requested from local web control")
      for _ in range(TRIGGER_REPEATS):
        self.pm.send("testJoystick", joystick_message(True, "resume"))
        await asyncio.sleep(TRIGGER_INTERVAL)
      self.pm.send("testJoystick", joystick_message(False, "resume"))
    return web.json_response({"ok": True})


def main():
  controller = VolvoBrakeTestWeb()
  app = web.Application()
  app.router.add_get("/", controller.index)
  app.router.add_get("/status", controller.get_status)
  app.router.add_get("/events", controller.events)
  app.router.add_post("/trigger/{mode}", controller.trigger)
  app.router.add_post("/resume", controller.resume)
  cloudlog.warning(f"Volvo brake test control listening on port {PORT}")
  web.run_app(app, access_log=None, host="0.0.0.0", port=PORT)


if __name__ == "__main__":
  main()
