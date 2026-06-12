#!/usr/bin/env python3
"""
web_gcs.py - A lightweight web-based PX4 ground control station.

Features:
  1. Pick a serial port (e.g. ttyACM0 / ttyACM1) and connect over MAVLink.
  2. Visualize the function assigned to every PWM output (raw param values).
  3. Visualize the role of every serial port (MAVLink, GPS, RC, ...).
  4. List detected sensors with model name and bus type (SPI / I2C / ...).
  5. An interactive shell to run PX4 nsh commands over MAVLink.

Only depends on pymavlink (which pulls in pyserial). Install with:
    pip3 install --user pymavlink pyserial

Run:
    python3 Tools/web_gcs.py            # then open http://127.0.0.1:8080
    python3 Tools/web_gcs.py --port 8080 --host 127.0.0.1
"""

import argparse
import json
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from queue import Queue, Empty

try:
    from pymavlink import mavutil
except ImportError:
    raise SystemExit(
        "Failed to import pymavlink.\n"
        "Install it with:  pip3 install --user pymavlink pyserial")

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None


# --------------------------------------------------------------------------
# Decoding tables
# --------------------------------------------------------------------------

# device_id bitfield: bits[0:2]=bus_type, [3:7]=bus, [8:15]=address, [16:23]=devtype
BUS_TYPE = {0: "UNKNOWN", 1: "I2C", 2: "SPI", 3: "UAVCAN",
            4: "SIMULATION", 5: "SERIAL", 6: "MAVLINK"}

# Subset of src/drivers/drv_sensor.h DRV_*_DEVTYPE_* (devtype -> model name)
DEVTYPE = {
    0x01: "HMC5883", 0x03: "MAGSIM", 0x04: "AK8963", 0x05: "LIS3MDL",
    0x06: "IST8310", 0x07: "RM3100", 0x08: "QMC5883L", 0x09: "AK09916",
    0x0A: "VCM1193L", 0x0B: "IST8308", 0x0D: "MMC5983MA", 0x0E: "IIS2MDC",
    0x0F: "QMC5883P", 0x10: "AF9838",
    0x11: "LSM303D", 0x12: "LSM6DSV", 0x13: "LSM6DSV32X", 0x14: "IMU_SIM",
    0x17: "LSM6DSK320X",
    0x21: "MPU6000", 0x22: "L3GD20", 0x24: "MPU9250", 0x25: "ICM20649",
    0x26: "ICM42688P", 0x27: "ICM40609D", 0x28: "ICM20948", 0x29: "ICM42605",
    0x2A: "ICM42670P", 0x2B: "IIM42652", 0x2C: "IAM20680HP", 0x2D: "ICM42686P",
    0x2E: "IIM42653",
    0x33: "MPU6050", 0x34: "ICM45686", 0x35: "MPU6050(gyr)", 0x36: "MPU6500",
    0x37: "BMI270", 0x38: "ICM20602", 0x3A: "ICM20608G", 0x3C: "ICM20689",
    0x3D: "MS5611", 0x3E: "MS5607", 0x3F: "BMP280", 0x40: "LPS25H",
    0x41: "BMI055(acc)", 0x42: "BMI055(gyr)", 0x43: "BMM150",
    0x44: "LSM9DS1_AG", 0x45: "LSM9DS1_M",
    0x4D: "TCBP001TA", 0x4E: "MS5837", 0x4F: "SPL06",
    0x50: "LPS33HW", 0x51: "MPL3115A2", 0x52: "FXOS8701C", 0x54: "FXAS2100C",
    0x57: "ADIS16448", 0x58: "ADIS16470", 0x59: "ADIS16477", 0x5A: "ADIS16507",
    0x5B: "SCH16T", 0x5F: "MPC2520",
    0x60: "LPS22HB", 0x61: "LSM303AGR(acc)", 0x62: "LSM303AGR(mag)",
    0x65: "BAROSIM", 0x66: "BMI088(gyr)", 0x67: "BMP388", 0x68: "DPS310",
    0x6A: "BMI088(acc)", 0x6B: "ATXXXX", 0x6C: "BMI085(acc)", 0x6D: "BMI085(gyr)",
    0x6E: "BMP390", 0x6F: "BMP581",
    0x80: "UAVCAN_ACC", 0x81: "UAVCAN_BARO", 0x85: "UAVCAN_GPS",
    0x86: "UAVCAN_GYR", 0x87: "UAVCAN_IMU", 0x88: "UAVCAN_MAG",
}

# Serial port identifier values used by *_CONFIG params
SERIAL_PORTS = {
    0: "Disabled", 101: "TELEM 1", 102: "TELEM 2", 103: "TELEM 3",
    104: "TELEM/SERIAL 4", 105: "TELEM 5", 201: "GPS 1", 202: "GPS 2",
    203: "GPS 3", 204: "GPS 4", 300: "Radio Controller", 1000: "Wifi",
}


def decode_device_id(devid):
    bus_type = devid & 0x7
    bus = (devid >> 3) & 0x1F
    address = (devid >> 8) & 0xFF
    devtype = (devid >> 16) & 0xFF
    return {
        "bus_type": BUS_TYPE.get(bus_type, str(bus_type)),
        "bus": bus,
        "address": address,
        "model": DEVTYPE.get(devtype, "0x%02X" % devtype),
        "devid": devid,
    }


def output_function_name(v):
    v = int(v)
    if v == 0:
        return "Disabled"
    if v == 1:
        return "Constant_Min"
    if v == 2:
        return "Constant_Max"
    if 101 <= v <= 148:
        return "Motor %d" % (v - 100)
    if 201 <= v <= 248:
        return "Servo %d" % (v - 200)
    if 400 <= v <= 415:
        return "Peripheral via Actuator %d" % (v - 399)
    return "Func %d" % v


def serial_port_name(v):
    v = int(v)
    return SERIAL_PORTS.get(v, "Port %d" % v)


# --------------------------------------------------------------------------
# MAVLink connection (all MAV I/O happens in one background thread)
# --------------------------------------------------------------------------

class Connection:
    def __init__(self):
        self.lock = threading.Lock()
        self.thread = None
        self.stop_flag = False
        self.cmd_q = Queue()

        self.port = None
        self.baud = None
        self.status = "disconnected"     # disconnected | connecting | connected | error
        self.error = ""
        self.target_system = 1
        self.target_component = 1

        self.heartbeat = {}
        self.sys_status = {}
        self.last_heartbeat = 0.0

        self.params = {}
        self.param_count = 0
        self.params_loaded = False

        self.shell_buf = ""
        self.capturing = False
        self.cap_buf = ""

        self.sensors = []
        self.scanning = False

    # -- public API (called from HTTP threads) -----------------------------

    def connect(self, port, baud):
        if self.thread and self.thread.is_alive():
            self.disconnect()
        self.port = port
        self.baud = int(baud)
        self.stop_flag = False
        self.status = "connecting"
        self.error = ""
        self.params = {}
        self.param_count = 0
        self.params_loaded = False
        self.shell_buf = ""
        self.sensors = []
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def disconnect(self):
        self.stop_flag = True
        if self.thread:
            self.thread.join(timeout=3)
        self.status = "disconnected"

    def send_shell(self, cmd):
        self.cmd_q.put(("shell", cmd))

    def scan_sensors(self):
        if not self.scanning:
            self.scanning = True
            self.cmd_q.put(("scan", None))

    def snapshot(self):
        with self.lock:
            return {
                "status": self.status,
                "error": self.error,
                "port": self.port,
                "baud": self.baud,
                "heartbeat": dict(self.heartbeat),
                "sys_status": dict(self.sys_status),
                "last_heartbeat_age": (time.time() - self.last_heartbeat)
                if self.last_heartbeat else None,
                "param_count": self.param_count,
                "params_received": len(self.params),
                "params_loaded": self.params_loaded,
                "scanning": self.scanning,
            }

    # -- background thread -------------------------------------------------

    def _run(self):
        try:
            self.master = mavutil.mavlink_connection(
                self.port, baud=self.baud, source_system=255,
                source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER)
        except Exception as e:  # noqa: BLE001
            with self.lock:
                self.status = "error"
                self.error = "open failed: %s" % e
            return

        self._send_heartbeat()
        hb = self.master.wait_heartbeat(timeout=10)
        if hb is None:
            with self.lock:
                self.status = "error"
                self.error = "no heartbeat (check port/baud)"
            return

        self.target_system = self.master.target_system
        self.target_component = self.master.target_component
        with self.lock:
            self.status = "connected"

        # request all parameters
        self.master.mav.param_request_list_send(
            self.target_system, self.target_component)

        last_hb = time.time()
        last_param_chase = time.time()
        while not self.stop_flag:
            # outgoing commands
            try:
                while True:
                    kind, payload = self.cmd_q.get_nowait()
                    if kind == "shell":
                        self._write_shell(payload + "\n")
                    elif kind == "scan":
                        self._do_scan()
            except Empty:
                pass

            now = time.time()
            if now - last_hb > 1.0:
                self._send_heartbeat()
                last_hb = now

            # chase missing params over lossy links
            if (not self.params_loaded and self.param_count
                    and now - last_param_chase > 3.0):
                self._chase_params()
                last_param_chase = now

            self._pump(0.1)

        try:
            self.master.close()
        except Exception:  # noqa: BLE001
            pass

    def _send_heartbeat(self):
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def _pump(self, timeout):
        msg = self.master.recv_match(blocking=True, timeout=timeout)
        if msg is not None:
            self._dispatch(msg)

    def _dispatch(self, msg):
        t = msg.get_type()
        if t == "HEARTBEAT":
            with self.lock:
                self.last_heartbeat = time.time()
                self.heartbeat = {
                    "type": msg.type,
                    "autopilot": msg.autopilot,
                    "base_mode": msg.base_mode,
                    "custom_mode": msg.custom_mode,
                    "system_status": msg.system_status,
                    "armed": bool(msg.base_mode
                                  & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                }
        elif t == "SYS_STATUS":
            with self.lock:
                self.sys_status = {
                    "voltage_battery": msg.voltage_battery,
                    "current_battery": msg.current_battery,
                    "battery_remaining": msg.battery_remaining,
                    "load": msg.load,
                }
        elif t == "PARAM_VALUE":
            name = msg.param_id
            if isinstance(name, bytes):
                name = name.decode("utf-8", "replace")
            name = name.split("\x00")[0]
            value = msg.param_value
            is_int = msg.param_type in (
                mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
                mavutil.mavlink.MAV_PARAM_TYPE_INT8,
                mavutil.mavlink.MAV_PARAM_TYPE_UINT16,
                mavutil.mavlink.MAV_PARAM_TYPE_INT16,
                mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
                mavutil.mavlink.MAV_PARAM_TYPE_INT32,
            )
            with self.lock:
                self.param_count = msg.param_count
                self.params[name] = int(round(value)) if is_int else value
                if len(self.params) >= self.param_count:
                    self.params_loaded = True
        elif t == "SERIAL_CONTROL":
            if msg.count == 0:
                return
            data = bytes(bytearray(msg.data[:msg.count]))
            text = data.decode("utf-8", "replace")
            with self.lock:
                if self.capturing:
                    self.cap_buf += text
                else:
                    self.shell_buf += text
                    if len(self.shell_buf) > 200000:
                        self.shell_buf = self.shell_buf[-200000:]

    def _write_shell(self, s):
        b = s.encode("utf-8")
        while len(b) > 0:
            chunk = b[:70]
            b = b[70:]
            buf = list(chunk) + [0] * (70 - len(chunk))
            self.master.mav.serial_control_send(
                mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
                mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
                | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                0, 0, len(chunk), buf)

    def _collect(self, duration):
        """Run the shell and capture its output for `duration` seconds."""
        with self.lock:
            self.cap_buf = ""
            self.capturing = True
        t0 = time.time()
        last_hb = t0
        while time.time() - t0 < duration:
            now = time.time()
            if now - last_hb > 1.0:
                self._send_heartbeat()
                last_hb = now
            self._pump(0.05)
        with self.lock:
            self.capturing = False
            return self.cap_buf

    def _do_scan(self):
        results = []
        seen = set()
        topics = ["sensor_accel", "sensor_gyro", "sensor_baro", "sensor_mag"]
        # make sure a shell session exists
        self._write_shell("\n")
        self._collect(0.3)
        for topic in topics:
            for inst in range(3):
                self._write_shell("listener %s -i %d -n 1\n" % (topic, inst))
                out = self._collect(0.5)
                for m in re.findall(r"device_id:\s*(\d+)", out):
                    devid = int(m)
                    if devid == 0 or devid in seen:
                        continue
                    seen.add(devid)
                    info = decode_device_id(devid)
                    info["topic"] = topic
                    results.append(info)
        with self.lock:
            self.sensors = results
            self.scanning = False

    def _chase_params(self):
        # re-request the full list to recover params dropped on lossy links
        self.master.mav.param_request_list_send(
            self.target_system, self.target_component)


CONN = Connection()


# --------------------------------------------------------------------------
# Derived views
# --------------------------------------------------------------------------

def view_pwm():
    with CONN.lock:
        params = dict(CONN.params)
    rows = []
    pat = re.compile(r"^(PWM_(?:MAIN|AUX|EXTRA))_FUNC(\d+)$")
    for name, value in params.items():
        m = pat.match(name)
        if m:
            rows.append({
                "param": name,
                "group": m.group(1),
                "index": int(m.group(2)),
                "value": int(value),
                "function": output_function_name(value),
            })
    rows.sort(key=lambda r: (r["group"], r["index"]))
    return rows


def view_serial():
    with CONN.lock:
        params = dict(CONN.params)
    rows = []
    for name, value in params.items():
        if name.endswith("_CONFIG") and not name.startswith("RC"):
            role = name[:-len("_CONFIG")]
            rows.append({
                "param": name,
                "role": role,
                "value": int(value),
                "port": serial_port_name(value),
            })
        elif name == "RC_PORT_CONFIG":
            rows.append({
                "param": name, "role": "RC Input",
                "value": int(value), "port": serial_port_name(value),
            })
    rows.sort(key=lambda r: r["role"])
    # collect SER_*_BAUD as standalone info
    bauds = {n: int(v) for n, v in params.items()
             if n.startswith("SER_") and n.endswith("_BAUD")}
    return {"roles": rows, "bauds": bauds}


# --------------------------------------------------------------------------
# HTTP server
# --------------------------------------------------------------------------

def list_serial_ports():
    ports = []
    if list_ports is not None:
        for p in list_ports.comports():
            ports.append({"device": p.device,
                          "description": p.description or ""})

    def rank(x):
        d = x["device"]
        if "ACM" in d:
            return (0, d)
        if "USB" in d:
            return (1, d)
        return (2, d)

    ports.sort(key=rank)
    return ports


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _send(self, code, body, ctype="application/json"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body)
        data = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _read_json(self):
        length = int(self.headers.get("Content-Length", 0))
        if length == 0:
            return {}
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except Exception:  # noqa: BLE001
            return {}

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            self._send(200, HTML, "text/html; charset=utf-8")
        elif self.path == "/api/ports":
            self._send(200, {"ports": list_serial_ports()})
        elif self.path == "/api/state":
            self._send(200, CONN.snapshot())
        elif self.path == "/api/pwm":
            self._send(200, {"rows": view_pwm()})
        elif self.path == "/api/serial":
            self._send(200, view_serial())
        elif self.path == "/api/sensors":
            with CONN.lock:
                self._send(200, {"sensors": list(CONN.sensors),
                                 "scanning": CONN.scanning})
        elif self.path == "/api/shell_output":
            with CONN.lock:
                self._send(200, {"text": CONN.shell_buf})
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self):
        body = self._read_json()
        if self.path == "/api/connect":
            port = body.get("port")
            baud = body.get("baud", 57600)
            if not port:
                self._send(400, {"error": "no port"})
                return
            CONN.connect(port, baud)
            self._send(200, {"ok": True})
        elif self.path == "/api/disconnect":
            CONN.disconnect()
            self._send(200, {"ok": True})
        elif self.path == "/api/shell":
            cmd = body.get("cmd", "")
            CONN.send_shell(cmd)
            self._send(200, {"ok": True})
        elif self.path == "/api/scan_sensors":
            CONN.scan_sensors()
            self._send(200, {"ok": True})
        else:
            self._send(404, {"error": "not found"})


HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>PX4 Web GCS</title>
<style>
  :root { --bg:#0f1419; --panel:#1a2027; --panel2:#222b34; --fg:#e6edf3;
          --muted:#8b98a5; --accent:#3fb950; --accent2:#58a6ff; --err:#f85149; }
  * { box-sizing: border-box; }
  body { margin:0; font-family: ui-sans-serif, system-ui, "Segoe UI", Roboto, sans-serif;
         background:var(--bg); color:var(--fg); }
  header { display:flex; align-items:center; gap:16px; padding:10px 16px;
           background:var(--panel); border-bottom:1px solid #30363d; flex-wrap:wrap; }
  header h1 { font-size:16px; margin:0; font-weight:600; }
  select, input, button { font-size:13px; padding:6px 10px; border-radius:6px;
           border:1px solid #30363d; background:var(--panel2); color:var(--fg); }
  button { cursor:pointer; }
  button.primary { background:var(--accent); color:#04260f; border-color:var(--accent); font-weight:600; }
  button.danger { background:var(--err); color:#2b0b0a; border-color:var(--err); }
  .status { margin-left:auto; display:flex; align-items:center; gap:8px; font-size:13px; }
  .dot { width:10px; height:10px; border-radius:50%; background:#555; }
  .dot.connected { background:var(--accent); }
  .dot.connecting { background:#d29922; }
  .dot.error { background:var(--err); }
  nav { display:flex; gap:4px; padding:8px 16px; background:var(--panel);
        border-bottom:1px solid #30363d; }
  nav button { background:transparent; border:none; color:var(--muted); padding:8px 14px; }
  nav button.active { color:var(--fg); border-bottom:2px solid var(--accent2); border-radius:0; }
  main { padding:16px; }
  .tab { display:none; }
  .tab.active { display:block; }
  table { border-collapse:collapse; width:100%; font-size:13px; }
  th, td { text-align:left; padding:7px 10px; border-bottom:1px solid #283038; }
  th { color:var(--muted); font-weight:600; }
  tr:hover td { background:#161b22; }
  .pill { display:inline-block; padding:2px 8px; border-radius:10px; font-size:11px;
          background:var(--panel2); border:1px solid #30363d; }
  .pill.SPI { color:#79c0ff; border-color:#1f6feb; }
  .pill.I2C { color:#d2a8ff; border-color:#8957e5; }
  .pill.disabled { color:var(--muted); }
  .pill.active { color:var(--accent); border-color:var(--accent); }
  .muted { color:var(--muted); }
  .cards { display:flex; gap:12px; flex-wrap:wrap; margin-bottom:14px; }
  .card { background:var(--panel); border:1px solid #30363d; border-radius:8px;
          padding:10px 14px; min-width:120px; }
  .card .v { font-size:18px; font-weight:600; }
  .card .k { font-size:11px; color:var(--muted); text-transform:uppercase; }
  #shellOut { background:#05080b; color:#c9d1d9; font-family: ui-monospace, monospace;
        font-size:12.5px; padding:12px; border-radius:8px; height:52vh; overflow:auto;
        white-space:pre-wrap; border:1px solid #30363d; }
  .shellbar { display:flex; gap:8px; margin-top:10px; }
  .shellbar input { flex:1; font-family: ui-monospace, monospace; }
  .hint { font-size:12px; color:var(--muted); margin:6px 0 14px; }
</style>
</head>
<body>
<header>
  <h1>PX4 Web GCS</h1>
  <select id="portSel"></select>
  <button onclick="refreshPorts()">&#x21bb;</button>
  <select id="baudSel">
    <option value="57600">57600</option>
    <option value="115200">115200</option>
    <option value="921600">921600</option>
    <option value="500000">500000</option>
  </select>
  <button class="primary" id="connectBtn" onclick="toggleConnect()">Connect</button>
  <div class="status">
    <span class="dot" id="dot"></span>
    <span id="statusText">disconnected</span>
  </div>
</header>
<nav>
  <button class="active" onclick="showTab('overview', this)">Overview</button>
  <button onclick="showTab('pwm', this)">PWM Outputs</button>
  <button onclick="showTab('serial', this)">Serial Ports</button>
  <button onclick="showTab('sensors', this)">Sensors</button>
  <button onclick="showTab('shell', this)">Shell</button>
</nav>
<main>
  <section id="overview" class="tab active">
    <div class="cards" id="ovCards"></div>
    <div class="hint" id="paramProgress"></div>
  </section>

  <section id="pwm" class="tab">
    <div class="hint">Each output and its assigned function (raw value from PWM_*_FUNC* params).</div>
    <table><thead><tr><th>Param</th><th>Group</th><th>Channel</th>
      <th>Raw value</th><th>Function</th></tr></thead>
      <tbody id="pwmBody"></tbody></table>
  </section>

  <section id="serial" class="tab">
    <div class="hint">Role assigned to each port and the port baud rates.</div>
    <table><thead><tr><th>Role</th><th>Param</th><th>Raw value</th><th>Port</th></tr></thead>
      <tbody id="serialBody"></tbody></table>
    <h3>Baud rates</h3>
    <table><thead><tr><th>Param</th><th>Baud</th></tr></thead>
      <tbody id="baudBody"></tbody></table>
  </section>

  <section id="sensors" class="tab">
    <div class="hint">Detected sensors with model and bus type (decoded from device_id via uORB).
      <button onclick="scanSensors()" id="scanBtn">Scan sensors</button></div>
    <table><thead><tr><th>uORB topic</th><th>Model</th><th>Bus</th>
      <th>Bus #</th><th>Address</th><th>device_id</th></tr></thead>
      <tbody id="sensorBody"></tbody></table>
  </section>

  <section id="shell" class="tab">
    <div id="shellOut"></div>
    <div class="shellbar">
      <input id="shellIn" placeholder="type a PX4 command, e.g. 'top', 'listener vehicle_status', 'param show SYS_AUTOSTART'"
             onkeydown="if(event.key==='Enter')sendShell()">
      <button class="primary" onclick="sendShell()">Send</button>
    </div>
    <div class="hint">Tip: press Enter to send. Try <code>help</code>, <code>top</code>,
      <code>ver all</code>, <code>param show</code>.</div>
  </section>
</main>

<script>
let connected = false;
let activeTab = 'overview';

function showTab(id, btn) {
  activeTab = id;
  document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
  document.getElementById(id).classList.add('active');
  document.querySelectorAll('nav button').forEach(b => b.classList.remove('active'));
  btn.classList.add('active');
  if (id === 'pwm') loadPwm();
  if (id === 'serial') loadSerial();
  if (id === 'sensors') loadSensors();
}

async function refreshPorts() {
  const r = await fetch('/api/ports'); const d = await r.json();
  const sel = document.getElementById('portSel');
  sel.innerHTML = '';
  if (d.ports.length === 0) {
    const o = document.createElement('option'); o.textContent = 'no ports found'; sel.appendChild(o);
  }
  d.ports.forEach(p => {
    const o = document.createElement('option');
    o.value = p.device;
    o.textContent = p.device + (p.description ? '  (' + p.description + ')' : '');
    sel.appendChild(o);
  });
}

async function toggleConnect() {
  if (connected || document.getElementById('statusText').textContent === 'connecting') {
    await fetch('/api/disconnect', {method:'POST'});
  } else {
    const port = document.getElementById('portSel').value;
    const baud = document.getElementById('baudSel').value;
    await fetch('/api/connect', {method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({port, baud: parseInt(baud)})});
  }
}

async function pollState() {
  try {
    const r = await fetch('/api/state'); const s = await r.json();
    const dot = document.getElementById('dot');
    const txt = document.getElementById('statusText');
    dot.className = 'dot ' + s.status;
    connected = (s.status === 'connected');
    document.getElementById('connectBtn').textContent = connected ? 'Disconnect' : 'Connect';
    document.getElementById('connectBtn').className = connected ? 'danger' : 'primary';
    let label = s.status;
    if (s.status === 'error') label = 'error: ' + s.error;
    if (connected) label = 'connected ' + (s.port || '');
    txt.textContent = label;

    renderOverview(s);
    document.getElementById('paramProgress').textContent =
      s.param_count ? ('parameters: ' + s.params_received + ' / ' + s.param_count
        + (s.params_loaded ? ' (loaded)' : ' (loading...)')) : 'waiting for parameters...';
  } catch(e) {}
}

function renderOverview(s) {
  const hb = s.heartbeat || {};
  const sys = s.sys_status || {};
  const cards = [
    ['Link', connected ? 'UP' : 'DOWN'],
    ['Armed', hb.armed === undefined ? '-' : (hb.armed ? 'ARMED' : 'disarmed')],
    ['System status', hb.system_status === undefined ? '-' : hb.system_status],
    ['Battery', sys.voltage_battery && sys.voltage_battery !== 65535 ?
        (sys.voltage_battery/1000).toFixed(2) + ' V' : 'n/a'],
    ['CPU load', sys.load !== undefined ? (sys.load/10).toFixed(1) + ' %' : '-'],
    ['HB age', s.last_heartbeat_age === null ? '-' : s.last_heartbeat_age.toFixed(1) + ' s'],
  ];
  document.getElementById('ovCards').innerHTML = cards.map(c =>
    '<div class="card"><div class="k">'+c[0]+'</div><div class="v">'+c[1]+'</div></div>').join('');
}

async function loadPwm() {
  const r = await fetch('/api/pwm'); const d = await r.json();
  document.getElementById('pwmBody').innerHTML = d.rows.map(x =>
    '<tr><td>'+x.param+'</td><td>'+x.group+'</td><td>'+x.index+'</td>'
    +'<td>'+x.value+'</td><td><span class="pill '+(x.value? 'active':'disabled')+'">'
    +x.function+'</span></td></tr>').join('') || '<tr><td colspan="5" class="muted">no PWM params (connect & wait for params)</td></tr>';
}

async function loadSerial() {
  const r = await fetch('/api/serial'); const d = await r.json();
  document.getElementById('serialBody').innerHTML = (d.roles||[]).map(x =>
    '<tr><td>'+x.role+'</td><td>'+x.param+'</td><td>'+x.value+'</td>'
    +'<td><span class="pill '+(x.value? 'active':'disabled')+'">'+x.port+'</span></td></tr>').join('')
    || '<tr><td colspan="4" class="muted">no serial config params</td></tr>';
  const bauds = d.bauds || {};
  document.getElementById('baudBody').innerHTML = Object.keys(bauds).sort().map(k =>
    '<tr><td>'+k+'</td><td>'+bauds[k]+'</td></tr>').join('')
    || '<tr><td colspan="2" class="muted">none</td></tr>';
}

async function loadSensors() {
  const r = await fetch('/api/sensors'); const d = await r.json();
  document.getElementById('scanBtn').textContent = d.scanning ? 'Scanning...' : 'Scan sensors';
  document.getElementById('sensorBody').innerHTML = (d.sensors||[]).map(x =>
    '<tr><td>'+x.topic+'</td><td><b>'+x.model+'</b></td>'
    +'<td><span class="pill '+x.bus_type+'">'+x.bus_type+'</span></td>'
    +'<td>'+x.bus+'</td><td>'+x.address+'</td><td>'+x.devid+'</td></tr>').join('')
    || '<tr><td colspan="6" class="muted">no sensors yet - click "Scan sensors"</td></tr>';
}

async function scanSensors() {
  await fetch('/api/scan_sensors', {method:'POST'});
  document.getElementById('scanBtn').textContent = 'Scanning...';
}

async function sendShell() {
  const inp = document.getElementById('shellIn');
  const cmd = inp.value;
  inp.value = '';
  await fetch('/api/shell', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({cmd})});
}

async function pollShell() {
  if (activeTab !== 'shell') return;
  try {
    const r = await fetch('/api/shell_output'); const d = await r.json();
    const el = document.getElementById('shellOut');
    const atBottom = el.scrollTop + el.clientHeight >= el.scrollHeight - 20;
    el.textContent = d.text;
    if (atBottom) el.scrollTop = el.scrollHeight;
  } catch(e) {}
}

refreshPorts();
setInterval(pollState, 1000);
setInterval(pollShell, 500);
setInterval(() => { if (activeTab === 'sensors') loadSensors(); }, 1500);
setInterval(() => { if (activeTab === 'pwm' && connected) loadPwm(); }, 3000);
setInterval(() => { if (activeTab === 'serial' && connected) loadSerial(); }, 3000);
</script>
</body>
</html>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="127.0.0.1", help="bind address")
    ap.add_argument("--port", type=int, default=8080, help="web server port")
    args = ap.parse_args()

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    url = "http://%s:%d" % (args.host, args.port)
    print("PX4 Web GCS running at %s  (Ctrl-C to stop)" % url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down...")
    finally:
        CONN.disconnect()
        server.server_close()


if __name__ == "__main__":
    main()
