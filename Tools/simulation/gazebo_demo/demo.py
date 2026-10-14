#!/usr/bin/env python3
"""Launch the local X500 contrast demo using only Python's standard library."""

import argparse
import copy
import hashlib
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
import zlib

DEMO_DIR = Path(__file__).resolve().parent
REPO_ROOT = DEMO_DIR.parents[2]
BUILD_DIR = REPO_ROOT / "build/px4_sitl_default"
MODEL_DIR = REPO_ROOT / "Tools/simulation/gz/models"
MODEL_NAME = "x500_0"
GRID_PIXELS = 2000
GRID_EXTENT_METRES = 500
GRID_CELL_PIXELS = 8  # Two metre cells over the 500 metre ground plane.
STARTUP_TIMEOUT_SECONDS = 90


def png_chunk(kind, data):
    return (struct.pack("!I", len(data)) + kind + data
            + struct.pack("!I", zlib.crc32(kind + data)))


def make_grid(path):
    pixels = bytearray()
    for y in range(GRID_PIXELS):
        pixels.append(0)  # PNG scanline filter: none.
        for x in range(GRID_PIXELS):
            if x % (GRID_CELL_PIXELS * 5) == 0 or y % (GRID_CELL_PIXELS * 5) == 0:
                color = (65, 86, 108)
            elif x % GRID_CELL_PIXELS == 0 or y % GRID_CELL_PIXELS == 0:
                color = (100, 122, 144)
            else:
                color = (220, 228, 235)
            pixels.extend(color)
    header = struct.pack("!2I5B", GRID_PIXELS, GRID_PIXELS, 8, 2, 0, 0, 0)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + png_chunk(b"IHDR", header)
                     + png_chunk(b"IDAT", zlib.compress(pixels)) + png_chunk(b"IEND", b""))


def prepare_assets(run_dir):
    """Reuse upstream vehicle dynamics, changing only visual materials."""
    texture = run_dir / "light_grid.png"
    make_grid(texture)
    world_doc = ET.parse(REPO_ROOT / "Tools/simulation/gz/worlds/default.sdf")
    world = world_doc.getroot().find("world")
    world.find("scene/background").text = "0.55 0.75 0.95 1"
    world.find("scene/ambient").text = "0.7 0.7 0.7 1"
    drone = ET.parse(MODEL_DIR / "x500_base/model.sdf").getroot().find("model")
    drone.set("name", MODEL_NAME)
    for uri in drone.findall(".//mesh/uri"):
        if uri.text.startswith("model://"):
            uri.text = (MODEL_DIR / uri.text[len("model://"):]).as_uri()
    for visual in drone.findall(".//visual"):
        old = visual.find("material")
        if old is not None:
            visual.remove(old)
        material = ET.SubElement(visual, "material")
        if visual.get("name") == "base_link_visual":
            color = "1 0.8 0.02 1"
        elif "rotor" in visual.get("name", "") or "prop" in visual.get("name", ""):
            color = "1 0.15 0.02 1"
        else:
            color = "0.1 0.65 1 1"
        for field in ("ambient", "diffuse"):
            ET.SubElement(material, field).text = color
        ET.SubElement(material, "emissive").text = "0.12 0.08 0.01 1"
    drone.find("link[@name='base_link']").append(ET.fromstring('''
      <visual name="demo_yellow_panel"><pose>0 0 0.075 0 0 0</pose>
        <geometry><box><size>0.18 0.22 0.04</size></box></geometry>
        <material><ambient>1 0.8 0 1</ambient><diffuse>1 0.8 0 1</diffuse>
          <emissive>0.3 0.2 0 1</emissive></material>
      </visual>'''))
    x500 = ET.parse(MODEL_DIR / "x500/model.sdf").getroot().find("model")
    for plugin in x500.findall("plugin"):
        # This presentation demo does not inject motor failures or require
        # checkout-built custom plugins. Keep the four normal motor plugins.
        if plugin.get("filename") != "MotorFailurePlugin":
            drone.append(copy.deepcopy(plugin))
    world.append(drone)
    ground = ET.SubElement(world, "model", name="demo_light_grid")
    ET.SubElement(ground, "static").text = "true"
    ET.SubElement(ground, "pose").text = "0 0 0.006 0 0 0"
    visual = ET.SubElement(ET.SubElement(ground, "link", name="ground"), "visual", name="grid")
    plane = ET.SubElement(ET.SubElement(visual, "geometry"), "plane")
    ET.SubElement(plane, "normal").text = "0 0 1"
    ET.SubElement(plane, "size").text = f"{GRID_EXTENT_METRES} {GRID_EXTENT_METRES}"
    material = ET.SubElement(visual, "material")
    for field in ("ambient", "diffuse"):
        ET.SubElement(material, field).text = "1 1 1 1"
    metal = ET.SubElement(ET.SubElement(material, "pbr"), "metal")
    ET.SubElement(metal, "albedo_map").text = str(texture)
    ET.SubElement(metal, "metalness").text = "0"
    ET.SubElement(metal, "roughness").text = "1"
    ET.SubElement(visual, "cast_shadows").text = "false"
    world_file = run_dir / "world.sdf"
    world_doc.write(world_file, encoding="utf-8", xml_declaration=True)

    # Reuse PX4's sensor configuration, omitting camera/optical-flow plugins
    # unused by X500 that require optional checkout-built shared libraries.
    server_doc = ET.parse(REPO_ROOT / "src/modules/simulation/gz_bridge/server.config")
    plugins = server_doc.getroot().find("plugins")
    for plugin in list(plugins):
        if not plugin.get("filename", "").startswith("gz-sim-"):
            plugins.remove(plugin)
    server_file = run_dir / "server.config"
    server_doc.write(server_file, encoding="utf-8", xml_declaration=True)
    return world_file, server_file


def start_process(command, log_path, env, processes):
    log = log_path.open("w")
    try:
        process = subprocess.Popen(command, cwd=REPO_ROOT, env=env,
                                   stdin=subprocess.DEVNULL, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
    finally:
        log.close()
    processes.append(process)
    return process


def stop_processes(processes):
    for process in reversed(processes):
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
    deadline = time.monotonic() + 5
    for process in reversed(processes):
        try:
            process.wait(timeout=max(0.01, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()


def wait_for_px4(log_path, processes, require_gcs):
    deadline = time.monotonic() + STARTUP_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        if any(process.poll() is not None for process in processes):
            raise RuntimeError("A simulation process exited during startup; inspect the logs.")
        text = log_path.read_text(errors="replace") if log_path.exists() else ""
        if "Ready for takeoff!" in text:
            return
        if not require_gcs and "Startup script returned successfully" in text:
            return
        time.sleep(0.5)
    raise RuntimeError("PX4 did not become ready within 90 seconds; inspect px4.log.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="store_true", help="Build PX4 SITL before launching")
    parser.add_argument("--headless", action="store_true", help="Run without the Gazebo GUI")
    parser.add_argument("--no-qgc", action="store_true", help="Do not open QGroundControl")
    parser.add_argument("--prepare-only", action="store_true", help="Generate assets without launching")
    args = parser.parse_args()
    if sys.platform != "darwin":
        raise RuntimeError("This launcher currently supports macOS; see README.md.")
    if args.build:
        subprocess.run(["make", "px4_sitl_default"], cwd=REPO_ROOT, check=True)
    run_root = BUILD_DIR / "demo"
    run_root.mkdir(parents=True, exist_ok=True)
    if args.prepare_only:
        run_dir = Path(tempfile.mkdtemp(prefix="prepared-", dir=run_root))
        prepare_assets(run_dir)
        print(f"Prepared demo assets: {run_dir}", flush=True)
        return
    px4 = BUILD_DIR / "bin/px4"
    if not px4.is_file():
        raise RuntimeError("PX4 SITL is not built. Run this launcher with --build.")
    if not shutil.which("gz"):
        raise RuntimeError("Gazebo is missing from PATH. Run Tools/setup/macos.sh --sim-tools.")
    prefix = os.environ.get("CONDA_PREFIX")
    if not prefix:
        raise RuntimeError("The pinned Gazebo environment is not active. Use run_demo.sh.")
    qgc = Path("/Applications/QGroundControl.app")
    if not args.no_qgc and not qgc.is_dir():
        raise RuntimeError("Install QGroundControl in /Applications or use --no-qgc.")
    # Instance zero and its ports must be free. Do not kill other demos.
    for port in (18570, 14580):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            try:
                probe.bind(("0.0.0.0", port))
            except OSError as error:
                raise RuntimeError(f"UDP port {port} is in use. Stop your existing SITL demo first.") from error
    lock = run_root / "running.lock"
    try:
        lock.mkdir()
    except FileExistsError as error:
        raise RuntimeError(f"Demo lock exists: {lock}. See README.md for stale-lock recovery.") from error
    processes = []
    try:
        (lock / "launcher.pid").write_text(str(os.getpid()))
        run_dir = Path(tempfile.mkdtemp(prefix=time.strftime("%Y%m%d-%H%M%S-"), dir=run_root))
        world, server_config = prepare_assets(run_dir)
        env = os.environ.copy()
        repo_hash = hashlib.sha256(str(REPO_ROOT).encode()).hexdigest()[:8]
        env.update(GZ_IP="127.0.0.1", GZ_PARTITION=f"px4-demo-{repo_hash}-{os.getpid()}",
                   GZ_SIM_SERVER_CONFIG_PATH=str(server_config),
                   GZ_SIM_RESOURCE_PATH=f"{MODEL_DIR}:{REPO_ROOT / 'Tools/simulation/gz/worlds'}",
                   GZ_GUI_PLUGIN_PATH=f"{prefix}/lib/gz-gui-8/plugins:{prefix}/lib/gz-sim-8/plugins/gui",
                   PX4_SYS_AUTOSTART="4001", PX4_GZ_STANDALONE="1", PX4_GZ_WORLD="default",
                   PX4_GZ_MODEL_NAME=MODEL_NAME, PX4_SIM_MODEL="gz_x500")
        # Avoid inherited switches that would change this particular demo.
        for variable in ("PX4_SIM_SPEED_FACTOR", "PX4_GZ_MODEL_POSE", "PX4_GZ_SIM_RENDER_ENGINE"):
            env.pop(variable, None)
        print(f"Demo assets and logs: {run_dir}", flush=True)
        start_process(["gz", "sim", "-r", "-s", str(world)], run_dir / "server.log", env, processes)
        start_process([str(px4), "-d"], run_dir / "px4.log", env, processes)
        if not args.no_qgc:
            subprocess.run(["open", "-a", str(qgc)], check=True)
        wait_for_px4(run_dir / "px4.log", processes, not args.no_qgc)
        # Finish the PX4 startup service requests before the GUI requests the
        # scene. Concurrent startup requests can stall Gazebo transport on Mac.
        if not args.headless:
            start_process(["gz", "sim", "-g", "-v", "3", "--gui-config", str(DEMO_DIR / "gui.config")],
                          run_dir / "gui.log", env, processes)
        camera_configured = args.headless
        camera_request = (f'track_mode: FOLLOW, follow_target: {{name: "{MODEL_NAME}"}}, '
                          'follow_offset: {x: -1.5, y: -1.5, z: 1.5}, follow_pgain: 1, track_pgain: 1')
        print("PX4 started. Check Ready to Fly in QGroundControl before taking off. "
              "Press Ctrl+C here to stop the demo.", flush=True)
        while all(process.poll() is None for process in processes):
            # macOS may defer rendering while the GUI is obscured. Configure
            # following when its render thread has initialised, even if the
            # user brings the window forward well after PX4 becomes ready.
            if not camera_configured:
                gui_text = (run_dir / "gui.log").read_text(errors="replace")
                gui_text = re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]", "", gui_text)
                if "Camera pose topic advertised" in gui_text:
                    subprocess.run(["gz", "topic", "-t", "/gui/track", "-m", "gz.msgs.CameraTrack",
                                    "-p", camera_request], env=env, stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL, timeout=10, check=True)
                    camera_configured = True
            time.sleep(0.5)
        raise RuntimeError("A demo process closed; stopping the remaining simulation processes.")
    finally:
        stop_processes(processes)
        (lock / "launcher.pid").unlink(missing_ok=True)
        lock.rmdir()


def interrupted(*_):
    raise KeyboardInterrupt()


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, interrupted)
    try:
        main()
    except KeyboardInterrupt:
        print("Demo stopped.")
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        print(f"Demo failed: {error}", file=sys.stderr)
        sys.exit(1)
