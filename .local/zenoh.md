# Zenoh variant

## Install zenohd

```bash
curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg
echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null

sudo apt update
sudo apt install zenoh
```

Run `zenohd` in the background before starting SITL — PX4 defaults to Zenoh
**client** mode (`tcp/127.0.0.1:7447` on SITL/POSIX, `CONFIG_ZENOH_DEFAULT_LOCATOR`)
and logs "make sure zenohd is running" if it can't connect. (Peer mode with
UDP multicast, no router, is also possible via the `zenoh net peer <locator>`
PX4 shell command, but everything below assumes client mode.)

## Firmware side

Custom board `boards/px4/sitl/navput.px4board` (union of `sih.px4board` +
`zenoh.px4board`):
```
CONFIG_MODULES_SIMULATION_GZ_BRIDGE=n
CONFIG_MODULES_SIMULATION_GZ_MSGS=n
CONFIG_MODULES_SIMULATION_GZ_PLUGINS=n
CONFIG_MODULES_UXRCE_DDS_CLIENT=n
CONFIG_MODULES_ZENOH=y
CONFIG_ZENOH_PUB_OPTION_OVERRIDE=y
```

Custom airframe `ROMFS/px4fmu_common/init.d-posix/airframes/10046_navput_quadx`
(copy of `10040_sihsim_quadx` + our own defaults, registered in that dir's
`CMakeLists.txt`):
```sh
param set-default ZENOH_ENABLE 1
param set-default EKF2_RNGBC_CTRL 1
```
`ZENOH_ENABLE` defaults to 0 (`reboot_required: true` in `zenoh_params.yaml`)
and isn't a Kconfig option, so it has to be set as a param default, not baked
into the board config.

One build-system fix was needed: `src/lib/cdrstream/CMakeLists.txt` only
built `dds_cdrstream.c` for the `cdr` target, but that file calls
`ddsrt_bswap2u/4u/8u`, defined in `cyclonedds/src/ddsrt/src/bswap.c`, which
was never added as a source — undefined-reference link error. Fixed by
adding that file to `target_sources(cdr ...)`.

## Build & run

The `sihsim_<model>` convenience ninja targets (`make px4_sitl_sih sihsim_quadx`)
are hardcoded per-model in `src/modules/simulation/simulator_sih/CMakeLists.txt`
and only exist for filenames matching `*_sihsim_*` with the model in that
file's `models` list — `10046_navput_quadx` doesn't match, so there's no
`navput_quadx` ninja target. Build and run separately instead
(`.local/sitl_quad.bash`):
```bash
make px4_sitl_navput
cd build/px4_sitl_navput/rootfs
PX4_SIM_MODEL=navput_quadx PX4_SIMULATOR=sihsim ../bin/px4
```
(`rootfs` matches `SITL_WORKING_DIR` in `platforms/posix/CMakeLists.txt`, so
`dataman`/`parameters.bson` land where they always have.)

If a topic you need isn't in `src/modules/zenoh/dds_topics.yaml`, add it
there under `publications:`/`subscriptions:` and rebuild — same mechanism as
the uxrce-dds path (see `.local/dds.md`), just a different yaml file. Watch
for shared struct types (e.g. all `estimator_aid_src_*` topics map to
`EstimatorAidSource1d`/`2d`/`3d`, not a per-topic type — check the `.msg`
file's `# TOPICS` comment).

## Python: subscribe

`pip install eclipse-zenoh` — that's the only dependency. PX4's key
expressions are built to match `rmw_zenoh_cpp`'s scheme:
`<domain_id><topic>/px4_msgs::msg::dds_::<TypeCamelCase>_/RIHS01_<32-byte type hash>`
(`ZENOH::generate_rmw_zenoh_topic_keyexpr` in `zenoh.cpp`) — computing the
hash by hand isn't worth it, use a wildcard instead. `.local/test/zenoh_sub.py`:
```python
import zenoh
from msgs.SensorCombined import SensorCombined

conf = zenoh.Config()
conf.insert_json5("mode", '"client"')
conf.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')

session = zenoh.open(conf)

routers = session.info.routers_zid()  # NOTE: property, not a method call
print("connected routers:", routers)
if not routers:
    print("WARNING: not connected to any zenohd router")

domain_id = 0
topic = "/fmu/out/sensor_combined"
keyexpr = f"{domain_id}{topic}/**"  # ** skips the px4_msgs/RIHS01 hash suffix

def listener(sample):
    print(sample.key_expr, "->", SensorCombined.deserialize(bytes(sample.payload)))

sub = session.declare_subscriber(keyexpr, listener)

import time
while True:
    time.sleep(1)
```

## Python: decoding (no ROS2, no idlc, no cyclonedds)

`sample.payload` is raw CDR: a 4-byte encapsulation header (`00 01 00 00` =
CDR_LE) followed by the fields in `.msg` order, each aligned to its own
size (natural C-struct-style alignment, confirmed by decoding a real
captured `sensor_combined` sample and checking every field against sane
physical values — `gyro_integral_dt`/`accelerometer_integral_dt` = 4000,
matching the SITL "4000 us sim time interval" log line, `accelerometer_m_s2`
≈ (0, 0, -9.81) for a stationary quad, etc.)

We looked at generating typed decoders via `idlc -l py` (the pip `cyclonedds`
package's IDL compiler) and hit a real dead end: the pip wheel's bundled
`idlc` segfaults standalone (bad ELF, confirmed with gdb — crashes inside
the dynamic linker before `main()`), and building `cyclonedds` from source
against the system's `cyclonedds-dev` (0.8.2) fails on a missing private
header (`dds/ddsi/ddsi_radmin.h`) that the distro package doesn't ship. Not
worth chasing further — see the alternative below, which needs nothing but
the Python stdlib.

PX4 already generates a `<Msg>.json` for every message as part of a normal
build (`msg/CMakeLists.txt`, using the vendored `rosidl_adapter` — no ROS2
install needed), at:
```
build/px4_sitl_navput/uORB/idl/px4_msgs/msg/<Msg>.json
```
It's the ROS 2 `type_description_interfaces` format: field names in order,
a numeric `type_id` (see `rosidl_runtime_c/type_description/field_type__struct.h`
in the vendored `src/lib/cdrstream/rosidl`), and an array `capacity` (0 for
scalars). That's a complete, machine-readable schema — enough to generate a
decoder without touching idlc/cyclonedds/ROS2 at all.

`.local/test/gen_msg.py` reads `<Msg>.json` and writes
`.local/test/msgs/<Msg>.py`: one precompiled `struct.Struct` covering the
whole message (CDR alignment padding inserted as explicit `x` bytes in the
format string, not relying on native struct alignment matching by luck) and
a `deserialize()` classmethod that's a single `unpack_from` call. Codegen
runs once per message, ahead of time — no JSON parsing or per-field looping
at decode time.

```bash
./gen_msg.py <build_dir> SensorCombined EstimatorAidSource1d
./gen_msg.py <build_dir> --all   # every message in one shot
```

`<build_dir>` is a PX4 build directory, e.g. `../../build/px4_sitl_navput`.
