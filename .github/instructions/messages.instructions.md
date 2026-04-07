---
applyTo: "msg/**,src/modules/mavlink/**,src/modules/uxrce_dds_client/**"
---

# Messages/Protocol Review Guidelines

In addition to the core code review guidelines:

- Backwards compatibility: will this break QGC, post-flight tools, or uLog parsers?
- uORB: `timestamp` for publication metadata, `timestamp_sample` close to physical sample, include `device_id`
- Don't version messages unless strictly needed
- Parameter UX: will this confuse users in a GCS? Every new param is a configuration burden
- MAVLink: use library constants, don't implement custom stream rates
