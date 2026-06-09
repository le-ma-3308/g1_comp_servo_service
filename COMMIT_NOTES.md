# Commit Notes

This file is a long-lived pre-commit log. Before each git commit, add or update
the entry for the commit being prepared. Keep entries in reverse chronological
order, and write all entries in English.

## Entry Template

```markdown
## YYYY-MM-DD - Short Commit Title

Suggested commit message:
`type: short imperative summary`

Purpose:
- Why this change is being made.
- What workflow, bug, or user need it supports.

Changed files:
- `path/to/file`: what changed and why.

Runtime or data-flow notes:
- Any important command, topic, schema, port, or behavior change.

Verification:
- Exact commands that were run.
- Result of each command.

Known limitations:
- Missing dependencies, skipped tests, follow-up work, or intentional scope boundaries.
```

## 2026-06-09 - ZMQ head servo bridge and MCAP telemetry

Suggested commit message:
`feat: add ZMQ head servo bridge telemetry`

Purpose:
- Add a Python bridge that consumes mocap head/pelvis orientation over ZMQ and
  publishes G1 head servo yaw/pitch commands through the existing ROS2
  `g1_comp_servo/cmd` interface.
- Capture the same command stream and actual servo state stream into
  DataCollector without blocking the live servo-control path.
- Make launch arguments explicit: mocap input is configured with
  `--mocap-ip` / `--mocap-port`, and DataCollector output is configured with
  `--data-collector-host`.

Changed files:
- `zmq_head_servo_bridge.py`: added the standalone bridge script. It decodes
  mocap payloads from JSON, msgpack, packed fixed-header topic payloads, or
  22-link float32 pose payloads; extracts head and pelvis quaternions; computes
  relative head pitch/yaw; maps them to servo pitch/yaw commands with the
  existing `38 deg` pitch offset and configured clamp limits; publishes
  `unitree_go.msg.MotorCmds` to `g1_comp_servo/cmd`; subscribes to
  `g1_comp_servo/state` when MCAP telemetry is enabled; and emits command/state
  telemetry with non-blocking ZMQ `PUSH` sockets.
- `test/test_zmq_head_servo_bridge.py`: added unit coverage for payload
  decoding, quaternion extraction, servo command computation and clamping,
  motor command ordering, DataCollector multipart/msgpack publishing, state
  payload construction, non-blocking drop accounting, and CLI endpoint
  resolution.
- `COMMIT_NOTES.md`: recorded this repository-side commit entry.

Runtime or data-flow notes:
- Mocap input is a ZMQ `SUB` connection to
  `tcp://<mocap-ip>:<mocap-port>`, with an optional `--topic` prefix.
- Servo command output remains the ROS2 topic `g1_comp_servo/cmd`.
- Servo state input for telemetry is the ROS2 topic `g1_comp_servo/state`.
- Motor order is fixed as `yaw,pitch`; `MotorCmds.cmds[0].q` is yaw degrees
  and `MotorCmds.cmds[1].q` is pitch degrees.
- MCAP/DataCollector telemetry is enabled by default. Use
  `--no-mcap-telemetry` only when DataCollector output should be intentionally
  disabled for local diagnostics.
- Default DataCollector endpoints are derived from
  `--data-collector-host`, which defaults to `127.0.0.1`:
  `tcp://<data-collector-host>:6017` for command telemetry and
  `tcp://<data-collector-host>:6018` for state telemetry.
- Full endpoint overrides are available with `--mcap-command-endpoint` and
  `--mcap-state-endpoint`.
- Command telemetry payloads use schema `g1_head_servo_command.v1` and include
  the bridge-computed yaw/pitch command, relative head/pelvis angles, clamp
  flags, input head/pelvis quaternions, mocap endpoint/topic, sequence, and
  timestamp.
- State telemetry payloads use schema `g1_head_servo_state.v1` and include
  actual yaw/pitch state, raw state `q`, sequence, and timestamp.
- Telemetry uses the DataCollector multipart contract
  `[header_json, msgpack_payload]`; sends are best-effort and non-blocking.

Verification:
- `PYTHONDONTWRITEBYTECODE=1 /home/delta/miniconda3/bin/python -B -m unittest discover -s /home/delta/code/g1_comp_servo_service/test -p 'test_zmq_head_servo_bridge.py' -v`
  passed with 11 tests.
- `git diff --check`
  passed with no whitespace errors.

Known limitations:
- This entry records the `g1_comp_servo_service` repository-side bridge
  implementation. The paired DataCollector repository has its own
  `COMMIT_NOTES.md` entry for receiver-side stream routing on ports `6017` and
  `6018`.
- Hardware head servo motion and live MCAP recording were not run while
  preparing this commit note.
- DataCollector must be running with a config that binds
  `/g1/head_servo/command` on `6017` and `/g1/head_servo/state` on `6018` for
  MCAP recording to receive these streams.
