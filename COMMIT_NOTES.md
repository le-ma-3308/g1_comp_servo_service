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

## 2026-06-10 - Observed mocap axis remap for head servo bridge

Suggested commit message:
`fix: align ZMQ head servo axis mapping with mocap observations`

Purpose:
- Correct the ZMQ head servo bridge to match live observed mocap behavior:
  human yaw was entering the old pitch path, human roll was entering the old
  yaw path, and human pitch was landing on the ignored roll axis.
- Keep the existing payload contract intact: input quaternions are still
  scalar-first `wxyz`, and the relative head orientation is still computed as
  `inverse(pelvis) * head`.
- Apply the correction only at the relative-quaternion-to-servo-angle boundary
  so ZMQ decoding, ROS2 topic names, motor command ordering, pitch offset,
  clamp limits, and telemetry schemas remain unchanged.
- Add a small local ignore file so editor settings and Python bytecode caches do
  not appear as accidental working-tree noise.

Changed files:
- `zmq_head_servo_bridge.py`: added
  `_remap_observed_human_axes_to_servo_axes()` and applied it immediately after
  computing the relative quaternion. The remap is `[w, x, y, z] -> [w, z, x, y]`,
  which makes current input `X` drive servo pitch, current input `Y` drive servo
  yaw, and current input `Z` become roll that is ignored by the two-axis servo
  command.
- `test/test_zmq_head_servo_bridge.py`: updated quaternion tests to encode the
  observed axis mapping. The tests now verify that current `Y` rotation produces
  yaw, current `X` rotation produces pitch, current `Z` rotation does not drive
  yaw/pitch, and clamp behavior still applies on the remapped yaw/pitch axes.
- `.gitignore`: added local ignores for `.vscode/` and `__pycache__/`.
- `COMMIT_NOTES.md`: added this commit preparation entry.

Runtime or data-flow notes:
- Mocap input remains a ZMQ `SUB` connection to
  `tcp://<mocap-ip>:<mocap-port>` with optional `--topic`.
- Supported input payload formats and quaternion keys are unchanged.
- Servo command output remains `g1_comp_servo/cmd`, with
  `MotorCmds.cmds[0].q` as yaw degrees and `MotorCmds.cmds[1].q` as pitch
  degrees.
- The pitch command formula remains `pitch_offset_deg - relative_pitch`, so
  positive remapped input pitch lowers the commanded pitch angle from the
  default `38 deg` offset.
- MCAP/DataCollector command and state telemetry behavior is unchanged.

Review notes:
- No blocking issues found in the current diff.
- The main residual risk is calibration/runtime-specific sign convention: this
  change fixes the observed axis permutation but intentionally does not flip
  positive/negative signs. If live testing shows only one direction reversed,
  that should be handled as a narrow sign correction on the affected axis.
- This is a global bridge behavior change for all accepted payload formats. It
  matches the current live sender observation, but an older sender that already
  matched the previous `Y=pitch, Z=yaw` convention would need a separate toggle
  or compatibility path before using this bridge.

Verification:
- `git diff --check`
  passed with no whitespace errors.
- `python3 -m unittest discover -s test`
  passed with 11 tests.

Known limitations:
- Hardware head servo motion was not run as part of this review.
- The change does not add a runtime CLI switch for old versus observed axis
  mapping; the observed mapping is now the bridge default.

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
