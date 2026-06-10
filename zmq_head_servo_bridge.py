#!/usr/bin/env python3
"""Bridge ZMQ head/pelvis quaternions to the G1 head servo ROS2 command topic."""

from __future__ import annotations

import argparse
import json
import math
import struct
import time
from dataclasses import dataclass
from typing import Any, Callable, Iterable, Sequence

import msgpack
import zmq


HEADER_SIZE = 1280
LINK_POSE_FLOAT_COUNT = 154
LINK_POSE_BYTE_SIZE = LINK_POSE_FLOAT_COUNT * 4
LINK_POSE_STRIDE = 7
LINK_NAMES = (
    "Hips",
    "LeftUpLeg",
    "LeftLeg",
    "LeftFoot",
    "LeftToe",
    "RightUpLeg",
    "RightLeg",
    "RightFoot",
    "RightToe",
    "Spine1",
    "Spine2",
    "Chest",
    "Neck",
    "Head",
    "LeftShoulder",
    "LeftArm",
    "LeftForeArm",
    "LeftHand",
    "RightShoulder",
    "RightArm",
    "RightForeArm",
    "RightHand",
)
HEAD_KEYS = (
    "head",
    "head_quat",
    "head_quaternion",
    "head_orientation",
    "head_rot",
    "head_wxyz",
    "head_quat_wxyz",
)
PELVIS_KEYS = (
    "pelvis",
    "pelvis_quat",
    "pelvis_quaternion",
    "pelvis_orientation",
    "pelvis_rot",
    "pelvis_wxyz",
    "pelvis_quat_wxyz",
    "hip",
    "hip_quat",
    "hip_rot",
    "root_quat",
)
HEAD_SERVO_MOTOR_ORDER = ("yaw", "pitch")
DEFAULT_MCAP_COMMAND_PORT = 6017
DEFAULT_MCAP_STATE_PORT = 6018


@dataclass(frozen=True)
class HeadServoCommand:
    relative_pitch_deg: float
    relative_yaw_deg: float
    pitch_deg: float
    yaw_deg: float
    pitch_clamped: bool
    yaw_clamped: bool


def _to_float_quat(quat: Any) -> list[float]:
    if hasattr(quat, "tolist"):
        quat = quat.tolist()
    if not isinstance(quat, (list, tuple)) or len(quat) != 4:
        raise ValueError(f"Quaternion must be length 4 in wxyz order, got {quat!r}")
    values = [float(v) for v in quat]
    norm = math.sqrt(sum(v * v for v in values))
    if norm == 0.0:
        raise ValueError("Zero-length quaternion provided")
    return [v / norm for v in values]


def _quat_conjugate(quat: Sequence[float]) -> list[float]:
    w, x, y, z = quat
    return [w, -x, -y, -z]


def _quat_mul(a: Sequence[float], b: Sequence[float]) -> list[float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]


def _remap_observed_human_axes_to_servo_axes(quat: Sequence[float]) -> list[float]:
    """Map observed sender axes into the yaw/pitch axes used by the servo bridge."""
    w, x, y, z = quat
    return [w, z, x, y]


def relative_head_angles(head_quat: Sequence[float], pelvis_quat: Sequence[float]) -> tuple[float, float]:
    """Return relative pitch/yaw degrees for head relative to pelvis.

    Inputs and internal math follow server_collect.py: q_rel = inverse(pelvis) * head,
    where both quaternions are scalar-first wxyz. The live mocap stream is observed
    as human pitch on current X, human yaw on current Y, and human roll on current Z,
    so the relative quaternion is remapped before extracting servo pitch/yaw.
    """
    q_head = _to_float_quat(head_quat)
    q_pelvis = _to_float_quat(pelvis_quat)
    w, x, y, z = _remap_observed_human_axes_to_servo_axes(
        _quat_mul(_quat_conjugate(q_pelvis), q_head)
    )

    sinp = 2.0 * (w * y - z * x)
    if sinp >= 1.0:
        pitch = math.pi / 2.0
    elif sinp <= -1.0:
        pitch = -math.pi / 2.0
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(pitch), math.degrees(yaw)


def _clamp(value: float, lower: float, upper: float) -> tuple[float, bool]:
    clamped = min(max(value, lower), upper)
    return clamped, clamped != value


def compute_head_servo_command(
    head_quat: Sequence[float],
    pelvis_quat: Sequence[float],
    *,
    pitch_offset_deg: float = 38.0,
    yaw_limits: tuple[float, float] = (-50.0, 50.0),
    pitch_limits: tuple[float, float] = (-20.0, 85.0),
) -> HeadServoCommand:
    relative_pitch, relative_yaw = relative_head_angles(head_quat, pelvis_quat)
    pitch, pitch_clamped = _clamp(pitch_offset_deg - relative_pitch, *pitch_limits)
    yaw, yaw_clamped = _clamp(relative_yaw, *yaw_limits)
    return HeadServoCommand(
        relative_pitch_deg=relative_pitch,
        relative_yaw_deg=relative_yaw,
        pitch_deg=pitch,
        yaw_deg=yaw,
        pitch_clamped=pitch_clamped,
        yaw_clamped=yaw_clamped,
    )


def default_mcap_endpoint(host: str, port: int) -> str:
    return f"tcp://{host}:{int(port)}"


def resolve_mcap_endpoints(args: argparse.Namespace) -> tuple[str, str]:
    command_endpoint = args.mcap_command_endpoint or default_mcap_endpoint(
        args.data_collector_host,
        DEFAULT_MCAP_COMMAND_PORT,
    )
    state_endpoint = args.mcap_state_endpoint or default_mcap_endpoint(
        args.data_collector_host,
        DEFAULT_MCAP_STATE_PORT,
    )
    return command_endpoint, state_endpoint


def build_head_servo_command_payload(
    *,
    sequence: int,
    source_timestamp_ns: int,
    command: HeadServoCommand,
    head_quat: Sequence[float],
    pelvis_quat: Sequence[float],
    zmq_endpoint: str,
    zmq_topic: str,
    dry_run: bool,
) -> dict[str, Any]:
    return {
        "schema": "g1_head_servo_command.v1",
        "sequence": int(sequence),
        "source_timestamp_ns": int(source_timestamp_ns),
        "dry_run": bool(dry_run),
        "command": {
            "motor_order": list(HEAD_SERVO_MOTOR_ORDER),
            "yaw_deg": float(command.yaw_deg),
            "pitch_deg": float(command.pitch_deg),
            "relative_yaw_deg": float(command.relative_yaw_deg),
            "relative_pitch_deg": float(command.relative_pitch_deg),
            "yaw_clamped": bool(command.yaw_clamped),
            "pitch_clamped": bool(command.pitch_clamped),
            "mode": 1,
        },
        "input": {
            "head_quat_wxyz": _to_float_quat(head_quat),
            "pelvis_quat_wxyz": _to_float_quat(pelvis_quat),
            "zmq_endpoint": str(zmq_endpoint),
            "zmq_topic": str(zmq_topic),
        },
    }


def _message_field(value: Any, field_name: str) -> Any:
    field = getattr(value, field_name)
    if callable(field):
        return field()
    return field


def extract_motor_state_q_degrees(state_msg: Any) -> tuple[float, float]:
    states = _message_field(state_msg, "states")
    if len(states) < 2:
        raise ValueError(f"Expected at least two head servo motor states, got {len(states)}")
    yaw_deg = float(_message_field(states[0], "q"))
    pitch_deg = float(_message_field(states[1], "q"))
    return yaw_deg, pitch_deg


def build_head_servo_state_payload(
    *,
    sequence: int,
    source_timestamp_ns: int,
    state_msg: Any,
) -> dict[str, Any]:
    yaw_deg, pitch_deg = extract_motor_state_q_degrees(state_msg)
    return {
        "schema": "g1_head_servo_state.v1",
        "sequence": int(sequence),
        "source_timestamp_ns": int(source_timestamp_ns),
        "state": {
            "motor_order": list(HEAD_SERVO_MOTOR_ORDER),
            "yaw_deg": yaw_deg,
            "pitch_deg": pitch_deg,
            "raw_q": [yaw_deg, pitch_deg],
        },
    }


class DataCollectorTelemetryPublisher:
    def __init__(
        self,
        *,
        endpoint: str,
        source: str,
        frame_id: str,
        socket: Any | None = None,
        packer: Callable[[dict[str, Any]], bytes] | None = None,
        again_exception: type[BaseException] | tuple[type[BaseException], ...] | None = None,
        noblock_flag: int | None = None,
        snd_hwm: int = 10,
    ) -> None:
        self.endpoint = endpoint
        self.source = source
        self.frame_id = frame_id
        self.dropped_count = 0
        self._packer = packer or (lambda payload: msgpack.packb(payload, use_bin_type=True))

        if socket is None:
            self._context = zmq.Context.instance()
            self._socket = self._context.socket(zmq.PUSH)
            self._socket.setsockopt(zmq.LINGER, 0)
            self._socket.setsockopt(zmq.SNDHWM, int(snd_hwm))
            if hasattr(zmq, "IMMEDIATE"):
                self._socket.setsockopt(zmq.IMMEDIATE, 1)
            self._socket.connect(endpoint)
            self._again_exception = zmq.Again if again_exception is None else again_exception
            self._noblock_flag = zmq.NOBLOCK if noblock_flag is None else noblock_flag
        else:
            self._context = None
            self._socket = socket
            self._again_exception = RuntimeError if again_exception is None else again_exception
            self._noblock_flag = 0 if noblock_flag is None else noblock_flag

    def publish(
        self,
        *,
        sequence: int,
        payload: dict[str, Any],
        source_timestamp_ns: int | None = None,
    ) -> bool:
        if source_timestamp_ns is None:
            source_timestamp_ns = time.time_ns()
        header = {
            "sequence": int(sequence),
            "source_timestamp_ns": int(source_timestamp_ns),
            "source": self.source,
            "encoding": "msgpack",
            "frame_id": self.frame_id,
        }
        parts = [
            json.dumps(header, separators=(",", ":")).encode("utf-8"),
            self._packer(payload),
        ]
        try:
            self._socket.send_multipart(parts, flags=self._noblock_flag)
        except self._again_exception:
            self.dropped_count += 1
            return False
        except Exception:
            self.dropped_count += 1
            return False
        return True

    def close(self) -> None:
        close = getattr(self._socket, "close", None)
        if callable(close):
            close(linger=0)


class HeadServoMcapTelemetry:
    def __init__(
        self,
        *,
        command_endpoint: str,
        state_endpoint: str,
        source: str,
        snd_hwm: int,
    ) -> None:
        self.command_publisher = DataCollectorTelemetryPublisher(
            endpoint=command_endpoint,
            source=source,
            frame_id="g1_head_servo_command",
            snd_hwm=snd_hwm,
        )
        self.state_publisher = DataCollectorTelemetryPublisher(
            endpoint=state_endpoint,
            source=source,
            frame_id="g1_head_servo_state",
            snd_hwm=snd_hwm,
        )
        self.command_sequence = 0
        self.state_sequence = 0
        self.dropped_command_payloads = 0
        self.dropped_state_payloads = 0

    def publish_command(
        self,
        *,
        command: HeadServoCommand,
        head_quat: Sequence[float],
        pelvis_quat: Sequence[float],
        zmq_endpoint: str,
        zmq_topic: str,
        dry_run: bool,
        source_timestamp_ns: int | None = None,
    ) -> bool:
        self.command_sequence += 1
        if source_timestamp_ns is None:
            source_timestamp_ns = time.time_ns()
        try:
            payload = build_head_servo_command_payload(
                sequence=self.command_sequence,
                source_timestamp_ns=source_timestamp_ns,
                command=command,
                head_quat=head_quat,
                pelvis_quat=pelvis_quat,
                zmq_endpoint=zmq_endpoint,
                zmq_topic=zmq_topic,
                dry_run=dry_run,
            )
        except Exception:
            self.dropped_command_payloads += 1
            return False
        return self.command_publisher.publish(
            sequence=self.command_sequence,
            payload=payload,
            source_timestamp_ns=source_timestamp_ns,
        )

    def publish_state_msg(self, state_msg: Any) -> bool:
        self.state_sequence += 1
        source_timestamp_ns = time.time_ns()
        try:
            payload = build_head_servo_state_payload(
                sequence=self.state_sequence,
                source_timestamp_ns=source_timestamp_ns,
                state_msg=state_msg,
            )
        except Exception:
            self.dropped_state_payloads += 1
            return False
        return self.state_publisher.publish(
            sequence=self.state_sequence,
            payload=payload,
            source_timestamp_ns=source_timestamp_ns,
        )

    def close(self) -> None:
        self.command_publisher.close()
        self.state_publisher.close()


def _normalize_payload(value: Any) -> Any:
    if isinstance(value, dict):
        normalized = {}
        for key, item in value.items():
            if isinstance(key, bytes):
                key = key.decode("utf-8")
            normalized[str(key)] = _normalize_payload(item)
        return normalized
    if isinstance(value, tuple):
        return [_normalize_payload(item) for item in value]
    if isinstance(value, list):
        return [_normalize_payload(item) for item in value]
    if hasattr(value, "tolist"):
        return _normalize_payload(value.tolist())
    return value


def _coerce_quat(value: Any) -> list[float] | None:
    value = _normalize_payload(value)
    if isinstance(value, dict):
        for key in ("quat", "quaternion", "orientation", "rot", "wxyz"):
            if key in value:
                quat = _coerce_quat(value[key])
                if quat is not None:
                    return quat
        if all(key in value for key in ("w", "x", "y", "z")):
            return _to_float_quat([value["w"], value["x"], value["y"], value["z"]])
        return None

    if isinstance(value, list):
        if len(value) == 4:
            try:
                return _to_float_quat(value)
            except (TypeError, ValueError):
                return None
        if len(value) == 2:
            second = _coerce_quat(value[1])
            if second is not None:
                return second
            return _coerce_quat(value[0])
    return None


def _find_quat(payload: dict[str, Any], keys: Iterable[str]) -> list[float] | None:
    for key in keys:
        if key in payload:
            quat = _coerce_quat(payload[key])
            if quat is not None:
                return quat
    return None


def extract_head_pelvis_quats(payload: Any) -> tuple[list[float], list[float]]:
    payload = _normalize_payload(payload)
    if not isinstance(payload, dict):
        raise ValueError(f"Expected decoded payload to be a mapping, got {type(payload).__name__}")

    head = _find_quat(payload, HEAD_KEYS)
    pelvis = _find_quat(payload, PELVIS_KEYS)
    if head is None:
        raise ValueError(f"Could not find head quaternion. Tried keys: {', '.join(HEAD_KEYS)}")
    if pelvis is None:
        raise ValueError(f"Could not find pelvis quaternion. Tried keys: {', '.join(PELVIS_KEYS)}")
    return head, pelvis


def _strip_topic_prefix(raw: bytes, topic: str | None) -> bytes:
    if not topic:
        return raw
    topic_bytes = topic.encode("utf-8")
    if not raw.startswith(topic_bytes):
        return raw
    stripped = raw[len(topic_bytes) :]
    if stripped.startswith((b" ", b"\t", b":")):
        stripped = stripped[1:]
    return stripped


def _decode_json(raw: bytes) -> Any:
    return json.loads(raw.decode("utf-8").strip())


def _decode_msgpack(raw: bytes) -> Any:
    return msgpack.unpackb(raw, raw=False)


def _decode_link_pose_float32_payload(raw: bytes) -> dict[str, Any]:
    if len(raw) != LINK_POSE_BYTE_SIZE:
        raise ValueError(f"Link pose float32 payload must be {LINK_POSE_BYTE_SIZE} bytes, got {len(raw)}")

    values = struct.unpack(f"<{LINK_POSE_FLOAT_COUNT}f", raw)
    links = {}
    for link_index, name in enumerate(LINK_NAMES):
        offset = link_index * LINK_POSE_STRIDE
        x, y, z, qw, qx, qy, qz = values[offset : offset + LINK_POSE_STRIDE]
        links[name] = {
            "position": [x, y, z],
            "quat": [qw, qx, qy, qz],
        }

    return {
        "format": "link_pose_float32",
        "links": links,
        "pelvis": links["Hips"],
        "head": links["Head"],
    }


def _decode_packed_topic_payload(raw: bytes) -> dict[str, Any]:
    if len(raw) < HEADER_SIZE:
        raise ValueError("Packed topic payload is smaller than the fixed header")

    header_bytes = raw[:HEADER_SIZE]
    null_idx = header_bytes.find(b"\x00")
    if null_idx >= 0:
        header_bytes = header_bytes[:null_idx]
    header = json.loads(header_bytes.decode("utf-8"))
    fields = header.get("fields")
    if not isinstance(fields, list):
        raise ValueError("Packed topic header does not contain fields")

    import numpy as np

    dtype_map = {
        "f32": np.float32,
        "f64": np.float64,
        "i32": np.int32,
        "i64": np.int64,
        "u8": np.uint8,
        "bool": bool,
    }
    decoded: dict[str, Any] = {
        "version": header.get("v", 0),
        "endian": header.get("endian", "le"),
    }
    offset = HEADER_SIZE
    for field in fields:
        dtype = dtype_map.get(field.get("dtype", "f32"), np.float32)
        shape = tuple(field.get("shape", [1]))
        n_bytes = int(np.prod(shape)) * np.dtype(dtype).itemsize
        decoded[field["name"]] = (
            np.frombuffer(raw[offset : offset + n_bytes], dtype=dtype).reshape(shape).copy().tolist()
        )
        offset += n_bytes
    return decoded


def decode_payload_from_frames(frames: Sequence[bytes], topic: str | None = None) -> Any:
    if not frames:
        raise ValueError("No ZMQ frames received")
    raw = frames[-1]
    if not isinstance(raw, bytes):
        raise ValueError(f"Expected bytes ZMQ frame, got {type(raw).__name__}")

    candidates = []
    stripped = _strip_topic_prefix(raw, topic)
    candidates.append(stripped)
    if stripped != raw:
        candidates.append(raw)

    errors = []
    for candidate in candidates:
        for decoder in (_decode_json, _decode_msgpack, _decode_packed_topic_payload, _decode_link_pose_float32_payload):
            try:
                return decoder(candidate)
            except Exception as exc:  # noqa: BLE001 - keep decoder fallbacks local.
                errors.append(f"{decoder.__name__}: {exc}")
    raise ValueError(
        "Could not decode ZMQ payload as JSON, msgpack, packed topic payload, or 22-link float32 payload: "
        + "; ".join(errors)
    )


def make_motor_cmds(
    yaw_deg: float,
    pitch_deg: float,
    *,
    motor_cmds_cls: Any,
    motor_cmd_cls: Any,
    mode: int = 1,
) -> Any:
    msg = motor_cmds_cls()

    yaw_cmd = motor_cmd_cls()
    yaw_cmd.mode = int(mode)
    yaw_cmd.q = float(yaw_deg)
    yaw_cmd.dq = 0.0
    yaw_cmd.tau = 0.0
    yaw_cmd.kp = 0.0
    yaw_cmd.kd = 0.0
    yaw_cmd.reserve = [0, 0, 0]

    pitch_cmd = motor_cmd_cls()
    pitch_cmd.mode = int(mode)
    pitch_cmd.q = float(pitch_deg)
    pitch_cmd.dq = 0.0
    pitch_cmd.tau = 0.0
    pitch_cmd.kp = 0.0
    pitch_cmd.kd = 0.0
    pitch_cmd.reserve = [0, 0, 0]

    msg.cmds.append(yaw_cmd)
    msg.cmds.append(pitch_cmd)
    return msg


class HeadServoRosPublisher:
    def __init__(
        self,
        topic: str,
        *,
        state_topic: str | None = None,
        state_callback: Callable[[Any], None] | None = None,
    ):
        import rclpy
        from unitree_go.msg import MotorCmd, MotorCmds

        self.rclpy = rclpy
        self.MotorCmd = MotorCmd
        self.MotorCmds = MotorCmds
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node("zmq_head_servo_bridge")
        self.publisher = self.node.create_publisher(MotorCmds, topic, 1)
        self.state_subscription = None
        if state_topic and state_callback is not None:
            from unitree_go.msg import MotorStates

            self.state_subscription = self.node.create_subscription(MotorStates, state_topic, state_callback, 1)

    def publish(self, command: HeadServoCommand) -> None:
        msg = make_motor_cmds(
            yaw_deg=command.yaw_deg,
            pitch_deg=command.pitch_deg,
            motor_cmds_cls=self.MotorCmds,
            motor_cmd_cls=self.MotorCmd,
            mode=1,
        )
        self.publisher.publish(msg)
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def release(self) -> None:
        msg = make_motor_cmds(
            yaw_deg=0.0,
            pitch_deg=0.0,
            motor_cmds_cls=self.MotorCmds,
            motor_cmd_cls=self.MotorCmd,
            mode=0,
        )
        self.publisher.publish(msg)
        self.rclpy.spin_once(self.node, timeout_sec=0.05)
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def _format_command(command: HeadServoCommand) -> str:
    flags = []
    if command.yaw_clamped:
        flags.append("yaw_clamped")
    if command.pitch_clamped:
        flags.append("pitch_clamped")
    suffix = f" [{' '.join(flags)}]" if flags else ""
    return (
        f"rel_pitch={command.relative_pitch_deg:7.2f} rel_yaw={command.relative_yaw_deg:7.2f} "
        f"cmd_pitch={command.pitch_deg:7.2f} cmd_yaw={command.yaw_deg:7.2f}{suffix}"
    )


def run_bridge(args: argparse.Namespace) -> None:
    endpoint = f"tcp://{args.mocap_ip}:{args.mocap_port}"
    context = zmq.Context.instance()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.RCVHWM, 1)
    socket.setsockopt(zmq.SUBSCRIBE, args.topic.encode("utf-8") if args.topic else b"")
    socket.connect(endpoint)

    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)
    telemetry = None
    if args.mcap_telemetry:
        command_endpoint, state_endpoint = resolve_mcap_endpoints(args)
        telemetry = HeadServoMcapTelemetry(
            command_endpoint=command_endpoint,
            state_endpoint=state_endpoint,
            source=args.mcap_source,
            snd_hwm=args.mcap_snd_hwm,
        )
    publisher = (
        None
        if args.dry_run
        else HeadServoRosPublisher(
            args.ros_topic,
            state_topic=args.ros_state_topic if telemetry is not None else None,
            state_callback=telemetry.publish_state_msg if telemetry is not None else None,
        )
    )
    last_pub_time = 0.0
    last_log_time = 0.0
    period = 1.0 / args.max_hz if args.max_hz > 0.0 else 0.0

    print(f"[bridge] connected ZMQ SUB to {endpoint} topic={args.topic!r}")
    print(f"[bridge] ROS2 topic={args.ros_topic!r} dry_run={args.dry_run}")
    if telemetry is not None:
        print(
            "[bridge] MCAP telemetry enabled "
            f"command={telemetry.command_publisher.endpoint} state={telemetry.state_publisher.endpoint}"
        )

    try:
        while True:
            events = dict(poller.poll(args.recv_timeout_ms))
            now = time.monotonic()
            if socket not in events:
                if publisher is not None:
                    publisher.spin_once(timeout_sec=0.0)
                if now - last_log_time >= args.log_interval:
                    print("[bridge] waiting for ZMQ head/pelvis quaternion payload...")
                    last_log_time = now
                continue

            frames = socket.recv_multipart()
            payload = decode_payload_from_frames(frames, topic=args.topic)
            head_quat, pelvis_quat = extract_head_pelvis_quats(payload)
            command = compute_head_servo_command(
                head_quat=head_quat,
                pelvis_quat=pelvis_quat,
                pitch_offset_deg=args.pitch_offset,
                yaw_limits=(args.yaw_min, args.yaw_max),
                pitch_limits=(args.pitch_min, args.pitch_max),
            )

            if period > 0.0 and now - last_pub_time < period:
                continue
            last_pub_time = now

            if publisher is not None:
                publisher.publish(command)
            if telemetry is not None:
                telemetry.publish_command(
                    command=command,
                    head_quat=head_quat,
                    pelvis_quat=pelvis_quat,
                    zmq_endpoint=endpoint,
                    zmq_topic=args.topic,
                    dry_run=args.dry_run,
                )
            if args.dry_run or now - last_log_time >= args.log_interval:
                print("[bridge] " + _format_command(command))
                last_log_time = now
    finally:
        poller.unregister(socket)
        socket.close(0)
        if publisher is not None:
            publisher.release()
        if telemetry is not None:
            telemetry.close()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mocap-ip", required=True, help="Mocap ZMQ PUB host/IP to connect to")
    parser.add_argument("--mocap-port", required=True, type=int, help="Mocap ZMQ PUB port to connect to")
    parser.add_argument("--topic", default="", help="Optional ZMQ SUB topic prefix")
    parser.add_argument("--ros-topic", default="g1_comp_servo/cmd", help="ROS2 MotorCmds topic")
    parser.add_argument("--ros-state-topic", default="g1_comp_servo/state", help="ROS2 MotorStates topic")
    parser.add_argument("--pitch-offset", type=float, default=38.0, help="Servo pitch command offset in degrees")
    parser.add_argument("--yaw-min", type=float, default=-50.0)
    parser.add_argument("--yaw-max", type=float, default=50.0)
    parser.add_argument("--pitch-min", type=float, default=-20.0)
    parser.add_argument("--pitch-max", type=float, default=85.0)
    parser.add_argument("--max-hz", type=float, default=50.0, help="Maximum ROS2 publish frequency; <=0 disables")
    parser.add_argument("--recv-timeout-ms", type=int, default=1000)
    parser.add_argument("--log-interval", type=float, default=1.0)
    parser.add_argument("--dry-run", action="store_true", help="Print computed commands without publishing ROS2")
    parser.add_argument(
        "--mcap-telemetry",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Publish command/state telemetry to DataCollector",
    )
    parser.add_argument(
        "--data-collector-host",
        default="127.0.0.1",
        help="DataCollector host used for default head servo telemetry endpoints",
    )
    parser.add_argument("--mcap-command-endpoint", default=None, help="Explicit DataCollector command endpoint")
    parser.add_argument("--mcap-state-endpoint", default=None, help="Explicit DataCollector state endpoint")
    parser.add_argument("--mcap-source", default="g1_head_servo_bridge", help="DataCollector telemetry source name")
    parser.add_argument("--mcap-snd-hwm", type=int, default=10, help="DataCollector telemetry ZMQ send high-water mark")
    return parser


def main() -> None:
    run_bridge(build_arg_parser().parse_args())


if __name__ == "__main__":
    main()
