import json
import math
import pathlib
import struct
import sys
import unittest

import msgpack


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from zmq_head_servo_bridge import (  # noqa: E402
    DataCollectorTelemetryPublisher,
    HeadServoCommand,
    build_arg_parser,
    build_head_servo_command_payload,
    build_head_servo_state_payload,
    compute_head_servo_command,
    decode_payload_from_frames,
    default_mcap_endpoint,
    extract_head_pelvis_quats,
    extract_motor_state_q_degrees,
    make_motor_cmds,
    resolve_mcap_endpoints,
)


def quat_from_axis_angle(axis, degrees):
    radians = math.radians(degrees)
    half = radians / 2.0
    scale = math.sin(half)
    return [
        math.cos(half),
        axis[0] * scale,
        axis[1] * scale,
        axis[2] * scale,
    ]


class FakeMotorCmds:
    def __init__(self):
        self.cmds = []


class FakeMotorCmd:
    def __init__(self):
        self.mode = 0
        self.q = 0.0
        self.dq = 0.0
        self.tau = 0.0
        self.kp = 0.0
        self.kd = 0.0
        self.reserve = [0, 0, 0]


class FakeMotorState:
    def __init__(self, q):
        self.q = q


class FakeMotorStates:
    def __init__(self, q_values):
        self.states = [FakeMotorState(value) for value in q_values]


class FakeAgain(Exception):
    pass


class FakeSocket:
    def __init__(self, fail=False):
        self.fail = fail
        self.sent = []
        self.closed = False

    def send_multipart(self, parts, flags=0):
        if self.fail:
            raise FakeAgain("would block")
        self.sent.append((parts, flags))

    def close(self, linger=0):
        self.closed = True


class ZmqHeadServoBridgeTest(unittest.TestCase):
    def test_decodes_json_payload_and_extracts_nested_quats(self):
        payload = {
            "head": {"quat": [1.0, 0.0, 0.0, 0.0]},
            "pelvis": {"quat": [0.7071068, 0.0, 0.0, 0.7071068]},
        }

        decoded = decode_payload_from_frames([json.dumps(payload).encode("utf-8")])
        head, pelvis = extract_head_pelvis_quats(decoded)

        self.assertEqual(head, [1.0, 0.0, 0.0, 0.0])
        self.assertAlmostEqual(pelvis[0], 0.7071068, places=6)
        self.assertEqual(pelvis[1:3], [0.0, 0.0])
        self.assertAlmostEqual(pelvis[3], 0.7071068, places=6)

    def test_decodes_msgpack_multipart_with_topic(self):
        payload = {
            "head_quat": [1.0, 0.0, 0.0, 0.0],
            "pelvis_quat": [1.0, 0.0, 0.0, 0.0],
        }
        packed = msgpack.packb(payload, use_bin_type=True)

        decoded = decode_payload_from_frames([b"head_pose", packed], topic="head_pose")
        head, pelvis = extract_head_pelvis_quats(decoded)

        self.assertEqual(head, [1.0, 0.0, 0.0, 0.0])
        self.assertEqual(pelvis, [1.0, 0.0, 0.0, 0.0])

    def test_decodes_22_link_float32_pose_payload(self):
        floats = []
        head_quat = quat_from_axis_angle((0.0, 1.0, 0.0), 30.0)
        for link_index in range(22):
            if link_index == 0:
                floats.extend([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
            elif link_index == 13:
                floats.extend([1.0, 2.0, 3.0, *head_quat])
            else:
                floats.extend([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        raw = struct.pack("<154f", *floats)

        decoded = decode_payload_from_frames([raw])
        head, pelvis = extract_head_pelvis_quats(decoded)
        command = compute_head_servo_command(head, pelvis)

        self.assertEqual(pelvis, [1.0, 0.0, 0.0, 0.0])
        self.assertAlmostEqual(head[0], head_quat[0], places=6)
        self.assertAlmostEqual(head[2], head_quat[2], places=6)
        self.assertAlmostEqual(command.relative_yaw_deg, 30.0, places=5)

    def test_computes_observed_human_axis_mapping_from_relative_quats(self):
        pelvis = [1.0, 0.0, 0.0, 0.0]

        yaw_command = compute_head_servo_command(
            quat_from_axis_angle((0.0, 1.0, 0.0), 30.0),
            pelvis,
        )
        pitch_command = compute_head_servo_command(
            quat_from_axis_angle((1.0, 0.0, 0.0), 20.0),
            pelvis,
        )
        roll_command = compute_head_servo_command(
            quat_from_axis_angle((0.0, 0.0, 1.0), 30.0),
            pelvis,
        )

        self.assertAlmostEqual(yaw_command.relative_pitch_deg, 0.0, places=5)
        self.assertAlmostEqual(yaw_command.relative_yaw_deg, 30.0, places=5)
        self.assertAlmostEqual(yaw_command.pitch_deg, 38.0, places=5)
        self.assertAlmostEqual(yaw_command.yaw_deg, 30.0, places=5)

        self.assertAlmostEqual(pitch_command.relative_pitch_deg, 20.0, places=5)
        self.assertAlmostEqual(pitch_command.relative_yaw_deg, 0.0, places=5)
        self.assertAlmostEqual(pitch_command.pitch_deg, 18.0, places=5)
        self.assertAlmostEqual(pitch_command.yaw_deg, 0.0, places=5)

        self.assertAlmostEqual(roll_command.relative_pitch_deg, 0.0, places=5)
        self.assertAlmostEqual(roll_command.relative_yaw_deg, 0.0, places=5)

    def test_clamps_pitch_and_yaw_to_servo_limits(self):
        yaw_command = compute_head_servo_command(
            head_quat=quat_from_axis_angle((0.0, 1.0, 0.0), 120.0),
            pelvis_quat=[1.0, 0.0, 0.0, 0.0],
        )
        pitch_command = compute_head_servo_command(
            head_quat=quat_from_axis_angle((1.0, 0.0, 0.0), 90.0),
            pelvis_quat=[1.0, 0.0, 0.0, 0.0],
        )

        self.assertEqual(yaw_command.yaw_deg, 50.0)
        self.assertEqual(pitch_command.pitch_deg, -20.0)

    def test_motor_command_order_is_yaw_then_pitch(self):
        msg = make_motor_cmds(
            yaw_deg=12.5,
            pitch_deg=34.0,
            motor_cmds_cls=FakeMotorCmds,
            motor_cmd_cls=FakeMotorCmd,
            mode=1,
        )

        self.assertEqual(len(msg.cmds), 2)
        self.assertEqual(msg.cmds[0].mode, 1)
        self.assertEqual(msg.cmds[0].q, 12.5)
        self.assertEqual(msg.cmds[1].mode, 1)
        self.assertEqual(msg.cmds[1].q, 34.0)

    def test_builds_head_servo_command_payload_for_mcap(self):
        command = HeadServoCommand(
            relative_pitch_deg=-4.0,
            relative_yaw_deg=12.0,
            pitch_deg=42.0,
            yaw_deg=12.0,
            pitch_clamped=False,
            yaw_clamped=True,
        )

        payload = build_head_servo_command_payload(
            sequence=9,
            source_timestamp_ns=123456789,
            command=command,
            head_quat=[1.0, 0.0, 0.0, 0.0],
            pelvis_quat=[0.7071068, 0.0, 0.0, 0.7071068],
            zmq_endpoint="tcp://10.0.0.4:7003",
            zmq_topic="pose",
            dry_run=False,
        )

        self.assertEqual(payload["schema"], "g1_head_servo_command.v1")
        self.assertEqual(payload["sequence"], 9)
        self.assertEqual(payload["source_timestamp_ns"], 123456789)
        self.assertEqual(payload["command"]["motor_order"], ["yaw", "pitch"])
        self.assertEqual(payload["command"]["yaw_deg"], 12.0)
        self.assertEqual(payload["command"]["pitch_deg"], 42.0)
        self.assertEqual(payload["command"]["yaw_clamped"], True)
        self.assertEqual(payload["input"]["head_quat_wxyz"], [1.0, 0.0, 0.0, 0.0])
        self.assertAlmostEqual(payload["input"]["pelvis_quat_wxyz"][3], 0.7071068, places=6)
        self.assertEqual(payload["input"]["zmq_endpoint"], "tcp://10.0.0.4:7003")
        self.assertEqual(payload["input"]["zmq_topic"], "pose")
        self.assertEqual(payload["dry_run"], False)

    def test_builds_head_servo_state_payload_from_motor_states(self):
        msg = FakeMotorStates([12.5, 34.0])

        yaw_deg, pitch_deg = extract_motor_state_q_degrees(msg)
        payload = build_head_servo_state_payload(
            sequence=10,
            source_timestamp_ns=22334455,
            state_msg=msg,
        )

        self.assertEqual(yaw_deg, 12.5)
        self.assertEqual(pitch_deg, 34.0)
        self.assertEqual(payload["schema"], "g1_head_servo_state.v1")
        self.assertEqual(payload["sequence"], 10)
        self.assertEqual(payload["state"]["motor_order"], ["yaw", "pitch"])
        self.assertEqual(payload["state"]["yaw_deg"], 12.5)
        self.assertEqual(payload["state"]["pitch_deg"], 34.0)
        self.assertEqual(payload["state"]["raw_q"], [12.5, 34.0])

    def test_data_collector_telemetry_publisher_sends_multipart_msgpack(self):
        socket = FakeSocket()
        publisher = DataCollectorTelemetryPublisher(
            endpoint="tcp://127.0.0.1:6017",
            source="unit_test_source",
            frame_id="head_servo_command",
            socket=socket,
            again_exception=FakeAgain,
            noblock_flag=99,
        )

        ok = publisher.publish(
            sequence=11,
            source_timestamp_ns=987654321,
            payload={"schema": "unit_test", "value": 3.5},
        )

        self.assertTrue(ok)
        self.assertEqual(len(socket.sent), 1)
        parts, flags = socket.sent[0]
        self.assertEqual(flags, 99)
        header = json.loads(parts[0].decode("utf-8"))
        unpacked = msgpack.unpackb(parts[1], raw=False)
        self.assertEqual(header["sequence"], 11)
        self.assertEqual(header["source_timestamp_ns"], 987654321)
        self.assertEqual(header["source"], "unit_test_source")
        self.assertEqual(header["encoding"], "msgpack")
        self.assertEqual(header["frame_id"], "head_servo_command")
        self.assertEqual(unpacked["value"], 3.5)

    def test_data_collector_telemetry_publisher_counts_nonblocking_drops(self):
        socket = FakeSocket(fail=True)
        publisher = DataCollectorTelemetryPublisher(
            endpoint="tcp://127.0.0.1:6017",
            source="unit_test_source",
            frame_id="head_servo_command",
            socket=socket,
            again_exception=FakeAgain,
        )

        ok = publisher.publish(sequence=12, payload={"schema": "unit_test"})

        self.assertFalse(ok)
        self.assertEqual(publisher.dropped_count, 1)

    def test_mcap_cli_defaults_and_endpoint_resolution(self):
        parser = build_arg_parser()
        defaults = parser.parse_args(["--mocap-ip", "127.0.0.1", "--mocap-port", "7003"])
        custom = parser.parse_args(
            [
                "--mocap-ip",
                "127.0.0.1",
                "--mocap-port",
                "7003",
                "--no-mcap-telemetry",
                "--data-collector-host",
                "10.1.2.3",
                "--mcap-snd-hwm",
                "4",
            ]
        )

        self.assertEqual(defaults.mocap_ip, "127.0.0.1")
        self.assertEqual(defaults.mocap_port, 7003)
        self.assertEqual(defaults.data_collector_host, "127.0.0.1")
        self.assertTrue(defaults.mcap_telemetry)
        self.assertEqual(default_mcap_endpoint("10.1.2.3", 6017), "tcp://10.1.2.3:6017")
        self.assertFalse(custom.mcap_telemetry)
        self.assertEqual(custom.data_collector_host, "10.1.2.3")
        self.assertEqual(custom.mcap_snd_hwm, 4)
        self.assertEqual(resolve_mcap_endpoints(custom), ("tcp://10.1.2.3:6017", "tcp://10.1.2.3:6018"))


if __name__ == "__main__":
    unittest.main()
