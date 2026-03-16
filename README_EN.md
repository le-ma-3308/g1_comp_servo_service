# g1_comp_servo_service

[中文说明](./README.md)

`g1_comp_servo_service` controls the compression-joint servos through a serial port and exposes a DDS interface.

This document is organized from `README.pdf` and supplemented with the actual constraints found in the current codebase for direct deployment and execution.

## Installation and Deployment

1. Install `unitree_sdk2` first.
2. Copy the project to `/home/unitree` on the G1 computer at `192.168.123.164`.

```bash
scp -r ~/g1_comp_servo_service unitree@192.168.123.164:/home/unitree
```

Note: the current code hardcodes the config path as `/home/unitree/g1_comp_servo_service/config/config.yaml`, so the project directory should remain at the following path.

```bash
/home/unitree/g1_comp_servo_service
```

## Build

The entire build and runtime process must be carried out on the G1 computer with IP `192.168.123.164`.

```bash
cd /home/unitree/g1_comp_servo_service
mkdir -p build
cd build
cmake ..
make
```

After a successful build, the following executables are typically generated in the `build` directory.

- `test_calibration`: servo calibration program. This must be run first before the other programs can work correctly.
- `test_read_angle`: reads the mapped joint angles and can be used to verify direction and joint limits.
- `test_joint0_control`: tests single-joint control for `joint 0` after calibration.
- `test_joint1_control`: tests single-joint control for `joint 1` after calibration.
- `test_servo_control`: tests continuous reciprocating motion of both joints after calibration.
- `main`: the main program for controlling joints through DDS.
- `test_servo_homePostion`: slowly moves the joints back to the zero position.

## Run Guide

### 1. Calibration `test_calibration`

```bash
cd /home/unitree/g1_comp_servo_service/build
./test_calibration
```

After launch, the program will trigger three countdowns.

1. Before the first countdown ends, move `joint 0` to its limit position.
2. After the second countdown ends, `joint 0` returns to the center, which indicates successful calibration of `joint 0`.
3. During the third countdown, press `joint 1` downward toward the G1 head direction.

Calibration is complete after all three countdowns finish. The program updates the following fields in `config/config.yaml`.

- `servo0_calibration`
- `servo1_calibration`
- `has_calibrate`

### 2. Read Joint Angles `test_read_angle`

```bash
cd /home/unitree/g1_comp_servo_service/build
./test_read_angle
```

The program continuously prints the mapped angles of both joints and can be used to check the following items.

- Whether the calibration result is correct
- Whether the positive joint direction matches expectations
- Whether the current limits are reasonable

### 3. Single-Joint Control `test_joint0_control`

```bash
cd /home/unitree/g1_comp_servo_service/build
./test_joint0_control
```

After launch, the terminal prints the following prompt.

```text
Enter the desired angle:
```

Enter the target angle and press Enter.

Notes:

- You must run `test_calibration` first
- The input angle must stay within the `joint0` limit range in `config/config.yaml`

### 4. Single-Joint Control `test_joint1_control`

```bash
cd /home/unitree/g1_comp_servo_service/build
./test_joint1_control
```

This program runs the same way as `test_joint0_control`, but it controls `joint 1`. The input angle must stay within the `joint1` limit range in `config/config.yaml`.

### 5. Continuous Motion Test `test_servo_control`

```bash
cd /home/unitree/g1_comp_servo_service/build
./test_servo_control
```

After launch, the joints first move to the initial target positions and then start reciprocating.

### 6. Return-to-Zero Test `test_servo_homePostion`

```bash
cd /home/unitree/g1_comp_servo_service/build
./test_servo_homePostion
```

After launch, both joints slowly return to zero and then the program exits.

### 7. DDS Control `main`

`main` is used to publish and subscribe joint commands and states through DDS. `README.pdf` only describes its purpose and does not provide a complete startup command, so you should confirm the DDS network initialization arguments from the source code before using it in the target environment.

## Configuration File

The default configuration file is `config/config.yaml`, and its current contents in this repository are shown below.

```yaml
dt: 0.02
servo0_calibration: 0
servo1_calibration: 0

joint0: [-50, 50]
joint1: [-20, 85]

direction: [1, -1]

has_calibrate: 0
```

Notes:

- `joint0` and `joint1` define the angle limit ranges
- `direction` defines the positive-direction mapping of the angles
- When `has_calibrate` is `0`, the other test programs will directly ask you to run `test_calibration` first

## Notes

- It is recommended to complete calibration before running any control-related program.
- This project is currently intended to be built and run on the G1 computer itself, and it is not recommended to execute it directly on another host.
- If the joint mounting direction or limit configuration changes, run the calibration procedure again.
