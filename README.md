# g1_comp_servo_service

[English Version](./README_EN.md)

`g1_comp_servo_service` 用于通过串口控制压缩关节舵机，并对外提供 DDS 接口。

本文档依据 `README.pdf` 整理，并补充了当前代码中的实际约束，方便直接部署和运行。

## 安装与部署

1. 先安装 `unitree_sdk2`。
2. 将工程拷贝到 G1 电脑 `192.168.123.164` 的 `/home/unitree` 目录下。

```bash
scp -r ~/g1_comp_servo_service unitree@192.168.123.164:/home/unitree
```

注意：当前代码将配置文件路径写死为 `/home/unitree/g1_comp_servo_service/config/config.yaml`，因此工程目录应保持为以下路径。

```bash
/home/unitree/g1_comp_servo_service
```

## 编译

整个编译和程序运行过程都需要在 IP 为 `192.168.123.164` 的 G1 电脑上完成。

```bash
cd /home/unitree/g1_comp_servo_service
mkdir -p build
cd build
cmake ..
make
```

编译成功后，`build` 目录中通常会生成以下可执行文件。

- `test_calibration`：舵机标定程序。必须先执行，后续其他程序才可正常使用。
- `test_read_angle`：读取映射后的关节角度，可用于检查正方向和关节限位。
- `test_joint0_control`：标定完成后，用于测试 `joint 0` 的单关节控制。
- `test_joint1_control`：标定完成后，用于测试 `joint 1` 的单关节控制。
- `test_servo_control`：标定完成后，用于测试两个关节的连续往复运动。
- `main`：通过 DDS 控制关节的主程序。
- `test_servo_homePostion`：将关节缓慢回到零位。

## 运行说明

### 1. 标定 `test_calibration`

```bash
cd /home/unitree/g1_comp_servo_service/build
./test_calibration
```

运行后会出现三次倒计时。

1. 第一次倒计时结束前，将 `joint 0` 即沿`yaw`轴推到极限位置。
2. 第二次倒计时结束后，`joint 0` 会自动回到中心位置，这表示 `joint 0` 标定成功。
3. 第三次倒计时期间，将 `joint 1` 向下压到 G1 头部方向。

三次倒计时全部完成后，标定结束。程序会更新 `config/config.yaml` 中的以下字段。

- `servo0_calibration`
- `servo1_calibration`
- `has_calibrate`

### 2. 读取关节角度 `test_read_angle`

```bash
cd /home/unitree/g1_comp_servo_service/build
./test_read_angle
```

程序会持续输出两个关节的映射角度，可用于检查以下内容。

- 标定结果是否正确
- 关节正方向是否符合预期
- 当前限位是否合理

### 3. 单关节控制 `test_joint0_control`

```bash
cd /home/unitree/g1_comp_servo_service/build
./test_joint0_control
```

运行后终端会提示以下内容。

```text
Enter the desired angle:
```

输入目标角度并回车即可。

注意：

- 必须先运行 `test_calibration`
- 输入角度必须在 `config/config.yaml` 的 `joint0` 限位范围内

### 4. 单关节控制 `test_joint1_control`

```bash
cd /home/unitree/g1_comp_servo_service/build
./test_joint1_control
```

运行方式与 `test_joint0_control` 相同，但控制对象为 `joint 1`。输入角度必须位于 `config/config.yaml` 的 `joint1` 限位范围内。

### 5. 连续运动测试 `test_servo_control`

```bash
cd /home/unitree/g1_comp_servo_service/build
./test_servo_control
```

运行后，关节会先移动到初始目标位，然后开始往复运动。

### 6. 回零测试 `test_servo_homePostion`

```bash
cd /home/unitree/g1_comp_servo_service/build
./test_servo_homePostion
```

运行后，两个关节会缓慢回到零位后退出。

### 7. DDS 控制 `main`

`main` 用于将舵机的角度值读取和角度控制，通过ROS2 topic发布出去，topic name为 `g1_comp_servo/state` 和 `g1_comp_servo/cmd`

使用方法(如果onboard网卡为eth0)：
```bash
cd /home/unitree/g1_comp_servo_service/build
sudo ./main eth0
```
可以将其配置成机器人开机自启g1_comp_servo.service

## 配置文件

默认配置文件为 `config/config.yaml`，当前仓库内容如下。

```yaml
dt: 0.02
servo0_calibration: 0
servo1_calibration: 0

joint0: [-50, 50]
joint1: [-20, 85]

direction: [1, -1]

has_calibrate: 0
```

说明：

- `joint0`、`joint1` 定义角度限位范围
- `direction` 定义角度正方向映射
- `has_calibrate` 为 `0` 时，其余测试程序会直接提示先执行 `test_calibration`

## 注意事项

- 所有控制类程序在运行前都建议先完成一次标定。
- 该工程当前面向 G1 电脑本机环境编译和运行，不建议在其他主机上直接执行。
- 如果修改了关节机械安装方向或限位配置，应重新执行标定流程。
