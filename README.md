<div align="center">
  <a href="https://developer.d-robotics.cc">
    <img src="images/d_robotics_logo.png" alt="" width="180"/>
  </a>

[简体中文](README_cn.md) | English

# RDK IMU Module SDK

The RDK IMU module is implemented with Bosch Sensortec’s high-performance 6-axis inertial measurement unit (IMU), the BMI088, which contains a tri-axis gyroscope and a tri-axis accelerometer, both with 16-bit resolution. The BMI088 is designed for applications that require high accuracy and vibration robustness, making it especially suitable for drones and robots operating in high-vibration environments. The BMI088 supports extended ranges of ±24 g for acceleration and ±2000°/s for angular rate. It also demonstrates excellent temperature drift characteristics (low TCO/TCS) and comes factory-calibrated for stable attitude and motion sensing.

<img src="images/rdk_imu_product.png" alt="" width="400"/>

rdk-imu-module-sdk is a software development kit created by D‑Robotics for the RDK IMU module. It includes C, Python and ROS 2 components. No kernel driver is required, hardware timestamping is supported, and the build process is simple. Developers can use this SDK with their preferred language, quickly explore the RDK IMU module’s features, and build their own robotics applications on top of it.

</div>

---

# 🚀 Quick Start

## 1. Hardware Connection

Using the RDK X5 as an example, the connection is shown below:

<div align="center">

<img src="images/install.png" alt="" width="750"/>

</div>

---

## 2. Prerequisites

Install Python build tools:

```shell
pip install setuptools wheel
```

Install colcon build tools:

```shell
sudo apt update
sudo apt install python3-colcon-common-extensions
```

Install stress-ng:

```shell
sudo apt update
sudo apt install stress-ng
```

---

## 3. Example Usage

### C example

In the `./core` directory run `make` to build everything.

Then run `sudo out/test` to start the test program; it prints IMU frame IDs, 6-axis data, timestamps, software FIFO remaining capacity, and real-time frequency statistics on interrupt.

Or run `make test` to quickly compile and run the example.

Use `make install` to install the header and shared library into the system environment.

---

### Python example

In the `./core` directory run `make install` to build and install the generated `.whl` and shared library into Python’s path.

Then run:

```shell
python3 examples/test_imu.py
```

to run the Python example.

---

### ROS 2 example

First source your ROS 2 environment:

```shell
source /opt/xxx/xxx/setup.bash
```

In the `./ros2` directory run `colcon build` to build the ROS 2 package.
> Make sure `core/lib/librdkimu.so` and related files exist before building.

After the build completes, run:

```shell
source install/setup.bash
```

to source the built package.

Launch the IMU data node with:

```shell
ros2 launch rdk_imu_module rdk_imu.launch.py
```

Open another terminal, source the ROS 2 environment there as well, and verify IMU topics:

```shell
source /opt/xxx/xxx/setup.bash

ros2 topic echo /rdkimu/data
ros2 topic hz /rdkimu/data
```

---

# 📚 Documentation

For detailed SDK documentation, please refer to the D‑Robotics developer community:

https://developer.d-robotics.cc

The documentation covers SDK principles, build steps, API reference, and usage notes.

---

# 💪 Performance

The RDK IMU Module SDK lets you tune IMU parameters; available options follow the BMI088 datasheet. When adjusting ODR (output data rate), please consider the limitations of the communication interface used.

## For I2C

The approximate sum ODR limit (accel + gyro) for I2C can be estimated as:

$$
	ext{ODR}_{\text{sum}} \approx \frac{f_{\text{I2C}}}{2 \times N_{\text{read}} \times 9}
$$

where:
- f_I2C is the configured I2C bus speed in Hz (standard mode is 100 kHz);
- N_read is the number of bytes transferred on the bus per batch read, here **6 bytes** (three axes × 16 bits).

Example: for RDK X5 default I2C speed f_I2C = 100 kHz, the theoretical maximum sum ODR is:

$$
	ext{ODR}_{\text{sum}} \approx \frac{100000}{2 \times 6 \times 9} \approx 1042\text{Hz}
$$

Therefore, the sum of accelerometer and gyroscope ODRs should not exceed about 1042 Hz. In real applications, also consider other devices on the bus and interrupt handling overhead; it's recommended to leave some margin.

### For SPI

The approximate sum ODR limit (accel + gyro) for SPI can be estimated as:

$$
	ext{ODR}_{\text{sum}} \approx \frac{f_{\text{SPI}}}{(B_{\text{accel}} + B_{\text{gyro}}) \times 8}
$$

where:
- f_SPI is the configured SPI clock frequency in Hz (for example, 1 MHz);
- B_accel and B_gyro are the bytes transferred per SPI read for accel and gyro respectively (here 7 and 8).

Example: for RDK X5 default SPI speed f_SPI = 1 MHz, the theoretical maximum sum ODR is:

$$
	ext{ODR}_{\text{sum}} \approx \frac{1000000}{(7 + 8) \times 8} = \frac{1000000}{112} \approx 8333\text{Hz}
$$

## Hardware Timestamping

The SDK’s timestamps come from user‑space hardware interrupt timestamps via `gpiod`. The figure below shows the IMU timestamp interval under combined CPU load and memory bandwidth stress tests.

<div align="center">
    <img src="images/timestamp_test.png" alt=""/>
</div>

---

# 📜 License

This project is open-sourced under the MIT License. See the [LICENSE](LICENSE) file.

# 📩 Contact

If you have questions or suggestions, please contact us via:

Developer community: https://developer.d-robotics.cc

---

# 🏷️ Release Notes

| Version | Date | Main changes |
|--------:|------|--------------|
| `v1.0.0` | 2026-06-10 | Initial release: support for 6-axis fused output and hardware timestamping |

---

<div align="center">

**Copyright (c) 2026 D‑Robotics.**

</div>