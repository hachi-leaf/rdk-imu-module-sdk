#!/usr/bin/env python3
from rdkimu import IMU
from rdkimu.imu import RDK_IMU_DEVICE

imu = IMU()
try:
    imu.bus(spi_accel_speed=1000000, spi_gyro_speed=1000000)
    imu.config()
    imu.enable()

    print("Continuous fused read (Ctrl+C to stop)...")
    last_ts, freq, frame = 0, 0.0, 0

    while True:
        data = imu.read_fused(fuse_by=RDK_IMU_DEVICE.ACCEL, max_age_ns=50000000)
        ts = data.accel.timestamp_ns

        if last_ts != 0:
            dt = (ts - last_ts) / 1e9
            if dt > 0.0:
                freq = 1.0 / dt
        last_ts = ts

        avail = imu.fifo_available()
        print(f"[{frame:6d}] TS={ts:15d} | "
              f"Accel: {data.accel.x:+8.4f} {data.accel.y:+8.4f} {data.accel.z:+8.4f} | "
              f"Gyro:  {data.gyro.x:+8.4f} {data.gyro.y:+8.4f} {data.gyro.z:+8.4f} | "
              f"FIFO={avail:3d} | Freq={freq:6.1f} Hz")
        frame += 1

except KeyboardInterrupt:
    print("\nInterrupted.")
finally:
    imu.disable()
    print("Stopped.")