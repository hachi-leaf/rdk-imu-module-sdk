/**
 * Copyright (c) 2026 Leaf. D-Robotics.
 * SPDX-License-Identifier: MIT
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <inttypes.h>

#include "rdkimu.h"

static volatile int keep_running = 1;

static void sigint_handler(int sig) {
    (void)sig;
    keep_running = 0;
}

int main(void) {
    signal(SIGINT, sigint_handler);

    rdk_imu_state_t *st = rdk_imu_create_default();
    if(!st){
        fprintf(stderr, "create failed\n");
        return 1;
    }

    /* 总线自动扫描 */
    rdk_imu_bus_info_t bus;
    bus.interface = RDK_IMU_AUTO;
    bus.bus.spi.accel.speed_hz = 1000000;
    bus.bus.spi.gyro.speed_hz = 1000000;
    if(rdk_imu_bus_init(st, bus) != RDK_IMU_OK){
        fprintf(stderr, "bus init failed\n");
        rdk_imu_destroy(st);
        return 1;
    }

    /* 设备配置 */
    rdk_imu_config_t cfg = RDK_IMU_X5_DEFAULT_CONFIG;
    if(rdk_imu_device_init(st, cfg) != RDK_IMU_OK){
        fprintf(stderr, "device init failed\n");
        rdk_imu_bus_deinit(st);
        rdk_imu_destroy(st);
        return 1;
    }

    /* 使能中断采集 */
    if(rdk_imu_enable(st) != RDK_IMU_OK){
        fprintf(stderr, "enable failed\n");
        rdk_imu_device_deinit(st);
        rdk_imu_bus_deinit(st);
        rdk_imu_destroy(st);
        return 1;
    }

    printf("Continuous fused read (Ctrl+C to stop)...\n");

    uint64_t last_ts = 0;
    double freq = 0.0;
    unsigned long long frame = 0;

    while(keep_running){
        rdk_imu_6_axis_data_t data;
        rdk_imu_err_t ret = rdk_imu_read_fused(st, &data, RDK_IMU_ACCEL, 50000000ULL);
        if(ret != RDK_IMU_OK){
            if (ret != RDK_IMU_FIFO_EMPTY)
                fprintf(stderr, "read_fused error: %d\n", ret);
            break;
        }

        uint64_t ts = data.accel.timestamp_ns;

        if(last_ts != 0){
            double dt = (ts - last_ts) / 1e9;
            if(dt > 0.0)freq = 1.0 / dt;
        }
        last_ts = ts;

        uint32_t avail = 0;
        rdk_imu_fifo_available(st, &avail);

        printf("[%6llu] TS=%15" PRIu64 " | "
               "Accel: %+8.4f %+8.4f %+8.4f | "
               "Gyro: %+8.4f %+8.4f %+8.4f | "
               "FIFO=%3u | Freq=%6.1f Hz\n",
               frame, ts,
               data.accel.x, data.accel.y, data.accel.z,
               data.gyro.x, data.gyro.y, data.gyro.z,
               avail, freq);
        frame++;
    }

    /* 释放资源 */
    rdk_imu_disable(st);
    rdk_imu_device_deinit(st);
    rdk_imu_bus_deinit(st);
    rdk_imu_destroy(st);

    printf("Stopped.\n");
    return 0;
}