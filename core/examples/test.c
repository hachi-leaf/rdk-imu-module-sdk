#include <stdio.h>
#include <stdint.h>
#include "rdkimu.h"


int main(){
    rdk_imu_err_t ret;

    rdk_imu_state_t *st = rdk_imu_create_default();
    if (!st) {
        printf("create failed\n");
        return -1;
    }

    rdk_imu_bus_info_t bus_info;
    bus_info.interface = RDK_IMU_AUTO;
    bus_info.bus.spi.accel.speed_hz = 1000000;
    bus_info.bus.spi.gyro.speed_hz  = 1000000;

    rdk_imu_config_t config;
    config.accel_drdy_int       = RDK_IMU_INT1;
    config.accel_int_gpio_mode  = RDK_IMU_PP_H;
    config.accel_drdy_gpio_chip = 4;
    config.accel_drdy_gpio_line = 2;

    config.gyro_drdy_int        = RDK_IMU_INT3;
    config.gyro_int_gpio_mode   = RDK_IMU_PP_H;
    config.gyro_drdy_gpio_chip  = 3;
    config.gyro_drdy_gpio_line  = 12;

    config.irq_priority          = -1;
    config.irq_thread_timeout_ns = 1000000000;

    config.accel_bwp   = RDK_IMU_OSR4;
    config.accel_range = RDK_IMU_ACCEL_24G;
    config.accel_odr   = RDK_IMU_ACCEL_100;

    config.gyro_range     = RDK_IMU_GYRO_2000DPS;
    config.gyro_bandwidth = RDK_IMU_ODR400_BW47;

    config.fifo_length = 256;
    config.fifo_mode   = RDK_IMU_FIFO_OVERWRITE;

    ret = rdk_imu_bus_init(st, bus_info);
    printf("bus_init: %d\n", ret);
    if (ret) goto cleanup;

    ret = rdk_imu_device_init(st, config);
    printf("device_init: %d\n", ret);
    if (ret) goto cleanup;

    ret = rdk_imu_enable(st);
    printf("enable: %d\n", ret);
    if (ret) goto cleanup;

    printf("Reading fused data (Accel primary, Gyro interpolated)...\n");

    uint64_t start_ts = 0;
    int frame_count = 0;

    while (1) {
        rdk_imu_6_axis_data_t data;
        ret = rdk_imu_read_fused(st, &data, RDK_IMU_ACCEL, 20000000ULL);
        if (ret != RDK_IMU_OK) {
            printf("read_fused error: %d\n", ret);
            break;
        }

        frame_count++;
        if (frame_count == 1) {
            start_ts = data.accel.timestamp_ns;  // 记录第一帧时间戳
        }

        // 每 100 帧计算并输出频率
        if (frame_count % 100 == 0) {
            uint64_t now_ts = data.accel.timestamp_ns;
            double dt_sec = (now_ts - start_ts) / 1e9;
            double freq = frame_count / dt_sec;
            printf("--- Frames: %d, Freq: %.2f Hz ---\n", frame_count, freq);

            // 重置计数器，从下一帧开始重新统计
            frame_count = 0;
            start_ts = 0;
        }

        // 输出当前帧数据（若不想刷屏可注释掉下面这行）
        printf("A[%7.3f %7.3f %7.3f] G[%7.3f %7.3f %7.3f] ts: %llu\n",
               data.accel.x, data.accel.y, data.accel.z,
               data.gyro.x, data.gyro.y, data.gyro.z,
               (unsigned long long)data.accel.timestamp_ns);
    }

cleanup:
    rdk_imu_disable(st);
    rdk_imu_device_deinit(st);
    rdk_imu_destroy(st);
    return ret;
}