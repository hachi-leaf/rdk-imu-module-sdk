/*
 * Copyright (c) 2026 D-Robotics
 * SPDX-License-Identifier: MIT
 */
#ifndef __RDK_IMU_MODULE_H
#define __RDK_IMU_MODULE_H

#include <stdint.h>
#include <pthread.h>

typedef enum {
    RDK_IMU_OK = 0,
    RDK_IMU_ERR_PARAM,
} rdk_imu_err_t;

typedef enum rdk_imu_interface_e{
    RDK_IMU_AUTO,
    RDK_IMU_I2C,
    RDK_IMU_SPI,
}rdk_imu_interface_t;

typedef struct rdk_imu_i2c_info_s{
    uint8_t bus;
    uint8_t addr;
}rdk_imu_i2c_info_t;

typedef struct rdk_imu_spi_info_s{
    uint8_t bus;
    uint8_t cs;
    uint32_t speed_hz;
}rdk_imu_spi_info_t;

typedef struct rdk_imu_bus_s{
    rdk_imu_interface_t interface;
    union{
        struct{
            rdk_imu_i2c_info_t accel;
            rdk_imu_i2c_info_t gyro;
        }i2c;
        struct{
            rdk_imu_spi_info_t accel;
            rdk_imu_spi_info_t gyro;
        }spi;
    }bus;
    int32_t valid;
}rdk_imu_bus_t;

typedef enum rdk_imu_accel_drdy_int_e{
    RDK_IMU_INT1,
    RDK_IMU_INT2,
}rdk_imu_accel_drdy_int_t;

typedef enum rdk_imu_gyro_drdy_int_e{
    RDK_IMU_INT3,
    RDK_IMU_INT4,
}rdk_imu_gyro_drdy_int_t;

typedef enum rdk_imu_accel_bwp_e{
    RDK_IMU_OSR4,
    RDK_IMU_OSR2,
    RDK_IMU_NORMAL,
}rdk_imu_accel_bwp_t;

typedef enum rdk_imu_accel_odr_e{
    RDK_IMU_ACCEL_12_5,
    RDK_IMU_ACCEL_25,
    RDK_IMU_ACCEL_50,
    RDK_IMU_ACCEL_100,
    RDK_IMU_ACCEL_200,
    RDK_IMU_ACCEL_400,
    RDK_IMU_ACCEL_800,
    RDK_IMU_ACCEL_1600,
}rdk_imu_accel_odr_t;

typedef enum rdk_imu_accel_range_e{
    RDK_IMU_ACCEL_3G,
    RDK_IMU_ACCEL_6G,
    RDK_IMU_ACCEL_12G,
    RDK_IMU_ACCEL_24G,
}rdk_imu_accel_range_t;

typedef enum rdk_imu_gyro_range_e{
    RDK_IMU_GYRO_2000DPS,
    RDK_IMU_GYRO_1000DPS,
    RDK_IMU_GYRO_500DPS,
    RDK_IMU_GYRO_250DPS,
    RDK_IMU_GYRO_125DPS,
}rdk_imu_gyro_range_t;

typedef enum rdk_imu_gyro_bandwidth_e{
    RDK_IMU_ODR2000_BW532,
    RDK_IMU_ODR2000_BW230,
    RDK_IMU_ODR1000_BW116,
    RDK_IMU_ODR400_BW47,
    RDK_IMU_ODR200_BW23,
    RDK_IMU_ODR100_BW12,
    RDK_IMU_ODR200_BW64,
    RDK_IMU_ODR100_BW32,
}rdk_imu_gyro_bandwidth_t;

typedef struct rdk_imu_config_s{
    rdk_imu_accel_drdy_int_t accel_drdy_int;
    uint32_t accel_drdy_gpiochip;
    uint32_t accel_drdy_gpiochip_offset;

    rdk_imu_gyro_drdy_int_t gyro_drdy_int;
    uint32_t gyro_drdy_gpiochip;
    uint32_t gyro_drdy_gpiochip_offset;

    rdk_imu_accel_bwp_t accel_bwp;
    rdk_imu_accel_odr_t accel_odr;
    rdk_imu_accel_range_t accel_range;

    rdk_imu_gyro_range_t gyro_range;
    rdk_imu_gyro_bandwidth_t gyro_bandwidth;

    uint32_t fifo_length;

    int32_t valid;
}rdk_imu_config_t;

typedef struct rdk_imu_3_axis_data_s{
    float x, y, z;
    uint64_t timestamp_ns;
    int32_t valid;
}rdk_imu_3_axis_data_t;

typedef struct rdk_imu_6_axis_data_s{
    rdk_imu_3_axis_data_t accel, gyro;
}rdk_imu_6_axis_data_t;

typedef struct rdk_imu_fifo_s{
    rdk_imu_6_axis_data_t *buffer;  
    unsigned int size; 
    unsigned int mask; 
    unsigned int head; 
    unsigned int tail; 
}rdk_imu_fifo_t;

typedef struct rdk_imu_state_s{
    rdk_imu_bus_t bus;
    rdk_imu_config_t config;

    uint32_t enable;
    pthread_mutex_t mutex;
    rdk_imu_fifo_t imu_fifo;
}rdk_imu_state_t;

rdk_imu_state_t rdk_imu_get_default_state();

rdk_imu_err_t rdk_imu_bus_init(
    rdk_imu_state_t* st,
    rdk_imu_bus_t *bus
);

rdk_imu_err_t rdk_imu_device_init(
    rdk_imu_state_t* st,
    rdk_imu_config_t *config
);

rdk_imu_err_t rdk_imu_enable(
    rdk_imu_state_t* st
);

rdk_imu_err_t rdk_imu_disable(
    rdk_imu_state_t* st
);

rdk_imu_err_t rdk_imu_device_deinit(
    rdk_imu_state_t* st
);

rdk_imu_err_t rdk_imu_bus_deinit(
    rdk_imu_state_t* st
);

rdk_imu_err_t rdk_imu_fifo_available(
    rdk_imu_state_t *st,
    uin32_t count
);

rdk_imu_err_t rdk_imu_read_indep(
    rdk_imu_state_t *st, 
    rdk_imu_6_axis_data_t *data,
    uin32_t count
);

typedef enum rdk_imu_device_e{
    RDK_IMU_ACCEL,
    RDK_IMU_GYRO,
}rdk_imu_device_t;

rdk_imu_err_t rdk_imu_read_fused(
    rdk_imu_state_t *st, 
    rdk_imu_6_axis_data_t *data,
    rdk_imu_device_t fuse_by,
    uint64_t max_age_ns
);

#endif
