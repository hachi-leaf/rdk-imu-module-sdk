/*
 * Copyright (c) 2026 Leaf. D-Robotics.
 * SPDX-License-Identifier: MIT
 */
#ifndef __RDK_IMU_MODULE_H
#define __RDK_IMU_MODULE_H

#include <stdint.h>
#include <pthread.h>

/* ============================== BMI088 Register and Feature  ============================== */
#define BMI088_ACCEL_REG_CHIP_REG 0x00
#define BMI088_ACCEL_CHIP_ID 0x1E
#define BMI088_GYRO_REG_CHIP_REG 0x00
#define BMI088_GYRO_CHIP_ID 0x0F

#define BMI088_ACCEL_REG_SOFTRESET 0x7E
#define BMI088_ACCEL_SOFTRESET_CMD 0xB6
#define BMI088_ACCEL_RESET_SLEEP_MS 1
#define BMI088_GYRO_REG_SOFTRESET 0x14
#define BMI088_GYRO_SOFTRESET_CMD 0xB6
#define BMI088_GYRO_RESET_SLEEP_MS 30

#define BMI088_SPI_ACCEL_RD_DUMMY 1
#define BMI088_SPI_ACCEL_WR_DUMMY 0
#define BMI088_SPI_GYRO_RD_DUMMY 0
#define BMI088_SPI_GYRO_WR_DUMMY 0

#define RDK_IMU_SPI_SCAN_SPEED_HZ 1000000
#define RDK_IMU_SPID_MODE SPI_MODE_3

/* ============================== Type Define ============================== */
/* ---------- error code ---------- */
typedef enum {
    RDK_IMU_OK = 0,
    RDK_IMU_ERR_PARAM,
    RDK_IMU_I2C_WRITE_ERR,
    RDK_IMU_I2C_READ_ERR,
    RDK_IMU_SPI_WRITE_ERR,
    RDK_IMU_SPI_READ_ERR,
    RDK_IMU_OPEN_DIR_ERR,
    RDK_IMU_OPEN_FILE_ERR,
    RDK_IMU_NO_DEV_ERR,
    RDK_IMU_CHIP_ID_ERR,
    RDK_IMU_CAN_NOT_RELEASE,
}rdk_imu_err_t;

/* ---------- about bus info ---------- */
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

typedef struct rdk_imu_bus_info_s{
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
}rdk_imu_bus_info_t;

/* ---------- bus handle ---------- */
typedef struct rdk_imu_bus_s rdk_imu_bus_t; // 前向声明

typedef struct rdk_imu_bus_ops_s{
    rdk_imu_err_t (*read)(
        rdk_imu_bus_t *bus, 
        uint8_t reg, 
        uint8_t *data, 
        uint8_t len);

    rdk_imu_err_t (*write)(
        rdk_imu_bus_t *bus, 
        uint8_t reg, 
        const uint8_t *data, 
        uint8_t len);

    rdk_imu_err_t (*update)(
        rdk_imu_bus_t *bus, 
        uint8_t reg, 
        uint8_t data, 
        uint8_t mask);
}rdk_imu_bus_ops_t;

struct rdk_imu_bus_s {
    int fd;
    uint8_t addr; // I2C address, SPI does not use this
    uint32_t speed_hz; // SPI speed, 0 for default, I2C does not use this
    uint8_t rd_dummy; // SPI only
    uint8_t wr_dummy; // SPI only
    const rdk_imu_bus_ops_t *ops;
};

/* ---------- about interrupt ---------- */
typedef enum rdk_imu_accel_drdy_int_e{
    RDK_IMU_INT1,
    RDK_IMU_INT2,
}rdk_imu_accel_drdy_int_t;

typedef enum rdk_imu_gyro_drdy_int_e{
    RDK_IMU_INT3,
    RDK_IMU_INT4,
}rdk_imu_gyro_drdy_int_t;

/* ---------- IMU sensor parameters ---------- */
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

typedef enum rdk_imu_fifo_mode_e{
    RDK_IMU_CIRCULAR,
    RDK_IMU_OVERWRITE,
}rdk_imu_fifo_mode_t;

typedef struct rdk_imu_config_s{
    /* accel 和 gyro 的 chip 端和 SoC 端中断脚选择 */
    rdk_imu_accel_drdy_int_t accel_drdy_int;
    uint32_t accel_drdy_gpiochip;
    uint32_t accel_drdy_gpiochip_offset;

    rdk_imu_gyro_drdy_int_t gyro_drdy_int;
    uint32_t gyro_drdy_gpiochip;
    uint32_t gyro_drdy_gpiochip_offset;

    /* accel 和 gyro 性能设置 */
    rdk_imu_accel_bwp_t accel_bwp;
    rdk_imu_accel_odr_t accel_odr;
    rdk_imu_accel_range_t accel_range;

    rdk_imu_gyro_range_t gyro_range;
    rdk_imu_gyro_bandwidth_t gyro_bandwidth;

    /* 缓冲区长度，必须是 2 的倍数 */
    uint32_t fifo_length;
    /* fifo 模式 */
    rdk_imu_fifo_mode_t fifo_mode;
}rdk_imu_config_t;

/* ---------- data package ---------- */
typedef struct rdk_imu_3_axis_data_s{
    float x, y, z;
    uint64_t timestamp_ns;
    int32_t valid;
}rdk_imu_3_axis_data_t;

typedef struct rdk_imu_6_axis_data_s{
    rdk_imu_3_axis_data_t accel, gyro;
}rdk_imu_6_axis_data_t;

/* ---------- private ---------- */
typedef struct rdk_imu_fifo_s{
    rdk_imu_6_axis_data_t *buffer;  
    unsigned int size; 
    unsigned int mask; 
    unsigned int head; 
    unsigned int tail; 
}rdk_imu_fifo_t;

typedef struct rdk_imu_state_s{
    rdk_imu_bus_t accel_bus;
    rdk_imu_bus_t gyro_bus;

    uint32_t enable;
    pthread_mutex_t mutex;
    rdk_imu_fifo_t imu_fifo;
}rdk_imu_state_t;

typedef enum rdk_imu_device_e{
    RDK_IMU_ACCEL,
    RDK_IMU_GYRO,
}rdk_imu_device_t;

/* ============================== API ============================== */
rdk_imu_state_t *rdk_imu_create_default(
    void);

rdk_imu_err_t rdk_imu_destroy(
    rdk_imu_state_t *st);

rdk_imu_err_t rdk_imu_bus_init(
    rdk_imu_state_t* st,
    rdk_imu_bus_info_t bus_info);

rdk_imu_err_t rdk_imu_bus_deinit(
    rdk_imu_state_t* st);

rdk_imu_err_t rdk_imu_device_init(
    rdk_imu_state_t* st,
    rdk_imu_config_t config);

rdk_imu_err_t rdk_imu_device_deinit(
    rdk_imu_state_t* st);

rdk_imu_err_t rdk_imu_enable(
    rdk_imu_state_t* st);

rdk_imu_err_t rdk_imu_disable(
    rdk_imu_state_t* st);

rdk_imu_err_t rdk_imu_fifo_available(
    rdk_imu_state_t *st,
    uint32_t count);

rdk_imu_err_t rdk_imu_read_indep(
    rdk_imu_state_t *st, 
    rdk_imu_6_axis_data_t *data,
    uint32_t count);

rdk_imu_err_t rdk_imu_read_fused(
    rdk_imu_state_t *st, 
    rdk_imu_6_axis_data_t *data,
    rdk_imu_device_t fuse_by,
    uint64_t max_age_ns);


#endif
