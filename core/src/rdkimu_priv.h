/**
 * Copyright (c) 2026 Leaf. D-Robotics.
 * SPDX-License-Identifier: MIT
 */
#ifndef __RDK_IMU_PRIV_H
#define __RDK_IMU_PRIV_H

#include <pthread.h>
#include <gpiod.h>

#include "rdkimu.h"

/* ============================== Chip-specific Registers ============================== */
#if RDK_IMU_CHIP == RDK_IMU_BMI088

/* Chip ID */
#define IMU_ACCEL_REG_CHIP_ID    0x00
#define IMU_ACCEL_CHIP_ID        0x1E
#define IMU_GYRO_REG_CHIP_ID     0x00
#define IMU_GYRO_CHIP_ID         0x0F

/* Soft Reset */
#define IMU_ACCEL_REG_SOFTRESET  0x7E
#define IMU_ACCEL_SOFTRESET_CMD  0xB6
#define IMU_ACCEL_RESET_SLEEP_MS 1
#define IMU_GYRO_REG_SOFTRESET   0x14
#define IMU_GYRO_SOFTRESET_CMD   0xB6
#define IMU_GYRO_RESET_SLEEP_MS  30

/* Accel Power/Enable */
#define IMU_ACCEL_REG_ENABLE      0x7D
#define IMU_ACCEL_ENABLE_VALUE    0x04
#define IMU_ACCEL_REG_PWR_CONF    0x7C
#define IMU_ACCEL_ACTIVATE_MODE   0x00
#define IMU_ACCEL_ENABLE_SLEEP_MS 1

/* Accel Config */
#define IMU_ACCEL_REG_CONF   0x40
#define IMU_ACCEL_BWP_MASK   0x0F
#define IMU_ACCEL_BWP_OFFSET 4
#define IMU_ACCEL_ODR_MASK   0x0F
#define IMU_ACCEL_ODR_OFFSET 0

#define IMU_ACCEL_REG_RANGE   0x41
#define IMU_ACCEL_RANGE_MASK  0x03
#define IMU_ACCEL_RANGE_OFFSET 0

/* Gyro Config */
#define IMU_GYRO_REG_RANGE   0x0F
#define IMU_GYRO_RANGE_MASK  0x07
#define IMU_GYRO_RANGE_OFFSET 0

#define IMU_GYRO_REG_BANDWIDTH     0x10
#define IMU_GYRO_BANDWIDTH_MASK    0x07
#define IMU_GYRO_BANDWIDTH_OFFSET  0

/* Interrupts */
#define IMU_ACCEL_REG_INT_MAP     0x58
#define IMU_ACCEL_INT1_DRDY_BIT   0x04
#define IMU_ACCEL_INT2_DRDY_BIT   0x40

#define IMU_GYRO_REG_INT_MAP      0x18
#define IMU_GYRO_INT3_DRDY_BIT    0x01
#define IMU_GYRO_INT4_DRDY_BIT    0x80

#define IMU_GYRO_REG_INT_CTRL     0x15
#define IMU_GYRO_DRDY_EN_BIT      0x80

/* Accel GPIO properties */
#define IMU_ACCEL_REG_INT1_IOCFG  0x53
#define IMU_ACCEL_REG_INT2_IOCFG  0x54
#define IMU_ACCEL_INT_OUT_BIT     0x08
#define IMU_ACCEL_INT_OD_BIT      0x04
#define IMU_ACCEL_INT_H_BIT       0x02

/* Gyro GPIO properties */
#define IMU_GYRO_REG_INT_IOCFG    0x16
#define IMU_GYRO_INT3_H_BIT       0x01
#define IMU_GYRO_INT3_OD_BIT      0x02
#define IMU_GYRO_INT4_H_BIT       0x04
#define IMU_GYRO_INT4_OD_BIT      0x08

/* Data registers */
#define IMU_ACCEL_REG_DATA   0x12
#define IMU_ACCEL_DATA_LEN   6
#define IMU_ACCEL_X_OFFSET   0
#define IMU_ACCEL_Y_OFFSET   2
#define IMU_ACCEL_Z_OFFSET   4

#define IMU_GYRO_REG_DATA    0x02
#define IMU_GYRO_DATA_LEN    6
#define IMU_GYRO_X_OFFSET    0
#define IMU_GYRO_Y_OFFSET    2
#define IMU_GYRO_Z_OFFSET    4

/* SPI dummy bytes */
#define IMU_SPI_ACCEL_RD_DUMMY  1
#define IMU_SPI_ACCEL_WR_DUMMY  0
#define IMU_SPI_GYRO_RD_DUMMY   0
#define IMU_SPI_GYRO_WR_DUMMY   0

/* SPI scan speed */
#define RDK_IMU_SPI_SCAN_SPEED_HZ  1000000

/* SPI mode (requires <linux/spi/spidev.h> in .c file) */
#define RDK_IMU_SPI_MODE  SPI_MODE_3

#else
#error "Unsupported RDK_IMU_CHIP value"
#endif

/* ============================== Internal Structures ============================== */
/* 前向声明 bus 结构体类型 */
typedef struct rdk_imu_bus_s rdk_imu_bus_t;

/* Bus operation method table (private) */
typedef struct{
    rdk_imu_err_t (*read)(struct rdk_imu_bus_s *bus, uint8_t reg, uint8_t *data, uint8_t len);
    rdk_imu_err_t (*write)(struct rdk_imu_bus_s *bus, uint8_t reg, uint8_t *data, uint8_t len);
    rdk_imu_err_t (*update)(struct rdk_imu_bus_s *bus, uint8_t reg, uint8_t data, uint8_t mask);
} rdk_imu_bus_ops_t;

/* Bus handle (private) */
typedef struct rdk_imu_bus_s{
    int fd;
    uint8_t addr;
    uint32_t speed_hz;
    uint8_t rd_dummy;
    uint8_t wr_dummy;
    rdk_imu_bus_ops_t *ops;
} rdk_imu_bus_t;

/* Software FIFO handle (private) */
typedef struct{
    rdk_imu_6_axis_data_t *buffer;
    unsigned int size;
    unsigned int mask;
    unsigned int head;
    unsigned int tail;
    rdk_imu_fifo_mode_t mode;
} rdk_imu_fifo_t;

/* Main device state (private, now complete) */
struct rdk_imu_state_s{
    rdk_imu_bus_t accel_bus;
    rdk_imu_bus_t gyro_bus;

    float accel_data_scale;
    float accel_data_bais;
    float gyro_data_scale;
    float gyro_data_bais;

    struct gpiod_chip *accel_drdy_gpio_chip;
    struct gpiod_line *accel_drdy_gpio_line;
    struct gpiod_chip *gyro_drdy_gpio_chip;
    struct gpiod_line *gyro_drdy_gpio_line;

    pthread_mutex_t mutex;
    uint32_t enable;
    int32_t irq_priority;
    uint64_t irq_thread_timeout_ns;
    pthread_t accel_thread;
    pthread_t gyro_thread;
    volatile int accel_running;
    volatile int gyro_running;

    rdk_imu_fifo_t imu_fifo;
    rdk_imu_6_axis_data_t fuse_win[3];
};

#endif /* __RDK_IMU_PRIV_H */