/*
 * Copyright (c) 2026 Leaf. D-Robotics.
 * SPDX-License-Identifier: MIT
 */
#ifndef __RDK_IMU_MODULE_H
#define __RDK_IMU_MODULE_H

#include <stdint.h>
#include <pthread.h>
#include <gpiod.h>

/* ============================== BMI088 Register and Feature  ============================== */
#define IMU_REG_BMI088

#ifdef IMU_REG_BMI088
#define IMU_ACCEL_REG_CHIP_ID 0x00
#define IMU_ACCEL_CHIP_ID 0x1E
#define IMU_GYRO_REG_CHIP_ID 0x00
#define IMU_GYRO_CHIP_ID 0x0F

#define IMU_ACCEL_REG_SOFTRESET 0x7E
#define IMU_ACCEL_SOFTRESET_CMD 0xB6
#define IMU_ACCEL_RESET_SLEEP_MS 1
#define IMU_GYRO_REG_SOFTRESET 0x14
#define IMU_GYRO_SOFTRESET_CMD 0xB6
#define IMU_GYRO_RESET_SLEEP_MS 30

#define IMU_ACCEL_REG_ENABLE 0x7D
#define IMU_ACCEL_ENABLE_VALUE 0x04
#define IMU_ACCEL_REG_PWR_CONF 0x7C
#define IMU_ACCEL_ACTIVATE_MODE 0x00

#define IMU_ACCEL_REG_CONF 0x40
#define IMU_ACCEL_BWP_MASK 0x0F
#define IMU_ACCEL_BWP_OFFSET 4
#define IMU_ACCEL_ODR_MASK 0x0F
#define IMU_ACCEL_ODR_OFFSET 0

#define IMU_ACCEL_REG_RANGE 0x41
#define IMU_ACCEL_RANGE_MASK 0x03
#define IMU_ACCEL_RANGE_OFFSET 0

#define IMU_GYRO_REG_RANGE 0x0F
#define IMU_GYRO_RANGE_MASK 0x07
#define IMU_GYRO_RANGE_OFFSET 0

#define IMU_GYRO_REG_BANDWIDTH 0x10
#define IMU_GYRO_BANDWIDTH_MASK 0x07
#define IMU_GYRO_BANDWIDTH_OFFSET 0

#define IMU_ACCEL_REG_INT_MAP 0x58
#define IMU_ACCEL_INT1_DRDY_BIT 0x04
#define IMU_ACCEL_INT2_DRDY_BIT 0x40

#define IMU_GYRO_REG_INT_MAP 0x18
#define IMU_GYRO_INT3_DRDY_BIT 0x01
#define IMU_GYRO_INT4_DRDY_BIT 0x80

#define IMU_GYRO_REG_INT_CTRL 0x15
#define IMU_GYRO_DRDY_EN_BIT 0x80

#define IMU_ACCEL_REG_INT1_IOCFG 0x53
#define IMU_ACCEL_REG_INT2_IOCFG 0x54
#define IMU_ACCEL_INT_OUT_BIT 0x08
#define IMU_ACCEL_INT_OD_BIT 0x04
#define IMU_ACCEL_INT_H_BIT 0x02

#define IMU_GYRO_REG_INT_IOCFG 0x16
#define IMU_GYRO_INT3_H_BIT 0x01
#define IMU_GYRO_INT3_OD_BIT 0x02
#define IMU_GYRO_INT4_H_BIT 0x04
#define IMU_GYRO_INT4_OD_BIT 0x08

#define IMU_ACCEL_REG_DATA 0X12
#define IMU_ACCEL_DATA_LEN 6
#define IMU_ACCEL_X_OFFSET 0
#define IMU_ACCEL_Y_OFFSET 2
#define IMU_ACCEL_Z_OFFSET 4

#define IMU_GYRO_REG_DATA 0X02
#define IMU_GYRO_DATA_LEN 6
#define IMU_GYRO_X_OFFSET 0
#define IMU_GYRO_Y_OFFSET 2
#define IMU_GYRO_Z_OFFSET 4

#endif /* IMU_REG_BMI088 */

#define IMU_SPI_ACCEL_RD_DUMMY 1
#define IMU_SPI_ACCEL_WR_DUMMY 0
#define IMU_SPI_GYRO_RD_DUMMY 0
#define IMU_SPI_GYRO_WR_DUMMY 0

#define RDK_IMU_SPI_SCAN_SPEED_HZ 1000000
#define RDK_IMU_SPI_MODE SPI_MODE_3

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
    RDK_IMU_GYRO_ENABLE_FAILED,
    RDK_IMU_ACCEL_ENABLE_FAILED,
    RDK_IMU_I2C_UPDATE_FAILED,
    RDK_IMU_SPI_UPDATE_FAILED,
    RDK_IMU_GPIO_INIT_FAILED,
    RDK_IMU_FIFO_FULL,
    RDK_IMU_FIFO_EMPTY,
    RDK_IMU_DEVICE_ENABLING,
    RDK_IMU_THREAD_CREATE_FAILED,
    RDK_IMU_FIFO_ALLOC_FAILED,
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
        uint8_t *data, 
        uint8_t len);

    rdk_imu_err_t (*update)(
        rdk_imu_bus_t *bus, 
        uint8_t reg, 
        uint8_t data, 
        uint8_t mask);
}rdk_imu_bus_ops_t;

struct rdk_imu_bus_s{
    int fd;
    uint8_t addr; // I2C address, SPI does not use this
    uint32_t speed_hz; // SPI speed, 0 for default, I2C does not use this
    uint8_t rd_dummy; // SPI only
    uint8_t wr_dummy; // SPI only
    rdk_imu_bus_ops_t *ops;
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
    RDK_IMU_OSR4 = 0x08,
    RDK_IMU_OSR2 = 0x09,
    RDK_IMU_NORMAL = 0x0A,
}rdk_imu_accel_bwp_t;

typedef enum rdk_imu_accel_odr_e{
    RDK_IMU_ACCEL_12_5 = 0x05,
    RDK_IMU_ACCEL_25 = 0x06,
    RDK_IMU_ACCEL_50 = 0x07,
    RDK_IMU_ACCEL_100 = 0x08,
    RDK_IMU_ACCEL_200 = 0x09,
    RDK_IMU_ACCEL_400 = 0x0A,
    RDK_IMU_ACCEL_800 = 0x0B,
    RDK_IMU_ACCEL_1600 = 0x0C,
}rdk_imu_accel_odr_t;

typedef enum rdk_imu_accel_range_e{
    RDK_IMU_ACCEL_3G = 0x00,
    RDK_IMU_ACCEL_6G = 0x01,
    RDK_IMU_ACCEL_12G = 0x02,
    RDK_IMU_ACCEL_24G = 0x03,
}rdk_imu_accel_range_t;

typedef enum rdk_imu_gyro_range_e{
    RDK_IMU_GYRO_2000DPS = 0x00,
    RDK_IMU_GYRO_1000DPS = 0x01,
    RDK_IMU_GYRO_500DPS = 0x02,
    RDK_IMU_GYRO_250DPS = 0x03,
    RDK_IMU_GYRO_125DPS = 0x04,
}rdk_imu_gyro_range_t;

typedef enum rdk_imu_gyro_bandwidth_e{
    RDK_IMU_ODR2000_BW532 = 0x00,
    RDK_IMU_ODR2000_BW230 = 0x01,
    RDK_IMU_ODR1000_BW116 = 0x02,
    RDK_IMU_ODR400_BW47 = 0x03,
    RDK_IMU_ODR200_BW23 = 0x04,
    RDK_IMU_ODR100_BW12 = 0x05,
    RDK_IMU_ODR200_BW64 = 0x06,
    RDK_IMU_ODR100_BW32 = 0x07,
}rdk_imu_gyro_bandwidth_t;

typedef enum rdk_imu_fifo_mode_e{
    RDK_IMU_FIFO_DROP, /*  FIFO 满时直接丢弃新数据（非覆盖） */
    RDK_IMU_FIFO_OVERWRITE, /*  FIFO 满时覆盖最旧的数据 */
}rdk_imu_fifo_mode_t;

typedef enum rdk_imu_gpio_mode_e{
    /*  高有效        低有效  */
    RDK_IMU_PP_H, RDK_IMU_PP_L, /* 推挽 */
    RDK_IMU_OD_H, RDK_IMU_OD_L, /* 开漏 */
}rdk_imu_gpio_mode_t;

typedef struct rdk_imu_config_s{
    /* accel 和 gyro 的 chip 端和 SoC 端中断配置 */
    rdk_imu_accel_drdy_int_t accel_drdy_int;
    rdk_imu_gpio_mode_t accel_int_gpio_mode;
    uint32_t accel_drdy_gpio_chip;
    uint32_t accel_drdy_gpio_line;

    rdk_imu_gyro_drdy_int_t gyro_drdy_int;
    rdk_imu_gpio_mode_t gyro_int_gpio_mode;
    uint32_t gyro_drdy_gpio_chip;
    uint32_t gyro_drdy_gpio_line;

    int32_t irq_priority; // 中断线程的实时优先级（1~99），-1 表示自动使用最高优先级
    uint64_t irq_thread_timeout_ns; // 中断轮询线程的timeout

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
    rdk_imu_fifo_mode_t mode;
}rdk_imu_fifo_t;

typedef struct rdk_imu_state_s{
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

    uint32_t enable;

    rdk_imu_fifo_t imu_fifo;

    int32_t irq_priority;
    uint64_t irq_thread_timeout_ns;

    pthread_mutex_t mutex;
    pthread_t accel_thread;     
    pthread_t gyro_thread;      
    volatile int accel_running;    
    volatile int gyro_running;     

    rdk_imu_6_axis_data_t fuse_win[3];
    
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
    uint32_t *count);

rdk_imu_err_t rdk_imu_read_indep(
    rdk_imu_state_t *st, 
    rdk_imu_6_axis_data_t *data,
    uint32_t *count);

rdk_imu_err_t rdk_imu_read_fused(
    rdk_imu_state_t *st, 
    rdk_imu_6_axis_data_t *data,
    rdk_imu_device_t fuse_by,
    uint64_t max_age_ns);


#endif
