/**
 * Copyright (c) 2026 Leaf. D-Robotics.
 * SPDX-License-Identifier: MIT
 */
#ifndef __RDK_IMU_H
#define __RDK_IMU_H

#include <stdint.h>

#ifndef RDK_IMU_CHIP
#define RDK_IMU_CHIP RDK_IMU_BMI088
#endif

/* ============================== Error Code ============================== */
typedef enum{
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
    RDK_IMU_DEVICE_BUSY,
    RDK_IMU_THREAD_CREATE_FAILED,
    RDK_IMU_FIFO_ALLOC_FAILED,
} rdk_imu_err_t;

/* ============================== Data Package ============================== */
typedef struct{
    float x, y, z;
    uint64_t timestamp_ns;
    int32_t valid;
} rdk_imu_3_axis_data_t;

typedef struct{
    rdk_imu_3_axis_data_t accel, gyro;
} rdk_imu_6_axis_data_t;

/* ============================== FIFO Mode ============================== */
typedef enum{
    RDK_IMU_FIFO_DROP,
    RDK_IMU_FIFO_OVERWRITE,
} rdk_imu_fifo_mode_t;

/* ============================== Device Type ============================== */
typedef enum{
    RDK_IMU_ACCEL,
    RDK_IMU_GYRO,
} rdk_imu_device_t;

/* ============================== Sensor Configuration Enums ============================== */
/* Accel settings */
typedef enum{
    RDK_IMU_OSR4   = 0x08,
    RDK_IMU_OSR2   = 0x09,
    RDK_IMU_NORMAL = 0x0A,
} rdk_imu_accel_bwp_t;

typedef enum{
    RDK_IMU_ACCEL_12_5 = 0x05,
    RDK_IMU_ACCEL_25   = 0x06,
    RDK_IMU_ACCEL_50   = 0x07,
    RDK_IMU_ACCEL_100  = 0x08,
    RDK_IMU_ACCEL_200  = 0x09,
    RDK_IMU_ACCEL_400  = 0x0A,
    RDK_IMU_ACCEL_800  = 0x0B,
    RDK_IMU_ACCEL_1600 = 0x0C,
} rdk_imu_accel_odr_t;

typedef enum{
    RDK_IMU_ACCEL_3G  = 0x00,
    RDK_IMU_ACCEL_6G  = 0x01,
    RDK_IMU_ACCEL_12G = 0x02,
    RDK_IMU_ACCEL_24G = 0x03,
} rdk_imu_accel_range_t;

/* Gyro settings */
typedef enum{
    RDK_IMU_GYRO_2000DPS = 0x00,
    RDK_IMU_GYRO_1000DPS = 0x01,
    RDK_IMU_GYRO_500DPS  = 0x02,
    RDK_IMU_GYRO_250DPS  = 0x03,
    RDK_IMU_GYRO_125DPS  = 0x04,
} rdk_imu_gyro_range_t;

typedef enum{
    RDK_IMU_ODR2000_BW532 = 0x00,
    RDK_IMU_ODR2000_BW230 = 0x01,
    RDK_IMU_ODR1000_BW116 = 0x02,
    RDK_IMU_ODR400_BW47   = 0x03,
    RDK_IMU_ODR200_BW23   = 0x04,
    RDK_IMU_ODR100_BW12   = 0x05,
    RDK_IMU_ODR200_BW64   = 0x06,
    RDK_IMU_ODR100_BW32   = 0x07,
} rdk_imu_gyro_bandwidth_t;

/* ============================== Interrupt Settings ============================== */
typedef enum{
    RDK_IMU_INT1,
    RDK_IMU_INT2,
} rdk_imu_accel_drdy_int_t;

typedef enum{
    RDK_IMU_INT3,
    RDK_IMU_INT4,
} rdk_imu_gyro_drdy_int_t;

typedef enum{
    RDK_IMU_PP_H, RDK_IMU_PP_L,
    RDK_IMU_OD_H, RDK_IMU_OD_L,
} rdk_imu_gpio_mode_t;

/* ============================== Bus Info ============================== */
typedef enum{
    RDK_IMU_AUTO,
    RDK_IMU_I2C,
    RDK_IMU_SPI,
} rdk_imu_interface_t;

typedef struct{
    uint8_t bus;
    uint8_t addr;
} rdk_imu_i2c_info_t;

typedef struct{
    uint8_t bus;
    uint8_t cs;
    uint32_t speed_hz;
} rdk_imu_spi_info_t;

typedef struct{
    rdk_imu_interface_t interface;
    union{
        struct{
            rdk_imu_i2c_info_t accel;
            rdk_imu_i2c_info_t gyro;
        } i2c;
        struct{
            rdk_imu_spi_info_t accel;
            rdk_imu_spi_info_t gyro;
        } spi;
    } bus;
} rdk_imu_bus_info_t;

/* ============================== Configuration Structure ============================== */
typedef struct{
    /* Accel */
    rdk_imu_accel_bwp_t   accel_bwp;
    rdk_imu_accel_odr_t   accel_odr;
    rdk_imu_accel_range_t accel_range;

    /* Gyro */
    rdk_imu_gyro_range_t     gyro_range;
    rdk_imu_gyro_bandwidth_t gyro_bandwidth;

    /* IMU interrupt pins */
    rdk_imu_accel_drdy_int_t accel_drdy_int;
    rdk_imu_gpio_mode_t      accel_int_gpio_mode;
    rdk_imu_gyro_drdy_int_t  gyro_drdy_int;
    rdk_imu_gpio_mode_t      gyro_int_gpio_mode;

    /* SoC GPIO lines */
    uint32_t accel_drdy_gpio_chip;
    uint32_t accel_drdy_gpio_line;
    uint32_t gyro_drdy_gpio_chip;
    uint32_t gyro_drdy_gpio_line;

    /* FIFO */
    uint32_t           fifo_length;
    rdk_imu_fifo_mode_t fifo_mode;

    /* IRQ thread */
    int32_t  irq_priority;
    uint64_t irq_thread_timeout_ns;
} rdk_imu_config_t;

/* ============================== Opaque Device Handle ============================== */
typedef struct rdk_imu_state_s rdk_imu_state_t;

/* ============================== API ============================== */
/**
 * @note 正确的初始化、反初始化和数据采集顺序如下：
 * rdk_imu_create_default
 *  ↓ ↓ ↓
 * rdk_imu_bus_init
 *  ↓ ↓ ↓
 * rdk_imu_device_init
 *  ↓ ↓ ↓
 * rdk_imu_enable
 *  ↓ ↓ ↓
 * Operate FIFO, read IMU data
 *  ↓ ↓ ↓
 * rdk_imu_disable
 *  ↓ ↓ ↓
 * rdk_imu_device_deinit
 *  ↓ ↓ ↓
 * rdk_imu_bus_deinit
 *  ↓ ↓ ↓
 * rdk_imu_destroy
 *  ↓ ↓ ↓
 * exit
 */

/**
 * @brief 合法方式获取一个初始 rdk_imu_state_t 句柄
 * @param None
 */
rdk_imu_state_t *rdk_imu_create_default(
    void);

/**
 * @brief 合法方式销毁 rdk_imu_state_t 句柄
 * @param st 指向需要操作 rdk_imu_state_t 结构体
 */
rdk_imu_err_t rdk_imu_destroy(
    rdk_imu_state_t *st);

/**
 * @brief 查询 bus_info 中配置的总线是否可用并以此初始化 st
 * @param st 指向需要操作 rdk_imu_state_t 结构体
 * @param bus_info 配置的总线信息
 */
rdk_imu_err_t rdk_imu_bus_init(
    rdk_imu_state_t* st,
    rdk_imu_bus_info_t bus_info);

/**
 * @brief 反初始化 st 中的总线内容
 * @param st 指向需要操作 rdk_imu_state_t 结构体
 */
rdk_imu_err_t rdk_imu_bus_deinit(
    rdk_imu_state_t* st);

/**
 * @brief 初始化 IMU 设备，配置设备性能参数
 * @param st 指向需要操作 rdk_imu_state_t 结构体
 * @param config 需要设置性能参数信息
 */
rdk_imu_err_t rdk_imu_device_init(
    rdk_imu_state_t* st,
    rdk_imu_config_t config);

/**
 * @brief 反初始化 st 中的 IMU 设备设置
 * @param st 指向需要操作 rdk_imu_state_t 结构体
 */
rdk_imu_err_t rdk_imu_device_deinit(
    rdk_imu_state_t* st);

/**
 * @brief 使能 IMU 开始进行数据采集
 * @param st 指向需要操作 rdk_imu_state_t 结构体
 */
rdk_imu_err_t rdk_imu_enable(
    rdk_imu_state_t* st);

/**
 * @brief 关闭 IMU 的数据采集行为
 * @param st 指向需要操作 rdk_imu_state_t 结构体
 */
rdk_imu_err_t rdk_imu_disable(
    rdk_imu_state_t* st);

/**
 * @brief 查看当前软件 FIFO 中的可读原始数据包数量
 * @param st 指向需要操作 rdk_imu_state_t 结构体
 * @param count 当前 FIFO 中可读原始数据包数量
 * @note 线程安全
 */
rdk_imu_err_t rdk_imu_fifo_available(
    rdk_imu_state_t *st,
    uint32_t *count);

/**
 * @brief 从 FIFO 中取出一帧数据包，消耗一条 FIFO 容量
 * @param st 指向需要操作 rdk_imu_state_t 结构体
 * @param data 指向返回的数据包变量
 * @param count 当前 FIFO 中剩余可读原始数据包数量
 * @note 读取的数据包仅单边有效，具体是 Accel/Gyro 哪一侧需要判断
 * @note 线程安全
 */
rdk_imu_err_t rdk_imu_read_indep(
    rdk_imu_state_t *st, 
    rdk_imu_6_axis_data_t *data,
    uint32_t *count);

/**
 * @brief 从 FIFO 中取出一帧融合数据包，消耗若干条 FIFO 容量进行插值
 * @param st 指向需要操作 rdk_imu_state_t 结构体
 * @param data 指向返回的数据包变量
 * @param fuse_by 融合数据包以哪一边的数据为真值依据
 * @param max_age_ns 最大插值间隔
 * @note 该函数将堵塞上下文，直到 FIFO 数据流达到插值条件
 * @note 线程安全
 */
rdk_imu_err_t rdk_imu_read_fused(
    rdk_imu_state_t *st, 
    rdk_imu_6_axis_data_t *data,
    rdk_imu_device_t fuse_by,
    uint64_t max_age_ns);

#endif /* __RDK_IMU_H */
