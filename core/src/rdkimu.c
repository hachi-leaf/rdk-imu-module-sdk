/*
 * Copyright (c) 2026 Leaf. D-Robotics.
 * SPDX-License-Identifier: MIT
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <gpiod.h>

#include "rdkimu_priv.h"

/* ============================== Macro ============================== */
#ifdef RDK_IMU_DEBUG
#define RDK_DEBUG_PRINTF(fmt, ...) \
    printf("[%s:%d] %s(): " fmt, __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#else
#define RDK_DEBUG_PRINTF(fmt, ...) ((void)0)
#endif

#define rdk_imu_accel_read(reg, data, len) \
    (st->accel_bus.ops->read(&st->accel_bus, (reg), (data), (len)))
#define rdk_imu_accel_write(reg, data, len) \
    (st->accel_bus.ops->write(&st->accel_bus, (reg), (data), (len)))
#define rdk_imu_accel_update(reg, data, mask) \
    (st->accel_bus.ops->update(&st->accel_bus, (reg), (data), (mask)))

#define rdk_imu_gyro_read(reg, data, len) \
    (st->gyro_bus.ops->read(&st->gyro_bus, (reg), (data), (len)))
#define rdk_imu_gyro_write(reg, data, len) \
    (st->gyro_bus.ops->write(&st->gyro_bus, (reg), (data), (len)))
#define rdk_imu_gyro_update(reg, data, mask) \
    (st->gyro_bus.ops->update(&st->gyro_bus, (reg), (data), (mask)))

/* ============================== Utility functions ============================== */
/**
 * @brief 毫秒级 Delay
 */
static void sleep_ms(int milliseconds) {
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

/**
 * @brief 初始化 rdk_imu_fifo_t 结构体
 */
static rdk_imu_err_t rdk_imu_fifo_init(
    rdk_imu_fifo_t *fifo, 
    unsigned int size,
    rdk_imu_fifo_mode_t mode)
{
    // size 必须为 2 的幂
    if((size & (size - 1)) != 0)return RDK_IMU_ERR_PARAM;

    fifo->buffer = (rdk_imu_6_axis_data_t *)malloc(size * sizeof(rdk_imu_6_axis_data_t));
    if(!fifo->buffer)return RDK_IMU_FIFO_ALLOC_FAILED;

    fifo->size = size;
    fifo->mask = size - 1;
    fifo->head = 0;
    fifo->tail = 0;
    fifo->mode = mode; 

    memset(fifo->buffer, 0, size * sizeof(rdk_imu_6_axis_data_t));

    return RDK_IMU_OK;
}

/**
 * @brief 反初始化 rdk_imu_fifo_t 结构体
 */
static void rdk_imu_fifo_deinit(
    rdk_imu_fifo_t *fifo)
{
    free(fifo->buffer);
    fifo->buffer = NULL;
    fifo->size = fifo->mask = 0;
    fifo->head = fifo->tail = 0;
}

/**
 * @brief 返回 bool：FIFO 是否为空
 */
static inline int rdk_imu_fifo_is_empty(
    rdk_imu_fifo_t *fifo)
{
    return (fifo->head == fifo->tail);
}

/**
 * @brief 返回 bool：FIFO 是否为满
 */
static inline int rdk_imu_fifo_is_full(
    rdk_imu_fifo_t *fifo)
{
    return ((fifo->head + 1) & fifo->mask) == fifo->tail;
}

/**
 * @brief 返回 FIFO 计数
 */
static unsigned int rdk_imu_fifo_count(
    const rdk_imu_fifo_t *fifo)
{
    return (fifo->head - fifo->tail) & fifo->mask;
}

/**
 * @brief FIFO 存数据
 */
static rdk_imu_err_t rdk_imu_fifo_push(
    rdk_imu_fifo_t *fifo, 
    rdk_imu_6_axis_data_t *data)
{
    
    if(rdk_imu_fifo_is_full(fifo)){
        if(fifo->mode == RDK_IMU_FIFO_OVERWRITE){
            /* 覆盖最旧数据：移动尾指针丢弃最早的元素 */
            fifo->tail = (fifo->tail + 1) & fifo->mask;
        }
        else{
            /* RDK_IMU_FIFO_DROP：满时直接丢弃新数据 */
            return RDK_IMU_FIFO_FULL;
        }
    }

    // fifo->buffer[fifo->head] = *data;
    memcpy(&fifo->buffer[fifo->head], data, sizeof(rdk_imu_6_axis_data_t));
    fifo->head = (fifo->head + 1) & fifo->mask;

    return RDK_IMU_OK;
}
    
/**
 * @brief FIFO 取数据
 */
static rdk_imu_err_t rdk_imu_fifo_pop(
    rdk_imu_fifo_t *fifo, 
    rdk_imu_6_axis_data_t *data)
{
    if(rdk_imu_fifo_is_empty(fifo))return RDK_IMU_FIFO_EMPTY;

    *data = fifo->buffer[fifo->tail];
    fifo->tail = (fifo->tail + 1) & fifo->mask;

    return RDK_IMU_OK;
}

/**
 * @brief 解析数据
 */
static void parse_accel_data(
    uint8_t *raw,
    rdk_imu_6_axis_data_t *data,
    rdk_imu_state_t *st)
{
    int16_t raw16[3];
    raw16[0] = raw[IMU_ACCEL_X_OFFSET] | raw[IMU_ACCEL_X_OFFSET+1]<<8;
    raw16[1] = raw[IMU_ACCEL_Y_OFFSET] | raw[IMU_ACCEL_Y_OFFSET+1]<<8;
    raw16[2] = raw[IMU_ACCEL_Z_OFFSET] | raw[IMU_ACCEL_Z_OFFSET+1]<<8;

    data->accel.valid = 1;
    data->accel.x = raw16[0] * st->accel_data_scale + st->accel_data_bais;
    data->accel.y = raw16[1] * st->accel_data_scale + st->accel_data_bais;
    data->accel.z = raw16[2] * st->accel_data_scale + st->accel_data_bais;

    return ;
}

/**
 * @brief 解析数据
 */
static void parse_gyro_data(
    uint8_t *raw,
    rdk_imu_6_axis_data_t *data,
    rdk_imu_state_t *st)
{
    int16_t raw16[3];
    raw16[0] = raw[IMU_GYRO_X_OFFSET] | raw[IMU_GYRO_X_OFFSET+1]<<8;
    raw16[1] = raw[IMU_GYRO_Y_OFFSET] | raw[IMU_GYRO_Y_OFFSET+1]<<8;
    raw16[2] = raw[IMU_GYRO_Z_OFFSET] | raw[IMU_GYRO_Z_OFFSET+1]<<8;

    data->gyro.valid = 1;
    data->gyro.x = raw16[0] * st->gyro_data_scale + st->gyro_data_bais;
    data->gyro.y = raw16[1] * st->gyro_data_scale + st->gyro_data_bais;
    data->gyro.z = raw16[2] * st->gyro_data_scale + st->gyro_data_bais;

    return ;
}

/**
 * @brief 线程函数
 */
static void *rdk_imu_accel_irq_thread(
    void *arg)
{
    rdk_imu_state_t *st = (rdk_imu_state_t *)arg;
    struct timespec timeout = {
        .tv_sec = st->irq_thread_timeout_ns / 100000000, 
        .tv_nsec = st->irq_thread_timeout_ns % 100000000,
    };

    while(st->accel_running){
        int wait_ret = gpiod_line_event_wait(st->accel_drdy_gpio_line, &timeout);
        if(wait_ret < 0){
            perror("gpiod_line_event_wait (accel)");
            break;   // 发生严重错误，退出线程
        }
        else if(wait_ret == 0){
            // 仅在设置超时且超时发生时进入这里，阻塞等待不会触发
            continue;
        }
        /* 读取事件，清除中断标志，同时获取时间戳 */
        struct gpiod_line_event event;
        if(gpiod_line_event_read(st->accel_drdy_gpio_line, &event) < 0){
            // perror("gpiod_line_event_read (accel)");
            continue;
        }
        uint8_t raw[IMU_ACCEL_DATA_LEN] = {0};
        if(rdk_imu_accel_read(IMU_ACCEL_REG_DATA, raw, IMU_ACCEL_DATA_LEN) != RDK_IMU_OK){
            RDK_DEBUG_PRINTF("Failed to read accel data\n");
            continue;
        }
        rdk_imu_6_axis_data_t data;
        memset(&data, 0, sizeof(data));
        parse_accel_data(raw, &data, st);
        data.accel.timestamp_ns = (uint64_t)event.ts.tv_sec * 1000000000ULL + (uint64_t)event.ts.tv_nsec;

        pthread_mutex_lock(&st->mutex);
        rdk_imu_fifo_push(&st->imu_fifo, &data);
        pthread_mutex_unlock(&st->mutex);
    }
    return NULL;
}

/**
 * @brief 线程函数
 */
static void *rdk_imu_gyro_irq_thread(
    void *arg)
{
    rdk_imu_state_t *st = (rdk_imu_state_t *)arg;
    struct timespec timeout = {
        .tv_sec = st->irq_thread_timeout_ns / 100000000, 
        .tv_nsec = st->irq_thread_timeout_ns % 100000000,
    };

    while(st->gyro_running){
        int wait_ret = gpiod_line_event_wait(st->gyro_drdy_gpio_line, &timeout);
        if(wait_ret < 0){
            // perror("gpiod_line_event_wait (gyro)");
            break;   // 发生严重错误，退出线程
        }
        else if(wait_ret == 0){
            // 仅在设置超时且超时发生时进入这里，阻塞等待不会触发
            continue;
        }
        /* 读取事件，清除中断标志，同时获取时间戳 */
        struct gpiod_line_event event;
        if(gpiod_line_event_read(st->gyro_drdy_gpio_line, &event) < 0){
            // perror("gpiod_line_event_read (gyro)");
            continue;
        }
        uint8_t raw[IMU_GYRO_DATA_LEN] = {0};
        if(rdk_imu_gyro_read(IMU_GYRO_REG_DATA, raw, IMU_GYRO_DATA_LEN) != RDK_IMU_OK){
            RDK_DEBUG_PRINTF("Failed to read gyro data\n");
            continue;
        }
        rdk_imu_6_axis_data_t data = {0};
        memset(&data, 0, sizeof(data));
        parse_gyro_data(raw, &data, st);
        data.gyro.timestamp_ns = (uint64_t)event.ts.tv_sec * 1000000000ULL + (uint64_t)event.ts.tv_nsec;

        pthread_mutex_lock(&st->mutex);
        rdk_imu_fifo_push(&st->imu_fifo, &data);
        pthread_mutex_unlock(&st->mutex);
    }
    return NULL;
}

/**
 * @brief 在两个有效三轴数据之间进行线性插值，得到目标时刻的值
 */
static void interpolate_3axis(
    const rdk_imu_3_axis_data_t *prev,
    const rdk_imu_3_axis_data_t *next,
    uint64_t target_ns,
    rdk_imu_3_axis_data_t *out)
{
    if(!prev || !next || !out)return;

    if(next->timestamp_ns == prev->timestamp_ns){
        *out = *prev;
        out->timestamp_ns = target_ns;
        return;
    }

    /* 防止时间戳无符号下溢出 */
    int64_t dt = (int64_t)(target_ns - prev->timestamp_ns);
    int64_t range = (int64_t)(next->timestamp_ns - prev->timestamp_ns);
    float ratio = (float)dt / (float)range;  
    // float ratio = (float)(target_ns - prev->timestamp_ns) /
    //                (float)(next->timestamp_ns - prev->timestamp_ns);

    out->x = prev->x + (next->x - prev->x) * ratio;
    out->y = prev->y + (next->y - prev->y) * ratio;
    out->z = prev->z + (next->z - prev->z) * ratio;
    out->timestamp_ns = target_ns;
    out->valid = 1;
}

/**
 * @brief 对比并返回 rdk_imu_6_axis_data_t 中两侧时间戳中数值更大的一方
 */
static inline uint64_t rdk_imu_data_timestamp_ns(
    rdk_imu_6_axis_data_t data)
{
    return data.accel.timestamp_ns > data.gyro.timestamp_ns ?
        data.accel.timestamp_ns : data.gyro.timestamp_ns;
}

/* ============================== API Implementation ============================== */
rdk_imu_state_t *rdk_imu_create_default(
    void)
{
    rdk_imu_state_t *st = malloc(sizeof(rdk_imu_state_t));
    if(!st)return NULL;
    
    memset(st, 0, sizeof(*st));
    st->enable = 0;

    return st;
}

rdk_imu_err_t rdk_imu_destroy(
    rdk_imu_state_t *st)
{
    if(!st)return RDK_IMU_ERR_PARAM;

    if(st->imu_fifo.buffer)free(st->imu_fifo.buffer);

    if(st)free(st);

    return RDK_IMU_OK;
}


rdk_imu_err_t rdk_imu_device_init(
    rdk_imu_state_t* st,
    rdk_imu_config_t config)
{
    if(!st)return RDK_IMU_CAN_NOT_RELEASE;

    rdk_imu_err_t ret;
    uint8_t reg_value[2];

    /* Reset IMU */ 
    reg_value[0] = IMU_ACCEL_SOFTRESET_CMD;
    ret = rdk_imu_accel_write(IMU_ACCEL_REG_SOFTRESET, &reg_value[0], 1);
    // if(ret)return ret;
    sleep_ms(IMU_ACCEL_RESET_SLEEP_MS);

    reg_value[0] = IMU_GYRO_SOFTRESET_CMD;
    ret = rdk_imu_gyro_write(IMU_GYRO_REG_SOFTRESET, &reg_value[0], 1);
    // if(ret)return ret;
    sleep_ms(IMU_GYRO_RESET_SLEEP_MS);
    /* BMI088 的 softreset 不提供停止信号，所以这里不做 reset 的 ret 检查 */

    /* Enable accel pwr */
    reg_value[0] = IMU_ACCEL_ENABLE_VALUE;
    reg_value[1] = 0xFF;
    for(int i=0; i<10; i++){
        RDK_DEBUG_PRINTF("Attempting to enable accel, time %d\n", i);

        ret = rdk_imu_accel_write(IMU_ACCEL_REG_ENABLE, &reg_value[0], 1);
        if(ret)return ret;
        ret = rdk_imu_accel_read(IMU_ACCEL_REG_ENABLE, &reg_value[1], 1);
        if(ret)return ret;
        if(reg_value[0] == reg_value[1])break;
    }

    if(reg_value[0] != reg_value[1]){
        RDK_DEBUG_PRINTF("Failed to enable accel.\n");
        return RDK_IMU_ACCEL_ENABLE_FAILED;
    }
    else RDK_DEBUG_PRINTF("Successfully enabled accel.\n");

    reg_value[0] = IMU_ACCEL_ACTIVATE_MODE;
    reg_value[1] = 0xFF;
    ret = rdk_imu_accel_write(IMU_ACCEL_REG_PWR_CONF, &reg_value[0], 1);
    if(ret)return ret;
    ret = rdk_imu_accel_read(IMU_ACCEL_REG_PWR_CONF, &reg_value[1], 1);
    if(ret)return ret;
    if(reg_value[0] != reg_value[1]){
        RDK_DEBUG_PRINTF("Failed to make accel enter activate mode.\n");
        return RDK_IMU_ACCEL_ENABLE_FAILED;
    }
    else RDK_DEBUG_PRINTF("Successfully switched accel into activate mode.\n");

    /* Enable gyro spi */
    reg_value[1] = 0xFF;
    for(int i=0; i<3; i++){
        ret = rdk_imu_gyro_read(IMU_GYRO_REG_CHIP_ID, &reg_value[1], 1);
        if(ret)return ret;
        RDK_DEBUG_PRINTF("read gyro chip id %02X\n", reg_value[1]);
        if(reg_value[1] == IMU_GYRO_CHIP_ID)break;
    }
    if(reg_value[1] != IMU_GYRO_CHIP_ID){
        RDK_DEBUG_PRINTF("Failed to enable gyro bus.\n");
        return RDK_IMU_GYRO_ENABLE_FAILED;
    }
    else RDK_DEBUG_PRINTF("Successfully enabled gyro bus.\n");

    /* Set the accel attribute  */
    /* bwp */
    ret = rdk_imu_accel_update(
        IMU_ACCEL_REG_CONF, 
        config.accel_bwp << IMU_ACCEL_BWP_OFFSET, 
        IMU_ACCEL_BWP_MASK <<IMU_ACCEL_BWP_OFFSET);
    if(ret)return ret;
    /* odr */
    ret = rdk_imu_accel_update(
        IMU_ACCEL_REG_CONF, 
        config.accel_odr << IMU_ACCEL_ODR_OFFSET, 
        IMU_ACCEL_ODR_MASK << IMU_ACCEL_ODR_OFFSET);
    if(ret)return ret;
    /* range */
    ret = rdk_imu_accel_update(
        IMU_ACCEL_REG_RANGE, 
        config.accel_range << IMU_ACCEL_RANGE_OFFSET, 
        IMU_ACCEL_RANGE_MASK << IMU_ACCEL_RANGE_OFFSET);
    if(ret)return ret;
    switch(config.accel_range){
        case RDK_IMU_ACCEL_3G: st->accel_data_scale = 9.80665 * 3.0 / 32768; st->accel_data_bais = 0.0; break;
        case RDK_IMU_ACCEL_6G: st->accel_data_scale = 9.80665 * 6.0 / 32768; st->accel_data_bais = 0.0; break;
        case RDK_IMU_ACCEL_12G: st->accel_data_scale = 9.80665 * 12.0 / 32768; st->accel_data_bais = 0.0; break;
        case RDK_IMU_ACCEL_24G: st->accel_data_scale = 9.80665 * 24.0 / 32768; st->accel_data_bais = 0.0; break;
    }
    RDK_DEBUG_PRINTF("Successfully set accel configures.\n");
    
    /* Set the gyro attribute  */
    /* range */
    ret = rdk_imu_gyro_update(
        IMU_GYRO_REG_RANGE, 
        config.gyro_range << IMU_GYRO_RANGE_OFFSET, 
        IMU_GYRO_RANGE_MASK << IMU_GYRO_RANGE_OFFSET);
    if(ret)return ret;
    switch(config.gyro_range){
        case RDK_IMU_GYRO_2000DPS: st->gyro_data_scale = M_PI / 180.0 * 2000.0 / 32768; st->gyro_data_bais = 0.0; break;
        case RDK_IMU_GYRO_1000DPS: st->gyro_data_scale = M_PI / 180.0 * 1000.0 / 32768; st->gyro_data_bais = 0.0; break;
        case RDK_IMU_GYRO_500DPS: st->gyro_data_scale = M_PI / 180.0 * 500.0 / 32768; st->gyro_data_bais = 0.0; break;
        case RDK_IMU_GYRO_250DPS: st->gyro_data_scale = M_PI / 180.0 * 250.0 / 32768; st->gyro_data_bais = 0.0; break;
        case RDK_IMU_GYRO_125DPS: st->gyro_data_scale = M_PI / 180.0 * 125.0 / 32768; st->gyro_data_bais = 0.0; break;
    }
    /* bandwidth */
    ret = rdk_imu_gyro_update(
        IMU_GYRO_REG_BANDWIDTH, 
        config.gyro_bandwidth << IMU_GYRO_BANDWIDTH_OFFSET, 
        IMU_GYRO_BANDWIDTH_MASK << IMU_GYRO_BANDWIDTH_OFFSET);
    if(ret)return ret;

    /* Set the interrupt configure */
    /* 开启 drdy 中断并映射到用户设定的 INT 引脚 */
    ret = rdk_imu_accel_update(
        IMU_ACCEL_REG_INT_MAP,
        config.accel_drdy_int == RDK_IMU_INT1 ? IMU_ACCEL_INT1_DRDY_BIT : IMU_ACCEL_INT2_DRDY_BIT,
        config.accel_drdy_int == RDK_IMU_INT1 ? IMU_ACCEL_INT1_DRDY_BIT : IMU_ACCEL_INT2_DRDY_BIT);
    if(ret)return ret;
    ret = rdk_imu_gyro_update(
        IMU_GYRO_REG_INT_CTRL,
        IMU_GYRO_DRDY_EN_BIT,
        IMU_GYRO_DRDY_EN_BIT);
    if(ret)return ret;
    ret = rdk_imu_gyro_update(
        IMU_GYRO_REG_INT_MAP,
        config.gyro_drdy_int == RDK_IMU_INT3 ? IMU_GYRO_INT3_DRDY_BIT : IMU_GYRO_INT4_DRDY_BIT,
        config.gyro_drdy_int == RDK_IMU_INT3 ? IMU_GYRO_INT3_DRDY_BIT : IMU_GYRO_INT4_DRDY_BIT);
    if(ret)return ret;

    /* 设定 accel INT 引脚的电气属性 */
    ret = rdk_imu_accel_update( /* PIN OUT */
        config.accel_drdy_int == RDK_IMU_INT1 ? IMU_ACCEL_REG_INT1_IOCFG : IMU_ACCEL_REG_INT2_IOCFG,
        IMU_ACCEL_INT_OUT_BIT,
        IMU_ACCEL_INT_OUT_BIT);
    if(ret)return ret;
    ret = rdk_imu_accel_update( /* PIN Push pull or Open drain */
        config.accel_drdy_int == RDK_IMU_INT1 ? IMU_ACCEL_REG_INT1_IOCFG : IMU_ACCEL_REG_INT2_IOCFG,
        config.accel_int_gpio_mode == RDK_IMU_OD_H || config.accel_int_gpio_mode == RDK_IMU_OD_L
        ? IMU_ACCEL_INT_OD_BIT : 0x00,
        IMU_ACCEL_INT_OD_BIT);
    if(ret)return ret;
    ret = rdk_imu_accel_update( /* PIN Activate H/L */
        config.accel_drdy_int == RDK_IMU_INT1 ? IMU_ACCEL_REG_INT1_IOCFG : IMU_ACCEL_REG_INT2_IOCFG,
        config.accel_int_gpio_mode == RDK_IMU_PP_H || config.accel_int_gpio_mode == RDK_IMU_OD_H
        ? IMU_ACCEL_INT_H_BIT : 0x00,
        IMU_ACCEL_INT_H_BIT);
    if(ret)return ret;

    /* 设定 gyro INT 引脚的电气属性 */
    ret = rdk_imu_gyro_update( /* PIN Push pull or Open drain */
        IMU_GYRO_REG_INT_IOCFG,
        config.gyro_int_gpio_mode == RDK_IMU_OD_H || config.gyro_int_gpio_mode == RDK_IMU_OD_L
        ? IMU_GYRO_INT3_OD_BIT | IMU_GYRO_INT4_OD_BIT : 0x00,
        config.gyro_drdy_int == RDK_IMU_INT3 ? IMU_GYRO_INT3_OD_BIT : IMU_GYRO_INT4_OD_BIT);
    if(ret)return ret;
    ret = rdk_imu_gyro_update( /* PIN Activate H/L */
        IMU_GYRO_REG_INT_IOCFG,
        config.gyro_int_gpio_mode == RDK_IMU_PP_H || config.gyro_int_gpio_mode == RDK_IMU_OD_H
        ? IMU_GYRO_INT3_H_BIT | IMU_GYRO_INT4_H_BIT : 0x00,
        config.gyro_drdy_int == RDK_IMU_INT3 ? IMU_GYRO_INT3_H_BIT : IMU_GYRO_INT4_H_BIT);
    if(ret)return ret;

    /* Configure the SoC-side interrupt GPIO */
    char chip_path[32];
    struct gpiod_line *line = NULL;

    /* --- Accel DRDY line --- */
    snprintf(chip_path, sizeof(chip_path), "/dev/gpiochip%u", config.accel_drdy_gpio_chip);
    st->accel_drdy_gpio_chip = gpiod_chip_open(chip_path);
    if(!st->accel_drdy_gpio_chip){
        RDK_DEBUG_PRINTF("Failed to open accel gpiochip %s\n", chip_path);
        return RDK_IMU_GPIO_INIT_FAILED;
    }
    line = gpiod_chip_get_line(st->accel_drdy_gpio_chip, config.accel_drdy_gpio_line);
    if(!line){
        RDK_DEBUG_PRINTF("Failed to get accel line %u\n", config.accel_drdy_gpio_line);
        gpiod_chip_close(st->accel_drdy_gpio_chip);
        return RDK_IMU_GPIO_INIT_FAILED;
    }
    /* 根据 GPIO 输出模式决定触发边沿：
     *   PP_H / OD_H → 中断时引脚变高 → 上升沿
     *   PP_L / OD_L → 中断时引脚变低 → 下降沿
     */
    if(config.accel_int_gpio_mode == RDK_IMU_PP_H || 
        config.accel_int_gpio_mode == RDK_IMU_OD_H){
        ret = gpiod_line_request_rising_edge_events(line, "accel-drdy");
    }
    else{
        ret = gpiod_line_request_falling_edge_events(line, "accel-drdy");
    }
    if(ret < 0){
        RDK_DEBUG_PRINTF("Failed to request accel IRQ.\n");
        gpiod_line_release(line);
        gpiod_chip_close(st->accel_drdy_gpio_chip);
        return RDK_IMU_GPIO_INIT_FAILED;
    }
    st->accel_drdy_gpio_line = line;

    /* --- Gyro DRDY line --- */
    /* 若与 accel 使用完全相同的 gpiochip 和 line，则跳过请求（共用） */
    if(config.accel_drdy_gpio_chip == config.gyro_drdy_gpio_chip &&
        config.accel_drdy_gpio_line == config.gyro_drdy_gpio_line){
        /* 共用同一线：直接复制 accel 的句柄 */
        st->gyro_drdy_gpio_chip = st->accel_drdy_gpio_chip;
        st->gyro_drdy_gpio_line = st->accel_drdy_gpio_line;
    }
    else{
        snprintf(chip_path, sizeof(chip_path), "/dev/gpiochip%u", config.gyro_drdy_gpio_chip);
        st->gyro_drdy_gpio_chip = gpiod_chip_open(chip_path);
        if(!st->gyro_drdy_gpio_chip){
            RDK_DEBUG_PRINTF("Failed to open gyro gpiochip %s\n", chip_path);
            gpiod_line_release(st->accel_drdy_gpio_line);
            gpiod_chip_close(st->accel_drdy_gpio_chip);
            return RDK_IMU_GPIO_INIT_FAILED;
        }

        line = gpiod_chip_get_line(st->gyro_drdy_gpio_chip, config.gyro_drdy_gpio_line);
        if(!line) {
            RDK_DEBUG_PRINTF("Failed to get gyro line %u\n", config.gyro_drdy_gpio_line);
            gpiod_chip_close(st->gyro_drdy_gpio_chip);
            gpiod_line_release(st->accel_drdy_gpio_line);
            gpiod_chip_close(st->accel_drdy_gpio_chip);
            return RDK_IMU_GPIO_INIT_FAILED;
        }

        if(config.gyro_int_gpio_mode == RDK_IMU_PP_H 
            || config.gyro_int_gpio_mode == RDK_IMU_OD_H){
            ret = gpiod_line_request_rising_edge_events(line, "gyro-drdy");
        }
        else{
            ret = gpiod_line_request_falling_edge_events(line, "gyro-drdy");
        }
        if(ret < 0){
            RDK_DEBUG_PRINTF("Failed to request gyro IRQ.\n");
            gpiod_line_release(line);
            gpiod_chip_close(st->gyro_drdy_gpio_chip);
            gpiod_line_release(st->accel_drdy_gpio_line);
            gpiod_chip_close(st->accel_drdy_gpio_chip);
            return RDK_IMU_GPIO_INIT_FAILED;
        }
        st->gyro_drdy_gpio_line = line;
    }
    RDK_DEBUG_PRINTF("GPIO interrupts configured successfully.\n");

    /* Software FIFO Initialization */
    ret = rdk_imu_fifo_init(&st->imu_fifo, config.fifo_length, config.fifo_mode);
    if(ret)return ret;

    /* irq */
    st->irq_priority = config.irq_priority;
    st->irq_thread_timeout_ns = config.irq_thread_timeout_ns;

    pthread_mutex_init(&st->mutex, NULL);

    return RDK_IMU_OK;
}

rdk_imu_err_t rdk_imu_device_deinit(
    rdk_imu_state_t* st)
{
    if(!st)return RDK_IMU_ERR_PARAM;

    if(st->enable)return RDK_IMU_DEVICE_BUSY;

    /* 释放 GPIO 线（注意共用线不要重复释放） */
    if(st->accel_drdy_gpio_line){
        gpiod_line_release(st->accel_drdy_gpio_line);
        st->accel_drdy_gpio_line = NULL;
    }

    if(st->gyro_drdy_gpio_line && st->gyro_drdy_gpio_line != st->accel_drdy_gpio_line){
        gpiod_line_release(st->gyro_drdy_gpio_line);
        st->gyro_drdy_gpio_line = NULL;
    }

    /* 关闭 GPIO 芯片 */
    if(st->accel_drdy_gpio_chip){
        gpiod_chip_close(st->accel_drdy_gpio_chip);
        st->accel_drdy_gpio_chip = NULL;
    }

    if(st->gyro_drdy_gpio_chip && st->gyro_drdy_gpio_chip != st->accel_drdy_gpio_chip){
        gpiod_chip_close(st->gyro_drdy_gpio_chip);
        st->gyro_drdy_gpio_chip = NULL;
    }

    /* 销毁互斥锁 */
    pthread_mutex_destroy(&st->mutex);

    /* 销毁 FIFO（若尚未释放） */
    rdk_imu_fifo_deinit(&st->imu_fifo);

    return RDK_IMU_OK;
}


rdk_imu_err_t rdk_imu_enable(
    rdk_imu_state_t* st)
{
    if(!st)return RDK_IMU_ERR_PARAM;

    /* 防止重复 enable */
    pthread_mutex_lock(&st->mutex); /* 消除竞态条件 */
    if(st->enable){
        pthread_mutex_unlock(&st->mutex);
        return RDK_IMU_DEVICE_BUSY;
    }
    pthread_mutex_unlock(&st->mutex);
    
    pthread_attr_t attr;
    struct sched_param param;
    int ret;

    /* 初始化线程属性，显式设置调度策略（不从父线程继承） */
    pthread_attr_init(&attr);
    if(pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) != 0){
        RDK_DEBUG_PRINTF("Failed to set explicit scheduling\n");
        pthread_attr_destroy(&attr);
        return RDK_IMU_THREAD_CREATE_FAILED;
    }

    /* 设置调度策略为 SCHED_FIFO */
    if(pthread_attr_setschedpolicy(&attr, SCHED_FIFO) != 0){
        RDK_DEBUG_PRINTF("Failed to set SCHED_FIFO policy\n");
        pthread_attr_destroy(&attr);
        return RDK_IMU_THREAD_CREATE_FAILED;
    }

    /* 解析优先级：
     *   -1 → 自动使用系统允许的最高实时优先级
     *   1~99 → 直接使用指定的优先级（需经范围限制） */
    int priority;
    if(st->irq_priority == -1){
        priority = sched_get_priority_max(SCHED_FIFO);
    }
    else{
        priority = st->irq_priority;
        /* 钳位到合法范围内 */
        int min_prio = sched_get_priority_min(SCHED_FIFO);
        int max_prio = sched_get_priority_max(SCHED_FIFO);
        if(priority < min_prio) priority = min_prio;
        if(priority > max_prio) priority = max_prio;
    }
    param.sched_priority = priority;
    if(pthread_attr_setschedparam(&attr, &param) != 0){
        RDK_DEBUG_PRINTF("Failed to set scheduler priority %d\n", priority);
        pthread_attr_destroy(&attr);
        return RDK_IMU_THREAD_CREATE_FAILED;
    }
    /* 设置线程退出标志，线程函数应在循环中检查这些标志 */
    st->accel_running = 1;
    st->gyro_running  = 1;

    /* 创建 accel 中断处理线程 */
    ret = pthread_create(&st->accel_thread, &attr, rdk_imu_accel_irq_thread, st);
    if(ret){
        RDK_DEBUG_PRINTF("Failed to create accel thread (error %d)\n", ret);
        st->accel_running = 0;
        st->gyro_running = 0;
        pthread_attr_destroy(&attr);
        return RDK_IMU_THREAD_CREATE_FAILED;
    }
    /* 创建 gyro 中断处理线程（复用线程属性） */
    ret = pthread_create(&st->gyro_thread, &attr, rdk_imu_gyro_irq_thread, st);
    if(ret){
        RDK_DEBUG_PRINTF("Failed to create gyro thread (error %d)\n", ret);
        /* 已创建的加速度线程需要安全终止 */
        st->accel_running = 0;
        st->gyro_running = 0;
        pthread_join(st->accel_thread, NULL);
        pthread_attr_destroy(&attr);
        return RDK_IMU_THREAD_CREATE_FAILED;
    }

    /* 销毁线程属性对象 */
    pthread_attr_destroy(&attr);

    /* 标记设备已使能 */
    st->enable = 1;

    /* 初始化插值窗口 */
    memset(st->fuse_win, 0, sizeof(st->fuse_win));

    RDK_DEBUG_PRINTF("IMU interrupt threads started (SCHED_FIFO prio=%d)\n", priority);

    return RDK_IMU_OK;
}

rdk_imu_err_t rdk_imu_disable(
    rdk_imu_state_t* st)
{
    if(!st)return RDK_IMU_ERR_PARAM;

    if(!st->enable)return RDK_IMU_OK;

    st->accel_running = 0;
    st->gyro_running  = 0;

    pthread_join(st->accel_thread, NULL);
    pthread_join(st->gyro_thread, NULL);

    st->enable = 0;

    return RDK_IMU_OK;
}

rdk_imu_err_t rdk_imu_fifo_available(rdk_imu_state_t *st,
                                     uint32_t *count)
{
    if(!st || !count)return RDK_IMU_ERR_PARAM;

    pthread_mutex_lock(&st->mutex);
    *count = rdk_imu_fifo_count(&st->imu_fifo);
    pthread_mutex_unlock(&st->mutex);

    return RDK_IMU_OK;
}

/* ======================== 独立读取 ======================== */
rdk_imu_err_t rdk_imu_read_indep(
    rdk_imu_state_t *st,
    rdk_imu_6_axis_data_t *data,
    uint32_t *count)
{
    if(!st || !data)return RDK_IMU_ERR_PARAM;

    pthread_mutex_lock(&st->mutex);

    if(rdk_imu_fifo_is_empty(&st->imu_fifo)){
        pthread_mutex_unlock(&st->mutex);
        return RDK_IMU_FIFO_EMPTY;
    }

    rdk_imu_fifo_pop(&st->imu_fifo, data);

    if(count)*count = rdk_imu_fifo_count(&st->imu_fifo);

    pthread_mutex_unlock(&st->mutex);

    return RDK_IMU_OK;
}

rdk_imu_err_t rdk_imu_read_fused(
    rdk_imu_state_t *st,
    rdk_imu_6_axis_data_t *data,
    rdk_imu_device_t fuse_by,
    uint64_t max_age_ns)
{
    if(!st || !data)return RDK_IMU_ERR_PARAM;
    if(fuse_by != RDK_IMU_ACCEL && fuse_by != RDK_IMU_GYRO)return RDK_IMU_ERR_PARAM;

    rdk_imu_err_t ret;

    while(1){
        /* 等待可读fifo */
        pthread_mutex_lock(&st->mutex);
        while(rdk_imu_fifo_count(&st->imu_fifo) < 1){
            pthread_mutex_unlock(&st->mutex);
            sleep_ms(10);
            pthread_mutex_lock(&st->mutex);
        }
        // pthread_mutex_unlock(&st->mutex);

        // pthread_mutex_lock(&st->mutex);
        ret = rdk_imu_fifo_pop(&st->imu_fifo, &st->fuse_win[2]);
        pthread_mutex_unlock(&st->mutex);
        if(ret)continue;

        /* 窗口还未初始化完全 */
        if(st->fuse_win[0].accel.valid == st->fuse_win[0].gyro.valid){
            memmove(&st->fuse_win[0], &st->fuse_win[1], 2 * sizeof(st->fuse_win[0]));
            continue;
        }
        /* 时间跨度过大 */
        if(rdk_imu_data_timestamp_ns(st->fuse_win[2]) - rdk_imu_data_timestamp_ns(st->fuse_win[0]) > max_age_ns){
            memmove(&st->fuse_win[0], &st->fuse_win[1], 2 * sizeof(st->fuse_win[0]));
            continue;
        }
        /* 合并重叠项 */
        if(st->fuse_win[1].accel.valid == st->fuse_win[2].accel.valid){
            memcpy(&st->fuse_win[1], &st->fuse_win[2], sizeof(st->fuse_win[0]));
            continue;
        }
        /* 可插值 */
        if(st->fuse_win[0].accel.valid != st->fuse_win[1].accel.valid &&
            st->fuse_win[0].accel.valid == st->fuse_win[2].accel.valid)
        {
            if(st->fuse_win[1].accel.valid==1 && fuse_by==RDK_IMU_ACCEL)break;
            if(st->fuse_win[1].gyro.valid==1 && fuse_by==RDK_IMU_GYRO)break;
        }
        /*正常滑动窗口*/
        memmove(&st->fuse_win[0], &st->fuse_win[1], 2 * sizeof(st->fuse_win[0]));
    }
    
    /* 插值计算 */
    memcpy(data, &st->fuse_win[1], sizeof(*data));
    if(fuse_by==RDK_IMU_ACCEL){
        interpolate_3axis(
            &st->fuse_win[0].gyro, 
            &st->fuse_win[2].gyro, 
            data->accel.timestamp_ns,
            &data->gyro);
    }
    else{
        interpolate_3axis(
            &st->fuse_win[0].accel, 
            &st->fuse_win[2].accel, 
            data->gyro.timestamp_ns,
            &data->accel);
    }
    
    /* 滑动窗口 */
    memmove(&st->fuse_win[0], &st->fuse_win[1], 2 * sizeof(st->fuse_win[0]));

    return RDK_IMU_OK;
}
