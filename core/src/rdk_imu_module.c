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

#include "rdk_imu_module.h"

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

static void sleep_ms(int milliseconds) {
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

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
    if(st->imu_fifo.buffer)free(st->imu_fifo.buffer);

    if(st)free(st);

    return RDK_IMU_OK;
}

rdk_imu_err_t rdk_imu_device_init(
    rdk_imu_state_t* st,
    rdk_imu_config_t config)
{
    rdk_imu_err_t ret;

    uint8_t reg_value[2];

    /* Reset IMU */
    ret = rdk_imu_accel_write(BMI088_ACCEL_SOFTRESET, BMI088_ACCEL_SOFTRESET_CMD, 1);
    if(ret)return ret;
    sleep_ms(BMI088_ACCEL_RESET_SLEEP_MS);

    rdk_imu_gyro_write(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_CMD, 1);
    if(ret)return ret;
    sleep_ms(BMI088_GYRO_RESET_SLEEP_MS);

    /* Enable gyro spi */
    if(st->)
    for(int i=0; i<3; i++){
        rdk_imu_gyro_read(BMI088_ACCEL_SOFTRESET)
    }

    /* Enable accel pwr */

    /* Set the accel attribute  */
    
    /* Set the gyro attribute  */

    /* Set the interrupt configure */

}

int main(){
    rdk_imu_err_t ret = RDK_IMU_OK;

    rdk_imu_state_t *st = rdk_imu_create_default();

    rdk_imu_bus_info_t bus_info;
    bus_info.interface = RDK_IMU_AUTO;

    ret = rdk_imu_bus_init(st, bus_info);

    uint8_t buf[5];
    // ret = rdk_imu_gyro_read(0x00, &buf[0], 5);

    return ret;
}

