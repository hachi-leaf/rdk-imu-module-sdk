/*
 * Copyright (c) 2026 Leaf. D-Robotics.
 * SPDX-License-Identifier: MIT
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <dirent.h>

#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/spi/spidev.h>

#include "rdk_imu_module.h"

/* ============================== I2C 底层实现 ============================== */
static rdk_imu_err_t rdk_imu_i2c_write(
    int fd, 
    uint8_t addr, 
    uint8_t reg, 
    const uint8_t *data, 
    uint8_t len) 
{
    if(len == 0)return RDK_IMU_ERR_PARAM;

    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(buf + 1, data, len);

    struct i2c_msg msg = {
        .addr = addr,
        .flags = 0, // 写标志
        .len = len + 1,
        .buf = buf,
    };

    struct i2c_rdwr_ioctl_data ioctl_data = {
        .msgs  = &msg,
        .nmsgs = 1,
    };

    if(ioctl(fd, I2C_RDWR, &ioctl_data) < 0) {
        // perror("i2c_write failed");
        return RDK_IMU_I2C_WRITE_ERR;
    }
    return RDK_IMU_OK;
}

static rdk_imu_err_t rdk_imu_i2c_read(
    int fd,
    uint8_t addr,
    uint8_t reg,
    uint8_t *data,
    uint8_t len)
{
    if(len == 0)return RDK_IMU_ERR_PARAM;

    struct i2c_msg msgs[2] = {
        {
            .addr = addr,
            .flags = 0, // 先写寄存器地址
            .len = 1, // 8 bit 地址
            .buf = &reg,
        },
        {
            .addr = addr,
            .flags = I2C_M_RD, // 再读数据
            .len = len,
            .buf = data,
        },
    };

    struct i2c_rdwr_ioctl_data ioctl_data = {
        .msgs  = msgs,
        .nmsgs = 2,
    };

    if(ioctl(fd, I2C_RDWR, &ioctl_data) < 0){
        // perror("i2c_read failed");
        return RDK_IMU_I2C_READ_ERR;
    }
    return RDK_IMU_OK;
}

static rdk_imu_err_t rdk_imu_i2c_update(
    int fd,
    uint8_t addr,
    uint8_t reg,
    uint8_t data,
    uint8_t mask)
{
    if(mask == 0)return RDK_IMU_OK;

    uint8_t cur_val;

    rdk_imu_err_t ret = rdk_imu_i2c_read(fd, addr, reg, &cur_val, 1);
    if(ret)return ret;

    uint8_t new_val = (cur_val & ~mask) | (data & mask);
    return rdk_imu_i2c_write(fd, addr, reg, &new_val, 1);
}

/* ============================== SPI 底层实现 ============================== */
static rdk_imu_err_t rdk_imu_spi_write(
    int fd, 
    uint32_t speed_hz, 
    uint8_t reg,
    uint8_t dummy_before,
    const uint8_t *data, 
    uint8_t len)
{
    if(len == 0)return RDK_IMU_ERR_PARAM;

    uint16_t total = 1 + dummy_before + len;
    uint8_t tx_buf[total];
    tx_buf[0] = reg;

    if(dummy_before > 0)memset(tx_buf + 1, 0, dummy_before);
    memcpy(tx_buf + 1 + dummy_before, data, len);

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .rx_buf = 0,
        .len = total,
        .speed_hz = speed_hz,
        .delay_usecs = 10,
        .bits_per_word = 8,
        .cs_change = 0,
    };

    if(ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        // perror("spi_write failed");
        return RDK_IMU_SPI_WRITE_ERR;
    }
    return RDK_IMU_OK;
}

static rdk_imu_err_t rdk_imu_spi_read(
    int fd, 
    uint32_t speed_hz, 
    uint8_t reg,
    uint8_t dummy_before,
    uint8_t *data, 
    uint8_t len) // BMI088 在 SPI 读 accel 时有特殊dummy，这里需要设置额外参数
{
    if (len == 0) return RDK_IMU_ERR_PARAM;

    uint16_t total = 1 + dummy_before + len;    // 总传输字节数
    uint8_t tx_buf[total];
    tx_buf[0] = reg | 0x80;
    memset(tx_buf + 1, 0, dummy_before + len); // 填充 dummy 和读时钟

    uint8_t rx_buf[total];

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .rx_buf = (unsigned long)rx_buf,
        .len = total,
        .speed_hz = speed_hz,
        .delay_usecs = 10,
        .bits_per_word = 8,
        .cs_change = 0,
    };

    if(ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        // perror("spi_read failed");
        return RDK_IMU_SPI_READ_ERR;
    }

    // 跳过命令回显 + dummy 字节，取出有效数据
    memcpy(data, rx_buf + 1 + dummy_before, len);
    return RDK_IMU_OK;
}

static rdk_imu_err_t rdk_imu_spi_update(
    int fd, 
    uint32_t speed_hz, 
    uint8_t reg,
    uint8_t rd_dummy_before,
    uint8_t wr_dummy_before,
    uint8_t data, 
    uint8_t mask)
{
    if (mask == 0) return RDK_IMU_OK;

    uint8_t cur_val;

    rdk_imu_err_t ret = rdk_imu_spi_read(fd, speed_hz, reg, rd_dummy_before, &cur_val, 1);
    if(ret)return ret;

    uint8_t new_val = (cur_val & ~mask) | (data & mask);
    return rdk_imu_spi_write(fd, speed_hz, reg, wr_dummy_before, &new_val, 1);
}

/* ============================== Ops 适配层（将 bus 接口映射到底层实现） ============================== */
static rdk_imu_err_t rdk_imu_i2c_write_op(
    rdk_imu_bus_t *bus, 
    uint8_t reg,
    const uint8_t *data, 
    uint8_t len)
{
    return rdk_imu_i2c_write(bus->fd, bus->addr, reg, data, len);
}

static rdk_imu_err_t rdk_imu_i2c_read_op(
    rdk_imu_bus_t *bus, 
    uint8_t reg,
    uint8_t *data, 
    uint8_t len)
{
    return rdk_imu_i2c_read(bus->fd, bus->addr, reg, data, len);
}

static rdk_imu_err_t rdk_imu_i2c_update_op(
    rdk_imu_bus_t *bus, 
    uint8_t reg,
    uint8_t data, 
    uint8_t mask)
{
    return rdk_imu_i2c_update(bus->fd, bus->addr, reg, data, mask);
}

static const rdk_imu_bus_ops_t i2c_ops = {
    .write  = rdk_imu_i2c_write_op,
    .read   = rdk_imu_i2c_read_op,
    .update = rdk_imu_i2c_update_op,
};

static rdk_imu_err_t rdk_imu_spi_read_op(
    rdk_imu_bus_t *bus, 
    uint8_t reg,
    uint8_t *data, 
    uint8_t len)
{
    return rdk_imu_spi_read(bus->fd, bus->speed_hz, reg, bus->rd_dummy, data, len);
}

static rdk_imu_err_t rdk_imu_spi_write_op(
    rdk_imu_bus_t *bus, 
    uint8_t reg,
    const uint8_t *data, 
    uint8_t len)
{
    return rdk_imu_spi_write(bus->fd, bus->speed_hz, reg, bus->wr_dummy, data, len);
}

static rdk_imu_err_t rdk_imu_spi_update_op(
    rdk_imu_bus_t *bus, 
    uint8_t reg,
    uint8_t data, 
    uint8_t mask) 
{
    return rdk_imu_spi_update(bus->fd, bus->speed_hz, reg, bus->rd_dummy, bus->wr_dummy, data, mask);
}

static const rdk_imu_bus_ops_t spi_ops = {
    .read   = rdk_imu_spi_read_op,
    .write  = rdk_imu_spi_write_op,
    .update = rdk_imu_spi_update_op,
};

/* ============================== 总线工具函数 ============================== */
/**
 * @brief 列出 /dev 下所有可用的 I2C 总线号
 * @param bus_numbers  输出数组（调用方保证至少 256 字节）
 * @param count        输出参数，实际填充的个数（最多 256）
 */
rdk_imu_err_t list_i2c_bus(
    uint8_t *bus_numbers, 
    uint8_t *count)
{
    if(!bus_numbers || !count)return RDK_IMU_ERR_PARAM;
    *count = 0;

    DIR *dir = opendir("/dev");
    if(!dir)return RDK_IMU_OPEN_DIR_ERR;

    struct dirent *entry;
    while((entry = readdir(dir)) != NULL) {
        if(strncmp(entry->d_name, "i2c-", 4) != 0)continue;

        int bus = atoi(entry->d_name + 4);
        if(bus < 0 || bus > 255)continue;

        uint8_t num = (uint8_t)bus;

        // 插入排序 + 去重
        int pos = 0;
        while(pos < *count && bus_numbers[pos] < num)pos++;
        if(pos < *count && bus_numbers[pos] == num)continue;  // 重复，跳过

        if(*count < 256){
            for(int i = *count; i > pos; i--)bus_numbers[i] = bus_numbers[i - 1];
            bus_numbers[pos] = num;
            (*count)++;
        }
    }
    closedir(dir);
    return RDK_IMU_OK;
}

/**
 * @brief 列出 /dev 下所有可用的 SPI 总线号（spidevX.Y 中的 X）
 * @param devices  输出数组（调用方保证至少 256 字节）
 * @param count    输出参数，实际填充的个数
 */
rdk_imu_err_t list_spi_bus(
    uint8_t *devices, 
    uint8_t *count)
{
    if(!devices || !count)return RDK_IMU_ERR_PARAM;
    *count = 0;

    DIR *dir = opendir("/dev");
    if(!dir)return RDK_IMU_OPEN_DIR_ERR;

    struct dirent *entry;

    while((entry = readdir(dir)) != NULL){
        if(strncmp(entry->d_name, "spidev", 6) != 0)continue;

        const char *p = entry->d_name + 6;
        char *endptr;
        long bus = strtol(p, &endptr, 10);
        if(endptr == p || *endptr != '.' || bus < 0 || bus > 255)continue;

        uint8_t new_bus = (uint8_t)bus;

        // 检查是否已存在
        uint8_t i;
        for(i = 0; i < *count; i++){
            if (devices[i] == new_bus) break;
        }
        if(i == *count && *count < 256){   // 未找到且未满
            devices[*count] = new_bus;
            (*count)++;
        }
    }
    closedir(dir);

    return RDK_IMU_OK;
}

/**
 * @brief 列出指定 SPI 总线上所有的片选号（spidevX.Y 中的 Y）
 * @param bus        SPI 总线号
 * @param cs_array   输出数组（调用方保证至少 256 字节）
 * @param count      输出参数，实际填充的个数
 */
rdk_imu_err_t list_spi_bus_cs(
    uint8_t bus, 
    uint8_t *cs_array, 
    uint8_t *count)
{
    if(!cs_array || !count)return RDK_IMU_ERR_PARAM;
    *count = 0;

    DIR *dir = opendir("/dev");
    if(!dir)return RDK_IMU_OPEN_DIR_ERR;

    char prefix[16];
    snprintf(prefix, sizeof(prefix), "spidev%u.", bus);
    size_t len = strlen(prefix);

    struct dirent *entry;
    while((entry = readdir(dir)) != NULL){
        if(strncmp(entry->d_name, prefix, len) != 0)continue;

        const char *cs_str = entry->d_name + len;
        char *endptr;
        long cs = strtol(cs_str, &endptr, 10);
        if(endptr == cs_str || *endptr != '\0' || cs < 0 || cs > 255)
            continue;

        if (*count < 256) {
            cs_array[*count] = (uint8_t)cs;
            (*count)++;
        }
    }
    closedir(dir);

    return RDK_IMU_OK;
}

/* ============================== 总线初始化实现 ============================== */
/**
 * @brief 通过 I2C 读取并验证单个设备的芯片 ID
 * @param fd 已打开的 I2C 总线文件描述符
 * @param addr 设备地址
 * @param chip_reg 芯片 ID 寄存器地址
 * @param expected 期望的芯片 ID 值
 */
static rdk_imu_err_t i2c_check_chip_id(
    int fd, 
    uint8_t addr, 
    uint8_t chip_reg, 
    uint8_t expected)
{
    uint8_t id;

    rdk_imu_err_t ret = rdk_imu_i2c_read(fd, addr, chip_reg, &id, 1);

    if(ret)return ret;
    if(id != expected)return RDK_IMU_CHIP_ID_ERR;   // 需定义该错误码

    return RDK_IMU_OK;
}

/**
 * @brief 通过 SPI 读取并验证单个设备的芯片 ID
 * @param fd 已打开的 SPI 设备文件描述符
 * @param speed_hz SPI 时钟速率
 * @param reg 芯片 ID 寄存器地址（不含读写标志）
 * @param dummy SPI 读额外 dummy 字节数
 * @param expected 期望的芯片 ID 值
 */
static rdk_imu_err_t spi_check_chip_id(
    int fd, 
    uint32_t speed_hz, 
    uint8_t reg, 
    uint8_t dummy, 
    uint8_t expected)
{
    uint8_t id;

    rdk_imu_err_t ret = rdk_imu_spi_read(fd, speed_hz, reg, dummy, &id, 1);

    // printf("spi check: reg %2X, id %02X\n", reg, id);

    if(ret != RDK_IMU_OK)return ret;
    if(id != expected)return RDK_IMU_CHIP_ID_ERR;

    return RDK_IMU_OK;
}

/**
 * @brief 自动扫描 I2C/SPI 总线，寻找 BMI088 加速度计和陀螺仪
 * @param bus_info 总线信息指针，扫描结果写入此处
 * @return RDK_IMU_OK 找到完整设备，否则返回错误码
 */
static rdk_imu_err_t rdk_imu_bus_auto_scan(
    rdk_imu_bus_info_t *bus_info)
{
    uint8_t i2c_bus_list[256];
    uint8_t spi_bus_list[256];
    uint8_t spi_bus_cs_list[256];

    uint8_t i2c_bus_num;
    uint8_t spi_bus_num;
    uint8_t spi_bus_cs_num;

    uint8_t reg_value[2];
    int fd = -1;
    char dev_path[32];
    uint8_t accel_found = 0, gyro_found = 0;
    rdk_imu_err_t ret;

#ifdef RDK_IMU_DEBUG
    printf("Starting I2C search...\n");
#endif

    /* 查看可用的 I2C 总线 */
    ret = list_i2c_bus(&i2c_bus_list[0], &i2c_bus_num);
    if(ret)return ret;

    if(i2c_bus_num == 0){
#ifdef RDK_IMU_DEBUG
        printf("No available i2c bus\n");
#endif
    }

    /* 遍历 I2C 总线 */
    for(uint16_t i2c_bus = 0; i2c_bus < /*i2c_bus_num*/5; i2c_bus++){
    /* 反向扫描，避免大总线号的虚拟总线造成超长延时 */
    // for(uint16_t i2c_bus = i2c_bus_num-1; i2c_bus < i2c_bus_num; i2c_bus--){
        snprintf(dev_path, sizeof(dev_path), "/dev/i2c-%u", i2c_bus_list[i2c_bus]);
        fd = open(dev_path, O_RDWR);
        if(fd < 0){
#ifdef RDK_IMU_DEBUG
            printf("Failed to open %s, skip.\n", dev_path);
#endif
            continue;
        }
#ifdef RDK_IMU_DEBUG
        else{
            printf("Open %s.\n", dev_path);
        }
#endif

        for(uint8_t addr = 0x08; addr < 0x77; addr++){
            if(!accel_found){
                ret = i2c_check_chip_id(fd, addr, BMI088_ACCEL_CHIP_REG, BMI088_ACCEL_CHIP_ID);
                if(ret == RDK_IMU_OK){
                    bus_info->interface = RDK_IMU_I2C;
                    bus_info->bus.i2c.accel.bus = i2c_bus_list[i2c_bus];
                    bus_info->bus.i2c.accel.addr = addr;
                    accel_found = 1;
#ifdef RDK_IMU_DEBUG
                    printf("Found accel on I2C bus %u addr 0x%02X\n", i2c_bus_list[i2c_bus], addr);
#endif
                }
            }
            if(!gyro_found){
                ret = i2c_check_chip_id(fd, addr, BMI088_GYRO_CHIP_REG, BMI088_GYRO_CHIP_ID);
                if(ret == RDK_IMU_OK){
                    bus_info->interface = RDK_IMU_I2C;
                    bus_info->bus.i2c.gyro.bus = i2c_bus_list[i2c_bus];
                    bus_info->bus.i2c.gyro.addr = addr;
                    gyro_found = 1;
#ifdef RDK_IMU_DEBUG
                    printf("Found gyro on I2C bus %u addr 0x%02X\n", i2c_bus_list[i2c_bus], addr);
#endif
                }
            }
            if(accel_found && gyro_found)break;
        }
        close(fd);
        if(accel_found && gyro_found)break;
    }

    /* 只找到一个设备则清空状态 */
    if(accel_found ^ gyro_found){
        accel_found = 0;
        gyro_found = 0;
        bus_info->interface = RDK_IMU_AUTO;
    }

    /* 如果 I2C 没找全，尝试 SPI */
    if(!accel_found || !gyro_found){
#ifdef RDK_IMU_DEBUG
        printf("Full I2C device not found, starting SPI search...\n");
#endif
        ret = list_spi_bus(&spi_bus_list[0], &spi_bus_num);
        if(ret)return ret;

        if(spi_bus_num == 0){
#ifdef RDK_IMU_DEBUG
            printf("No available spi bus\n");
#endif
        }

        for(uint16_t spi_bus = 0; spi_bus < spi_bus_num; spi_bus++){
            ret = list_spi_bus_cs(spi_bus_list[spi_bus], &spi_bus_cs_list[0], &spi_bus_cs_num);
            if(ret)return ret;

            for(uint16_t spi_bus_cs = 0; spi_bus_cs < spi_bus_cs_num; spi_bus_cs++){
                snprintf(dev_path, sizeof(dev_path), "/dev/spidev%u.%u", spi_bus_list[spi_bus], spi_bus_cs_list[spi_bus_cs]);
                fd = open(dev_path, O_RDWR);

                if(fd < 0){
#ifdef RDK_IMU_DEBUG
                    printf("Failed to open %s, skip.\n", dev_path);
#endif
                    continue;
                }
#ifdef RDK_IMU_DEBUG
                else{
                    printf("Open %s.\n", dev_path);
                }
#endif
                uint8_t mode = RDK_IMU_SPID_MODE;
                ioctl(fd, SPI_IOC_WR_MODE, &mode);

                if(!accel_found){
                    ret = spi_check_chip_id(fd, 
                        RDK_IMU_SPI_SCAN_SPEED_HZ, 
                        BMI088_ACCEL_CHIP_REG, 
                        BMI088_SPI_ACCEL_RD_DUMMY, 
                        BMI088_ACCEL_CHIP_ID);
                        
                    if(ret == RDK_IMU_OK){
                        bus_info->interface = RDK_IMU_SPI;
                        bus_info->bus.spi.accel.bus = spi_bus_list[spi_bus];
                        bus_info->bus.spi.accel.cs  = spi_bus_cs_list[spi_bus_cs];
                        accel_found = 1;
#ifdef RDK_IMU_DEBUG
                        printf("Found accel on %s\n", dev_path);
#endif
                    }
                }

                if(!gyro_found){
                    ret = spi_check_chip_id(fd, 
                        RDK_IMU_SPI_SCAN_SPEED_HZ, 
                        BMI088_GYRO_CHIP_REG, 
                        BMI088_SPI_GYRO_RD_DUMMY, 
                        BMI088_GYRO_CHIP_ID);

                    if(ret == RDK_IMU_OK){
                        bus_info->interface = RDK_IMU_SPI;
                        bus_info->bus.spi.gyro.bus = spi_bus_list[spi_bus];
                        bus_info->bus.spi.gyro.cs  = spi_bus_cs_list[spi_bus_cs];
                        gyro_found = 1;
#ifdef RDK_IMU_DEBUG
                        printf("Found gyro on %s\n", dev_path);
#endif
                    }
                }

                close(fd);
                if(accel_found && gyro_found)break;
            }
            if(accel_found && gyro_found)break;
        }

        /* SPI 也只找到一个则清空状态 */
        if(accel_found ^ gyro_found){
            accel_found = 0;
            gyro_found = 0;
            bus_info->interface = RDK_IMU_AUTO;
        }
    }

    /* 检查最终是否找到完整设备 */
    if(!accel_found || !gyro_found){
#ifdef RDK_IMU_DEBUG
        printf("Full device not found on any bus.\n");
#endif
        return RDK_IMU_NO_DEV_ERR;
    }

    return RDK_IMU_OK;
}

rdk_imu_err_t rdk_imu_bus_init(
    rdk_imu_state_t* st,
    rdk_imu_bus_info_t bus_info)
{
    rdk_imu_err_t ret = RDK_IMU_OK;
    
    char dev_path[32];

    int fd_accel, fd_gyro;

    /* 自动扫描 */
    if(bus_info.interface == RDK_IMU_AUTO) {
        ret = rdk_imu_bus_auto_scan(&bus_info);
        if(ret)return ret;

        // 打印找到的设备信息
#ifdef RDK_IMU_DEBUG
        if(bus_info.interface == RDK_IMU_I2C){
            printf("IMU found on I2C:\n");
            printf("  Accel: bus=%u addr=0x%02X\n",
                   bus_info.bus.i2c.accel.bus,
                   bus_info.bus.i2c.accel.addr);
            printf("  Gyro:  bus=%u addr=0x%02X\n",
                   bus_info.bus.i2c.gyro.bus,
                   bus_info.bus.i2c.gyro.addr);
        }
        else if(bus_info.interface == RDK_IMU_SPI){
            printf("IMU found on SPI:\n");
            printf("  Accel: bus=%u CS=%u\n",
                   bus_info.bus.spi.accel.bus,
                   bus_info.bus.spi.accel.cs);
            printf("  Gyro:  bus=%u CS=%u\n",
                   bus_info.bus.spi.gyro.bus,
                   bus_info.bus.spi.gyro.cs);
        }
#endif
    }
    /* 不使用自动扫描，检查用户提供的接口内容是否为BMI088芯片 */
    else{
        if(bus_info.interface == RDK_IMU_I2C){
            snprintf(dev_path, sizeof(dev_path), "/dev/i2c-%u", bus_info.bus.i2c.accel.bus);
            fd_accel = open(dev_path, O_RDWR);
            if(fd_accel<0)return RDK_IMU_OPEN_FILE_ERR;

            snprintf(dev_path, sizeof(dev_path), "/dev/i2c-%u", bus_info.bus.i2c.gyro.bus);
            fd_gyro = open(dev_path, O_RDWR);
            if(fd_gyro<0)return RDK_IMU_OPEN_FILE_ERR;

            ret = i2c_check_chip_id(fd_accel, bus_info.bus.i2c.accel.addr, BMI088_ACCEL_CHIP_REG, BMI088_ACCEL_CHIP_ID)
                | i2c_check_chip_id(fd_gyro, bus_info.bus.i2c.gyro.addr, BMI088_GYRO_CHIP_REG, BMI088_GYRO_CHIP_ID);

            close(fd_accel);
            close(fd_gyro);
        }
        if(bus_info.interface == RDK_IMU_SPI){
            snprintf(dev_path, sizeof(dev_path), "/dev/spidev%u.%u", bus_info.bus.spi.accel.bus, bus_info.bus.spi.accel.cs);
            fd_accel = open(dev_path, O_RDWR);
            if(fd_accel<0)return RDK_IMU_OPEN_FILE_ERR;

            snprintf(dev_path, sizeof(dev_path), "/dev/spidev%u.%u", bus_info.bus.spi.gyro.bus, bus_info.bus.spi.gyro.cs);
            fd_gyro = open(dev_path, O_RDWR);
            if(fd_gyro<0)return RDK_IMU_OPEN_FILE_ERR;

            ret = spi_check_chip_id(fd_accel, 
                    bus_info.bus.spi.accel.speed_hz, 
                    BMI088_ACCEL_CHIP_REG, 
                    BMI088_SPI_ACCEL_RD_DUMMY, 
                    BMI088_ACCEL_CHIP_ID)
                | spi_check_chip_id(fd_gyro, 
                    bus_info.bus.spi.gyro.speed_hz, 
                    BMI088_GYRO_CHIP_REG, 
                    BMI088_SPI_ACCEL_RD_DUMMY, 
                    BMI088_GYRO_CHIP_ID);

            close(fd_accel);
            close(fd_gyro);
        }
        if(ret)return RDK_IMU_CHIP_ID_ERR;
    }
    /* 至此 bus_info 确认可靠 */
    /* 将 ops 绑定到 imu_state 中 */
    /* 根据最终确定的接口创建总线对象 */
    if(bus_info.interface == RDK_IMU_I2C){
        snprintf(dev_path, sizeof(dev_path), "/dev/i2c-%u", bus_info.bus.i2c.accel.bus);
        st->accel_bus.fd = open(dev_path, O_RDWR);
        if(st->accel_bus.fd < 0)return RDK_IMU_OPEN_FILE_ERR;

        st->accel_bus.addr = bus_info.bus.i2c.accel.addr;
        st->accel_bus.ops = &i2c_ops;

        snprintf(dev_path, sizeof(dev_path), "/dev/i2c-%u", bus_info.bus.i2c.gyro.bus);
        st->gyro_bus.fd = open(dev_path, O_RDWR);
        if(st->gyro_bus.fd < 0)return RDK_IMU_OPEN_FILE_ERR;

        st->gyro_bus.addr = bus_info.bus.i2c.gyro.addr;
        st->gyro_bus.ops = &i2c_ops;
    }
    if(bus_info.interface == RDK_IMU_SPI){
        snprintf(dev_path, sizeof(dev_path), "/dev/spidev%u.%u", bus_info.bus.spi.accel.bus, bus_info.bus.spi.accel.cs);
        st->accel_bus.fd = open(dev_path, O_RDWR);
        if(st->accel_bus.fd < 0)return RDK_IMU_OPEN_FILE_ERR;

        st->accel_bus.speed_hz = bus_info.bus.spi.gyro.speed_hz;
        st->accel_bus.rd_dummy = BMI088_SPI_ACCEL_RD_DUMMY;
        st->accel_bus.wr_dummy = BMI088_SPI_ACCEL_WR_DUMMY;
        st->accel_bus.ops = &spi_ops;

        snprintf(dev_path, sizeof(dev_path), "/dev/spidev%u.%u", bus_info.bus.spi.gyro.bus, bus_info.bus.spi.gyro.cs);
        st->gyro_bus.fd = open(dev_path, O_RDWR);
        if(st->gyro_bus.fd < 0)return RDK_IMU_OPEN_FILE_ERR;

        st->gyro_bus.speed_hz = bus_info.bus.spi.gyro.speed_hz;
        st->gyro_bus.rd_dummy = BMI088_SPI_GYRO_RD_DUMMY;
        st->gyro_bus.wr_dummy = BMI088_SPI_GYRO_WR_DUMMY;
        st->gyro_bus.ops = &spi_ops;

        uint8_t mode = RDK_IMU_SPID_MODE;
        ioctl(st->accel_bus.fd, SPI_IOC_WR_MODE, &mode);
        ioctl(st->gyro_bus.fd, SPI_IOC_WR_MODE, &mode);
    }

    /* 至此总线初始化完成 */

    return RDK_IMU_OK;
}

rdk_imu_err_t rdk_imu_bus_deinit(
    rdk_imu_state_t *st)
{
    if(!st)return RDK_IMU_CAN_NOT_RELEASE;

    // 关闭并清理加速度计总线
    if (st->accel_bus.fd >= 0) {
        close(st->accel_bus.fd);
        st->accel_bus.fd = -1;
    }
    memset(&st->accel_bus, 0, sizeof(st->accel_bus));
    st->accel_bus.fd = -1; // 确保无效值

    // 关闭并清理陀螺仪总线
    if (st->gyro_bus.fd >= 0) {
        close(st->gyro_bus.fd);
        st->gyro_bus.fd = -1;
    }
    memset(&st->gyro_bus, 0, sizeof(st->gyro_bus));
    st->gyro_bus.fd = -1;

    return RDK_IMU_OK;
}
