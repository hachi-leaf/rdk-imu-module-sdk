"""
Python wrapper for librdkimu.so (RDK IMU C library).
"""
import ctypes
import os
from ctypes import (
    c_int32, c_uint8, c_uint32, c_uint64, c_float,
    c_char, c_void_p, Structure, Union, POINTER,
)
from enum import IntEnum


# ---------------- Error codes ----------------
class RDK_IMU_ERR(IntEnum):
    OK = 0
    ERR_PARAM = 1
    I2C_WRITE_ERR = 2
    I2C_READ_ERR = 3
    SPI_WRITE_ERR = 4
    SPI_READ_ERR = 5
    OPEN_DIR_ERR = 6
    OPEN_FILE_ERR = 7
    NO_DEV_ERR = 8
    CHIP_ID_ERR = 9
    CAN_NOT_RELEASE = 10
    GYRO_ENABLE_FAILED = 11
    ACCEL_ENABLE_FAILED = 12
    I2C_UPDATE_FAILED = 13
    SPI_UPDATE_FAILED = 14
    GPIO_INIT_FAILED = 15
    FIFO_FULL = 16
    FIFO_EMPTY = 17
    DEVICE_BUSY = 18
    THREAD_CREATE_FAILED = 19
    FIFO_ALLOC_FAILED = 20


# ---------------- Enums (from rdkimu.h) ----------------
class RDK_IMU_FIFO_MODE(IntEnum):
    DROP = 0
    OVERWRITE = 1

class RDK_IMU_DEVICE(IntEnum):
    ACCEL = 0
    GYRO = 1

class RDK_IMU_ACCEL_BWP(IntEnum):
    OSR4 = 0x08
    OSR2 = 0x09
    NORMAL = 0x0A

class RDK_IMU_ACCEL_ODR(IntEnum):
    RATE_12_5 = 0x05
    RATE_25   = 0x06
    RATE_50   = 0x07
    RATE_100  = 0x08
    RATE_200  = 0x09
    RATE_400  = 0x0A
    RATE_800  = 0x0B
    RATE_1600 = 0x0C

class RDK_IMU_ACCEL_RANGE(IntEnum):
    RANGE_3G  = 0x00
    RANGE_6G  = 0x01
    RANGE_12G = 0x02
    RANGE_24G = 0x03

class RDK_IMU_GYRO_RANGE(IntEnum):
    DPS_2000 = 0x00
    DPS_1000 = 0x01
    DPS_500  = 0x02
    DPS_250  = 0x03
    DPS_125  = 0x04

class RDK_IMU_GYRO_BANDWIDTH(IntEnum):
    ODR2000_BW532 = 0x00
    ODR2000_BW230 = 0x01
    ODR1000_BW116 = 0x02
    ODR400_BW47   = 0x03
    ODR200_BW23   = 0x04
    ODR100_BW12   = 0x05
    ODR200_BW64   = 0x06
    ODR100_BW32   = 0x07

class RDK_IMU_ACCEL_DRDY_INT(IntEnum):
    INT1 = 0
    INT2 = 1

class RDK_IMU_GYRO_DRDY_INT(IntEnum):
    INT3 = 0
    INT4 = 1

class RDK_IMU_GPIO_MODE(IntEnum):
    PP_H = 0
    PP_L = 1
    OD_H = 2
    OD_L = 3

class RDK_IMU_INTERFACE(IntEnum):
    AUTO = 0
    I2C  = 1
    SPI  = 2


# ---------------- Structures ----------------
class RDK_IMU_3_AXIS_DATA(Structure):
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("timestamp_ns", c_uint64),
        ("valid", c_int32),
    ]

class RDK_IMU_6_AXIS_DATA(Structure):
    _fields_ = [
        ("accel", RDK_IMU_3_AXIS_DATA),
        ("gyro", RDK_IMU_3_AXIS_DATA),
    ]

class RDK_IMU_I2C_INFO(Structure):
    _fields_ = [
        ("bus", c_uint8),
        ("addr", c_uint8),
    ]

class RDK_IMU_SPI_INFO(Structure):
    _fields_ = [
        ("bus", c_uint8),
        ("cs", c_uint8),
        ("speed_hz", c_uint32),
    ]

class _BusUnion(Union):
    _fields_ = [
        ("i2c", RDK_IMU_I2C_INFO * 2),   # accel, gyro
        ("spi", RDK_IMU_SPI_INFO * 2),
    ]

class RDK_IMU_BUS_INFO(Structure):
    _fields_ = [
        ("interface", c_int32),
        ("bus", _BusUnion),
    ]

class RDK_IMU_CONFIG(Structure):
    _fields_ = [
        ("accel_bwp", c_int32),
        ("accel_odr", c_int32),
        ("accel_range", c_int32),
        ("gyro_range", c_int32),
        ("gyro_bandwidth", c_int32),
        ("accel_drdy_int", c_int32),
        ("accel_int_gpio_mode", c_int32),
        ("gyro_drdy_int", c_int32),
        ("gyro_int_gpio_mode", c_int32),
        ("accel_drdy_gpio_chip", c_uint32),
        ("accel_drdy_gpio_line", c_uint32),
        ("gyro_drdy_gpio_chip", c_uint32),
        ("gyro_drdy_gpio_line", c_uint32),
        ("fifo_length", c_uint32),
        ("fifo_mode", c_int32),
        ("irq_priority", c_int32),
        ("irq_thread_timeout_ns", c_uint64),
    ]


# ---------------- Default configuration (same as RDK_IMU_X5_DEFAULT_CONFIG) ----------------
DEFAULT_CONFIG = {
    "accel_bwp":              RDK_IMU_ACCEL_BWP.OSR4,
    "accel_odr":              RDK_IMU_ACCEL_ODR.RATE_400,
    "accel_range":            RDK_IMU_ACCEL_RANGE.RANGE_24G,
    "gyro_range":             RDK_IMU_GYRO_RANGE.DPS_2000,
    "gyro_bandwidth":         RDK_IMU_GYRO_BANDWIDTH.ODR400_BW47,
    "accel_drdy_int":         RDK_IMU_ACCEL_DRDY_INT.INT1,
    "accel_int_gpio_mode":    RDK_IMU_GPIO_MODE.PP_H,
    "gyro_drdy_int":          RDK_IMU_GYRO_DRDY_INT.INT3,
    "gyro_int_gpio_mode":     RDK_IMU_GPIO_MODE.PP_H,
    "accel_drdy_gpio_chip":   4,
    "accel_drdy_gpio_line":   2,
    "gyro_drdy_gpio_chip":    3,
    "gyro_drdy_gpio_line":    12,
    "fifo_length":            256,
    "fifo_mode":              RDK_IMU_FIFO_MODE.OVERWRITE,
    "irq_priority":           -1,
    "irq_thread_timeout_ns":  1000000000,
}


class IMU:
    """
    Python wrapper for the RDK IMU C library.
    
    Basic usage:
        imu = IMU()
        imu.bus()                  # auto scan
        imu.config()               # use default X5 config
        imu.enable()
        data, count = imu.read_indep()
        ...
        imu.disable()
    """

    def __init__(self, lib_path: str = None):
        self._lib = self._load_lib(lib_path)
        self._setup_functions()
        self._handle = self._lib.rdk_imu_create_default()
        if not self._handle:
            raise MemoryError("rdk_imu_create_default returned NULL")
        self._enabled = False

    def _load_lib(self, path: str = None):
        """Load the shared library. Users must ensure librdkimu.so is accessible."""
        if path is None:
            # Try common locations
            lib_name = "librdkimu.so"
            try:
                return ctypes.CDLL(lib_name)
            except OSError:
                # Try next to this file
                candidate = os.path.join(os.path.dirname(__file__), lib_name)
                if os.path.exists(candidate):
                    return ctypes.CDLL(candidate)
                # Try env var
                env_path = os.environ.get("RDKIMU_LIB_PATH", "")
                if env_path:
                    return ctypes.CDLL(env_path)
                raise RuntimeError(
                    "Cannot find librdkimu.so. "
                    "Place it in a standard library path, next to imu.py, "
                    "or set RDKIMU_LIB_PATH environment variable."
                )
        else:
            return ctypes.CDLL(path)

    def _setup_functions(self):
        """Set argument and return types for all used C functions."""
        lib = self._lib

        # rdk_imu_create_default
        lib.rdk_imu_create_default.restype = c_void_p

        # rdk_imu_destroy
        lib.rdk_imu_destroy.argtypes = [c_void_p]
        lib.rdk_imu_destroy.restype = c_int32

        # rdk_imu_bus_init
        lib.rdk_imu_bus_init.argtypes = [c_void_p, RDK_IMU_BUS_INFO]
        lib.rdk_imu_bus_init.restype = c_int32

        # rdk_imu_bus_deinit
        lib.rdk_imu_bus_deinit.argtypes = [c_void_p]
        lib.rdk_imu_bus_deinit.restype = c_int32

        # rdk_imu_device_init
        lib.rdk_imu_device_init.argtypes = [c_void_p, RDK_IMU_CONFIG]
        lib.rdk_imu_device_init.restype = c_int32

        # rdk_imu_device_deinit
        lib.rdk_imu_device_deinit.argtypes = [c_void_p]
        lib.rdk_imu_device_deinit.restype = c_int32

        # rdk_imu_enable
        lib.rdk_imu_enable.argtypes = [c_void_p]
        lib.rdk_imu_enable.restype = c_int32

        # rdk_imu_disable
        lib.rdk_imu_disable.argtypes = [c_void_p]
        lib.rdk_imu_disable.restype = c_int32

        # rdk_imu_fifo_available
        lib.rdk_imu_fifo_available.argtypes = [c_void_p, POINTER(c_uint32)]
        lib.rdk_imu_fifo_available.restype = c_int32

        # rdk_imu_read_indep
        lib.rdk_imu_read_indep.argtypes = [
            c_void_p,
            POINTER(RDK_IMU_6_AXIS_DATA),
            POINTER(c_uint32),
        ]
        lib.rdk_imu_read_indep.restype = c_int32

        # rdk_imu_read_fused
        lib.rdk_imu_read_fused.argtypes = [
            c_void_p,
            POINTER(RDK_IMU_6_AXIS_DATA),
            c_int32,            # fuse_by
            c_uint64,           # max_age_ns
        ]
        lib.rdk_imu_read_fused.restype = c_int32

    def _check_ret(self, ret, msg="C function failed"):
        """Raise exception on non‑OK return codes."""
        if ret != RDK_IMU_ERR.OK:
            raise RuntimeError(f"{msg} (error code {ret})")

    # ---------- Public API ----------

    def bus(self,
            interface=RDK_IMU_INTERFACE.AUTO,
            i2c_accel_bus=0, i2c_accel_addr=0x18,
            i2c_gyro_bus=0,  i2c_gyro_addr=0x68,
            spi_accel_bus=0, spi_accel_cs=0, spi_accel_speed=1000000,
            spi_gyro_bus=0,  spi_gyro_cs=1,  spi_gyro_speed=1000000):
        """
        Initialize the communication bus.
        If interface is AUTO, performs automatic I2C/SPI scanning.
        """
        bus_info = RDK_IMU_BUS_INFO()
        bus_info.interface = int(interface)

        # Fill I2C part (not used if SPI is chosen but safe)
        bus_info.bus.i2c[0].bus = i2c_accel_bus
        bus_info.bus.i2c[0].addr = i2c_accel_addr
        bus_info.bus.i2c[1].bus = i2c_gyro_bus
        bus_info.bus.i2c[1].addr = i2c_gyro_addr

        # Fill SPI part
        bus_info.bus.spi[0].bus = spi_accel_bus
        bus_info.bus.spi[0].cs = spi_accel_cs
        bus_info.bus.spi[0].speed_hz = spi_accel_speed
        bus_info.bus.spi[1].bus = spi_gyro_bus
        bus_info.bus.spi[1].cs = spi_gyro_cs
        bus_info.bus.spi[1].speed_hz = spi_gyro_speed

        ret = self._lib.rdk_imu_bus_init(self._handle, bus_info)
        self._check_ret(ret, "rdk_imu_bus_init failed")

    def config(self, config_dict=None):
        """
        Initialize the IMU device with configuration parameters.
        
        Args:
            config_dict: A dict with key-value pairs to override defaults.
                        If None, the default X5 configuration is used.
                        Example:
                            imu.config({
                                'accel_range': RDK_IMU_ACCEL_RANGE.RANGE_12G,
                                'gyro_bandwidth': RDK_IMU_GYRO_BANDWIDTH.ODR100_BW32,
                            })
        """
        cfg = RDK_IMU_CONFIG()
        # fill in defaults
        for field_name, default_val in DEFAULT_CONFIG.items():
            setattr(cfg, field_name, int(default_val))

        if config_dict is not None:
            if not isinstance(config_dict, dict):
                raise TypeError("config_dict must be a dict")
            for key, value in config_dict.items():
                if not hasattr(cfg, key):
                    raise KeyError(f"Invalid config parameter: {key}")
                setattr(cfg, key, int(value))

        ret = self._lib.rdk_imu_device_init(self._handle, cfg)
        self._check_ret(ret, "rdk_imu_device_init failed")

    def enable(self):
        """Start data acquisition (interrupt threads)."""
        ret = self._lib.rdk_imu_enable(self._handle)
        self._check_ret(ret, "rdk_imu_enable failed")
        self._enabled = True

    def disable(self):
        """Stop data acquisition and join threads."""
        if self._enabled:
            ret = self._lib.rdk_imu_disable(self._handle)
            self._check_ret(ret, "rdk_imu_disable failed")
            self._enabled = False

    def fifo_available(self) -> int:
        """Return number of raw data packets currently in FIFO."""
        count = c_uint32()
        ret = self._lib.rdk_imu_fifo_available(self._handle, ctypes.byref(count))
        self._check_ret(ret, "rdk_imu_fifo_available failed")
        return count.value

    def read_indep(self):
        """
        Read one independent data packet from FIFO.
        Returns (data: RDK_IMU_6_AXIS_DATA, remaining_count: int).
        """
        data = RDK_IMU_6_AXIS_DATA()
        remaining = c_uint32()
        ret = self._lib.rdk_imu_read_indep(
            self._handle,
            ctypes.byref(data),
            ctypes.byref(remaining),
        )
        self._check_ret(ret, "rdk_imu_read_indep failed")
        return data, remaining.value

    def read_fused(self, fuse_by=RDK_IMU_DEVICE.ACCEL, max_age_ns=50000000):
        """
        Read a fused data packet (blocking until interpolation conditions are met).
        fuse_by: RDK_IMU_DEVICE.ACCEL or RDK_IMU_DEVICE.GYRO
        Returns RDK_IMU_6_AXIS_DATA.
        """
        data = RDK_IMU_6_AXIS_DATA()
        ret = self._lib.rdk_imu_read_fused(
            self._handle,
            ctypes.byref(data),
            c_int32(int(fuse_by)),
            c_uint64(max_age_ns),
        )
        self._check_ret(ret, "rdk_imu_read_fused failed")
        return data

    def close(self):
        """Release all resources (disable, deinit, destroy)."""
        if self._handle is not None:
            try:
                if self._enabled:
                    self.disable()
                self._lib.rdk_imu_device_deinit(self._handle)
                self._lib.rdk_imu_bus_deinit(self._handle)
                self._lib.rdk_imu_destroy(self._handle)
            except Exception:
                pass
            finally:
                self._handle = None
                self._enabled = False

    def __del__(self):
        self.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False
