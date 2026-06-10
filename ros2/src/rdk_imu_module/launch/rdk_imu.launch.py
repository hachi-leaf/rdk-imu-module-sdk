from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rdk_imu_module',
            executable='rdk_imu_node',
            name='rdk_imu_node',
            output='screen',
            parameters=[{
                'frame_id': 'imu_link',
                'fuse_by': 'accel',
                'max_age_ns': 50000000,
                'publish_rate': 0.0,
                'auto_start': True,
                # 话题和服务名称（可在此覆盖）
                'imu_topic': 'rdkimu/data',
                'enable_service_name': '~/enable',
                'disable_service_name': '~/disable',
                # IMU 传感器配置（默认值）
                'accel_bwp': 'NORMAL',
                'accel_odr': '400',
                'accel_range': '24G',
                'gyro_range': '2000DPS',
                'gyro_bandwidth': 'ODR400_BW47',
                'accel_drdy_int': 'INT1',
                'accel_int_gpio_mode': 'PP_H',
                'gyro_drdy_int': 'INT3',
                'gyro_int_gpio_mode': 'PP_H',
                'accel_drdy_gpio_chip': 4,
                'accel_drdy_gpio_line': 2,
                'gyro_drdy_gpio_chip': 3,
                'gyro_drdy_gpio_line': 12,
                'fifo_length': 256,
                'fifo_mode': 'OVERWRITE',
                'irq_priority': -1,
                'irq_thread_timeout_ns': 1000000000,
            }],
            # 可选：也可以通过重映射覆盖（优先级高于参数）
            # remappings=[
            #     ('imu/data', '/custom/imu'),
            #     ('~/enable', '/custom/enable'),
            #     ('~/disable', '/custom/disable'),
            # ]
        )
    ])