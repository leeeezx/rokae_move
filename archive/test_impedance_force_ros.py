"""
@brief:将三维力传感器数据接入ros系统
"""
import os
import logging
from typing import Optional, List, Dict, Any, Generator
import serial
import time
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import argparse


# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class AsciiSendModel:
    """
    关于传感器ascii模式的类，包括参数设置和工具函数
    """

    def __init__(self,
                 port_name: Optional[str] = None,
                 baudrate: int = 115200,
                 bytesize: int = 8,
                 parity: str = 'N',
                 stopbits: int = 1,
                 timeout: float = 1,
                 minimum_packet_interval: Optional[float] = None,
                 byte_num_of_one_message: int = 7,
                 buffer_size: Optional[int] = None,
                 **kwargs: Any):
        """
        参数初始化

        :param port_name: 串口名称
        :param baudrate: 波特率
        :param bytesize: 数据位
        :param parity: 校验位 ('N', 'E', 'O', 'M', 'S')
        :param stopbits: 停止位
        :param timeout: 超时时间
        :param minimum_packet_interval: 最小包间隔。ascii模式只能保证实际包间隔时间大于仪表的设定值
        :param byte_num_of_one_message: 一个报文的字节数
        :param buffer_size: 串口缓冲区大小
        :param kwargs: 其它参数
        """
        self.port_name = port_name
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = timeout
        self.minimum_packet_interval = minimum_packet_interval
        self.byte_num_of_one_message = byte_num_of_one_message

        # 检查额外的参数
        if kwargs:
            raise ValueError('猪头，压根没有这玩意！： {!r}'.format(kwargs))

        # 配置并打开串口
        self.ser = serial.Serial(
            port=self.port_name,
            baudrate=self.baudrate,
            bytesize=self.bytesize,
            parity=self.parity,
            stopbits=self.stopbits,
            timeout=self.timeout
        )

        logger.info(f"串口 {self.port_name} 已打开")

    def close(self) -> None:
        """显式关闭串口"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            logger.info("串口已关闭")


    """
    *********************工具函数***********************
    """

    def read_sensor_data(self,
                         standard_message_length: int = 7,
                         report_count: int = 50,
                         chunk_size: int = 512) -> Generator[str, None, None]:
        """
        在ascii通讯模式下读取串口数据：
        1. 首先找到第一个回车符(0D)作为数据同步点
        2. 之后的数据才开始正式接收和解码处理

        :param standard_message_length: 标准通讯模式下，符合标准的默认报文长度（以字节为单位）
        :param report_count: 一次抛出的报文数量限制（已经转换为仪表数值，kg为单位）
        :param chunk_size: 缓冲区大小

        :return: 解码后的报文列表
        """
        if not self.ser.is_open:
            logger.error("串口未打开")
            raise serial.SerialException("串口未打开")


        buffer = bytearray()  # 创建空的字节串
        reports = []  # 创建报文空列表

        # 进入数据处理循环
        while True:                     
            # 读取新数据并添加到buffer
            if self.ser.in_waiting:     # 如果串口中有数据等待
                # print('串口有数据')
                chunk = self.ser.read(min(self.ser.in_waiting, chunk_size))  # 从等待区和设置的chunk区中，选一个较小的区，进行读取操作
                buffer.extend(chunk)    # 添加到buffer中

            while len(buffer) >= standard_message_length:         # 当buffer超过默认报文长度7
                cr_index = buffer.find(b'\r')       # cr_index作为空格符的索引
                if cr_index == -1 or cr_index < standard_message_length - 1:  # 如果cr_index不存在或小于6。防止串口刚开始接收数据时，数据被截断。
                    break                   # 没有完整报文，退出循环

                valid_data = buffer[:cr_index + 1]
                buffer = buffer[cr_index + 1:]  # 更新buffer

                try:
                    # 将采集到的数据按照ascii编码进行解码，转换为str类型，然后去除首尾的空白符
                    decoded_data = valid_data.decode('ascii').strip()   
                    # reports.append(decoded_data)
                    # print('report添加完毕')
                    # if len(reports) >= report_count:
                        # print('report已抛出')
                        # total_reports += len(reports)       # 采集率计算
                        # yield reports
                    yield decoded_data  # 生成器，每次返回一个报文    
                        # reports = []
                except UnicodeDecodeError as e:
                    logging.error(f"解码错误 on {self.port_name}：{e}，丢弃无效数据")

            # 如果buffer过大，可能表示数据积压，清理旧数据
            if len(buffer) > chunk_size * 2:
                buffer = buffer[-chunk_size:]
                logger.warning(f"数据积压 on {self.port_name}")

            # 如果没有足够的报文，短暂等待更多数据到达
            if not reports:
                time.sleep(0.005)  # 等待时间，可根据需要调整


# 删除原有的PipeTransmitter类,替换为ROS2发布者节点
class ForceSensorPublisher(Node):
    def __init__(self, ascii_model, axis: str):
        # 初始化ROS2发布者节点名称
        super().__init__(f'force_sensor_{axis}publisher')
        self.publisher = self.create_publisher(Float32 , f'force_{axis}_data', 10)
        self.ascii_model = ascii_model
        self.axis = axis

    def run(self):
        for report in self.ascii_model.read_sensor_data():
            if not rclpy.ok():
                break
                
            try:
                force_value = float(report)
                msg = Float32()
                msg.data = force_value  
                self.publisher.publish(msg)

                # self.get_logger().info(f"发布 {self.axis} 轴力数据: {force_value}")
            except ValueError as e:
                self.get_logger().error(f"数据转换错误 on {self.axis}: {e}")


def main():
    parser = argparse.ArgumentParser(description="Publish force sensor data to ROS2 topic")
    parser.add_argument('--port', type=str, choices=['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2'], required=True, help='Serial port name (e.g., /dev/ttyUSB0 or COM1)')
    parser.add_argument('--axis', type=str, choices=['x', 'y', 'z'], required=True, help='Axis to publish (x, y, or z)')
    args = parser.parse_args()

    rclpy.init()
    
    ascii_model = AsciiSendModel(port_name=args.port, baudrate=115200)
    
    publisher = ForceSensorPublisher(ascii_model, args.axis)
    
    try:
        publisher.run()
    except KeyboardInterrupt:
        pass
    finally:
        ascii_model.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    # 命令行运行示例：
    # python3 src/test_impedance_force_ros.py --port /dev/ttyUSB0 --axis x
    # python3 src/test_impedance_force_ros.py --port /dev/ttyUSB1 --axis y
    # python3 src/test_impedance_force_ros.py --port /dev/ttyUSB2 --axis z
